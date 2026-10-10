#!/usr/bin/env python3
"""Offline qualitative driving audit; no ROS publishing or calibration edits."""
import argparse
import json
import shutil
import subprocess
from pathlib import Path
import cv2
import numpy as np
import rosbag
import rospy
import yaml
from sensor_msgs import point_cloud2

ROOT = Path('/home/nuc/lxw/neupan_ws')
OUT = ROOT / 'rosbag_videos/rosbag20261008_155045/extrinsics_driving'


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--indices', default='0,3,6,9,13', help='Comma-separated split indices, or all')
    parser.add_argument('--offset', type=float, default=2)
    parser.add_argument('--duration', type=float, default=6)
    parser.add_argument('--fps', type=float, default=5)
    parser.add_argument('--output', type=Path, default=OUT)
    parser.add_argument('--h264', action='store_true', help='Also encode browser/Ubuntu-compatible H.264 with ffmpeg')
    args = parser.parse_args()
    if args.duration <= 0 or args.fps <= 0 or args.offset < .5:
        parser.error('duration/fps must be positive, offset must be >= 0.5')
    if args.h264 and not shutil.which('ffmpeg'):
        parser.error('--h264 requires ffmpeg')
    OUT = args.output
    OUT.mkdir(parents=True, exist_ok=True)
    calibration = yaml.safe_load((ROOT/'src/usb_camera/config/livox_camera_extrinsics.yaml').read_text())
    lio = yaml.safe_load((ROOT/'src/FAST_LIO/config/mid360.yaml').read_text())['mapping']
    r = np.array(calibration['R_camera_lidar']['data']).reshape(3,3)
    t = np.array(calibration['t_camera_lidar'])
    ri = np.array(lio['extrinsic_R']).reshape(3,3)
    ti = np.array(lio['extrinsic_T'])
    report = {'limitations': 'Receive-time pairing; unknown exposure latency. Current camera and LIO extrinsics, not historical parameters. No motion compensation between camera and cloud times.', 'clips': []}
    indices = range(14) if args.indices == 'all' else [int(x) for x in args.indices.split(',')]
    for index in indices:
        path = ROOT / ('rosbags/rosbag20261008_155045/hybridnets_vehicle_data_20261008_155045_%d.bag' % index)
        images, clouds, odoms, commands, info = [], [], [], [], None
        with rosbag.Bag(str(path)) as bag:
            start = bag.get_start_time()+args.offset
            end = min(start+args.duration, bag.get_end_time())
            for topic, msg, stamp in bag.read_messages(topics=['/camera/remote_image_raw','/camera/remote_camera_info','/cloud_registered_body','/Odometry','/cmd_vel/selected'], start_time=rospy.Time.from_sec(start-.5), end_time=rospy.Time.from_sec(end+.5)):
                ts = stamp.to_sec()
                if topic.endswith('image_raw'):
                    if ts>=start and ts<end and (not images or ts-images[-1][0]>=.9/args.fps):
                        images.append((ts,msg))
                elif topic.endswith('camera_info'):
                    info = msg
                elif topic == '/cloud_registered_body':
                    points = np.array(list(point_cloud2.read_points(msg,field_names=('x','y','z'),skip_nans=True)))
                    # FAST-LIO body is IMU frame: undo its LiDAR-to-IMU extrinsic.
                    lidar = (points-ti) @ ri
                    camera = lidar @ r.T+t
                    camera = camera[np.isfinite(camera).all(axis=1)&(camera[:,2]>.5)&(camera[:,2]<25)]
                    clouds.append((ts,msg.header.stamp.to_sec(),camera))
                elif topic == '/Odometry':
                    q = msg.pose.pose.orientation
                    yaw = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    p = msg.pose.pose.position
                    odoms.append((ts,p.x,p.y,float(yaw)))
                else:
                    commands.append(msg.linear.x)
        if not images or not clouds or info is None:
            continue
        writer = cv2.VideoWriter(str(OUT/('clip_%02d_preview.mp4'%index)),cv2.VideoWriter_fourcc(*'mp4v'),args.fps,(1280,720))
        if not writer.isOpened():
            raise RuntimeError('Cannot open video writer')
        cloud_times = np.array([c[0] for c in clouds])
        deltas = []
        for number,(ts,msg) in enumerate(images):
            image=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
            if msg.encoding=='rgb8':
                image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            for shift in (0,):
                j=int(np.argmin(abs(cloud_times-(ts+shift))))
                cloud_ts,header,camera=clouds[j]
                delta=1000*(ts-cloud_ts)
                deltas.append(delta)
                uv,_=cv2.projectPoints(camera,np.zeros(3),np.zeros(3),np.array(info.K).reshape(3,3),np.array(info.D))
                uv=uv.reshape(-1,2)
                mask=(uv[:,0]>=0)&(uv[:,0]<msg.width)&(uv[:,1]>=0)&(uv[:,1]<msg.height)
                uv,depth=uv[mask],camera[mask,2]
                colors=cv2.applyColorMap(np.uint8(255*(1-np.clip(depth/25,0,1))),cv2.COLORMAP_TURBO).reshape(-1,3)
                for pixel,color in zip(uv,colors):
                    cv2.circle(image,tuple(np.rint(pixel).astype(int)),1,tuple(int(v) for v in color),-1)
            cv2.rectangle(image,(0,0),(1250,90),(0,0,0),-1)
            cv2.putText(image,'bag %02d | undistorted cloud | receive delta %.0f ms'%(index,delta),(15,32),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
            cv2.putText(image,'Camera exposure latency unknown; warm=near, cool=far',(15,70),cv2.FONT_HERSHEY_SIMPLEX,.8,(255,255,255),2)
            writer.write(cv2.resize(image,(1280,720)))
            if number in (0,len(images)//4,len(images)//2,3*len(images)//4,len(images)-1):
                cv2.imwrite(str(OUT/('clip_%02d_frame%02d.jpg'%(index,number))),image)
        writer.release()
        if args.h264:
            subprocess.run(['ffmpeg','-nostdin','-hide_banner','-loglevel','error',
                            '-n','-i',str(OUT/('clip_%02d_preview.mp4'%index)),
                            '-c:v','libx264','-preset','fast','-crf','20',
                            '-pix_fmt','yuv420p','-movflags','+faststart',
                            str(OUT/('clip_%02d.mp4'%index))],check=True)
        elapsed=odoms[-1][0]-odoms[0][0] if len(odoms)>1 else 0
        distance=float(np.linalg.norm(np.array(odoms[-1][1:3])-np.array(odoms[0][1:3]))) if elapsed else None
        yaw_change=float(np.degrees(np.unwrap(np.array([o[3] for o in odoms]))[-1]-np.unwrap(np.array([o[3] for o in odoms]))[0])) if elapsed else None
        report['clips'].append({'bag_index':index,'frames':len(images),'image_start':images[0][0],'image_end':images[-1][0],'seconds':elapsed,'net_distance_m':distance,'yaw_change_deg':yaw_change,'median_command_mps':float(np.median(commands)) if commands else None,'receive_delta_ms_minmax':[min(deltas),max(deltas)]})
        print(report['clips'][-1],flush=True)
    (OUT/'report.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
