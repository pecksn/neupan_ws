#!/usr/bin/env python3
"""Offline qualitative driving audit; no ROS publishing or calibration edits."""
import argparse
from collections import Counter
import json
import shutil
import subprocess
import sys
from pathlib import Path
import cv2
import numpy as np
import rosbag
import rospy
import yaml
from sensor_msgs import point_cloud2

ROOT = Path('/home/nuc/lxw/neupan_ws')
OUT = ROOT / 'rosbag_videos/rosbag20261008_155045/extrinsics_driving'


def dense_route(points):
    dense=[]
    for a,b in zip(points[:-1],points[1:]):
        n=max(1,int(np.ceil(np.linalg.norm(b-a)/.5)))
        dense.extend(a+(b-a)*fraction for fraction in np.arange(n)/n)
    return np.vstack((dense,points[-1]))


def compare_existing_cache(output):
    """Matched-frame temporal comparison, excluding live control safety gates."""
    from camera_right_path import SegmentCache,bounded_offsets,load_osm_junctions,osm_guidance_context,project_ground_path
    configuration=json.loads((output/'report.json').read_text())
    target_clearance=configuration.get('target_camera_clearance_m',2.)
    ground_z=-configuration.get('base_link_height_m',.20)
    junctions=load_osm_junctions('/home/nuc/lxw/ros_maps/map.osm',23.0398,113.39002)
    camera=yaml.safe_load((ROOT/'src/usb_camera/config/usb_camera.yaml').read_text())
    ext=yaml.safe_load((ROOT/'src/usb_camera/config/livox_camera_extrinsics.yaml').read_text())
    r=np.array(ext['R_camera_lidar']['data']).reshape(3,3).T
    origin=np.array([.1,0,.35])-r@np.array(ext['t_camera_lidar'])
    k=np.array(camera['camera_matrix']['data']).reshape(3,3)
    d=np.array(camera['distortion_coefficients']['data'])
    route=np.array(json.loads((output/'route_points.json').read_text()))
    direction=np.vstack((np.diff(route,axis=0),route[-1]-route[-2]))
    normals=np.column_stack((-direction[:,1],direction[:,0]))/np.maximum(np.linalg.norm(direction,axis=1),1.e-9)[:,None]
    cache=SegmentCache()
    last_time=None;first=np.zeros(len(route));second=first.copy();old_first=first.copy();old_second=second.copy()
    streak=0;scalar=0;metrics=[];writer=cv2.VideoWriter(str(output/'comparison_preview.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),3,(1920,540))
    if not writer.isOpened():raise RuntimeError('cannot create comparison video')
    for file in sorted(output.glob('clip_*_frames.json')):
        index=int(file.name.split('_')[1])
        frames=json.loads(file.read_text())
        cap=cv2.VideoCapture(str(output/('clip_%02d_preview.mp4'%index)))
        for number,frame in enumerate(frames):
            ok,image=cap.read()
            if not ok:raise RuntimeError('missing preview frame')
            now=frame['bag_time'];dt=min(1,now-last_time) if last_time else .33
            position=frame.get('vehicle_map_xy');yaw=frame.get('vehicle_map_yaw')
            context=osm_guidance_context(route,position,yaw,junctions) if position is not None else 'osm_pose_unavailable'
            obs=frame.get('cache_observations',[])
            if context!='straight' or frame.get('scene_type') in ('intersection','open_area'):
                if context=='straight':context='scene_junction'
                obs=[];cache.clear();second[:]=0
            if obs:
                streak+=1
                target=float(np.median([v for _,v in obs]))
                for i,v in obs:cache.observe(i,v,now,now)
            else:
                streak=0;scalar=0
            first=np.zeros(len(route))
            if streak>=3:
                scalar+=float(np.clip(target-scalar,-.1*dt,.1*dt))
                for i,_ in obs:first[i]=scalar
                first=bounded_offsets(route,first)
            forward=np.zeros(len(route))
            if position is not None:
                delta=route-np.array(position)
                forward=delta[:,0]*np.cos(yaw)+delta[:,1]*np.sin(yaw)
                for i in list(cache.committed):
                    if forward[i]<-2:cache.committed.pop(i)
            values=cache.values(now)
            goal=np.zeros(len(route))
            for i,v in values.items():goal[i]=v
            goal=bounded_offsets(route,goal)
            second+=np.clip(goal-second,-.15*dt,.15*dt)
            second[np.array([i not in values for i in range(len(route))])]=0
            second=bounded_offsets(route,second)
            ahead=(forward>0)&(forward<6) if position is not None else np.zeros(len(route),bool)
            active_first=bool(np.any(abs(first[ahead])>.001))
            active_second=bool(np.any(abs(second[ahead])>.001))
            metric={'bag_time':now,'realtime_active':active_first,'cache_active':active_second,'cached_points':len(values),
                    'osm_context':context,'input_reason':frame['reason'],
                    'realtime_max_shift_m':float(np.max(abs(first))),'cache_max_shift_m':float(np.max(abs(second))),
                    'realtime_change_m':float(np.mean(abs(first[ahead]-old_first[ahead]))) if ahead.any() and last_time and now-last_time<1 else None,
                    'cache_change_m':float(np.mean(abs(second[ahead]-old_second[ahead]))) if ahead.any() and last_time and now-last_time<1 else None}
            metrics.append(metric)
            panes=[]
            for label,offsets,active,color in [('REALTIME + OSM',first,active_first,(255,100,0)),('CACHE + OSM',second,active_second,(0,140,255))]:
                pane=cv2.resize(image,(960,540))
                if position is not None:
                    shifted=route+normals*offsets[:,None]
                    delta=shifted-np.array(position)
                    base=np.column_stack((delta[:,0]*np.cos(yaw)+delta[:,1]*np.sin(yaw),-delta[:,0]*np.sin(yaw)+delta[:,1]*np.cos(yaw),np.full(len(route),ground_z)))
                    for segment in project_ground_path(base,origin,r,k,d,1920,1080):
                        cv2.polylines(pane,[np.rint(segment/2).astype(np.int32)],False,color,3,cv2.LINE_AA)
                cv2.rectangle(pane,(0,0),(960,90),(0,0,0),-1)
                cv2.putText(pane,'%s | camera target %.1fm'%(label,target_clearance),(12,25),cv2.FONT_HERSHEY_SIMPLEX,.6,color,2)
                cv2.putText(pane,'OFFLINE GEOMETRY/STATE ONLY - NOT EXECUTED CONTROL',(12,52),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
                source='CAMERA' if active else 'OSM'
                cv2.putText(pane,'%s | %s | clip %02d frame %d'%(source,context,index,number),(12,78),cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
                panes.append(pane)
            writer.write(np.hstack(panes))
            old_first=first.copy();old_second=second.copy();last_time=now
        cap.release()
    writer.release()
    summary={'samples':len(metrics),'target_camera_center_clearance_m':target_clearance,'base_link_height_m':-ground_z,'camera_height_above_ground_m':float(origin[2]-ground_z),'comparison_scope':'same geometry inputs with OSM turn/junction fallback, gates for timing/localization not applied; not full NeuPAN replay',
             'osm_context_counts':dict(Counter(m['osm_context'] for m in metrics)),
             'realtime_active_samples':sum(m['realtime_active'] for m in metrics),'cache_active_samples':sum(m['cache_active'] for m in metrics),
             'realtime_mean_change_m':float(np.mean([m['realtime_change_m'] for m in metrics if m['realtime_change_m'] is not None])),
             'cache_mean_change_m':float(np.mean([m['cache_change_m'] for m in metrics if m['cache_change_m'] is not None]))}
    (output/'comparison_metrics.json').write_text(json.dumps({'summary':summary,'frames':metrics},indent=2))
    print(json.dumps(summary,indent=2),flush=True)


def plot_existing_audit(output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    frames=[]
    for file in sorted(output.glob('clip_*_frames.json')):
        frames.extend(json.loads(file.read_text()))
    if not frames:
        raise ValueError('no existing frame audit files')
    first=ROOT/'rosbags/rosbag20261008_155045/hybridnets_vehicle_data_20261008_155045_0.bag'
    with rosbag.Bag(str(first)) as bag:
        _,msg,_=next(bag.read_messages(topics=['/osm_global_path']))
        route=np.array([[pose.pose.position.x,pose.pose.position.y] for pose in msg.poses])
    fig,axes=plt.subplots(1,3,figsize=(18,6))
    axes[0].plot(route[:,0],route[:,1],color='cyan',label='Recorded global route')
    for candidate,color,label in ((False,'gray','Sampled vehicle TF poses'),(True,'green','Geometry-pass sample poses')):
        xy=np.array([f['vehicle_map_xy'] for f in frames if 'vehicle_map_xy' in f and f['geometry_pass']==candidate])
        if len(xy):axes[0].scatter(xy[:,0],xy[:,1],s=8,c=color,label=label)
    axes[0].set_aspect('equal',adjustable='datalim');axes[0].legend();axes[0].set_title('Recorded route vs vehicle poses (map)')
    times=np.array([f['bag_time'] for f in frames])-frames[0]['bag_time']
    axes[1].scatter(times,[f.get('source_age_ms',np.nan) for f in frames],s=5,label='GPU source_age_ms')
    axes[1].scatter(times,[f.get('capture_to_image_receive_ms',np.nan) for f in frames],s=5,label='Capture to NUC image receipt')
    axes[1].axhline(300,c='red',ls='--',label='Source-age gate 300 ms')
    axes[1].axhline(500,c='orange',ls='--',label='Capture-age gate 500 ms')
    axes[1].set_xlabel('Bag time relative to first sample (s)');axes[1].set_ylabel('Timestamp difference (ms)');axes[1].legend();axes[1].set_title('Timestamp gaps, not proven network latency')
    scales=['1.0','1.5','2.0']
    counts=[sum(f.get('height_sweep_geometry',{}).get(scale,False) for f in frames) for scale in scales]
    axes[2].bar(scales,counts);axes[2].set_xlabel('Assumed camera height multiplier');axes[2].set_ylabel('Geometry-pass samples');axes[2].set_title('Height sensitivity ONLY, not calibration')
    for i,n in enumerate(counts):axes[2].text(i,n+2,str(n),ha='center')
    fig.tight_layout();fig.savefig(str(output/'route_and_gates.png'),dpi=150);plt.close(fig)
    values=[f.get('source_age_ms') for f in frames if f.get('source_age_ms') is not None]
    capture=[f.get('capture_to_image_receive_ms') for f in frames if f.get('capture_to_image_receive_ms') is not None]
    comparisons=[f for f in frames if 'route_shift_gate_pass' in f]
    segments=route[1:]-route[:-1]
    squared=(segments**2).sum(axis=1)
    signed=[];tracking_times=[]
    for f in frames:
        if 'vehicle_map_xy' not in f:
            continue
        point=np.array(f['vehicle_map_xy'])
        fraction=np.clip(((point-route[:-1])*segments).sum(axis=1)/np.maximum(squared,1.e-12),0,1)
        closest=route[:-1]+fraction[:,None]*segments
        index=int(np.argmin(((point-closest)**2).sum(axis=1)))
        normal=np.array([-segments[index,1],segments[index,0]])/max(np.sqrt(squared[index]),1.e-12)
        signed.append(float((point-closest[index])@normal));tracking_times.append(f['bag_time']-frames[0]['bag_time'])
    fig,ax=plt.subplots(figsize=(10,4))
    ax.scatter(tracking_times,signed,s=8)
    ax.axhline(0,color='black',ls='--');ax.set_xlabel('Relative bag time (s)');ax.set_ylabel('Signed lateral error (m), left positive')
    ax.set_title('Recorded vehicle TF vs nearest global route segment; sampled poses')
    fig.tight_layout();fig.savefig(str(output/'recorded_tracking_error.png'),dpi=150);plt.close(fig)
    summary={'samples':len(frames),'geometry_pass':sum(f['geometry_pass'] for f in frames),
             'source_age_ms_percentiles':np.percentile(values,[0,50,95,100]).tolist(),
             'capture_to_image_receive_ms_percentiles':np.percentile(capture,[0,50,95,100]).tolist(),
             'height_sweep_pass_counts':dict(zip(scales,counts)),
             'global_route_comparable_samples':len(comparisons),
             'route_shift_gate_pass':sum(f['route_shift_gate_pass'] for f in comparisons),
             'required_shift_percentiles':np.percentile([v for f in comparisons for v in f['required_route_shift_m']],[0,50,95,100]).tolist() if comparisons else None}
    summary['recorded_tracking_abs_error_m_percentiles']=np.percentile(np.abs(signed),[50,95,100]).tolist() if signed else None
    (output/'audit_summary.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2),flush=True)


def candidate_overlay(image, stamp, packets, matrix, distortion, rotation, origin,
                      route=None, transforms=None,clearance=2.0,ground_z=-.2):
    """Reuse control geometry, without claiming to run the full planner/gates."""
    from camera_right_path import boundary_curve, lane_target,project_ground_path
    available = [entry for entry in packets if entry[0] <= stamp]
    reason, candidate = 'JSON missing or stale', False
    packet_time = None
    details = {}
    local_route = None
    if route is not None and transforms is not None:
        import tf.transformations as tr
        try:
            matrices=[]
            for edge in ('map->odom','odom->base_link'):
                translation,quaternion=transforms[edge]
                transform=tr.quaternion_matrix(quaternion)
                transform[:3,3]=translation
                matrices.append(transform)
            map_base=matrices[0]@matrices[1]
            base_map=np.linalg.inv(map_base)
            ground_map=np.column_stack((route[:,0],route[:,1],np.full(len(route),map_base[2,3]+ground_z),np.ones(len(route))))
            local_route=(ground_map@base_map.T)[:,:3]
            local_route[:,2]=ground_z
            dense=[]
            for a,b in zip(local_route[:-1],local_route[1:]):
                count=max(1,int(np.ceil(np.linalg.norm(b-a)/.25)))
                dense.extend(a+(b-a)*fraction for fraction in np.arange(count)/count)
            display_route=np.asarray(dense)
            for segment in project_ground_path(display_route,origin,rotation,matrix,distortion,image.shape[1],image.shape[0]):
                cv2.polylines(image,[np.rint(segment).astype(np.int32)],False,(255,255,0),4,cv2.LINE_AA)
            details['vehicle_map_xy']=map_base[:2,3].tolist()
            details['vehicle_map_yaw']=float(np.arctan2(map_base[1,0],map_base[0,0]))
        except (KeyError,ValueError,np.linalg.LinAlgError):
            details['route_check']='TF unavailable'
    if available:
        packet_time, packet = available[-1]
        details['source_age_ms']=packet.get('source_age_ms')
        details['capture_to_image_receive_ms']=(stamp-float(packet.get('capture_timestamp_ns',0))/1.e9)*1000
        details['confidence']=packet.get('confidence')
        details['scene_type']=packet.get('scene_type')
        details['height_sweep_geometry']={}
        # Sensitivity only: never use altered height for the control candidate.
        if all(packet.get(k) is True for k in ('valid','road_polygon_valid','left_boundary_valid','right_boundary_valid')):
            for scale in (1.,1.5,2.):
                alternate=origin.copy();alternate[2]=ground_z+(origin[2]-ground_z)*scale
                try:
                    cc=[boundary_curve(packet.get(side+'_valid_segments_px',[packet.get(side+'_boundary_px',[])]),matrix,distortion,rotation,alternate,ground_z) for side in ('left','right')]
                    lane_target(*cc,clearance,.75)
                    details['height_sweep_geometry'][str(scale)]=True
                except (ValueError,TypeError,KeyError,cv2.error):
                    details['height_sweep_geometry'][str(scale)]=False
        try:
            if stamp-packet_time > .5:
                raise ValueError(reason)
            if not all(packet.get(k) is True for k in ('valid','road_polygon_valid','left_boundary_valid','right_boundary_valid')):
                raise ValueError('both boundaries required')
            if packet.get('coordinate_space') != 'rotated_image_pixels' or (packet.get('image_width'),packet.get('image_height')) != (1920,1080):
                raise ValueError('image coordinates mismatch')
            if packet.get('scene_type') in ('intersection','open_area'):
                raise ValueError('junction or open area')
            # Draw valid measured segments, never join across missing sections.
            curves=[]
            for side,color in [('left',(255,0,255)),('right',(0,255,255))]:
                segments=packet.get(side+'_valid_segments_px')
                if segments is None:
                    segments=[packet.get(side+'_boundary_px',[])]
                for segment in segments:
                    points=np.asarray(segment,float).reshape(-1,2)
                    if len(points)>1:
                        cv2.polylines(image,[np.rint(points).astype(np.int32)],False,color,2,cv2.LINE_AA)
                curves.append(boundary_curve(segments,matrix,distortion,rotation,origin,ground_z))
            x,y,_,_=lane_target(*curves,clearance,.75)
            if local_route is not None:
                usable=(local_route[:,0]>=x[0])&(local_route[:,0]<=x[-1])&(abs(local_route[:,1])<5)
                if usable.any():
                    desired=np.interp(local_route[usable,0],x,y)-local_route[usable,1]
                    details['required_route_shift_m']=desired.tolist()
                    details['route_shift_gate_pass']=bool(np.all(np.abs(desired)<=1.5))
                    indices=np.flatnonzero(usable)
                    direction=np.diff(route,axis=0)
                    direction=np.vstack((direction,direction[-1]))
                    yaw=np.arctan2(direction[:,1],direction[:,0])-details['vehicle_map_yaw']
                    angle=np.arctan2(np.sin(yaw),np.cos(yaw))
                    if np.all(np.abs(desired)<=1.5) and np.all(np.abs(angle[usable])<=np.deg2rad(15)):
                        details['cache_observations']=[[int(i),float(v)] for i,v in zip(indices,desired)]
                else:
                    details['route_check']='no global points in shared camera range'
            ground=np.column_stack((x,y,np.full(len(x),ground_z)))
            for segment in project_ground_path(ground,origin,rotation,matrix,distortion,image.shape[1],image.shape[0]):
                cv2.polylines(image,[np.rint(segment).astype(np.int32)],False,(0,255,0),5,cv2.LINE_AA)
            candidate=True
            reason='GEOMETRY PASS - candidate only'
        except (ValueError,TypeError,KeyError,cv2.error) as error:
            reason=str(error)
    cv2.rectangle(image,(0,0),(image.shape[1],145),(0,0,0),-1)
    for row,text in enumerate(['OFFLINE BAG GEOMETRY - NOT NEUPAN CONTROL',reason,
                              'Green=candidate; cyan=global route; magenta=left; yellow=right']):
        cv2.putText(image,text,(15,35+43*row),cv2.FONT_HERSHEY_SIMPLEX,.8,
                    (0,255,0) if candidate and row==1 else (255,255,255),2)
    return dict({'bag_time':stamp,'json_bag_time':packet_time,'geometry_pass':candidate,'reason':reason},**details)


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--indices', default='0,3,6,9,13', help='Comma-separated split indices, or all')
    parser.add_argument('--offset', type=float, default=2)
    parser.add_argument('--duration', type=float, default=6)
    parser.add_argument('--fps', type=float, default=5)
    parser.add_argument('--output', type=Path, default=OUT)
    parser.add_argument('--h264', action='store_true', help='Also encode browser/Ubuntu-compatible H.264 with ffmpeg')
    parser.add_argument('--camera-right-candidate', action='store_true', help='Visualize camera ground geometry instead of LiDAR overlay; NOT a full control replay')
    parser.add_argument('--compare-global-path', action='store_true', help='Add recorded map path/TF comparison and height sensitivity audit')
    parser.add_argument('--plot-existing-audit', action='store_true', help='Summarize existing route comparison output without replaying frames')
    parser.add_argument('--center-clearance',type=float,default=2.0)
    parser.add_argument('--base-link-height',type=float,default=.20)
    parser.add_argument('--compare-cache-existing',action='store_true')
    args = parser.parse_args()
    if args.compare_global_path:
        args.camera_right_candidate=True
    if args.duration <= 0 or args.fps <= 0 or args.offset < .5:
        parser.error('duration/fps must be positive, offset must be >= 0.5')
    if not np.isfinite(args.base_link_height) or not 0<=args.base_link_height<=1:
        parser.error('base-link-height must be in [0,1] metres')
    if args.h264 and not shutil.which('ffmpeg'):
        parser.error('--h264 requires ffmpeg')
    OUT = args.output
    OUT.mkdir(parents=True, exist_ok=True)
    if args.compare_cache_existing:
        sys.path.insert(0,str(ROOT/'src/robot_base_driver/src'))
        compare_existing_cache(OUT)
        return
    if args.plot_existing_audit:
        plot_existing_audit(OUT)
        return
    calibration = yaml.safe_load((ROOT/'src/usb_camera/config/livox_camera_extrinsics.yaml').read_text())
    lio = yaml.safe_load((ROOT/'src/FAST_LIO/config/mid360.yaml').read_text())['mapping']
    r = np.array(calibration['R_camera_lidar']['data']).reshape(3,3)
    t = np.array(calibration['t_camera_lidar'])
    ri = np.array(lio['extrinsic_R']).reshape(3,3)
    ti = np.array(lio['extrinsic_T'])
    report = {'limitations': 'Receive-time pairing; unknown exposure latency. Current camera and LIO extrinsics, not historical parameters. No motion compensation between camera and cloud times.', 'clips': []}
    report['target_camera_clearance_m']=args.center_clearance
    report['base_link_height_m']=args.base_link_height
    if args.camera_right_candidate:
        sys.path.insert(0,str(ROOT/'src/robot_base_driver/src'))
        report['mode']='offline_camera_geometry_only_not_neupan_control'
        report['excluded_control_gates']=['global route alignment/corridor','TF freshness','localization safety','receiver health','capture freshness','confirmation and smoothing','NeuPAN planning']
    route=None
    if args.compare_global_path:
        first=ROOT/'rosbags/rosbag20261008_155045/hybridnets_vehicle_data_20261008_155045_0.bag'
        with rosbag.Bag(str(first)) as bag:
            route_entries=[(stamp.to_sec(),np.array([[p.pose.position.x,p.pose.position.y] for p in msg.poses])) for _,msg,stamp in bag.read_messages(topics=['/osm_global_path'])]
        if route_entries:
            route=route_entries[0][1]
            route=dense_route(route)
            (OUT/'route_points.json').write_text(json.dumps(route.tolist()))
            report['route_first_receive_time']=route_entries[0][0]
    indices = range(14) if args.indices == 'all' else [int(x) for x in args.indices.split(',')]
    for index in indices:
        path = ROOT / ('rosbags/rosbag20261008_155045/hybridnets_vehicle_data_20261008_155045_%d.bag' % index)
        images, clouds, odoms, commands, info = [], [], [], [], None
        packets=[]
        tf_entries=[]
        topics=['/camera/remote_image_raw','/camera/remote_camera_info','/Odometry','/cmd_vel/selected']
        topics.append('/hybridnets/result_json' if args.camera_right_candidate else '/cloud_registered_body')
        if args.compare_global_path:
            topics.extend(['/tf','/osm_global_path'])
        with rosbag.Bag(str(path)) as bag:
            start = bag.get_start_time()+args.offset
            end = min(start+args.duration, bag.get_end_time())
            for topic, msg, stamp in bag.read_messages(topics=topics, start_time=rospy.Time.from_sec(start-.5), end_time=rospy.Time.from_sec(end+.5)):
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
                elif topic == '/hybridnets/result_json':
                    packets.append((ts,json.loads(msg.data)))
                elif topic == '/osm_global_path':
                    route=dense_route(np.array([[p.pose.position.x,p.pose.position.y] for p in msg.poses]))
                elif topic == '/tf':
                    for transform in msg.transforms:
                        edge=transform.header.frame_id.lstrip('/')+'->'+transform.child_frame_id.lstrip('/')
                        if edge in ('map->odom','odom->base_link'):
                            translation=transform.transform.translation
                            quaternion=transform.transform.rotation
                            tf_entries.append((ts,edge,([translation.x,translation.y,translation.z],[quaternion.x,quaternion.y,quaternion.z,quaternion.w])))
                else:
                    commands.append(msg.linear.x)
        if not images or (not clouds and not args.camera_right_candidate) or info is None:
            continue
        writer = cv2.VideoWriter(str(OUT/('clip_%02d_preview.mp4'%index)),cv2.VideoWriter_fourcc(*'mp4v'),args.fps,(1280,720))
        if not writer.isOpened():
            raise RuntimeError('Cannot open video writer')
        cloud_times = np.array([c[0] for c in clouds])
        deltas = []
        frame_results=[]
        for number,(ts,msg) in enumerate(images):
            image=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
            if msg.encoding=='rgb8':
                image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
            if args.camera_right_candidate:
                origin=np.array([.1,0,.35])-r.T@t
                transforms={edge:value for timestamp,edge,value in tf_entries if timestamp<=ts}
                valid_route=route if not args.compare_global_path or ts>=report.get('route_first_receive_time',float('inf')) else None
                frame_results.append(candidate_overlay(image,ts,packets,np.array(info.K).reshape(3,3),np.array(info.D),r.T,origin,valid_route,transforms,args.center_clearance,-args.base_link_height))
            for shift in (() if args.camera_right_candidate else (0,)):
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
            if not args.camera_right_candidate:
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
        report['clips'].append({'bag_index':index,'frames':len(images),'image_start':images[0][0],'image_end':images[-1][0],'seconds':elapsed,'net_distance_m':distance,'yaw_change_deg':yaw_change,'median_command_mps':float(np.median(commands)) if commands else None,'receive_delta_ms_minmax':[min(deltas),max(deltas)] if deltas else None})
        if args.camera_right_candidate:
            report['clips'][-1]['geometry_reasons']=dict(Counter(f['reason'] for f in frame_results))
            (OUT/('clip_%02d_frames.json'%index)).write_text(json.dumps(frame_results,indent=2))
        print(report['clips'][-1],flush=True)
    (OUT/'report.json').write_text(json.dumps(report,indent=2))


if __name__=='__main__':
    main()
