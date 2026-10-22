"""Pure geometry tests; no ROS master or vehicle required."""
import ast
from pathlib import Path
import unittest
import copy
import hashlib
import json
import math
import threading
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import cv2
import numpy as np

workspace = Path(__file__).resolve().parents[3]
source = workspace / 'src/osm_map_handler/src/osm_camera_path_refinement.py'
tree = ast.parse(source.read_text())
functions = ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)], type_ignores=[])
namespace = {'np':np, 'cv2':cv2}
exec(compile(functions, str(source), 'exec'), namespace)
namespace.update({'copy':copy,'hashlib':hashlib,'json':json,'math':math,'time':time,'threading':threading,
                  'String':lambda **kw:NS(**kw),
                  'tf':NS(Exception=RuntimeError,transformations=NS(quaternion_from_euler=lambda *a:(0,0,0,1)))})
classes=ast.Module(body=[n for n in tree.body if isinstance(n,ast.ClassDef)],type_ignores=[])
exec(compile(classes,str(source),'exec'),namespace)


class GeometryTests(unittest.TestCase):
    def capture_namespace(self):
        return patch.dict(namespace, {
            'rospy': NS(Time=lambda sec, nsec: (sec, nsec)),
            'tf': NS(Exception=RuntimeError, transformations=NS(
                euler_from_quaternion=lambda quaternion: (0., 0., .4)))})

    def test_capture_pose_uses_exact_timestamp_not_latest_tf(self):
        listener = NS(lookupTransform=Mock(return_value=((1., 2., 0.), (0., 0., 0., 1.))))
        with self.capture_namespace():
            position, yaw = namespace['capture_vehicle_pose'](listener, 1792136337651834500)
        listener.lookupTransform.assert_called_once_with('map', 'base_link', (1792136337, 651834500))
        self.assertEqual(position, (1., 2., 0.))
        self.assertEqual(yaw, .4)

    def test_missing_capture_tf_is_rejected_without_latest_tf_fallback(self):
        listener = NS(lookupTransform=Mock(side_effect=RuntimeError('missing history')))
        with self.capture_namespace(), self.assertRaisesRegex(ValueError, 'capture-time vehicle TF unavailable'):
            namespace['capture_vehicle_pose'](listener, 1792136337651834500)
        listener.lookupTransform.assert_called_once()

    def test_nonfinite_capture_tf_is_rejected(self):
        listener = NS(lookupTransform=Mock(return_value=((float('nan'), 2., 0.), (0., 0., 0., 1.))))
        with self.capture_namespace(), self.assertRaisesRegex(ValueError, 'non-finite capture-time vehicle TF'):
            namespace['capture_vehicle_pose'](listener, 1792136337651834500)

    def test_ground_projection(self):
        r = np.array([[0,0,1],[-1,0,0],[0,-1,0]],float)
        p = namespace['ground_points']([[0,.2],[0,.1],[0,-.1]],np.eye(3),np.zeros(5),r,np.array([0,0,.5]))
        np.testing.assert_allclose(p[:,0],[2.5,5])
        np.testing.assert_allclose(p[:,2],0,atol=1e-12)

    def test_nonfinite(self):
        with self.assertRaises(ValueError):
            namespace['ground_points']([[np.nan,0]],np.eye(3),np.zeros(5),np.eye(3),np.array([0,0,.5]))

    def curves(self, left=2., right=-2.):
        x=np.arange(1,8,.25)
        return np.column_stack((x,np.full(len(x),left))),np.column_stack((x,np.full(len(x),right)))

    def test_right_target_sign(self):
        left,right=self.curves()
        x,target,_,_=namespace['lane_target'](left,right,.9)
        np.testing.assert_allclose(target,-1.1)

    def test_narrow_and_crossed(self):
        for l,r in ((.5,-.5),(-1,1)):
            with self.assertRaises(ValueError):
                namespace['lane_target'](*self.curves(l,r),.9)

    def test_sharp_boundary(self):
        left,right=self.curves()
        right[:,1]=right[:,0]-5
        with self.assertRaises(ValueError):
            namespace['lane_target'](left,right,.9)

    def test_disjoint(self):
        left,right=self.curves()
        right[:,0]+=10
        with self.assertRaises(ValueError):
            namespace['lane_target'](left,right,.9)

    def test_missing_and_stale_json_rejected(self):
        node=namespace['CameraRightPath'].__new__(namespace['CameraRightPath'])
        node.packet=None
        node.timeout=.5
        with self.assertRaisesRegex(ValueError,'json missing'):
            node.refine(10)
        node.packet={'valid':True}
        node.received=1
        with self.assertRaisesRegex(ValueError,'stale'):
            node.refine(10)

    def test_localization_failure_rejected(self):
        node=namespace['CameraRightPath'].__new__(namespace['CameraRightPath'])
        node.packet={'valid':True};node.received=10;node.timeout=.5
        node.receiver_valid=True;node.valid_received=10;node.safe=False;node.control=False
        with self.assertRaisesRegex(ValueError,'localization unsafe'):
            node.refine(10)

    def test_repeated_route_sequence_does_not_reset_confirmation(self):
        node=namespace['CameraRightPath'].__new__(namespace['CameraRightPath'])
        node.lock=threading.RLock();node.route_signature=None
        msg=NS(header=NS(seq=0,stamp=NS(to_nsec=lambda:123),frame_id='map'),
               poses=[NS(pose=NS(position=NS(x=x,y=0))) for x in (0,3,6)])
        node.path_cb(msg)
        node.confirmations=2;node.shift=-.05
        msg.header.seq=99
        node.path_cb(msg)
        self.assertEqual(node.confirmations,2)
        self.assertEqual(node.shift,-.05)

    def test_strong_distortion_inverse(self):
        r=np.array([[0,0,1],[-1,0,0],[0,-1,0]],float)
        k=np.array([[1056.,0,887.],[0,1057.,688.],[0,0,1.]])
        d=np.array([-.4013,.18456,.000347,.001498,-.041186])
        origin=np.array([0,0,.3])
        points=np.column_stack((np.linspace(2,7,30),np.full(30,2.),np.zeros(30)))
        uv=cv2.projectPoints((points-origin)@r,np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2)
        restored=namespace['ground_points'](uv,k,d,r,origin)
        np.testing.assert_allclose(restored,points,atol=.01)

    def test_neupan_route_identity_ignores_transport_sequence(self):
        core=workspace/'src/neupan_ros/src/neupan_core.py'
        assigns=[n for n in ast.walk(ast.parse(core.read_text())) if isinstance(n,ast.Assign)
                 and any(isinstance(t,ast.Name) and t.id=='path_key' for t in n.targets)]
        expression=compile(ast.Expression(body=assigns[0].value),str(core),'eval')
        state={'path_frame':'map','path_stamp':123,'initial_point_list':[np.array([[6.],[1.],[0.],[1.]])]}
        before=eval(expression,dict(state,path=NS(header=NS(seq=0))))
        after=eval(expression,dict(state,path=NS(header=NS(seq=99))))
        self.assertEqual(before,after)
        new_stamp=eval(expression,dict(state,path_stamp=124))
        self.assertNotEqual(before,new_stamp)

    def test_one_point_five_target_with_left_safety(self):
        left,right=self.curves(1.,-1.5)
        x,y,_,_=namespace['lane_target'](left,right,1.5,.75)
        np.testing.assert_allclose(y,0)

    def test_cache_confirm_freeze_and_expiry(self):
        cache=namespace['SegmentCache'](ttl=30)
        cache.observe(2,.5,0,1);cache.observe(2,.6,.5,2)
        self.assertFalse(cache.values(.5))
        cache.observe(2,.55,1,3)
        self.assertAlmostEqual(cache.values(1)[2],.55)
        cache.observe(2,.7,2,4)
        self.assertAlmostEqual(cache.values(2)[2],.55)
        self.assertFalse(cache.values(32))

    def test_cache_duplicate_and_clear(self):
        cache=namespace['SegmentCache']()
        for t in (0,.5,1):cache.observe(1,.5,t,1)
        self.assertFalse(cache.values(1))
        cache.observe(1,.5,1.5,2);cache.observe(1,.5,2,3)
        self.assertTrue(cache.values(2))
        cache.clear();self.assertFalse(cache.values(2))

    def test_taper_does_not_extend_into_unknown_road(self):
        points=np.column_stack((np.arange(20)*.5,np.zeros(20)))
        offsets=np.zeros(20);offsets[4:14]=1.
        result=namespace['bounded_offsets'](points,offsets)
        self.assertTrue(np.all(result[offsets==0]==0))
        self.assertLessEqual(np.max(np.abs(np.diff(result))),.125+1.e-12)

    def test_unverified_wheel_geometry_rejects_live_correction(self):
        node=namespace['CameraRightPath'].__new__(namespace['CameraRightPath'])
        node.packet={'valid':True};node.received=10;node.timeout=.5
        node.receiver_valid=True;node.valid_received=10;node.safe=True;node.safe_received=10
        node.control=True;node.wheel_geometry_verified=False
        with self.assertRaisesRegex(ValueError,'wheel geometry not verified'):
            node.refine(10)

    def test_camera_center_offset_sign(self):
        self.assertAlmostEqual(namespace['camera_center_clearance'](2,.1),1.9)
        self.assertAlmostEqual(namespace['camera_center_clearance'](2,-.1),2.1)
        self.assertAlmostEqual(namespace['camera_center_clearance'](2,0),2)

    def test_current_extrinsics_camera_center(self):
        import yaml
        cfg=workspace/'src/usb_camera/config/livox_camera_extrinsics.yaml'
        calibration=yaml.safe_load(cfg.read_text())
        r=np.array(calibration['R_camera_lidar']['data']).reshape(3,3)
        origin=np.array([.1,0,.35])-r.T@np.array(calibration['t_camera_lidar'])
        self.assertLess(abs(origin[1]),1.e-6)
        self.assertAlmostEqual(namespace['camera_center_clearance'](2,float(origin[1])),2,places=6)

    def test_osm_turn_fallback(self):
        points=np.array([[0,0],[2,0],[4,0],[4,2],[4,4]],float)
        self.assertEqual(namespace['osm_guidance_context'](points,[0,0],0,np.zeros((0,2))),'osm_turn')

    def test_osm_junction_even_when_driving_straight(self):
        points=np.column_stack((np.arange(20),np.zeros(20)))
        self.assertEqual(namespace['osm_guidance_context'](points,[0,0],0,np.array([[5,0]])),'osm_junction')
        self.assertEqual(namespace['osm_guidance_context'](points,[0,0],0,np.array([[5,20]])),'straight')

    def test_osm_turn_outside_lookahead(self):
        points=np.array([[0,0],[5,0],[10,0],[15,0],[15,5]],float)
        self.assertEqual(namespace['osm_guidance_context'](points,[0,0],0,np.zeros((0,2))),'straight')

    def test_ground_twenty_cm_below_base(self):
        r=np.array([[0,0,1],[-1,0,0],[0,-1,0]],float)
        p=namespace['ground_points']([[0,.2]],np.eye(3),np.zeros(5),r,np.array([0,0,.3]),-.2)
        np.testing.assert_allclose(p,[[2.5,0,-.2]],atol=1.e-8)

    def test_projection_does_not_join_across_rejected_points(self):
        r=np.array([[0,0,1],[-1,0,0],[0,-1,0]],float)
        points=np.array([[2,0,-.2],[2.5,0,-.2],[200,50,-.2],[3,0,-.2],[3.5,0,-.2]])
        k=np.array([[100,0,100],[0,100,100],[0,0,1]],float)
        segments=namespace['project_ground_path'](points,np.array([0,0,.3]),r,k,np.zeros(5),200,200)
        self.assertEqual(len(segments),2)
        self.assertEqual([len(s) for s in segments],[2,2])

    def test_projection_rejects_far_sideways_road(self):
        r=np.array([[0,0,1],[-1,0,0],[0,-1,0]],float)
        k=np.array([[100,0,100],[0,100,100],[0,0,1]],float)
        p=np.array([[3,50,-.2],[4,60,-.2],[4,-50,-.2]])
        self.assertEqual(namespace['project_ground_path'](p,np.array([0,0,.3]),r,k,np.zeros(5),200,200),[])

    def test_fallback_publishes_original_and_preserves_header(self):
        node=namespace['CameraRightPath'].__new__(namespace['CameraRightPath'])
        poses=[NS(pose=NS(position=NS(x=x,y=0,z=0),orientation=NS(x=0,y=0,z=0,w=1))) for x in (0,3,6)]
        node.path=NS(header=NS(seq=7,stamp=NS(to_nsec=lambda:123),frame_id='map'),poses=poses)
        node.lock=threading.RLock();node.last_tick=time.monotonic();node.packet=None
        node.shift=.3;node.confirmations=4;node.received=0;node.timeout=.5
        node.previous=None;node.control=True
        output=[];status=[]
        node.preview=NS(publish=lambda p:None);node.output=NS(publish=output.append);node.status=NS(publish=status.append)
        node.tick(None)
        self.assertEqual([p.pose.position.x for p in output[0].poses],[0,3,6])
        self.assertEqual([p.pose.position.y for p in output[0].poses],[0,0,0])
        self.assertEqual(output[0].header.seq,7)
        self.assertEqual(output[0].header.stamp.to_nsec(),123)
        self.assertFalse(json.loads(status[0].data)['active'])
        self.assertEqual(node.shift,0)
        node.tick(None)
        self.assertEqual(len(output),1)



def run_ros_test():
    """Integration smoke test for a shadow node on an isolated ROS master only."""
    import json
    import os
    import threading
    import time
    from pathlib import Path as FilePath

    import cv2
    import numpy as np
    import rosbag
    import rospy
    import tf
    import yaml
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path
    from std_msgs.msg import Bool, String


    def main():
        if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:11319':
            raise RuntimeError('Test must run on isolated localhost ROS port 11319')
        rospy.init_node('camera_right_integration_test', anonymous=True)
        if rospy.get_param('/camera_right_path/control_enabled', True):
            raise RuntimeError('Integration test requires control_enabled=false')
        statuses, paths, controls = [], [], []
        lock = threading.Lock()
        def status_cb(msg):
            with lock:
                statuses.append(json.loads(msg.data))
        rospy.Subscriber('/camera_right/status', String, status_cb)
        rospy.Subscriber('/camera_right/reference_path', Path, paths.append)
        rospy.Subscriber('/camera_right/control_path', Path, controls.append)
        pubs = {topic:rospy.Publisher(topic, kind, queue_size=1) for topic,kind in (
            ('/osm_global_path',Path),('/hybridnets/result_json',String),
            ('/hybridnets/valid',Bool),('/localization_safe',Bool))}
        broadcaster=tf.TransformBroadcaster()
        root=FilePath(__file__).resolve().parents[3]
        camera=yaml.safe_load((root/'src/usb_camera/config/usb_camera.yaml').read_text())
        ext=yaml.safe_load((root/'src/usb_camera/config/livox_camera_extrinsics.yaml').read_text())
        r=np.array(ext['R_camera_lidar']['data']).reshape(3,3).T
        origin=np.array([.1,0,.35])-r@np.array(ext['t_camera_lidar'])
        k=np.array(camera['camera_matrix']['data']).reshape(3,3)
        d=np.array(camera['distortion_coefficients']['data'])
        def pixels(y):
            x=np.linspace(2,7,40)
            points=np.column_stack((x,np.full(len(x),y),np.full(len(x),-rospy.get_param('/camera_right_path/base_link_height_m',.2))))
            optical=(points-origin)@r
            return cv2.projectPoints(optical,np.zeros(3),np.zeros(3),k,d)[0].reshape(-1,2).tolist()
        center=rospy.get_param('/camera_right_path/center_clearance_m',-1)
        if rospy.get_param('/camera_right_path/distance_reference','camera_center')=='camera_center':
            center=rospy.get_param('/camera_right_path/camera_clearance_m',2.)-float(origin[1])
        elif center<=0:
            offset=rospy.get_param('/camera_right_path/right_wheel_lateral_offset_m',-1)
            if offset<=0:
                offset=(rospy.get_param('/camera_right_path/track_width_m',.67)+rospy.get_param('/camera_right_path/tire_width_m',.08))/2
            center=rospy.get_param('/camera_right_path/wheel_clearance_m',1.5)+offset
        left,right=pixels(2),pixels(-center-.1)
        packet={'frame_seq':0,'valid':True,'road_polygon_valid':True,'left_boundary_valid':True,
                'right_boundary_valid':True,'image_width':1920,'image_height':1080,
                'coordinate_space':'rotated_image_pixels','confidence':.99,'source_age_ms':10,
                'scene_type':'unknown','left_valid_segments_px':[left],'right_valid_segments_px':[right]}
        path=Path();path.header.frame_id='map';path.header.seq=42;path.header.stamp=rospy.Time.now()
        for x in np.arange(0,9,.5):
            pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x=float(x);pose.pose.orientation.w=1
            path.poses.append(pose)
        def drive(seconds,send_json=True,safe=True,raw=None):
            stop=time.monotonic()+seconds
            while time.monotonic()<stop and not rospy.is_shutdown():
                now=rospy.Time.now()
                broadcaster.sendTransform((0,0,0),(0,0,0,1),now,'odom','map')
                broadcaster.sendTransform((0,0,0),(0,0,0,1),now,'base_link','odom')
                broadcaster.sendTransform((.1,0,.35),(0,0,0,1),now,'livox_frame','base_link')
                pubs['/osm_global_path'].publish(path)
                pubs['/hybridnets/valid'].publish(True)
                pubs['/localization_safe'].publish(safe)
                if send_json:
                    packet['frame_seq']+=1
                    packet['capture_timestamp_ns']=now.to_nsec()
                    if raw:
                        historical=json.loads(raw)
                        historical['frame_seq']=packet['frame_seq']
                        payload=json.dumps(historical)
                    else:
                        payload=json.dumps(packet)
                    pubs['/hybridnets/result_json'].publish(String(data=payload))
                time.sleep(.1)
        time.sleep(1)
        drive(4)
        assert any(s['active'] for s in statuses), statuses[-3:]
        assert any(any(p.pose.position.y < -.01 for p in path_msg.poses) for path_msg in paths), 'No rightward correction'
        assert paths[-1].header.stamp==path.header.stamp
        assert len(paths[-1].poses)>=len(path.poses) and paths[-1].poses[-1].pose.position.x==path.poses[-1].pose.position.x, 'Global route truncated'
        assert not controls, 'Shadow mode published control path'
        print('PASS: active shadow correction, complete path/header retained, no control publishing',flush=True)
        drive(1.5,send_json=False)
        if rospy.get_param('/camera_right_path/strategy','segment_cache')=='segment_cache':
            assert statuses[-1]['active'] and statuses[-1]['cached_points']>0,statuses[-1]
            print('PASS: short JSON dropout retains confirmed segment cache',flush=True)
        else:
            assert statuses[-1]['reason']=='json missing or stale', statuses[-1]
            assert all(abs(p.pose.position.y)<1e-9 for p in paths[-1].poses)
            print('PASS: JSON timeout returns original path',flush=True)
        drive(1,safe=False)
        assert 'localization unsafe' in statuses[-1]['reason'], statuses[-1]
        assert not statuses[-1]['active'] and all(abs(p.pose.position.y)<1e-9 for p in paths[-1].poses),statuses[-1]
        print('PASS: localization loss rejects correction',flush=True)
        bag=root/'rosbags/rosbag20261008_155045/hybridnets_vehicle_data_20261008_155045_0.bag'
        with rosbag.Bag(str(bag)) as reader:
            raw=None
            for _,message,_ in reader.read_messages(topics=['/hybridnets/result_json']):
                historic=json.loads(message.data)
                if all(historic.get(k) is True for k in ('valid','road_polygon_valid','left_boundary_valid','right_boundary_valid')):
                    raw=message.data
                    break
            assert raw, 'No valid historical JSON found in bag'
        drive(1.5,raw=raw)
        assert not statuses[-1]['active'], statuses[-1]
        assert any(reason in statuses[-1]['reason'] for reason in (
            'capture stale', 'low confidence or old inference', 'source_age_exceeded_',
            'capture_age_exceeded_', 'capture_timestamp_in_future_clock_skew')), statuses[-1]
        print('PASS: historical bag JSON is not treated as fresh live control input',flush=True)
        print('Latest rejection:',statuses[-1]['reason'],flush=True)
        old=packet.copy();old['capture_timestamp_ns']=rospy.Time.now().to_nsec()-10_000_000_000
        drive(1.5,raw=json.dumps(old))
        assert ('capture stale' in statuses[-1]['reason'] or
                'capture_age_exceeded_' in statuses[-1]['reason']),statuses[-1]
        print('PASS: stale capture timestamp rejected even when JSON has just arrived',flush=True)
        path.header.stamp=rospy.Time.now()
        path.poses=[]
        for x,y in ((0,0),(2,0),(4,0),(4,2),(4,4)):
            pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation.w=1
            path.poses.append(pose)
        drive(2)
        assert not statuses[-1]['active'] and 'osm_turn' in statuses[-1]['reason'],statuses[-1]
        assert statuses[-1]['guidance_source']=='osm',statuses[-1]
        print('PASS: upcoming OSM turn overrides camera correction and clears cache',flush=True)


    main()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Camera right-driving tests: geometry by default; isolated ROS test with --ros.')
    parser.add_argument('--ros', action='store_true', help='Requires isolated ROS master 127.0.0.1:11319 and shadow node; never use the vehicle master.')
    args = parser.parse_args()
    if args.ros:
        run_ros_test()
    else:
        unittest.main(argv=[__file__])
