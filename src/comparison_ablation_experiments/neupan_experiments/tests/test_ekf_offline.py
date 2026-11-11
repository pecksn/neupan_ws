"""Opt-in integration: private ROS master, only synthetic inputs, no vehicle nodes."""
import os
import unittest


@unittest.skipUnless(os.environ.get('NEUPAN_EXPERIMENT_INTEGRATION')=='1','private-master integration is opt-in')
class EkfOfflineTests(unittest.TestCase):
    def test_bad_gps_enters_real_ekf_and_dropout_does_not_quality_stop(self):
        import json
        import socket
        import subprocess
        import tempfile
        import time
        import xmlrpc.client
        from pathlib import Path
        import xml.etree.ElementTree as ET
        import contextlib
        import io
        import roslaunch
        import yaml
        from test_experiments import runner,ROOT

        with socket.socket() as sock:
            sock.bind(('127.0.0.1',0));port=sock.getsockname()[1]
        os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(port)
        with tempfile.TemporaryDirectory(prefix='experiment_private_master_') as folder:
            output=open(Path(folder)/'master.log','w')
            master=subprocess.Popen(['roscore','-p',str(port)],stdout=output,stderr=subprocess.STDOUT)
            parent=None
            try:
                deadline=time.monotonic()+8.
                while True:
                    try:xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/offline_test');break
                    except OSError:
                        if time.monotonic()>deadline:raise
                        time.sleep(.1)
                child=Path(folder)/'navigation.launch'
                root,nav=runner.compose('gps_always',ROOT,child)
                ET.ElementTree(nav).write(str(child),encoding='unicode')
                with contextlib.redirect_stdout(io.StringIO()):
                    config=roslaunch.config.load_config_default([],None,roslaunch_strs=[ET.tostring(root,encoding='unicode')],verbose=False)
                tiny=ET.Element('launch')
                for key,value in config.params.items():
                    if key.startswith(('/gps_fix_to_map_odom/','/ekf_global/')):
                        node=ET.SubElement(tiny,'rosparam',param=key);node.text=yaml.safe_dump(value.value)
                for node in config.nodes:
                    if node.name in ('gps_fix_to_map_odom','ekf_global'):
                        element=ET.SubElement(tiny,'node',pkg=node.package,type=node.type,name=node.name,output='log')
                        for source,target in node.remap_args:ET.SubElement(element,'remap',{'from':source,'to':target})
                uuid=roslaunch.rlutil.get_or_generate_uuid(None,False)
                parent=roslaunch.parent.ROSLaunchParent(uuid,[],roslaunch_strs=[ET.tostring(tiny,encoding='unicode')])
                parent.start()
                import rospy
                from nav_msgs.msg import Odometry
                from sensor_msgs.msg import NavSatFix
                from std_msgs.msg import Bool,String
                import tf
                rospy.init_node('experiment_offline_inputs',anonymous=True,disable_signals=True)
                local=rospy.Publisher('/odometry/local',Odometry,queue_size=10)
                gps=rospy.Publisher('/gps/fix',NavSatFix,queue_size=10)
                heading=rospy.Publisher('/gnss/solution_status',String,queue_size=10)
                healthy=rospy.Publisher('/lio_guard/healthy',Bool,queue_size=2)
                broadcaster=tf.TransformBroadcaster()
                poses=[];states=[]
                rospy.Subscriber('/odometry/global',Odometry,lambda m:poses.append(m.pose.pose.position.x))
                rospy.Subscriber('/gps_guard/status',String,lambda m:states.append(json.loads(m.data)))
                deadline=time.monotonic()+5.
                while local.get_num_connections()==0 or gps.get_num_connections()==0:
                    self.assertLess(time.monotonic(),deadline);time.sleep(.05)
                def feed(duration,gps_x=None,local_start=0.):
                    started=time.monotonic()
                    while time.monotonic()-started<duration:
                        stamp=rospy.Time.now();message=Odometry();message.header.stamp=stamp
                        message.header.frame_id='odom';message.child_frame_id='base_link'
                        message.pose.pose.position.x=local_start+.05*(time.monotonic()-started)
                        message.pose.pose.orientation.w=1.;message.pose.covariance[0]=.01
                        message.pose.covariance[7]=.01;message.pose.covariance[35]=.01
                        local.publish(message);healthy.publish(Bool(data=True))
                        broadcaster.sendTransform((message.pose.pose.position.x,0.,0.),(0.,0.,0.,1.),stamp,'base_link','odom')
                        if gps_x is not None:
                            source=stamp-rospy.Duration(3.)
                            fix=NavSatFix();fix.header.stamp=source;fix.status.status=-1
                            fix.latitude=23.0398;fix.longitude=113.39002+gps_x/(6378137.*3.141592653589793/180.*.920)
                            fix.position_covariance[0]=.01;fix.position_covariance[4]=.01
                            gps.publish(fix)
                            heading.publish(String(data=json.dumps(dict(stamp_ns=source.to_nsec(),heading_deg=0.,heading_fixed=False))))
                        time.sleep(.05)
                feed(4.,0.);feed(2.,1.)
                self.assertTrue(poses);self.assertGreater(poses[-1],.3)
                before=states[-1]['gps_observations']
                feed(2.,None,.1)
                self.assertTrue(states[-1]['localization_safe'],states[-1])
                self.assertGreaterEqual(states[-1]['gps_observations'],before)
                self.assertGreater(states[-1]['gps_receive_age_sec'],1.)
                drained=states[-1]['gps_observations'];feed(.4,None,.2)
                self.assertEqual(states[-1]['gps_observations'],drained)
                print('PRIVATE_EKF_PASS: 3s late finite status=-1 GPS fused, Float heading fused, GPS dropout did not quality-stop; no vehicle nodes')
            finally:
                if parent is not None:parent.shutdown()
                master.terminate()
                try:master.wait(timeout=8)
                except subprocess.TimeoutExpired:master.kill();master.wait(timeout=3)
                output.close()


if __name__=='__main__':unittest.main()
