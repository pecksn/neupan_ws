"""First-reference and long heading gap regressions; no live ROS outputs."""
import math
import ast
from collections import deque
import unittest
import copy
import json
import tempfile
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
from unittest.mock import Mock
import numpy as np
from geometry_msgs.msg import QuaternionStamped
import rospy
from test_camera_right_refinement import namespace as n
import test_runtime_position as runtime
import test_stable_reference_policy as stable
from reference_test_frames import stamp_reference_frame
from osm_camera_path_guard import CameraPathGuard,path_digest


class InitialReferenceTests(unittest.TestCase):
    def test_first_in_band_osm_does_not_pull_vehicle_onto_distant_line(self):
        for sign in (1., -1.):
            points=np.column_stack((np.arange(0.,30.5,.5),np.zeros(61)))*sign
            geometry={i:dict(points_odom=[[points[i,0],-1.2*sign],
                [float('nan'),float('nan')],[points[i,0],-2.5*sign]],
                normal_odom=[0.,sign],mode='right_only') for i in range(4,17)}
            candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),
                geometry,[0.,-1.2*sign],dict(wheelbase=.67,max_steer=.35,
                max_steer_rate=.08,speed=1.5),max_shift=6.,heading=0. if sign>0 else math.pi,
                reference_accepted=False)
            self.assertTrue(report['changed'],report)
            self.assertAlmostEqual(candidate[-1,1],-1.2*sign,places=4)
            self.assertTrue(report['shape']['ok'])

    def test_appropriate_original_is_confirmed_at_version_zero(self):
        node,guard,clock,points,_=stable.StableReferenceTests().repair_node(2.4)
        with stable.StableReferenceTests().repair_context(clock):
            for frame in (1,1,2,3):
                clock[0]+=.1;stamp_reference_frame(node,frame)
                result=copy.deepcopy(node.path);active,count=node.stable_result(result,clock[0],.1)
                node.publish_result(result,clock[0],'initial',active,count)
                state=guard.select(clock[0],round(clock[0]*1.e9))[2]
                self.assertEqual(state['source'],'camera' if frame==3 else 'wait')
            self.assertEqual(node.reference_version,0)
            self.assertTrue(state['reference_accepted'])
            np.testing.assert_array_equal(node.frozen_reference_map,points)
            node.cache_geometry.clear();node.cache.committed.clear()
            node.advisory_fallback('occlusion',clock[0])
            self.assertEqual(guard.select(clock[0]+5.,round((clock[0]+5)*1.e9))[2]['source'],'camera')

    def test_no_heartbeat_or_unconfirmed_forwarding_never_counts_as_first_reference(self):
        from test_camera_publication_protocol import path
        route=path();guard=CameraPathGuard(require_initial_reference=True)
        guard.original_cb(route);guard.candidate_cb(route)
        self.assertEqual(guard.select(10.)[2]['source'],'wait')
        status=dict(control_enabled=True,route_stamp_ns=123,route_frame_id='map',original_path_digest=path_digest(route),
            control_path_digest=path_digest(route),inference_diagnostics=dict(reference_role='advisory',reference_version=0,
            reference_accepted=False,reference_accepted_stamp=None))
        guard.status_cb(status,10.)
        self.assertEqual(guard.select(10.)[2]['source'],'wait')
        status['inference_diagnostics'].update(reference_accepted=True,reference_accepted_stamp=10.)
        guard.status_cb(status,10.)
        self.assertEqual(guard.select(14.)[2]['source'],'wait') # stale cannot initialize
        self.assertEqual(guard.select(10.)[2]['source'],'camera')
        guard.candidate_cb(path(middle=.2))
        self.assertEqual(guard.select(10.)[2]['source'],'camera') # retain accepted pair
        guard.original_cb(path(stamp=124))
        self.assertEqual(guard.select(10.)[2]['source'],'wait') # new task must confirm

    def receipt_case(self, corrected=False):
        import test_followup_safety as command
        from test_camera_publication_protocol import path
        from test_outdoor_start_connection import AdapterTests,resample_forward_reference
        route=path();guard=CameraPathGuard(require_initial_reference=True)
        guard.original_cb(route);guard.candidate_cb(route)
        state=dict(control_enabled=True,route_stamp_ns=123,route_frame_id='map',
            original_path_digest=path_digest(route),control_path_digest=path_digest(route),
            inference_diagnostics=dict(reference_role='advisory',reference_version=0,
                reference_accepted=False,reference_accepted_stamp=None))
        if corrected:
            producer,guard,clock,_,_=stable.StableReferenceTests().repair_node(3.6)
            producer.cache_pose=lambda now,force=False:([0.,-.4,0.],0.)
            with stable.StableReferenceTests().repair_context(clock):
                for frame in (1,2,3):
                    stamp_reference_frame(producer,frame)
                    result=copy.deepcopy(producer.path)
                    active,count=producer.stable_result(result,clock[0],.1)
                    producer.publish_result(result,clock[0],'confirmed',active,count)
            route=guard.candidate;state=copy.deepcopy(guard.status)
            self.assertTrue(state['inference_diagnostics']['reference_accepted'])
            self.assertEqual(state['inference_diagnostics']['reference_version'],1)
            self.assertEqual(state['inference_diagnostics']['final_path']['reference_connection']['kind'],'measured_pose_c2')
            state['inference_diagnostics'].update(reference_accepted=False,reference_accepted_stamp=None)
            guard.accepted_advisory=None;guard.require_initial_reference=True
        guard.status_cb(state,1000.)
        snapshots=[]
        def hook(node,clock,namespace):
            source=Path(__file__).resolve().parents[3]/'src/neupan_ros/src/neupan_core.py'
            cls=next(v for v in ast.parse(source.read_text()).body if isinstance(v,ast.ClassDef) and v.name=='neupan_core')
            methods=[v for v in cls.body if isinstance(v,ast.FunctionDef) and v.name in ('apply_camera_path_guard','apply_pending_route')]
            namespace.update(copy=copy,String=lambda **kw:NS(**kw),resample_forward_reference=resample_forward_reference)
            namespace['rospy'].logdebug_throttle=Mock();namespace['rospy'].loginfo=Mock()
            exec(compile(ast.Module(body=methods,type_ignores=[]),str(source),'exec'),namespace)
            adapter,_=AdapterTests().node()
            for key,value in vars(adapter).items():
                if not hasattr(node,key):setattr(node,key,value)
            node.arrival_path_key=('map',0);node.minimum_path_stamp=0
            node.camera_guard=guard;node.camera_guard_signature=None
            node.camera_guard_last_state=None;node.camera_guard_last_publish=0.
            node.camera_guard_pub=Mock();node.refresh_camera_hazard_scope=Mock()
            if corrected:
                node.listener.lookupTransform=lambda *args:((0.,-.4,0.),(0.,0.,0.,1.))
            node.goal_arrived_timer_callback=Mock();node.publish_connection_status=Mock()
            node.neupan_planner.reset=Mock()
            node.neupan_planner.set_initial_path=lambda data:setattr(node.neupan_planner,'initial_path',data)
            def callback(message):
                node.pending_route_update=('path',[np.array([[p.pose.position.x],[p.pose.position.y],[0.],[1.]])
                    for p in message.poses],('map',123))
                return True
            node.path_callback=callback
            count=[0]
            def select():
                count[0]+=1
                if count[0]==2:
                    state['inference_diagnostics'].update(reference_accepted=True,reference_accepted_stamp=1000.)
                    guard.status_cb(state,clock.value)
                return namespace['apply_camera_path_guard'](node)
            node.apply_camera_path_guard=select
            node.apply_pending_route=lambda:namespace['apply_pending_route'](node)
            original=node.vel_pub.publish
            def publish(message):
                if message.linear.x:snapshots.append(copy.deepcopy(node.reference_receipt))
                original(message)
            node.vel_pub.publish=publish
        node,commands=command.CommandBoundaryTests().run_loop(setup_hook=hook)
        self.assertEqual(commands[0],0.)
        self.assertIn(.8,commands)
        self.assertTrue(snapshots)
        self.assertTrue(snapshots[0]['reference_accepted'])
        self.assertEqual(snapshots[0]['selected_reference_version'],1 if corrected else 0)
        self.assertEqual(snapshots[0]['source_path_digest'],path_digest(route))
        np.testing.assert_allclose(snapshots[0]['loaded_endpoint_map'],
            [route.poses[-1].pose.position.x,route.poses[-1].pose.position.y])
        if corrected:self.assertEqual(snapshots[0]['connection_kind'],'measured_pose_c2')

    def test_first_nonzero_native_command_has_actual_accepted_load_receipt(self):
        self.receipt_case()

    def test_corrected_c2_suffix_is_loaded_before_first_nonzero(self):
        self.receipt_case(corrected=True)


class HeadingGapTests(unittest.TestCase):
    setUp=runtime.RuntimePositionTests.setUp
    bridge=runtime.RuntimePositionTests.bridge
    def test_expired_old_pose_uses_same_generation_alignment(self):
        for real_turn,gnss_error,reject in ((85.,2.6,False),(0.,87.6,True)):
            b=self.bridge();b.pvt_period_sec=1.
            b.last_heading_yaw=.3;b.last_heading_stamp=rospy.Time(1)
            b.heading_alignment_epoch=b.reference_frame_epoch
            b.local_pose_history=deque([(100000000000,(0.,0.,math.radians(real_turn)))])
            msg=QuaternionStamped();msg.header.stamp=rospy.Time(100)
            yaw=.3+math.radians(real_turn+gnss_error)
            msg.quaternion.z=math.sin(yaw/2);msg.quaternion.w=math.cos(yaw/2)
            b.heading_cb(msg)
            self.assertEqual(b.heading_bad_until>self.now,reject)

    def test_new_history_missing_or_reset_never_uses_old_alignment(self):
        for reset in (False,True):
            b=self.bridge();b.pvt_period_sec=1.;b.last_heading_yaw=.3;b.last_heading_stamp=rospy.Time(1)
            b.heading_alignment_epoch=b.reference_frame_epoch
            if reset:
                b.reference_frame_epoch+=1
                b.local_pose_history=deque([(100000000000,(0.,0.,0.))])
            msg=QuaternionStamped();msg.header.stamp=self.now
            msg.quaternion.z=math.sin(1.);msg.quaternion.w=math.cos(1.)
            with patch.object(b,'remember_observation',wraps=b.remember_observation):
                b.heading_cb(msg)
            self.assertFalse(b.heading_bad_until>self.now)
            # Callback does not certify a heading/alignment or initialize map.
            self.assertEqual(b.transform_yaw,.3)
            self.assertEqual(b.heading_alignment_epoch,0)


class EvidenceCaptureTests(unittest.TestCase):
    def test_raw_capture_is_bounded_selected_and_timed(self):
        from libnmea_navsat_driver.serial_diagnostics import BoundedRawCapture
        with tempfile.TemporaryDirectory() as folder:
            clock=[0.];capture=BoundedRawCapture(folder,30.,lambda:clock[0],max_bytes=500)
            capture.observe(b'$GNGSV,ignored\r\n',1,2,10)
            capture.observe(b'$GNGGA,raw*ab\r\n',3,4,20)
            capture.close()
            rows=[json.loads(line) for line in Path(capture.path).read_text().splitlines()]
            self.assertEqual(len(rows),1);self.assertEqual(rows[0]['raw'],'$GNGGA,raw*ab\r\n')
            self.assertEqual(rows[0]['in_waiting_bytes'],20)
            capture=BoundedRawCapture(folder,30.,lambda:clock[0],max_bytes=500)
            for _ in range(50):capture.observe(b'#PVTSLNA,raw\r\n',1,2,3)
            self.assertIsNone(capture.file);self.assertLessEqual(Path(capture.path).stat().st_size,500)
            capture=BoundedRawCapture(folder,30.,lambda:clock[0]);clock[0]=30.
            capture.observe(b'$GNRMC,raw\r\n',1,2,3)
            self.assertIsNone(capture.file);self.assertEqual(Path(capture.path).stat().st_size,0)

    def test_host_snapshot_rejects_stale_clock_step_or_reboot(self):
        from libnmea_navsat_driver import host_utc_evidence as host
        evidence=dict(qualified=True,utc_ns=100000000000,monotonic_ns=50000000000,boot_id='boot',
                      sample_age_sec=1.,max_sample_age_sec=90.)
        with patch.object(host.time,'time_ns',return_value=101000000000), \
             patch.object(host.time,'monotonic_ns',return_value=51000000000), \
             patch.object(host.Path,'read_text',return_value='boot'):
            self.assertTrue(host.validate(evidence))
            self.assertFalse(host.validate(dict(evidence,boot_id='other')))
            self.assertFalse(host.validate(dict(evidence,utc_ns=97000000000)))
            self.assertFalse(host.validate(dict(evidence,qualified=False)))
            self.assertFalse(host.validate(evidence,max_capture_age=.5))
