"""Production scan queue/tracking/run regressions; no ROS node or commands."""
import ast
from collections import deque
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
import numpy as np
import test_followup_safety as fixture
from neupan_safety import CandidateRejected, InputUnavailable, checked_input_ages
from osm_outdoor_start_connection import OutdoorTrackingGuard, resample_forward_reference

ROOT=Path(__file__).resolve().parents[3]


class Stamp:
    def __init__(self,value=0):self.value=value
    def to_nsec(self):return round(self.value*1.e9)
    def to_sec(self):return self.value
    def __sub__(self,other):return Stamp(self.value-other.value)


def scan_node():
    """AST extracts exact production methods; replaces only clock/TF/ROS APIs."""
    source=ROOT/'src/neupan_ros/src/neupan_core.py'
    cls=next(x for x in ast.parse(source.read_text()).body if isinstance(x,ast.ClassDef))
    methods={'scan_callback','refresh_scan_snapshot','convert_scan_snapshot','bind_scan_snapshot','mark_scan_fault','mark_scan_valid','scan_is_healthy'}
    tree=ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Scan',bases=[],keywords=[],
        body=[x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in methods],decorator_list=[])],type_ignores=[]))
    clock=NS(value=1000.,ready=True,body_ready=True)
    tf=NS(Exception=LookupError,LookupException=LookupError,ConnectivityException=ConnectionError,ExtrapolationException=TimeoutError)
    def transform(state):
        x,y,yaw=state[:,0];c,s=math.cos(yaw),math.sin(yaw)
        return np.array([[x],[y]]),np.array([[c,-s],[s,c]])
    ns=dict(np=np,time=NS(monotonic=lambda:clock.value),rospy=NS(Time=NS(now=lambda:Stamp(clock.value))),
        tf=tf,deque=deque,get_transform=transform,cos=math.cos,sin=math.sin,checked_input_ages=checked_input_ages,InputUnavailable=InputUnavailable)
    exec(compile(tree,str(source),'exec'),ns)
    node=ns['Scan']();node.scan_lock=threading.RLock();node.scan_queue=deque(maxlen=8)
    node.start_connector=None;node.scan_snapshot=None;node.safety_scan=None;node.obstacle_points=None
    node.scan_timeout=.5;node.scan_transform_timeout=.5;node.scan_valid_since=None;node.last_scan_valid_time=None
    node.last_scan_received_time=None;node.stop_on_scan_fault=True;node.scan_fault_reason='waiting';node.scan_mode='WAITING'
    node.flip_angle=False;node.scan_range=[.1,20.];node.scan_angle_range=[-3.14,3.14]
    node.scan_downsample=1;node.scan_self_filter=False;node.scan_self_filter_length=.95;node.scan_self_filter_width=.85;node.scan_self_filter_margin=.03
    node.lidar_frame='laser';node.map_frame='map';node.base_frame='base_link'
    node.quat_to_yaw_list=lambda _:0.
    calls=[]
    def lookup(target,source,stamp):
        calls.append((target,source,stamp.to_nsec()))
        if not clock.ready or (source=='base_link' and not clock.body_ready):raise LookupError('TF arrives later')
        return (0.,0.,0.),(0.,0.,0.,1.)
    node.listener=NS(lookupTransform=lookup,getLatestCommonTime=lambda *_:Stamp(clock.value-.01))
    return node,clock,calls


def message(stamp=999.99,ranges=(2.,3.,4.)):
    return NS(header=NS(stamp=Stamp(stamp),frame_id='laser'),ranges=ranges,angle_min=-.1,
        angle_increment=.1,range_min=.1,range_max=20.)


class ScanQueueTests(unittest.TestCase):
    def test_tf_delay_uses_original_capture_and_receive_times(self):
        node,clock,calls=scan_node();clock.ready=False
        node.scan_callback(message());node.refresh_scan_snapshot()
        self.assertIsNone(node.safety_scan)
        clock.value+=.02;clock.ready=True;node.refresh_scan_snapshot()
        self.assertEqual(node.safety_scan[1],message().header.stamp.to_nsec())
        self.assertEqual(node.safety_scan[2],1000.)
        self.assertTrue(all(c[2]==message().header.stamp.to_nsec() for c in calls))

    def test_pending_new_tf_does_not_invalidate_old_fresh_snapshot(self):
        node,clock,_=scan_node();node.scan_callback(message());node.refresh_scan_snapshot()
        old=node.scan_snapshot;since=node.scan_valid_since
        clock.value+=.1;clock.ready=False;node.scan_callback(message(clock.value-.01));node.refresh_scan_snapshot()
        self.assertIs(old,node.scan_snapshot);self.assertIs(since,node.scan_valid_since)
        self.assertTrue(node.scan_is_healthy(Stamp(clock.value))[0])
        self.assertEqual(node.scan_tf_diagnostics['state'],'using_previous_fresh_capture')

    def test_true_outage_over_timeout_stops_and_old_queue_cannot_renew_age(self):
        node,clock,_=scan_node();node.scan_callback(message());node.refresh_scan_snapshot()
        clock.ready=False;clock.value+=.6;node.scan_callback(message(clock.value-.01));node.refresh_scan_snapshot()
        self.assertFalse(node.scan_is_healthy(Stamp(clock.value))[0]);self.assertIsNone(node.scan_valid_since)
        old=node.safety_scan;clock.ready=True;clock.value+=.6;node.refresh_scan_snapshot()
        self.assertIs(old,node.safety_scan);self.assertFalse(node.scan_is_healthy(Stamp(clock.value))[0])

    def test_second_tf_failure_does_not_partly_commit_points(self):
        node,clock,_=scan_node();node.scan_self_filter=True
        node.scan_callback(message());node.refresh_scan_snapshot();old=node.scan_snapshot
        clock.value+=.1;clock.body_ready=False;node.scan_callback(message(clock.value-.01,(5.,6.,7.)));node.refresh_scan_snapshot()
        self.assertIs(old,node.scan_snapshot);self.assertIs(old['planner_points'],node.obstacle_points)
        self.assertIs(old['safety'],node.safety_scan)

    def test_newest_convertible_frame_and_out_of_order_do_not_roll_back(self):
        node,clock,_=scan_node()
        for stamp in (999.8,999.99,999.9):node.scan_callback(message(stamp))
        node.refresh_scan_snapshot();self.assertEqual(node.safety_scan[1],999990000000)
        node.scan_callback(message(999.9));node.refresh_scan_snapshot();self.assertEqual(node.safety_scan[1],999990000000)

    def test_queue_is_bounded(self):
        node,clock,_=scan_node();clock.ready=False
        for _ in range(100):node.scan_callback(message())
        self.assertEqual(len(node.scan_queue),8)

    def test_real_tf_outage_recovers_with_new_frame_not_old_reception_renewal(self):
        node,clock,_=scan_node();node.scan_callback(message());node.refresh_scan_snapshot()
        clock.value+=.7;clock.ready=False;node.refresh_scan_snapshot()
        self.assertFalse(node.scan_is_healthy(Stamp(clock.value))[0])
        clock.ready=True;node.scan_callback(message(clock.value-.01));node.refresh_scan_snapshot()
        self.assertTrue(node.scan_is_healthy(Stamp(clock.value))[0])
        self.assertEqual(node.safety_scan[2],clock.value)

    def test_future_capture_waits_and_nan_does_not_become_empty_clear_scan(self):
        node,clock,_=scan_node();node.scan_callback(message(1000.01));node.refresh_scan_snapshot()
        self.assertIsNone(node.scan_snapshot)
        clock.value+=.02;node.refresh_scan_snapshot();self.assertIsNotNone(node.scan_snapshot)
        previous=node.scan_snapshot;clock.value+=.1;node.scan_callback(message(clock.value-.01,(math.nan,)*3));node.refresh_scan_snapshot()
        self.assertIs(previous,node.scan_snapshot)

    def test_sensor_frame_not_configured_alias_and_scan_is_not_downsampled_for_safety(self):
        node,_,calls=scan_node();node.scan_downsample=2
        node.scan_callback(message());node.refresh_scan_snapshot()
        self.assertEqual(calls[0][1],'laser');self.assertEqual(node.safety_scan[0].shape[1],3)
        self.assertEqual(node.obstacle_points.shape[1],2)


class RecoveryTests(unittest.TestCase):
    def run_case(self,kind):
        def hook(node,clock,ns):
            if kind in ('blocked','removed','slow'):
                if kind=='slow':node.obstacle_points=np.array([[1.03],[0.]])
                else:node.obstacle_points=np.array([[.9],[0.]])
                node.safety_scan=(node.obstacle_points.copy(),round(clock.value*1.e9),clock.value)
                # The fixture's sleep refreshes the same full scan each cycle.
                if kind in ('blocked','removed'):
                    base=type(node.neupan_planner)
                    class NativeStopPlanner(base):
                        def __call__(self,*args):
                            action,info=super().__call__(*args)
                            if kind=='blocked' or self.calls==1:
                                info['stop']=True;action=np.zeros((2,1))
                            return action,info
                    node.neupan_planner=NativeStopPlanner()
                    node.neupan_planner.min_distance=NS(detach=lambda:NS(item=lambda:.1))
                    node.neupan_planner.collision_threshold=.2
            if kind=='startup':
                node.outdoor_tracking_guard=OutdoorTrackingGuard()
                node.neupan_planner.ipath.cur_curve=resample_forward_reference([[0.,1.54,0.,1.],[20.,1.54,0.,1.]])
            if kind=='nan':
                ns['validate_solution']=lambda *_:(_ for _ in ()).throw(ValueError('nonfinite optimized trajectory'))
        return fixture.CommandBoundaryTests().run_loop(recover=True,setup_hook=hook)

    def test_persistent_near_collision_stops_but_keeps_replanning(self):
        node,commands=self.run_case('blocked')
        self.assertEqual(node.neupan_planner.calls,9);self.assertTrue(all(v==0 for v in commands))
        self.assertIsNone(getattr(node,'planner_fault',None));self.assertIn('planner_obstacle',node.stop_registry.reasons)

    def test_obstacle_removed_automatically_resumes_original_task(self):
        node,commands=self.run_case('removed')
        self.assertEqual(commands[0],0.);self.assertIn(.8,commands)
        self.assertNotIn('candidate',node.stop_registry.reasons);self.assertIsNone(getattr(node,'planner_fault',None))

    def test_wrapper_does_not_replace_native_action_with_slow_alternative(self):
        node,commands=self.run_case('slow')
        self.assertTrue(commands);self.assertTrue(all(v==.8 for v in commands))
        self.assertEqual(node.neupan_planner.calls,9)

    def test_real_run_startup_154cm_is_diagnostic_not_outer_speed_rewrite(self):
        node,commands=self.run_case('startup')
        self.assertTrue(all(v==.8 for v in commands));self.assertFalse(node.outdoor_tracking_status['latched'])

    def test_nonfinite_is_still_internal_fault(self):
        node,commands=self.run_case('nan')
        self.assertEqual(node.neupan_planner.calls,1);self.assertTrue(all(v==0 for v in commands))
        self.assertIsNotNone(node.planner_fault)

    def test_preparation_rebinds_pose_before_solving_not_after_result(self):
        stamps=[]
        def hook(node,clock,ns):
            generate=node.generate_path_msg
            def prepare(states):
                if not stamps:
                    clock.value+=.05;stamps.append(round(clock.value*1.e9))
                return generate(states)
            node.generate_path_msg=prepare
        node,commands=fixture.CommandBoundaryTests().run_loop(recover=True,setup_hook=hook)
        self.assertIn(.8,commands)
        self.assertGreaterEqual(node.diagnostic_state_stamp_ns,stamps[0])

    def test_input_recovery_uses_one_shared_window_not_scan_then_input(self):
        def hook(node,clock,ns):
            node.scan_recovery_stable_sec=.5
            # Establish one actual input failure before run; run creates registry.
            node.safety_scan=(node.obstacle_points.copy(),round((clock.value-1)*1.e9),clock.value-1)
        node,commands=fixture.CommandBoundaryTests().run_loop(recover=True,setup_hook=hook)
        self.assertTrue(all(v==0 for v in commands[:6]))
        self.assertIn(.8,commands)
        self.assertNotIn('input',node.stop_registry.reasons)

    def test_diagnostic_thresholds_do_not_latch_and_pause_resets_continuity(self):
        guard=OutdoorTrackingGuard();route=resample_forward_reference([[0,0,0,1],[20,0,0,1]])
        for t in np.arange(0,1,.1):report=guard.check(route,[0,1.54,0],0,float(t))
        self.assertFalse(report['latched']);guard.pause()
        report=guard.check(route,[0,1.53,0],0,10.)
        self.assertEqual(report['continuous_bad_sec'],0.);self.assertGreater(report['convergence_progress_m'],0.)
        self.assertEqual(guard.check(route,[0,0,0],0,10.1)['phase'],'tracking')
        self.assertFalse(guard.check(route,[0,2.,0],0,10.2)['latched'])

    def test_invalid_reference_is_still_latched_until_explicit_reset(self):
        guard=OutdoorTrackingGuard()
        self.assertTrue(guard.check([[math.nan,0,0,1]], [0,0,0],0,0)['latched'])
        route=resample_forward_reference([[0,0,0,1],[20,0,0,1]])
        self.assertTrue(guard.check(route,[0,0,0],0,.1)['latched'])
        guard.reset();self.assertFalse(guard.check(route,[0,0,0],0,.2)['latched'])


if __name__=='__main__':unittest.main()
