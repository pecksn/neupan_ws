"""Three Oct28 review counterexamples; real methods, fake ROS outputs only."""
import copy
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
import test_followup_safety as fixture
import test_camera_publication_protocol as protocol
from test_camera_right_refinement import namespace as n
from neupan_navigation import MotionFeedback
from neupan_safety import MotionContinuity,continuous_solution,check_motion_feedback,CandidateRejected,candidate_snapshot
from neupan_diagnostics import CandidateEvidenceSink


class FeedbackRateTests(unittest.TestCase):
    def test_one_hz_average_exists_but_is_not_instantaneous(self):
        feedback=MotionFeedback(timeout=1.5)
        for second in range(1,6):
            feedback.odom([second*.4,0.,0.],second*10**9,float(second),'odom',second*10**9)
            state=feedback.snapshot(second*10**9,float(second))
            if second==1:continue
            self.assertAlmostEqual(state['average_speed_mps'],.4)
            self.assertEqual(state['sample_interval_sec'],1.)
            self.assertEqual(state['average_midpoint_age_ms'],500.)
            self.assertIsNone(state['speed_mps'])
            self.assertEqual(state['speed_source'],'wheel_odom_interval_average_diagnostic_only')

    def test_rates_have_distinct_sampling_and_age_limits(self):
        for interval in (.05,.1,.2,1.,1.4,2.):
            feedback=MotionFeedback(timeout=1.5)
            feedback.odom([0.,0.,0.],10**9,1.,'odom')
            stamp=round((1+interval)*1.e9)
            feedback.odom([.4*interval,0.,0.],stamp,1+interval,'odom')
            state=feedback.snapshot(stamp,1+interval)
            self.assertEqual(state['average_speed_mps'] is not None,interval<=1.5)
            self.assertEqual(state['speed_mps'] is not None,interval<=.2)

    def test_lagged_average_does_not_reject_current_command(self):
        feedback=MotionFeedback(timeout=1.5)
        feedback.odom([0.,0.,0.],10**9,1.,'odom')
        feedback.odom([.4,0.,0.],2*10**9,2.,'odom')
        policy=MotionContinuity();policy.record([.8,0.],1.9)
        policy.feedback_provider=lambda:feedback.snapshot(2*10**9,2.)
        robot=NS(kinematics='acker',max_acce=np.array([.8,.08]),max_speed=np.array([1.5,.35]),L=.67)
        info=dict(vel_tensor=np.array([[.8,.8],[0.,0.]]),opt_state_list=[np.zeros((3,1))])
        action,result=continuous_solution(np.array([[.8],[0.]]),info,robot,.1,policy,2.)
        self.assertAlmostEqual(action[0,0],.8)
        self.assertEqual(result['continuity_source'],'last_sent_command_not_actuator_feedback')
        check_motion_feedback(policy,action,robot,.1)

    def test_average_expires_and_duplicates_do_not_renew_it(self):
        feedback=MotionFeedback(timeout=1.5)
        feedback.odom([0.,0.,0.],10**9,1.,'odom')
        feedback.odom([.4,0.,0.],2*10**9,2.,'odom')
        feedback.odom([.8,0.,0.],2*10**9,3.,'odom')
        self.assertEqual(feedback.snapshot(3*10**9,3.)['average_end_ns'],2*10**9)
        self.assertIsNone(feedback.snapshot(3600000000,3.6)['average_speed_mps'])

    def test_high_rate_estimate_degrades_without_extra_stop(self):
        feedback=MotionFeedback(timeout=1.5)
        feedback.odom([0.,0.,0.],10**9,1.,'odom')
        feedback.odom([.04,0.,0.],1100000000,1.1,'odom')
        policy=MotionContinuity();policy.feedback_snapshot=feedback.snapshot(1100000000,1.1)
        policy.feedback_provider=lambda:feedback.snapshot(1400000000,1.4)
        check_motion_feedback(policy,[[.4],[0.]],NS(max_acce=np.array([.8,.08])),.1)
        self.assertEqual(policy.feedback_degraded_at_publication,['speed_mps'])


def history_road():
    points=np.column_stack((np.arange(0.,100.5,.5),np.zeros(201)))
    geometry={}
    for i in range(53):
        x=points[i,0];gap=3.8 if x>=22 else 2.4
        geometry[i]=dict(points_odom=np.array([[x,0.],[x,5.],[x,-gap]]),
            route_point_odom=[x,0.],normal_odom=[0.,1.],capture_timestamp_ns=10**12)
    return points,geometry


class LocalReferenceTests(unittest.TestCase):
    def test_old_two_point_four_cannot_hide_forward_three_point_eight(self):
        points,geometry=history_road()
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[20.,0.,0.])
        self.assertTrue(report['changed'])
        self.assertAlmostEqual(report['reference_gap_m'],3.8)
        self.assertEqual(report['evidence_points'],9)
        self.assertEqual(report['corridor_evidence_points'],53)
        self.assertEqual(report['decision_window']['route_indices'],[44,52])
        self.assertLess(candidate[80,1],-.7)

    def test_actual_node_uses_local_window_and_distinct_frame_confirmation(self):
        node,_,clock=protocol.RecoveryTests().node();points,geometry=history_road()
        node.path.poses=[NS(pose=NS(position=NS(x=float(x),y=float(y)))) for x,y in points]
        node.strategy='segment_reference';node.clearance_min=1.;node.clearance_max=3.;node.max_shift=1.5
        node.osm_exit_requested=False;node.osm_junctions=[];node.cache_geometry=geometry
        node.cache.committed={i:(0.,clock[0]) for i in geometry}
        node.cache_pose=lambda now,force=False:([20.,0.,0.],0.)
        with patch.dict(n,time=NS(monotonic=lambda:clock[0])):
            for seq in range(1,4):
                node.last_seq=seq;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        self.assertGreater(node.reference_version,0)
        self.assertLess(node.frozen_reference_map[80,1],-.7)
        self.assertEqual(node.final_path_diagnostics['decision_window']['used_points'],9)
        self.assertFalse(node.hazards.regions)

    def test_in_band_current_segment_still_does_not_chase_two(self):
        points,geometry=history_road()
        for i,item in geometry.items():item['points_odom'][2,1]=-2.8 if i>=44 else -3.8
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[20.,0.])
        np.testing.assert_array_equal(candidate,points);self.assertEqual(report['reason'],'hold_in_band')

    def test_historical_real_danger_scope_is_not_trimmed(self):
        points,geometry=history_road();geometry[2]['points_odom'][2,1]=-.1
        with self.assertRaises(n['CorridorUnsafe']):
            n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[20.,0.])

    def test_no_forward_statistics_cannot_skip_real_danger_check(self):
        points,geometry=history_road();geometry={2:geometry[2]};geometry[2]['points_odom'][2,1]=-.1
        with self.assertRaises(n['CorridorUnsafe']):
            n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[20.,0.])

    def test_return_road_heading_and_progress_do_not_merge(self):
        forward=np.column_stack((np.arange(0.,31.),np.zeros(31)))
        returning=np.column_stack((np.arange(30.,-1.,-1.),np.ones(31)*.5))
        points=np.vstack((forward,returning));geometry={i:{} for i in range(len(points))}
        keys,report=n['reference_evidence_window'](points,geometry,geometry,[10.,.1],progress=10.,heading=0.)
        self.assertTrue(keys);self.assertTrue(all(i<31 for i in keys));self.assertAlmostEqual(report['progress_s'],10.)

    def test_loop_without_progress_is_ambiguous_not_a_new_shift(self):
        route=np.array([[0.,0.],[20.,0.],[20.,20.],[0.,20.],[0.,0.],[20.,0.]])
        geometry={i:{} for i in range(len(route))}
        keys,report=n['reference_evidence_window'](route,geometry,geometry,[10.,0.],heading=0.)
        self.assertFalse(keys);self.assertEqual(report['reason'],'ambiguous_route_progress')


class EvidenceCycleTests(unittest.TestCase):
    def test_solver_validation_failure_has_no_previous_arrays_or_attempts(self):
        captured=[]
        def hook(node,clock,ns):
            node.candidate_original_info=dict(opt_state_list=[np.ones((3,1))*10])
            node.candidate_attempts=[node.candidate_original_info]
            node.diagnostic_raw_solver_states=np.ones((3,2))*10
            node.diagnostic_safety_scan=(np.ones((2,1)),123,1.)
            original=node.hold_planner_fault
            def hold(*args):
                captured.append(dict(original=node.candidate_original_info,attempts=list(node.candidate_attempts),
                    raw=node.diagnostic_raw_solver_states,scan=node.diagnostic_safety_scan))
                original(*args)
            node.hold_planner_fault=hold
        node,commands=fixture.CommandBoundaryTests().run_loop(empty_error=True,setup_hook=hook)
        self.assertTrue(all(v==0 for v in commands));self.assertEqual(len(captured),1)
        self.assertIsNone(captured[0]['original']);self.assertIsNone(captured[0]['raw'])
        self.assertEqual(captured[0]['attempts'],[])
        self.assertNotEqual(captured[0]['scan'][1],123)

    def test_candidate_snapshot_is_not_mutable_vendor_info_alias(self):
        info=dict(opt_state_list=[np.zeros((3,1))],vel_tensor=np.zeros((2,1)))
        result=candidate_snapshot(info)
        info['opt_state_list'][0][0]=10.;info['vel_tensor'][0]=1.
        self.assertEqual(result['opt_state_list'][0][0,0],0.)
        self.assertEqual(result['vel_tensor'][0,0],0.)

    def test_unbound_cycle_is_dropped_and_missing_data_not_fabricated(self):
        with tempfile.TemporaryDirectory() as folder:
            sink=CandidateEvidenceSink(folder,interval=0.)
            node=NS(diagnostic_cycle_id=2,diagnostic_context_cycle_id=1)
            sink.capture(node,CandidateRejected('mismatch'),None)
            self.assertEqual(sink.dropped,1);self.assertEqual(sink.count,0)
            node.diagnostic_context_cycle_id=2
            sink.capture(node,CandidateRejected('before candidate'),None)
            sink.queue.join()
            record=json.loads(next(Path(folder).glob('*.json')).read_text())
            self.assertEqual(record['cycle_id'],2)
            self.assertIn('original_states',record['missing_fields'])
            self.assertIn('scan_points',record['missing_fields'])

    def test_real_two_cycle_rejection_cannot_capture_previous_candidate(self):
        with tempfile.TemporaryDirectory() as folder:
            sink=CandidateEvidenceSink(folder,interval=0.)
            def hook(node,clock,ns):
                node.motion_policy=MotionContinuity();node.candidate_evidence_sink=sink
                node.safety_scan=(node.obstacle_points.copy(),10**12,1000.)
                base=type(node.neupan_planner)
                class Planner(base):
                    def __call__(self,state,points):
                        self.calls+=1;x=10. if self.calls==1 else 0.
                        if self.calls==1:
                            clock.value+=.1
                            node.safety_scan=(node.obstacle_points.copy(),1000100000000,clock.value)
                        return np.array([[.8],[0.]]),dict(stop=False,arrive=False,
                            opt_state_list=[np.array([[x+dx],[0.],[0.]]) for dx in (0.,.08,.16)],
                            ref_state_list=[np.array([[x],[0.],[0.]])],vel_tensor=np.array([[.8,.8],[0.,0.]]))
                node.neupan_planner=Planner()
                node.listener.getLatestCommonTime=lambda *_:NS(to_nsec=lambda:round(clock.value*1.e9))
                node.listener.lookupTransform=lambda *_:((10. if node.neupan_planner.calls==0 else 0.,0.,0.),(0.,0.,0.,1.))
                original_continuity=ns['continuous_solution'];first=[True]
                def continuous(*args):
                    if node.neupan_planner.calls==2:
                        self.assertIsNone(node.candidate_original_info)
                        self.assertEqual(node.candidate_attempts,[])
                        raise CandidateRejected('second-cycle early continuity rejection')
                    return original_continuity(*args)
                def collision(*args):
                    if first[0]:
                        first[0]=False;raise CandidateRejected('first-cycle collision',dict(point_body=[1.,0.]))
                    return dict(min_clearance_m=.2)
                ns['continuous_solution']=continuous;ns['trajectory_clearance']=collision
            node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=hook)
            sink.queue.join();self.assertIsNone(getattr(node,'planner_fault',None))
            records=[(json.loads(p.read_text()),p.with_suffix('.npz')) for p in sorted(Path(folder).glob('*.json'))]
            self.assertEqual(len(records),2)
            for expected,(record,path) in zip((10.,0.),records):
                with np.load(path) as arrays:
                    self.assertEqual(arrays['original_states'][0,0],expected)
                    self.assertEqual(arrays['raw_solver_states'][0,0],expected)
                    if expected==0.:self.assertFalse(any(k.startswith('alternative_') for k in arrays.files))
                self.assertTrue(all(v['cycle_id']==record['cycle_id'] for v in record['array_bindings'].values()))
            self.assertNotEqual(records[0][0]['scan_stamp_ns'],records[1][0]['scan_stamp_ns'])
            self.assertEqual(commands[-1],0.)


if __name__=='__main__':unittest.main()
