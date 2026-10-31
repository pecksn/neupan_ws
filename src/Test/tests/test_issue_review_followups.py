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
from test_startup_recovery import Stamp
from test_camera_right_refinement import namespace as n
from neupan_navigation import MotionFeedback
from neupan_safety import CandidateRejected,candidate_snapshot

def history_road():
    points=np.column_stack((np.arange(0.,100.5,.5),np.zeros(201)))
    geometry={i:dict(points_odom=np.array([[points[i,0],0.],[points[i,0],5.],
        [points[i,0],-3.8 if i>=44 else -2.4]]),route_point_odom=points[i].tolist(),
        normal_odom=[0.,1.],capture_timestamp_ns=10**12) for i in range(53)}
    return points,geometry


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

    def test_average_expires_and_duplicates_do_not_renew_it(self):
        feedback=MotionFeedback(timeout=1.5)
        feedback.odom([0.,0.,0.],10**9,1.,'odom')
        feedback.odom([.4,0.,0.],2*10**9,2.,'odom')
        feedback.odom([.8,0.,0.],2*10**9,3.,'odom')
        self.assertEqual(feedback.snapshot(3*10**9,3.)['average_end_ns'],2*10**9)
        self.assertIsNone(feedback.snapshot(3600000000,3.6)['average_speed_mps'])

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
        with patch.dict(n,time=NS(monotonic=lambda:clock[0]),rospy=NS(Time=NS(now=lambda:Stamp(clock[0])))):
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

    def test_historical_cache_does_not_veto_forward_reference(self):
        points,geometry=history_road();geometry[2]['points_odom'][2,1]=-.1
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[20.,0.])
        self.assertTrue(report['changed']);self.assertLess(candidate[80,1],-.7)

    def test_no_forward_statistics_holds_without_promoting_cache_to_danger(self):
        points,geometry=history_road();geometry={2:geometry[2]};geometry[2]['points_odom'][2,1]=-.1
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[20.,0.])
        np.testing.assert_array_equal(candidate,points);self.assertFalse(report['changed'])

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


if __name__=='__main__':unittest.main()
