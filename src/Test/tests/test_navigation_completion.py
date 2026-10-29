"""Outdoor adapters: no ROS master or vehicle control publishers."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np
import test_followup_safety as fixture
import test_camera_publication_protocol as protocol

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/neupan_ros/src'))
from neupan_navigation import MotionFeedback
from neupan_safety import CandidateRejected, ResultExpired, AuthorizationChanged, planner_obstacle_input, validate_solution
sys.path.insert(0,str(ROOT/'src/osm_map_handler/src'))
from osm_camera_path_guard import CameraPathGuard

BODY=np.array([[-.14,.81,.81,-.14],[-.425,-.425,.425,.425]])


class FeedbackTests(unittest.TestCase):
    def make(self):
        feedback=MotionFeedback()
        feedback.odom([0.,0.,0.],10**9,1.,'wheel_odom')
        return feedback

    def test_forward_and_reverse_signed_pose_speed(self):
        for distance in (.04,-.04):
            feedback=self.make();feedback.odom([distance,0.,0.],1100000000,1.1,'wheel_odom')
            state=feedback.snapshot(1100000000,1.1)
            self.assertAlmostEqual(state['speed_mps'],distance/.1)
            self.assertIsNone(state['steering_rad'])

    def test_duplicate_and_old_packets_do_not_renew_age(self):
        feedback=self.make();feedback.odom([.04,0.,0.],1100000000,1.1,'wheel_odom')
        feedback.odom([.08,0.,0.],1100000000,1.5,'wheel_odom')
        feedback.odom([.08,0.,0.],1000000000,1.6,'wheel_odom')
        self.assertIsNone(feedback.snapshot(1700000000,1.7)['speed_mps'])

    def test_time_jump_frame_change_and_pose_jump_invalidate(self):
        for pose,stamp,frame in (([.1,0.,0.],2*10**9,'wheel_odom'),
                                ([.1,0.,0.],1100000000,'new'),([10.,0.,0.],1100000000,'wheel_odom')):
            feedback=self.make();feedback.odom(pose,stamp,stamp/1.e9,frame)
            self.assertIsNone(feedback.snapshot(stamp,stamp/1.e9)['speed_mps'])

    def test_neither_future_stamp_nor_old_reception_accepted(self):
        feedback=self.make();feedback.odom([.04,0.,0.],1100000000,1.1,'wheel_odom')
        self.assertIsNone(feedback.snapshot(1000000000,1.1)['speed_mps'])
        self.assertIsNone(feedback.snapshot(1200000000,2.)['speed_mps'])

    def test_bad_future_packet_does_not_poison_normal_recovery(self):
        feedback=self.make()
        feedback.odom([.04,0.,0.],10**15,1.1,'wheel_odom',1100000000)
        feedback.odom([.08,0.,0.],1200000000,1.2,'wheel_odom',1200000000)
        feedback.odom([.12,0.,0.],1300000000,1.3,'wheel_odom',1300000000)
        self.assertAlmostEqual(feedback.snapshot(1300000000,1.3)['speed_mps'],.4)

    def test_encoder_independent_of_pose_and_expires(self):
        feedback=self.make();feedback.encoder(.2,1100000000,1.1)
        state=feedback.snapshot(1200000000,1.2)
        self.assertEqual(state['steering_rad'],.2);self.assertIsNone(state['speed_mps'])
        self.assertIsNone(feedback.snapshot(1800000000,1.8)['steering_rad'])

class HistoricalHazardScopeTests(unittest.TestCase):
    def guard(self,returning=False):
        guard=CameraPathGuard();guard.original_key=('map',1,40.,0.)
        guard.hazard_regions={'danger':dict(kind='boundary',region_odom=[0.,0.])}
        guard.corridor_hold_key=guard.original_key
        guard.status=dict(inference_diagnostics=dict(reference_role='advisory'))
        route=np.column_stack((np.arange(0.,40.5,.5),np.zeros(81)))
        if returning:route=np.vstack((route,route[::-1]))
        return guard,route

    def pass_region(self,guard,route):
        for i in range(14):
            stamp=round((1.+i*.1)*1.e9)
            guard.update_hazard_scope([8.+i*.1,0.,0.],[route,route],stamp,stamp)
        return stamp

    def test_past_disjoint_region_keeps_history_and_needs_prediction_ack(self):
        guard,route=self.guard();stamp=self.pass_region(guard,route)
        self.assertTrue(guard.scope_allows_route(stamp));self.assertIn('danger',guard.hazard_regions)
        self.assertFalse(guard.commit_recovery(guard.authorization_generation,0,stamp))
        self.assertTrue(guard.check_scoped_prediction(np.array([[9.,0.,0.],[11.,0.,0.]]),stamp))
        self.assertTrue(guard.commit_recovery(guard.authorization_generation,0,stamp))
        self.assertIn('danger',guard.hazard_regions);self.assertIsNotNone(guard.corridor_hold_key)

    def test_route_returning_to_danger_cannot_retire(self):
        guard,route=self.guard(returning=True);stamp=self.pass_region(guard,route)
        self.assertFalse(guard.scope_allows_route(stamp))

    def test_actual_detour_reentering_region_rejected(self):
        guard,route=self.guard();stamp=self.pass_region(guard,route)
        self.assertFalse(guard.check_scoped_prediction(np.array([[9.,0.,0.],[0.,0.,0.]]),stamp))
        self.assertFalse(guard.commit_recovery(guard.authorization_generation,0,stamp))

    def test_scope_expires_and_jump_restarts_measured_passage(self):
        guard,route=self.guard();stamp=self.pass_region(guard,route)
        self.assertFalse(guard.scope_allows_route(stamp+500000001))
        guard.update_hazard_scope([20.,0.,0.],[route,route],stamp+100000000,stamp+100000000)
        self.assertFalse(guard.scoped_hazards)

    def test_route_change_or_candidate_reentry_reactivates(self):
        guard,route=self.guard();stamp=self.pass_region(guard,route)
        candidate=np.vstack((route[:20],np.array([[0.,0.],[40.,0.]])))
        guard.update_hazard_scope([9.4,0.,0.],[route,candidate],stamp+100000000,stamp+100000000)
        self.assertFalse(guard.scoped_hazards)

    def test_legacy_unlocated_and_capacity_stops_not_bypassed(self):
        for flag in ('recovery_unlocated_required','hazard_capacity_exceeded'):
            guard,route=self.guard();guard.status['inference_diagnostics'][flag]=True
            stamp=self.pass_region(guard,route)
            self.assertFalse(guard.scope_allows_route(stamp))

    def scoped_run(self,delay=0.):
        route=protocol.path();guard=CameraPathGuard()
        guard.original_cb(route);guard.candidate_cb(route)
        state=protocol.status(route,None)
        state.update(active=False,corridor_stop_required=True,corridor_verified=False,
            hazard_regions={'past':dict(kind='boundary',region_odom=[-9.,0.])})
        state['inference_diagnostics']=dict(reference_role='advisory')
        guard.status_cb(state,1000.)
        points=np.array([[p.pose.position.x,p.pose.position.y] for p in route.poses])
        for i in range(14):
            stamp=round((998.7+i*.1)*1.e9)
            guard.update_hazard_scope([-1.3+i*.1,0.,0.],[points,points],stamp,stamp)
        def hook(node,clock,ns):
            node.camera_guard=guard
            def apply():
                selected=guard.select(clock.value,round(clock.value*1.e9))[2]
                node.camera_guard_selected_state=selected
                node.camera_execution_contract=selected['execution_contract']
                node.camera_safety_generation=selected['safety_generation']
                return selected['source']!='stop'
            node.apply_camera_path_guard=apply
        node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=hook,delay=delay,delay_at='camera_check')
        return node,commands,guard

    def test_actual_run_disjoint_scope_checks_prediction_and_keeps_history(self):
        node,commands,guard=self.scoped_run()
        self.assertTrue(commands and all(v>0 for v in commands))
        self.assertIsNone(getattr(node,'planner_fault',None))
        self.assertIn('past',guard.hazard_regions)

    def test_actual_run_scope_expiring_before_publish_cannot_release(self):
        node,commands,guard=self.scoped_run(delay=.6)
        self.assertTrue(commands and all(v==0 for v in commands))
        self.assertIn('past',guard.hazard_regions)


if __name__=='__main__':unittest.main()
