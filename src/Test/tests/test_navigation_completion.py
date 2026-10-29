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
from neupan_navigation import (polygon_distances, select_swept_points,
    GeometryObstacleAdapter, MotionFeedback, braking_envelope)
from neupan_safety import (MotionContinuity,continuous_solution,CandidateRejected,
    check_motion_feedback,ResultExpired,AuthorizationChanged,planner_obstacle_input,validate_solution)
sys.path.insert(0,str(ROOT/'src/osm_map_handler/src'))
from osm_camera_path_guard import CameraPathGuard

BODY=np.array([[-.14,.81,.81,-.14],[-.425,-.425,.425,.425]])


class SelectionTests(unittest.TestCase):
    def test_signed_distance_both_windings(self):
        points=np.array([[0.,1.81,0.],[0.,0.,1.425]])
        for body in (BODY,BODY[:,::-1]):
            np.testing.assert_allclose(polygon_distances(points,body),[-.14,1.,1.])

    def test_thin_pole_not_lost_to_scan_stride(self):
        points=np.vstack((np.linspace(-5,15,1001),np.full(1001,4.)))
        points[:,501]=[1.,.43]
        indices,report=select_swept_points(points,np.array([[0.,1.],[0.,0.],[0.,0.]]),BODY,100)
        self.assertIn(501,indices);self.assertEqual(len(indices),100)
        self.assertEqual(len(set(indices)),100)
        self.assertAlmostEqual(report['max_discrete_distance_loss_m'],0.)

    def test_both_turn_sides_and_current_nearest_retained(self):
        points=np.array([[.4,2.,2.,10.],[.5,1.5,-1.5,0.]])
        states=np.array([[0.,2.,2.],[0.,1.,-1.],[0.,.4,-.4]])
        indices,_=select_swept_points(points,states,BODY,6)
        self.assertEqual(set(indices),set(range(4)))

    def test_budget_and_nonfinite_validation(self):
        for budget in (0,-1,.5,True):
            with self.assertRaises(ValueError): select_swept_points(np.ones((2,2)),np.zeros((3,2)),BODY,budget)
        with self.assertRaises(ValueError):select_swept_points(np.full((2,2),np.nan),np.zeros((3,2)),BODY,2)

    def test_empty_has_no_invented_obstacles(self):
        indices,report=select_swept_points(np.zeros((2,0)),np.zeros((3,2)),BODY,100)
        self.assertFalse(len(indices));self.assertEqual(report['full_points'],0)

    def test_torch_hook_keeps_feature_associations(self):
        import torch
        class Dune(torch.nn.Module):
            def forward(self,flow,rotations,points):
                # Intentionally learned order: wall before nearby pole.
                mu=torch.tensor([[111.,222.,333.]])
                return [mu],[mu+1000],[points[0]]
        class Pan:
            no_obs=False;dune_max_num=100;nrmp_max_num=1
            dune_layer=Dune()
            def generate_point_flow(self,*args):return args
        adapter=GeometryObstacleAdapter(NS(pan=Pan(),robot=NS(vertices=BODY)))
        points=torch.tensor([[3.,1.,2.],[0.,.43,0.]])
        mu,lam,sorted_points=adapter.pan.dune_layer([points],[torch.eye(2)],[points])
        self.assertEqual(mu[0][0,0],222.)
        self.assertEqual(lam[0][0,0],1222.)
        np.testing.assert_allclose(sorted_points[0][:,0],[1.,.43])

    def test_pan_adapter_passes_bounded_points_each_iteration(self):
        import torch
        class Pan:
            no_obs=False;dune_max_num=4;nrmp_max_num=2
            dune_layer=torch.nn.Identity()
            def generate_point_flow(self,states,points,velocities=None):return points
        pan=Pan();GeometryObstacleAdapter(NS(pan=pan,robot=NS(vertices=BODY)))
        points=torch.stack((torch.arange(50.),torch.ones(50)))
        for x in (0.,30.):
            selected=pan.generate_point_flow(torch.tensor([[x,x+1],[0.,0.],[0.,0.]]),points)
            self.assertEqual(selected.shape,(2,4))
            self.assertLess(float(abs(selected[0]-x).min()),2.)

    def test_real_deployed_library_and_model_accept_instance_adapter(self):
        import torch
        from neupan import neupan
        checkpoint=ROOT/'src/neupan_ros/example/lxw_car/model/acker_robot_default_7/model_5000.pth'
        if not checkpoint.is_file():self.skipTest('deployed model checkpoint missing')
        torch.set_num_threads(1)
        planner=neupan.init_from_yaml(str(ROOT/'src/neupan_ros/example/lxw_car/config/neupan_config_outdoor.yaml'),
            pan={'dune_checkpoint':str(checkpoint)})
        adapter=GeometryObstacleAdapter(planner)
        planner.set_initial_path([np.array([[x],[0.],[0.],[1.]]) for x in np.arange(0.,20.,.5)])
        points=np.vstack((np.linspace(-5,15,1001),np.full(1001,3.)))
        points[:,501]=[1.,.9]
        action,info=planner(np.zeros((3,1)),planner_obstacle_input(planner,points))
        validate_solution(action,info,planner.robot,planner.dt)
        self.assertEqual(adapter.report['selected_points'],100)
        selected=adapter.numpy(planner.dune_points)
        self.assertLess(np.linalg.norm(selected-points[:,501:502],axis=0).min(),1.e-5)
        action,info=planner(np.zeros((3,1)),planner_obstacle_input(planner,np.empty((2,0))))
        validate_solution(action,info,planner.robot,planner.dt)
        self.assertIsNone(planner.dune_points);self.assertIsNone(planner.nrmp_points)


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

    def test_motion_bound_is_intersection_not_replacement(self):
        robot=NS(kinematics='acker',max_acce=np.array([.8,.08]),max_speed=np.array([1.5,.35]),L=.67)
        policy=MotionContinuity();policy.record([.2,.1],1.)
        policy.feedback_provider=lambda:dict(speed_mps=.15,steering_rad=None,age_ms=0)
        info=dict(vel_tensor=np.array([[.8,.8],[.1,.1]]),opt_state_list=[np.zeros((3,1))])
        action,result=continuous_solution(np.array([[.8],[.1]]),info,robot,.1,policy,1.1)
        self.assertLessEqual(action[0,0],.23+1.e-9)
        self.assertEqual(result['continuity_source'],'wheel_motion_and_command')

    def test_conflicting_motion_does_not_publish_unchecked_jump(self):
        robot=NS(kinematics='acker',max_acce=np.array([.8,.08]),max_speed=np.array([1.5,.35]),L=.67)
        policy=MotionContinuity();policy.record([0.,0.],1.)
        policy.feedback_provider=lambda:dict(speed_mps=1.,steering_rad=None,age_ms=0)
        info=dict(vel_tensor=np.zeros((2,3)),opt_state_list=[np.zeros((3,1))])
        with self.assertRaises(CandidateRejected):continuous_solution(np.zeros((2,1)),info,robot,.1,policy,1.1)

    def test_obstacle_target_cap_decelerates_without_instantaneous_jump(self):
        robot=NS(kinematics='acker',max_acce=np.array([.8,.08]),max_speed=np.array([1.5,.35]),L=.67)
        policy=MotionContinuity();policy.record([.8,0.],1.);policy.restrict(.2)
        info=dict(vel_tensor=np.vstack((np.full(20,.8),np.zeros(20))),opt_state_list=[np.zeros((3,1))])
        action,result=continuous_solution(np.array([[.8],[0.]]),info,robot,.1,policy,1.1)
        self.assertAlmostEqual(action[0,0],.72)
        self.assertAlmostEqual(result['vel_tensor'][0,-1],.2)

    def test_hard_camera_cap_cannot_be_bypassed_as_a_soft_target(self):
        robot=NS(kinematics='acker',max_acce=np.array([.8,.08]),max_speed=np.array([1.5,.35]),L=.67)
        policy=MotionContinuity();policy.record([.8,0.],1.)
        info=dict(speed_cap_mps=.2,vel_tensor=np.zeros((2,20)),opt_state_list=[np.zeros((3,1))])
        with self.assertRaises(CandidateRejected):continuous_solution(np.zeros((2,1)),info,robot,.1,policy,1.1)

    def test_optional_feedback_expiring_after_solve_degrades_without_stop(self):
        policy=MotionContinuity();policy.feedback_snapshot=dict(speed_mps=.2)
        policy.feedback_provider=lambda:dict(speed_mps=None,steering_rad=None)
        check_motion_feedback(policy,[[.2],[0.]],NS(max_acce=np.array([.8,.08])),.1)
        self.assertEqual(policy.feedback_degraded_at_publication,['speed_mps'])

    def test_new_feedback_rejects_old_action_without_latching_fault(self):
        policy=MotionContinuity();policy.feedback_snapshot=dict(speed_mps=.2)
        policy.feedback_provider=lambda:dict(speed_mps=.7,steering_rad=None,age_ms=0.)
        with self.assertRaises(AuthorizationChanged):
            check_motion_feedback(policy,[[.2],[0.]],NS(max_acce=np.array([.8,.08])),.1)


class BrakingShadowTests(unittest.TestCase):
    def test_tail_preserves_momentum_then_stops(self):
        states=np.array([[0.,.08,.16,.24],[0.,0.,0.,0.],[0.,0.,0.,0.]])
        controls=np.array([[.8,.8,.8],[0.,0.,0.]])
        result=braking_envelope(states,controls,NS(kinematics='acker',L=.67),.1,.2,.8)
        self.assertAlmostEqual(result[0,-1],.16+.8**2/(2*.8),places=6)
        self.assertGreater(result.shape[1],states.shape[1])

    def test_missing_or_unbounded_parameters_rejected(self):
        for delay,decel in ((0.,0.),(.1,0.),(10.,.5),(math.nan,.5)):
            with self.assertRaises(ValueError):braking_envelope(np.zeros((3,3)),np.zeros((2,2)),NS(kinematics='acker'),.1,delay,decel)


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
