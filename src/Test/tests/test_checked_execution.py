"""One native solver, finite ROS dispatch; fake publishers, never hardware."""
import ast
from contextlib import nullcontext
import importlib.util
import math
from pathlib import Path
import sys
import threading
import time
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'neupan_ros/src'))
from neupan_execution import (AcceptedExecution, RevokingPublisher, execution_clearance, validate_calibration)
from neupan_execution import check_measured_reserve_limits
from neupan_navigation import directional_stopping_clearance
from neupan_safety import (MotionContinuity, cap_solution_speed, CandidateRejected, AuthorizationChanged)
import test_followup_safety as fixture


def robot():
    return NS(kinematics='acker',L=.67,max_speed=np.array([1.5,.35]),max_acce=np.array([.8,.08]),
        vertices=np.array([[-.14,.81,.81,-.14],[-.425,-.425,.425,.425]]))


def candidate(speed=.4,steer=0.,steps=50):
    controls=np.tile([[speed],[steer]],(1,steps))
    _,info=cap_solution_speed(controls[:,:1],dict(stop=False,arrive=False,vel_tensor=controls,
        opt_state_list=[np.zeros((3,1))],ref_state_list=[]),robot(),.1,1.5)
    return info


def calibration():
    # SYNTHETIC TEST FIXTURE, deliberately NOT installed as vehicle parameters.
    return dict(verified=True,calibration_id='synthetic_test_only',speed_upper_bound_mps=1.5,
        total_delay_upper_bound_sec=.2,braking_deceleration_lower_bound_mps2=3.,
        steering_error_upper_bound_rad=.01,steering_rate_upper_bound_radps=.08,
        steering_abs_upper_bound_rad=.45,post_stop_clearance_m=.02)


class HorizonClassificationTests(unittest.TestCase):
    def test_no_calibration_adds_no_gate_to_full_clear_candidate(self):
        check=execution_clearance(candidate(),np.zeros((2,0)),robot(),.1)
        self.assertEqual(check['scope'],'full_native_horizon')
        self.assertFalse(check['full_scan_rejection'])

    def test_far_conflict_requires_measurements(self):
        with self.assertRaises(CandidateRejected) as cm:
            execution_clearance(candidate(.8),np.array([[4.2],[0.]]),robot(),.1)
        self.assertEqual(cm.exception.details['continuation_unavailable'],'no_measured_stopping_reserve')

    def test_far_conflict_can_request_native_replanning_not_zero(self):
        result=execution_clearance(candidate(.8),np.array([[4.2],[0.]]),robot(),.1,calibration())
        self.assertTrue(result['requires_replanning'])
        self.assertTrue(result['full_scan_rejection'])
        self.assertFalse(result['native_stop'])
        self.assertGreater(result['first_conflict']['step'],20)

    def test_near_obstacle_still_rejected(self):
        with self.assertRaises(CandidateRejected):
            execution_clearance(candidate(.8),np.array([[1.],[0.]]),robot(),.1,calibration())

    def test_unrelated_side_obstacle_does_not_trigger_old_omnidirectional_disk(self):
        result=execution_clearance(candidate(.8),np.array([[4.2,0.],[0.,1.]]),robot(),.1,calibration())
        self.assertTrue(result['requires_replanning'])
        self.assertEqual(result['directional_stopping_reserve']['direction_coverage'],'both')

    def test_side_point_inside_actual_near_footprint_still_rejected(self):
        with self.assertRaises(CandidateRejected):
            execution_clearance(candidate(.8),np.array([[4.2,0.],[0.,.45]]),robot(),.1,calibration())

    def test_old_profile_without_steering_bounds_cannot_grant_exception(self):
        profile=calibration()
        for name in ('steering_error_upper_bound_rad','steering_rate_upper_bound_radps',
                     'steering_abs_upper_bound_rad','post_stop_clearance_m'):profile.pop(name)
        with self.assertRaises(CandidateRejected) as cm:
            execution_clearance(candidate(.8),np.array([[4.2],[0.]]),robot(),.1,profile)
        self.assertEqual(cm.exception.details['classification'],'conflict_without_directional_bounds')

    def test_classification_keeps_far_time_and_near_rejection_separate(self):
        result=execution_clearance(candidate(.8),np.array([[4.2],[0.]]),robot(),.1,calibration())
        self.assertEqual(result['classification'],'far_conflict_near_and_stopping_clear')
        self.assertGreater(result['first_conflict_time_sec'],4.)
        with self.assertRaises(CandidateRejected) as cm:
            execution_clearance(candidate(.8),np.array([[1.],[0.]]),robot(),.1,calibration())
        self.assertEqual(cm.exception.details['classification'],'near_unsafe')

    def test_directional_reserve_covers_motion_not_just_native_near_speed(self):
        info=candidate(.8)
        for point in ([1.60,0.],[-.95,0.]):
            with self.subTest(point=point),self.assertRaises(CandidateRejected):
                directional_stopping_clearance(np.hstack(info['opt_state_list']),info['vel_tensor'],
                    np.asarray(point).reshape(2,1),robot(),.1,calibration())

    def test_directional_reserve_invariant_to_map_origin_and_rotation(self):
        info=candidate(.8);states=np.hstack(info['opt_state_list']);points=np.array([[4.2,0.],[0.,1.]])
        base=directional_stopping_clearance(states,info['vel_tensor'],points,robot(),.1,calibration())
        yaw=.8;rotation=np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
        offset=np.array([[1000.],[-1200.]])
        states[:2]=rotation@states[:2]+offset;states[2]+=yaw
        other=directional_stopping_clearance(states,info['vel_tensor'],rotation@points+offset,robot(),.1,calibration())
        np.testing.assert_allclose(other['occupancy_bounds_body'],base['occupancy_bounds_body'],atol=1.e-10)

    def test_turning_braking_with_changing_steer_is_covered_in_both_directions(self):
        for direction in (-1.,1.):
            for steering in (-.3,.3):
                profile=calibration();info=candidate(direction*.8,steering)
                pose=np.zeros(3);step=.002;delay=.3;duration=.8
                for t in np.arange(0.,duration,step):
                    velocity=direction*max(0.,1.5-3.*max(0.,t-delay))
                    # Real angle changes during braking, does NOT stay fixed.
                    actual_steer=steering-math.copysign(.08*max(0.,t-delay),steering)
                    pose+=np.array([velocity*math.cos(pose[2]),velocity*math.sin(pose[2]),
                        velocity*math.tan(actual_steer)/.67])*step
                angle=pose[2];rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
                vertex=robot().vertices[:,1 if direction>0 else 0]
                point=(pose[:2]+rotation@vertex).reshape(2,1)
                with self.subTest(direction=direction,steering=steering),self.assertRaises(CandidateRejected):
                    directional_stopping_clearance(np.hstack(info['opt_state_list']),info['vel_tensor'],
                                                   point,robot(),.1,profile)

    def test_directional_schema_rejects_unbounded_or_partial_steering_assumptions(self):
        for patch_value in (dict(steering_abs_upper_bound_rad=math.pi/2),
                dict(steering_rate_upper_bound_radps=-1.),dict(post_stop_clearance_m=-.1)):
            with self.subTest(patch_value=patch_value),self.assertRaises(ValueError):
                validate_calibration(dict(calibration(),**patch_value),robot())

    def test_native_stop_cannot_be_overridden_by_remote_conflict(self):
        info=candidate(.8);info['stop']=True
        with self.assertRaises(CandidateRejected):
            execution_clearance(info,np.array([[4.2],[0.]]),robot(),.1,calibration())

    def test_short_native_sequence_cannot_cover_stopping_reserve(self):
        with self.assertRaises(CandidateRejected):
            execution_clearance(candidate(.8,steps=3),np.array([[.95],[0.]]),robot(),.1,calibration())

    def test_profile_rejects_missing_unverified_nonfinite_and_inadequate_bounds(self):
        for updates in (dict(verified=False),dict(calibration_id=''),dict(speed_upper_bound_mps=.8),
                dict(total_delay_upper_bound_sec=0.),dict(braking_deceleration_lower_bound_mps2=math.nan)):
            with self.subTest(updates=updates),self.assertRaises(ValueError):
                validate_calibration(dict(calibration(),**updates),robot())
        self.assertEqual(validate_calibration(calibration(),robot()),calibration())

    def test_remote_exception_requires_actual_delay_and_feedback_inside_profile(self):
        for ages,feedback in ((dict(scan_measurement_age_ms=201),None),
                              (dict(scan_measurement_age_ms=100),dict(speed_mps=1.6))):
            with self.assertRaises(CandidateRejected):
                check_measured_reserve_limits(dict(requires_replanning=True),calibration(),ages,feedback)
        check_measured_reserve_limits(dict(requires_replanning=False),None,dict(state_age_ms=400))


class DispatchTests(unittest.TestCase):
    def setup_execution(self,real_clock=False):
        self.now=100.;self.context=1;self.points=np.zeros((2,0));self.stale=False
        self.output=[];self.reports=[];self.gate_lock=threading.RLock()
        self.clock=time.monotonic if real_clock else lambda:self.now
        self.policy=MotionContinuity()
        def inputs():
            now=self.clock();stamp=now-1. if self.stale else now
            return dict(pose=np.zeros(3),scan=(self.points.copy(),round(stamp*1.e9),stamp),
                state_stamp_ns=round(stamp*1.e9),context=self.context,now_ns=lambda:round(self.clock()*1.e9))
        self.manager=AcceptedExecution(robot(),.1,self.policy,inputs,lambda:self.context,
            lambda:self.gate_lock,lambda action:self.output.append((self.clock(),np.asarray(action).reshape(2).copy())),
            self.reports.append,clock=self.clock)
        self.manager.stage(candidate(),self.context,dict(cycle_id=1,scan_stamp_ns=123),action=np.array([[.4],[0.]]))
        self.addCleanup(self.manager.close)
        return self.manager

    def test_progress_without_renewing_origin_or_expiry(self):
        m=self.setup_execution();active=m.active;end=active['expires']
        for offset in (.11,.22,.61):self.now=100.+offset;self.assertTrue(m.tick())
        self.assertEqual(m.active['expires'],end)
        self.assertEqual(m.last_result['origin']['scan_stamp_ns'],123)
        self.assertEqual(m.last_result['consumed_steps'],6)
        self.assertAlmostEqual(m.last_result['remaining_sec'],4.39)
        self.assertTrue(all(action[0]>.0 for _,action in self.output))

    def test_finite_sequence_stops_even_with_fresh_sensors(self):
        m=self.setup_execution();self.now=104.6
        self.assertFalse(m.tick());self.assertIsNone(m.active)
        self.assertEqual(self.output[-1][1].tolist(),[0.,0.])
        self.assertEqual(self.reports[-1]['fault_kind'],'coverage')

    def test_stale_input_stops_and_does_not_restamp(self):
        m=self.setup_execution();self.stale=True
        self.assertFalse(m.resume());self.assertIsNone(m.active)
        self.assertEqual(self.reports[-1]['fault_kind'],'input')

    def test_new_obstacle_invalidates_previous_safe_native_sequence(self):
        m=self.setup_execution();self.points=np.array([[.5],[0.]])
        self.assertFalse(m.resume());self.assertEqual(self.reports[-1]['fault_kind'],'geometry')
        self.assertEqual(self.output[-1][1].tolist(),[0.,0.])

    def test_task_changed_during_solve_does_not_accept_stale_candidate(self):
        m=self.setup_execution();self.context=2
        with self.assertRaises(AuthorizationChanged):m.stage(candidate(),1,dict(cycle_id=2))
        self.assertFalse(m.tick());self.assertIsNone(m.active)

    def test_explicit_stop_revokes_before_publication(self):
        m=self.setup_execution();pub=RevokingPublisher(NS(publish=lambda message:self.output.append((self.now,np.zeros(2)))),m)
        pub.publish(NS(linear=NS(x=0.),angular=NS(z=0.)))
        self.assertIsNone(m.active);self.assertFalse(m.tick())
        self.assertEqual(self.output[-1][1].tolist(),[0.,0.])

    def test_native_stop_and_arrival_retire_sequence(self):
        for flag in ('stop','arrive'):
            m=self.setup_execution();info=candidate();info[flag]=True
            self.assertFalse(m.stage(info,1,{}));self.assertIsNone(m.active)

    def test_fresh_solve_installs_new_version_and_no_zero_between(self):
        m=self.setup_execution();version=m.version;self.now+=.1
        m.stage(candidate(),1,dict(cycle_id=2),action=np.array([[.4],[0.]]))
        self.assertGreater(m.version,version);self.assertEqual(m.active['origin']['cycle_id'],2)
        self.assertTrue(all(action[0]>.0 for _,action in self.output))

    def test_old_inflight_failed_check_cannot_cancel_newly_accepted_solve(self):
        m=self.setup_execution();original=m.inputs
        def newer():
            live=original();live['context']=99
            m.stage(candidate(),1,dict(cycle_id=2))
            return live
        m.inputs=newer
        self.assertFalse(m.tick());self.assertEqual(m.active['origin']['cycle_id'],2)
        self.assertEqual(len(self.output),1)

    def test_authority_rechecked_after_expensive_geometry(self):
        m=self.setup_execution()
        import neupan_execution as module
        original=module.execution_clearance
        def slow(*args):
            result=original(*args);self.context=2;return result
        with patch.object(module,'execution_clearance',side_effect=slow):self.assertFalse(m.tick())
        self.assertEqual(self.reports[-1]['fault_kind'],'authority')

    def test_exact_input_age_rechecked_after_geometry(self):
        m=self.setup_execution()
        import neupan_execution as module
        original=module.execution_clearance
        def slow(*args):
            result=original(*args);self.now+=.51;return result
        with patch.object(module,'execution_clearance',side_effect=slow):self.assertFalse(m.tick())
        self.assertEqual(self.reports[-1]['fault_kind'],'result_expired')
        self.assertIsNone(m.input_failure);self.assertIsNotNone(m.active)
        self.assertEqual(self.reports[-1]['retry_count'],1)
        self.assertEqual(self.output[-1][1].tolist(),[0.,0.])
        self.assertFalse(m.last_result['continued'])

    def test_490ms_plus_20ms_rebinds_complete_scan_pose_without_zero(self):
        m=self.setup_execution();base=m.inputs;calls=[]
        def inputs():
            live=base()
            if not calls:
                live['scan']=(self.points.copy(),round((self.now-.49)*1.e9),self.now-.49)
                live['state_stamp_ns']=round((self.now-.49)*1.e9)
            calls.append(live);return live
        m.inputs=inputs
        import neupan_execution as module
        original=module.execution_clearance;geometry=[]
        def slow(*args):
            geometry.append(args[1].copy())
            result=original(*args)
            if len(geometry)==1:self.now+=.02;self.points=np.array([[10.],[10.]])
            return result
        active=m.active;expires=active['expires'];version=m.version
        with patch.object(module,'execution_clearance',side_effect=slow):self.assertTrue(m.tick())
        self.assertEqual(len(geometry),2);self.assertEqual(geometry[1].shape,(2,1))
        self.assertIs(m.active,active);self.assertEqual(m.active['expires'],expires);self.assertEqual(m.version,version)
        self.assertIsNone(m.input_failure);self.assertEqual(m.last_result['retry_count'],1)
        self.assertAlmostEqual(m.last_result['rechecks'][0]['bind_scan_age_ms'],490.)
        self.assertTrue(all(action[0]>.0 for _,action in self.output))

    def test_expired_rebind_real_outage_sets_input_failure_and_zero(self):
        m=self.setup_execution()
        import neupan_execution as module
        original=module.execution_clearance
        def slow(*args):
            result=original(*args);self.now+=.51;self.stale=True;return result
        with patch.object(module,'execution_clearance',side_effect=slow):self.assertFalse(m.tick())
        self.assertIsNotNone(m.input_failure);self.assertIsNone(m.active)
        self.assertEqual(self.reports[-1]['fault_kind'],'input')

    def test_new_obstacle_in_rebound_frame_must_be_geometrically_rechecked(self):
        m=self.setup_execution()
        import neupan_execution as module
        original=module.execution_clearance
        def slow(*args):
            result=original(*args);self.now+=.51;self.points=np.array([[.2],[0.]])
            return result
        with patch.object(module,'execution_clearance',side_effect=slow):self.assertFalse(m.tick())
        self.assertEqual(self.reports[-1]['fault_kind'],'geometry')
        self.assertIsNone(m.input_failure);self.assertIsNone(m.active)

    def test_new_solve_during_expired_check_is_not_revoked_by_retry(self):
        m=self.setup_execution()
        import neupan_execution as module
        original=module.execution_clearance
        def slow(*args):
            result=original(*args);self.now+=.51
            m.stage(candidate(),1,dict(cycle_id=2),action=np.array([[.4],[0.]]))
            return result
        with patch.object(module,'execution_clearance',side_effect=slow):self.assertFalse(m.tick())
        self.assertEqual(m.active['origin']['cycle_id'],2);self.assertIsNone(m.input_failure)
        self.assertTrue(all(action[0]>.0 for _,action in self.output))

    def test_authority_revoked_during_expiry_retry_cannot_send_another_nonzero(self):
        m=self.setup_execution()
        import neupan_execution as module
        original=module.execution_clearance
        def slow(*args):
            result=original(*args);self.now+=.51;self.context=2;return result
        with patch.object(module,'execution_clearance',side_effect=slow):self.assertFalse(m.tick())
        self.assertIsNone(m.active);self.assertEqual(len(self.output),2)
        self.assertEqual(self.output[-1][1].tolist(),[0.,0.]);self.assertIsNone(m.input_failure)

    def test_two_expired_checks_do_not_claim_a_pending_valid_speed_after_zero(self):
        m=self.setup_execution()
        import neupan_execution as module
        original=module.execution_clearance
        def slow(*args):
            result=original(*args);self.now+=.51;return result
        with patch.object(module,'execution_clearance',side_effect=slow):self.assertFalse(m.tick())
        m.check_lock.acquire()
        try:self.assertFalse(m.resume())
        finally:m.check_lock.release()
        self.assertIsNone(m.input_failure)

    def test_internal_fault_is_latched_and_diagnostic_failure_cannot_unstop(self):
        m=self.setup_execution();m.inputs=lambda:1/0;m.report=lambda _:1/0
        self.assertFalse(m.tick());self.assertIn('ZeroDivisionError',m.fault)
        self.assertIsInstance(m.fault_info[1],ZeroDivisionError)
        with self.assertRaises(AuthorizationChanged):m.stage(candidate(),1,{})
        self.assertEqual(self.output[-1][1].tolist(),[0.,0.])

    def test_last_actual_command_continuity_is_rechecked_at_new_adoption(self):
        m=self.setup_execution()
        with self.assertRaises(AuthorizationChanged):
            m.stage(candidate(.8),1,{},action=np.array([[.8],[0.]]))

    def test_no_clock_rewind_renews_sequence(self):
        m=self.setup_execution();self.now=99.
        self.assertFalse(m.tick());self.assertIsNone(m.active)

    def test_worker_keeps_publish_interval_below_unchanged_watchdog_while_main_blocked(self):
        m=self.setup_execution(real_clock=True);m.start()
        # Test thread only: no ROS nodes, no real vehicle commands.
        threading.Event().wait(.66)
        m.close()
        self.assertGreaterEqual(len(self.output),5)
        times=[stamp for stamp,_ in self.output]
        self.assertLess(max(np.diff(times)),.5)
        self.assertTrue(all(action[0]>.0 for _,action in self.output))


class RealWrapperBranchTests(unittest.TestCase):
    def integrated_setup(self,behavior):
        def setup(node,clock,ns):
            from neupan_execution import execution_clearance
            ns['execution_clearance']=execution_clearance
            ns['check_measured_reserve_limits']=check_measured_reserve_limits
            node.motion_policy=MotionContinuity();node.execution_route_version=0
            node.neupan_planner.min_distance=NS(detach=lambda:NS(item=lambda:.1))
            node.neupan_planner.collision_threshold=.2
            node.execution_calibration=None
            node.safety_scan=(node.obstacle_points.copy(),round(clock.value*1.e9),clock.value)
            stamp_type=type(node.listener.getLatestCommonTime())
            node.listener.getLatestCommonTime=lambda *_:stamp_type(clock.value)
            gate=threading.RLock();node.command_gate=lambda:gate
            node.execution_authority=lambda:(node.execution_route_version,)
            node.trajectory_execution_check=lambda info,points:execution_clearance(info,points,node.neupan_planner.robot,.1)
            base=node.vel_pub
            def emit(action):base.publish(NS(linear=NS(x=float(action[0,0])),angular=NS(z=float(action[1,0]))))
            def live():
                return dict(pose=np.zeros(3),scan=node.safety_scan,state_stamp_ns=round(clock.value*1.e9),
                    context=node.execution_authority(),now_ns=lambda:round(clock.value*1.e9))
            m=AcceptedExecution(node.neupan_planner.robot,.1,node.motion_policy,live,node.execution_authority,
                node.command_gate,emit,lambda _:None,clock=lambda:clock.value)
            m.start=lambda:None  # Deterministic fake clock; thread timing tested separately.
            node.execution=m;node.vel_pub=RevokingPublisher(base,m);self.addCleanup(m.close)
            original=node.neupan_planner.__class__.__call__
            def solve(planner,*args):
                action,info=original(planner,*args)
                info=candidate(.8)
                if planner.calls==2:
                    if behavior=='slow':
                        clock.value+=.6
                        node.safety_scan=(node.obstacle_points.copy(),round(clock.value*1.e9),clock.value)
                    elif behavior=='new_goal':node.execution_route_version+=1
                    elif behavior=='native_stop':action=np.zeros((2,1));info['stop']=True
                return action,info
            node.neupan_planner.__class__.__call__=solve
        return setup

    def test_actual_run_slow_second_solve_continues_native_sequence_without_zero(self):
        node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=self.integrated_setup('slow'))
        self.assertEqual(commands,[.8,.8])
        self.assertEqual(node.execution.active['origin']['cycle_id'],1)
        self.assertGreaterEqual(node.execution.last_result['consumed_steps'],5)
        self.assertNotIn('input',node.stop_registry.reasons)

    def test_actual_run_new_valid_solution_takes_over(self):
        node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=self.integrated_setup('normal'))
        self.assertEqual(commands,[.8,.8]);self.assertEqual(node.execution.active['origin']['cycle_id'],2)

    def test_actual_run_native_stop_cannot_keep_previous_execution(self):
        node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=self.integrated_setup('native_stop'))
        self.assertEqual(commands[0],.8);self.assertEqual(commands[-1],0.)
        self.assertIsNone(node.execution.active)

    def test_actual_run_goal_updated_during_solve_retires_previous_execution(self):
        node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=self.integrated_setup('new_goal'))
        self.assertEqual(commands[0],.8);self.assertTrue(all(value==0. for value in commands[1:]))
        self.assertIsNone(node.execution.active)

    def test_expired_result_qualified_execution_skips_zero_and_history_reset(self):
        events=[]
        def setup(node,clock,ns):
            node.motion_policy=MotionContinuity();node.motion_policy.record([.4,0.],clock.value)
            node.safety_scan=(node.obstacle_points.copy(),round(clock.value*1.e9),clock.value)
            node.continue_checked_execution=lambda reason:events.append(reason) or True
            stamp_type=type(node.listener.getLatestCommonTime())
            node.listener.getLatestCommonTime=lambda *_:stamp_type(clock.value)
        node,commands=fixture.CommandBoundaryTests().run_loop(delay=.6,delay_at='solver',newer_scan=True,setup_hook=setup)
        self.assertTrue(events);self.assertEqual(commands,[])
        self.assertIsNotNone(node.motion_policy.history)
        self.assertAlmostEqual(node.motion_policy.history[0][0],.4)
        self.assertNotIn('input',node.stop_registry.reasons)

    def test_candidate_rejection_continues_only_checked_old_sequence(self):
        events=[]
        def setup(node,clock,ns):
            node.motion_policy=MotionContinuity()
            node.obstacle_points=np.array([[.3],[0.]])
            node.safety_scan=(node.obstacle_points.copy(),round(clock.value*1.e9),clock.value)
            node.continue_checked_execution=lambda reason:events.append(reason) or True
        _,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=setup)
        self.assertTrue(events);self.assertEqual(commands,[])

    def test_real_outage_cannot_use_healthy_result_expiry_shortcut(self):
        def setup(node,clock,ns):
            node.motion_policy=MotionContinuity()
            node.continue_checked_execution=lambda _:self.fail('must not continue on actual outage')
        node,commands=fixture.CommandBoundaryTests().run_loop(delay=.6,delay_at='solver',setup_hook=setup)
        self.assertIn(0.,commands);self.assertIn('input',node.stop_registry.reasons)


class AuthoritySemanticsTests(unittest.TestCase):
    def node(self):
        source=Path(__file__).resolve().parents[2]/'neupan_ros/src/neupan_core.py'
        klass=next(item for item in ast.parse(source.read_text()).body if isinstance(item,ast.ClassDef))
        functions=[item for item in klass.body if isinstance(item,ast.FunctionDef)
                   and item.name in ('execution_authority','execution_mode_callback')]
        tree=ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Node',bases=[],keywords=[],
            body=functions,decorator_list=[])],type_ignores=[]))
        ns=dict(Twist=lambda:NS(linear=NS(x=0.),angular=NS(z=0.)))
        exec(compile(tree,str(source),'exec'),ns)
        node=ns['Node']()
        for key in ('shutdown_requested','planner_fault','waiting_for_new_goal','arrive',
                    'pending_route_update','pending_goal_request_stamp'):setattr(node,key,None)
        node.execution_control_mode=0;node.execution_route_version=1
        node.command_gate=lambda:threading.RLock();node.messages=[]
        node.vel_pub=NS(publish=node.messages.append)
        node.camera_guard_selected_state=dict(reference_role='advisory',source='camera')
        node.camera_guard=NS(status=dict(active=True,control_enabled=True,
            inference_diagnostics=dict(reference_role='advisory')),hazard_regions={},corridor_hold_key=None,
            pending_recovery=None,authorization_generation=1,safety_generation=0,original_key=('map',123),original_digest='a')
        return node

    def test_advisory_dropout_or_new_observation_is_not_a_new_driving_licence(self):
        node=self.node();before=node.execution_authority()
        node.camera_guard.authorization_generation+=1
        node.camera_guard.status['active']=False
        self.assertEqual(node.execution_authority(),before)

    def test_true_hazard_and_new_explicit_contract_disable_unqualified_continuation(self):
        node=self.node();node.camera_guard.hazard_regions={'wall':{}}
        self.assertIsNone(node.execution_authority())
        node.camera_guard.hazard_regions={}
        node.camera_guard.status['inference_diagnostics']['execution_contract']={'expires_ns':999}
        self.assertIsNone(node.execution_authority())

    def test_original_road_and_adopted_task_changes_have_different_versions(self):
        node=self.node();before=node.execution_authority()
        node.camera_guard.original_digest='new road';self.assertNotEqual(node.execution_authority(),before)
        node.camera_guard.original_digest='a';node.execution_route_version+=1
        self.assertNotEqual(node.execution_authority(),before)

    def test_manual_and_auto_edges_retire_previous_execution_before_new_solution(self):
        node=self.node();before=node.execution_route_version
        node.execution_mode_callback(NS(data=1))
        self.assertIsNone(node.execution_authority());self.assertEqual(len(node.messages),1)
        node.execution_mode_callback(NS(data=0))
        self.assertIsNotNone(node.execution_authority());self.assertEqual(len(node.messages),2)
        self.assertEqual(node.execution_route_version,before+2)

    def test_pending_task_shutdown_and_other_stops_block_continuation(self):
        for name in ('shutdown_requested','planner_fault','waiting_for_new_goal','arrive',
                     'pending_route_update','pending_goal_request_stamp'):
            node=self.node();setattr(node,name,True)
            self.assertIsNone(node.execution_authority())


class NativeLibraryDispatchTests(unittest.TestCase):
    def test_deployed_native_model_output_dispatches_without_another_solver(self):
        import torch
        from neupan import neupan
        from neupan_safety import validate_solution
        root=Path(__file__).resolve().parents[3]
        torch.set_num_threads(1)
        planner=neupan.init_from_yaml(str(root/'src/neupan_ros/example/lxw_car/config/neupan_config_outdoor.yaml'),
            pan=dict(dune_checkpoint=str(root/'src/neupan_ros/example/lxw_car/model/acker_robot_default_7/model_5000.pth')))
        planner.set_initial_path([np.array([[x],[0.],[0.],[1.]]) for x in np.arange(0.,20.,.5)])
        action,info=planner(np.zeros((3,1)),None)
        validate_solution(action,info,planner.robot,planner.dt)
        before=[state.copy() for state in info['opt_state_list']]
        clock=[100.];commands=[];gate=threading.RLock()
        def live():
            stamp=round(clock[0]*1.e9)
            return dict(pose=before[1].reshape(3),scan=(np.empty((2,0)),stamp,clock[0]),
                state_stamp_ns=stamp,context=1,now_ns=lambda:round(clock[0]*1.e9))
        manager=AcceptedExecution(planner.robot,planner.dt,MotionContinuity(),live,lambda:1,
            lambda:gate,lambda action:commands.append(action.copy()),lambda _:None,clock=lambda:clock[0])
        self.addCleanup(manager.close)
        manager.stage(info,1,dict(cycle_id=1),action=action)
        clock[0]+=.11
        self.assertTrue(manager.tick());self.assertEqual(len(commands),2)
        for a,b in zip(before,info['opt_state_list']):np.testing.assert_array_equal(a,b)
        self.assertEqual(manager.last_result['origin']['cycle_id'],1)


class RecordedInputOrderingTests(unittest.TestCase):
    def test_split_ten_is_ordered_by_recording_time_not_lexical_name(self):
        script=Path(__file__).resolve().parents[1]/'scripts/verify_stable_navigation.py'
        spec=importlib.util.spec_from_file_location('recorded_input_order_test',script)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        starts={'part_1.bag':10.,'part_2.bag':20.,'part_10.bag':30.}
        fake=NS(Bag=lambda file:nullcontext(NS(get_start_time=lambda:starts[Path(file).name])))
        with tempfile.TemporaryDirectory() as temporary:
            for name in starts:(Path(temporary)/name).touch()
            with patch.dict(sys.modules,{'rosbag':fake}):files=module.ordered_bag_files(temporary)
        self.assertEqual([file.name for file in files],['part_1.bag','part_2.bag','part_10.bag'])


if __name__=='__main__':unittest.main()
