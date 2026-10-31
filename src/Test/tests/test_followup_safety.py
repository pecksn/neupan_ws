"""Oct26 follow-up: execute control loop with fake outputs; pure path regressions."""
import ast
from contextlib import ExitStack, contextmanager
import copy
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as NS, ModuleType
import unittest
from unittest.mock import Mock, patch
import numpy as np
from test_camera_right_refinement import namespace as n

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/neupan_ros/src'))
from neupan_safety import (checked_input_ages, validate_solution, trajectory_clearance,
    pose_snapshot, StopRegistry, InputUnavailable,ResultExpired,AuthorizationChanged,CandidateRejected,planner_obstacle_input,candidate_snapshot,reference_offsets)
sys.path.insert(0,str(ROOT/'src/osm_map_handler/src'))
from osm_camera_path_guard import CameraPathGuard, path_digest,check_execution_contract


class CommandBoundaryTests(unittest.TestCase):
    def run_loop(self, empty_error=False, delay=0., delay_at='check', newer_scan=False, disk_error=False,contract=None,recover=False,setup_hook=None):
        source=ROOT/'src/neupan_ros/src/neupan_core.py'
        original=next(x for x in ast.parse(source.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='neupan_core')
        body=[x for x in original.body if isinstance(x,ast.FunctionDef) and x.name in ('command_gate','control_mode_callback','run','generate_twist_msg','hold_planner_fault','temporary_input_stop','check_camera_execution','publish_stop_snapshot','cycle_metrics','candidate_stop','capture_candidate_rejection','stopping_shadow','refresh_camera_hazard_scope','continue_checked_execution','trajectory_execution_check','final_execution_input_check')]
        selected=ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(name='Run',bases=[],keywords=[],body=body,decorator_list=[])],type_ignores=[]))
        clock=NS(value=1000.,loops=0)
        class Stamp:
            def __init__(self,value):self.value=value
            def to_sec(self):return self.value
            def to_nsec(self):return round(self.value*1.e9)
            def __sub__(self,other):return Stamp(self.value-other.value)
        class Time:
            def __new__(cls,value=0,nsec=0):return Stamp(value+nsec/1.e9)
            @staticmethod
            def now():return Stamp(clock.value)
        class Twist:
            def __init__(self):self.linear=NS(x=0.);self.angular=NS(z=0.)
        def shutdown():
            clock.loops+=1
            return clock.loops>(9 if recover else 2)
        def sleep():
            if recover:
                clock.value+=.1
                node.safety_scan=(node.obstacle_points.copy(),round(clock.value*1.e9),clock.value)
        ros=NS(Time=Time,Rate=lambda _:NS(sleep=sleep),is_shutdown=shutdown,
            logerr=Mock(),logerr_throttle=Mock(),logwarn_throttle=Mock(),loginfo_throttle=Mock(),loginfo_once=Mock())
        def elapsed(stage):
            if stage==delay_at and (not recover or node.neupan_planner.calls==1):
                clock.value+=delay
                if newer_scan:node.safety_scan=(node.obstacle_points.copy(),round(clock.value*1.e9),clock.value)
        def checked(*args):
            result=trajectory_clearance(*args);elapsed('check');return result
        ns=dict(np=np,rospy=ros,time=NS(monotonic=lambda:clock.value),Twist=Twist,sys=sys,
            tf=NS(LookupException=LookupError,ConnectivityException=ConnectionError,ExtrapolationException=TimeoutError),
            validate_solution=lambda *a:(validate_solution(*a),elapsed('check'))[0],trajectory_clearance=checked,checked_input_ages=checked_input_ages,
            pose_snapshot=pose_snapshot,StopRegistry=StopRegistry,InputUnavailable=InputUnavailable,AuthorizationChanged=AuthorizationChanged,CandidateRejected=CandidateRejected,threading=threading,
            check_execution_contract=check_execution_contract,contextmanager=contextmanager,ExitStack=ExitStack,
            planner_obstacle_input=planner_obstacle_input,
            ResultExpired=ResultExpired,math=math,
            reference_offsets=reference_offsets,
            candidate_snapshot=candidate_snapshot,
            json=json,degrees=math.degrees,sin=math.sin,cos=math.cos,atan2=math.atan2,
            yellow_terminal_text=lambda x:x,red_terminal_text=lambda x:x)
        exec(compile(selected,str(source),'exec'),ns)
        node=ns['Run']();outputs=[]
        original_camera_check=node.check_camera_execution
        def camera_check(info,command):
            original_camera_check(info,command);elapsed('camera_check')
        node.check_camera_execution=camera_check
        node.vel_pub=NS(publish=lambda msg:outputs.append(msg.linear.x))
        for name in ('runtime_pub','ref_path_pub','plan_pub','ref_state_pub','point_markers_pub_dune','point_markers_pub_nrmp','robot_marker_pub'):
            setattr(node,name,NS(publish=Mock()))
        for name in ('apply_pending_goal_request','refresh_connection_scan','refresh_scan_snapshot','apply_pending_route','publish_scan_health','update_adaptive_adjust_parameters','update_goal_arrival'):
            setattr(node,name,Mock())
        node.apply_camera_path_guard=lambda:True;node.scan_is_healthy=lambda _:(True,'ok')
        def path(states):elapsed('path');return states
        node.generate_path_msg=path
        node.generate_dune_points_markers_msg=Mock();node.generate_nrmp_points_markers_msg=Mock();node.generate_robot_marker_msg=Mock()
        node.quat_to_yaw_list=lambda _:0.
        node.listener=NS(lookupTransform=lambda *_:((0.,0.,0.),(0.,0.,0.,1.)),getLatestCommonTime=lambda *_:Stamp(clock.value if recover else 1000.))
        node.command_lock=threading.RLock();node.control_mode=0;node.command_route_version=0
        node.route_update_lock=threading.RLock();node.pending_route_update=None;node.waiting_for_new_goal=False
        node.map_frame='map';node.base_frame='base_link';node.stop_on_scan_fault=True;node.scan_timeout=.5
        node.outdoor_tracking_guard=None;node.outdoor_tracking_status=None;node.start_connector=None;node.camera_guard=None
        node.verbose_runtime_log=False;node.last_command_monotonic=None
        node.camera_execution_contract=contract
        node.obstacle_points=np.array([[1.],[2.]])
        node.safety_scan=(node.obstacle_points.copy(),round(999.6*1.e9),999.6)
        node.bind_scan_snapshot=lambda:(node.safety_scan,node.obstacle_points,0)
        vertices=np.array([[-.14,.81,.81,-.14],[-.425,-.425,.425,.425]])
        class Planner:
            calls=0;waypoints=[np.array([[1.],[0.],[0.]])];initial_path=waypoints
            ref_speed=.8
            ipath=NS(point_index=0,interval=.5)
            robot=NS(max_speed=np.array([1.5,.35]),max_acce=np.array([.8,.08]),kinematics='acker',L=.67,vertices=vertices)
            dt=.1
            def set_reference_speed(self,speed):self.ref_speed=speed
            def __call__(self,*_):
                self.calls+=1
                if not hasattr(self,'used_reference_speeds'):self.used_reference_speeds=[]
                self.used_reference_speeds.append(self.ref_speed)
                if empty_error:raise ValueError()
                elapsed('solver')
                # A native solver stub; a legacy contract case supplies a
                # compliant native sequence, not ROS-side control retiming.
                active_contract=getattr(node,'camera_execution_contract',None)
                speed=active_contract['speed_cap_mps'] if active_contract else .8
                states=[np.array([[i*speed*.1],[0.],[0.]]) for i in range(3)]
                return np.array([[speed],[0.]]),dict(stop=False,arrive=False,opt_state_list=states,ref_state_list=copy.deepcopy(states),vel_tensor=np.array([[speed,speed],[0.,0.]]))
        node.neupan_planner=Planner()
        if setup_hook:setup_hook(node,clock,ns)
        diagnostic=ModuleType('neupan_diagnostics');diagnostic.report_failure=Mock(side_effect=OSError('disk full') if disk_error else None)
        with patch.dict(sys.modules,{'neupan_diagnostics':diagnostic}):node.run()
        return node,outputs

    def test_empty_exception_stays_latched_even_if_diagnostics_fail(self):
        for disk_error in (False,True):
            node,commands=self.run_loop(empty_error=True,disk_error=disk_error)
            self.assertEqual(node.neupan_planner.calls,1)
            self.assertEqual(node.planner_fault['type'],'ValueError')
            self.assertEqual(node.planner_fault['message'],'')
            self.assertTrue(commands);self.assertTrue(all(x==0 for x in commands))

    def test_delay_at_each_stage_cannot_publish_expired_action(self):
        for stage in ('solver','check','path','camera_check'):
            with self.subTest(stage=stage):
                node,commands=self.run_loop(delay=.2,delay_at=stage)
                self.assertTrue(commands);self.assertTrue(all(x==0 for x in commands))
                self.assertIn('input',node.stop_registry.reasons)
                self.assertIsNone(getattr(node,'planner_fault',None))

    def test_new_callback_does_not_renew_already_checked_snapshot(self):
        node,commands=self.run_loop(delay=.2,newer_scan=True)
        self.assertEqual(commands[0],0.) # First expired result never leaks.
        self.assertEqual(node.neupan_planner.calls,2)
        self.assertIn(.8,commands) # Next native solve uses the NEW snapshot.
        self.assertNotIn('input',node.stop_registry.reasons)

    def test_fresh_safe_command_still_passes(self):
        node,commands=self.run_loop()
        self.assertIn(.8,commands);self.assertIsNone(getattr(node,'planner_fault',None))

    def test_temporary_expiry_recovers_only_after_stable_new_inputs(self):
        node,commands=self.run_loop(delay=.2,recover=True)
        self.assertEqual(commands[0],0.)
        self.assertIn(.8,commands)
        self.assertNotIn('input',node.stop_registry.reasons)
        self.assertIsNone(getattr(node,'planner_fault',None))

    def test_real_run_applies_and_rechecks_the_hard_camera_speed_cap(self):
        contract=dict(version=1,path_digest='test',coordinate_frame='odom',speed_cap_mps=.3,expires_ns=1000500000000,
            end_odom=[8.,0.],direction_odom=[1.,0.],sections=[dict(route_point=[x,0.],normal=[0.,1.],right=[x,-2.],left=[x,2.]) for x in (1.,2.,3.,4.,5.,6.,7.,8.)])
        node,commands=self.run_loop(contract=contract)
        self.assertIn(.3,commands)
        self.assertTrue(all(abs(v)<=.3 for v in commands))
        self.assertIsNone(getattr(node,'planner_fault',None))
        self.assertEqual(node.neupan_planner.ref_speed,.8)
        self.assertTrue(all(v==.8 for v in node.neupan_planner.used_reference_speeds))

    def test_pose_age_and_clock_errors(self):
        for measurement,receive,state in ((999.9,999.9,999.),(1000.1,999.9,1000.),(999.9,1000.1,1000.)):
            with self.assertRaises(ValueError):checked_input_ages((None,round(measurement*1.e9),receive),10**12,1000.,.5,round(state*1.e9))


class NativeOutputTests(unittest.TestCase):
    """Native control stub only: verifies ROS integration, not obstacle safety."""
    def native_run(self,mode=None,replace_task=False,nonfinite=False,arrive=False,alternate=False):
        seen=[];returned=[]
        def hook(node,clock,ns):
            node.vel_pub=NS(publish=lambda msg:seen.append([float(msg.linear.x),float(msg.angular.z)]))
            node.control_mode=mode if mode is not None else 0
            node.motion_feedback=NS(snapshot=lambda *_:dict(average_speed_mps=-1.))
            base=type(node.neupan_planner)
            class Native(base):
                def __call__(self,*args):
                    action,info=super().__call__(*args)
                    steer=.35 if alternate and self.calls==1 else -.35
                    action[1,0]=steer;info['vel_tensor'][1,:]=steer
                    if nonfinite:action[0,0]=math.nan
                    if arrive:info['arrive']=True;action[:]=0.
                    if replace_task:node.command_route_version+=1
                    returned.append((action.copy(),info['vel_tensor'].copy()))
                    return action,info
            node.neupan_planner=Native()
            node.neupan_planner.cur_vel_array=np.full((2,2),.1234)
            node.neupan_planner.set_reference_speed=Mock(side_effect=AssertionError('ROS must not retarget native speed'))
        node,_=CommandBoundaryTests().run_loop(setup_hook=hook)
        return node,seen,returned

    def test_native_speed_and_steering_publish_without_postprocessing(self):
        node,commands,returned=self.native_run()
        np.testing.assert_allclose(commands,[[.8,-.35],[.8,-.35]])
        self.assertEqual(node.neupan_planner.calls,2)
        node.neupan_planner.set_reference_speed.assert_not_called()
        np.testing.assert_array_equal(node.neupan_planner.cur_vel_array,np.full((2,2),.1234))
        for action,controls in returned:np.testing.assert_array_equal(action,controls[:,:1])
        records=[json.loads(call.args[0]) for call in node.runtime_pub.publish.call_args_list]
        for row in records:
            if row.get('command_source')=='neupan_native':
                self.assertEqual(row['native_action'],[row['linear_x'],math.radians(row['steering_deg'])])

    def test_native_steering_reversal_is_not_delayed_by_ros(self):
        _,commands,_=self.native_run(alternate=True)
        np.testing.assert_allclose(commands,[[.8,.35],[.8,-.35]])

    def test_manual_mode_has_no_native_nonzero_commands(self):
        node,commands,_=self.native_run(mode=1)
        self.assertEqual(node.neupan_planner.calls,0)
        self.assertTrue(commands and all(value==[0.,0.] for value in commands))

    def test_route_replacement_during_solve_cancels_old_action(self):
        _,commands,_=self.native_run(replace_task=True)
        self.assertTrue(commands and all(value==[0.,0.] for value in commands))

    def test_mode_change_during_solve_cancels_old_action(self):
        def hook(node,clock,ns):
            base=type(node.neupan_planner)
            class Native(base):
                def __call__(self,*args):
                    result=super().__call__(*args)
                    node.control_mode=1;node.command_route_version+=1
                    return result
            node.neupan_planner=Native()
        node,commands=CommandBoundaryTests().run_loop(setup_hook=hook)
        self.assertEqual(node.neupan_planner.calls,1);self.assertTrue(all(v==0 for v in commands))

    def test_nonfinite_native_action_faults_and_holds_zero(self):
        node,commands,_=self.native_run(nonfinite=True)
        self.assertEqual(node.neupan_planner.calls,1);self.assertIsNotNone(node.planner_fault)
        self.assertTrue(commands and all(value==[0.,0.] for value in commands))

    def test_native_arrival_remains_zero(self):
        node,commands,_=self.native_run(arrive=True)
        self.assertTrue(all(value==[0.,0.] for value in commands));self.assertIsNone(getattr(node,'planner_fault',None))

    def test_removed_executor_and_control_rewriters_are_not_runtime_dependencies(self):
        core=(ROOT/'src/neupan_ros/src/neupan_core.py').read_text()
        for name in ('AcceptedExecution','continuous_solution','cap_solution_speed','GeometryObstacleAdapter','motion_policy'):
            self.assertNotIn(name,core)
        self.assertFalse((ROOT/'src/neupan_ros/src/neupan_execution.py').exists())


class PathBoundaryTests(unittest.TestCase):
    def route(self):return np.column_stack((np.arange(0,21,.5),np.zeros(42)))

    def geometry(self,points,keys,left=3.,right=-2.,target=.1):
        return {i:dict(points_odom=np.array([[points[i,0],target],[points[i,0],left],[points[i,0],right]])) for i in keys}

    def test_alignment_is_invariant_to_odom_origin(self):
        points=np.array([[4.,.5],[4.,2.],[4.,-1.5]])
        angle=.001;rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        values=[]
        for origin in (np.zeros(2),np.array([300.,-1000.])):
            values.append(n['alignment_impact'](points+origin,(-origin,0.),(-rotation@origin,angle)))
        self.assertAlmostEqual(*values,places=11)

    def test_cache_pose_retains_same_physical_correction_at_both_origins(self):
        class Stamp:
            def __init__(self,value):self.value=value
            def __sub__(self,other):return Stamp(self.value-other.value)
            def to_sec(self):return self.value
        class Time:
            def __new__(cls,value=0):return Stamp(value)
            @staticmethod
            def now():return Stamp(100.)
        angle=.001;outcomes=[]
        for distance,single in ((0.,False),(300.,False),(0.,True),(300.,True)):
            cls=n['CameraRightPath'];node=cls.__new__(cls)
            node.safe=True;node.safe_received=100.;node.control=True;node.distance_reference='camera_center'
            node.map_anchor=([-distance,0.],0.);node.anchor_time=99.8
            node.cache=n['SegmentCache']();node.cache.committed={8:(.5,99.)}
            fixed=np.array([[4.+distance,.5],[4.+distance,2.],[4.+distance,-1.5]])
            node.cache_geometry={8:dict(points_odom=fixed)}
            if single:
                fixed[1]=np.nan
                node.cache_geometry[8]['mode']='right_only'
            node.applied=np.ones(25)*.1;node.max_shift=1.5
            node.path=NS(poses=[NS(pose=NS(position=NS(x=x,y=0.))) for x in np.arange(25)*.5])
            anchor=([-distance*math.cos(angle),-distance*math.sin(angle)],angle)
            node.listener=NS(getLatestCommonTime=lambda *_:Stamp(100.),lookupTransform=lambda a,b,_:
                ((anchor[0][0],anchor[0][1],0.),(0.,0.,angle,1.)) if b=='odom' else ((0.,0.,0.),(0.,0.,angle,1.)))
            with patch.dict(n,rospy=NS(Time=Time),tf=NS(transformations=NS(euler_from_quaternion=lambda q:(0.,0.,q[2])))):
                node.cache_pose(100.)
            self.assertTrue(node.cache.committed)
            self.assertEqual(node.cache.committed[8][1],99.)
            outcomes.append(node.cache_alignment_diagnostics['translation_m'])
        self.assertAlmostEqual(outcomes[0],outcomes[1],places=11)
        self.assertAlmostEqual(outcomes[2],outcomes[3],places=11)

    def test_true_road_jump_is_not_hidden_by_origin_invariance(self):
        points=np.array([[4.,.5],[4.,2.],[4.,-1.5]])
        self.assertGreater(n['alignment_impact'](points,([0,0],0.),([2,0],0.)),.2)

    def test_short_or_single_point_cannot_form_sharp_bump(self):
        points=self.route()
        for keys in ([8],list(range(4,13))):
            values={i:-1. for i in keys}
            delivered,offsets,report=n['feasible_cached_path'](points,values,self.geometry(points,keys),([0,0],0),None,.2)
            self.assertTrue(report['shape']['ok'])
            self.assertLessEqual(float(abs(offsets).max()),.030001)
            if len(keys)==1:np.testing.assert_array_equal(delivered,points)

    def test_long_supported_transition_is_feasible_and_rate_limited(self):
        points=self.route();keys=list(range(2,39));values={i:.1 for i in keys}
        previous=points.copy();offsets=None
        for _ in range(10):
            delivered,offsets,report=n['feasible_cached_path'](points,values,self.geometry(points,keys),([0,0],0),offsets,.2,previous_path=previous)
            self.assertTrue(report['shape']['ok']);self.assertTrue(report['corridor']['ok'])
            self.assertLessEqual(float(np.linalg.norm(delivered-previous,axis=1).max()),.030001)
            previous=delivered
        self.assertGreater(float(abs(offsets).max()),.05)

    def test_unsafe_original_cannot_be_used_as_silent_fallback(self):
        points=self.route();keys=list(range(4,13));values={i:1.45 for i in keys}
        with self.assertRaises(n['CorridorUnsafe']):
            n['feasible_cached_path'](points,values,self.geometry(points,keys,right=-.55,target=1.45),([0,0],0),None,.2)

    def test_corridor_checks_actual_blended_path_and_interiors(self):
        points=self.route();keys=[4,5];geometry=self.geometry(points,keys,right=-.81)
        report=n['corridor_report'](points,geometry,([0,0],0),keys)
        self.assertFalse(report['ok']);self.assertGreater(report['checked_points'],len(keys))
        corrected=points.copy();corrected[:,1]=.1
        self.assertTrue(n['corridor_report'](corrected,geometry,([0,0],0),keys)['ok'])

    def test_same_fixed_corridor_is_rechecked_after_alignment(self):
        points=self.route();keys=list(range(4,13));geometry=self.geometry(points,keys,right=-1.)
        self.assertTrue(n['corridor_report'](points,geometry,([0,0],0),keys)['ok'])
        self.assertFalse(n['corridor_report'](points,geometry,([0,.4],0),keys)['ok'])

    def test_smoothing_does_not_extrapolate_across_unsupported_gaps(self):
        points=self.route();keys=list(range(2,15))+list(range(25,39))
        offsets=n['smooth_supported_offsets'](points,{i:.1 for i in keys})
        np.testing.assert_array_equal(offsets[15:25],0.)

    def test_unsupported_expired_curve_cannot_jump_back_in_one_tick(self):
        points=self.route();previous=points.copy();previous[8,1]=.5
        with self.assertRaises(n['CorridorUnsafe']):
            n['feasible_cached_path'](points,{}, {},([0,0],0),None,.2,previous_path=previous)

    def test_guard_holds_stop_until_verified_matching_release(self):
        def path(stamp):
            return NS(header=NS(frame_id='map',stamp=NS(to_nsec=lambda:stamp)),poses=[NS(pose=NS(position=NS(x=x,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))) for x in (0.,1.,2.)])
        guard=CameraPathGuard();route=path(123);guard.original_cb(route);guard.candidate_cb(route)
        status=dict(control_enabled=True,route_stamp_ns=123,route_frame_id='map',corridor_stop_required=True)
        guard.status_cb(status,10.)
        self.assertEqual(guard.select(10.)[2]['source'],'stop')
        self.assertEqual(guard.select(20.)[2]['source'],'stop')
        guard.status_cb(dict(status,route_stamp_ns=999,corridor_stop_required=False,corridor_verified=True,control_path_digest=path_digest(route)),20.)
        self.assertEqual(guard.select(20.)[2]['source'],'stop')
        guard.status_cb(dict(status,corridor_stop_required=False,corridor_verified=True,control_path_digest='wrong'),20.)
        self.assertEqual(guard.select(20.)[2]['source'],'stop')
        guard.status_cb(dict(status,corridor_stop_required=False,corridor_verified=True,control_path_digest=path_digest(route)),20.)
        self.assertEqual(guard.select(20.)[2]['source'],'stop')
        guard.status_cb(status,21.);guard.select(21.)
        guard.original_cb(path(124))
        self.assertEqual(guard.select(21.)[2]['source'],'stop')

    def test_rejected_published_reference_does_not_replace_temporal_baseline(self):
        source=ROOT/'src/osm_map_handler/src/osm_camera_path_refinement.py'
        cls=next(x for x in ast.parse(source.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='CameraRightPath')
        method=copy.deepcopy(next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='publish_result'))
        # Execute the unchanged safety/publication prefix, before unrelated
        # full diagnostic assembly (no ROS master, no output to a real topic).
        cutoff=next(i for i,x in enumerate(method.body) if isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='capture' for t in x.targets))
        method.body=method.body[:cutoff]
        namespace=dict(n)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(source),'exec'),namespace)
        def path(y):
            return NS(header=NS(stamp=NS(to_nsec=lambda:123),frame_id='map'),poses=[NS(pose=NS(position=NS(x=i*.5,y=dy))) for i,dy in enumerate(y)])
        original=path([0.]*8);candidate=path([0.,0.,0.,.5,0.,0.,0.,0.])
        baseline=np.array([[i*.5,0.] for i in range(8)])
        node=NS(packet=None,path=original,delivered_points=baseline.copy(),delivery_dt=.2,
                corridor_stop_required=False,corridor_verified=True,control=True,status=NS(publish=Mock()))
        node.remember_observation_gap=n['CameraRightPath'].remember_observation_gap.__get__(node)
        node.processing_stage=Mock()
        namespace['publish_result'](node,candidate,10.,'test',True,1)
        self.assertTrue(node.corridor_stop_required)
        np.testing.assert_array_equal(node.delivered_points,baseline)
        self.assertTrue(json.loads(node.status.publish.call_args[0][0].data)['corridor_stop_required'])


if __name__=='__main__':unittest.main()
