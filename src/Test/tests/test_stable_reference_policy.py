"""Oct28 stable advisory + persistent checked motion: no vehicle or ROS master."""
import copy
import ast
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np
import test_followup_safety as fixture
import test_camera_publication_protocol as protocol
from test_camera_right_refinement import namespace as n
from test_startup_recovery import Stamp
from neupan_safety import (MotionContinuity,continuous_solution,validate_solution,ZeroAwarePublisher,
                          trajectory_clearance,CandidateRejected)
from osm_camera_path_guard import CameraPathGuard,path_digest


def road(gap=2.4):
    points=np.column_stack((np.arange(0.,100.5,.5),np.zeros(201)))
    geometry={i:dict(points_odom=np.array([[points[i,0],0.],[points[i,0],4.],[points[i,0],-gap]]),
        route_point_odom=points[i].tolist(),normal_odom=[0.,1.],capture_timestamp_ns=1000000000000) for i in range(4,17)}
    return points,geometry


class StableReferenceTests(unittest.TestCase):
    def test_160_frame_straight_sequences_keep_zero_or_one_correction(self):
        for gap,expected in ((2.4,0),(3.6,1)):
            with self.subTest(gap=gap):
                node,guard,clock,points,_=self.repair_node(gap)
                with self.repair_context(clock):
                    for frame in range(160):
                        clock[0]+=.2;node.last_seq=frame;station=frame*.16
                        current=getattr(node,'frozen_reference_map',points)
                        y=float(np.interp(station,points[:,0],current[:,1]))
                        node.cache_pose=lambda now,force=False:([station,y,0.],0.)
                        measured=gap+(.04*math.sin(frame/3.) if expected else 0.)
                        keys=np.flatnonzero((points[:,0]>=station+2.)&(points[:,0]<=station+8.))
                        node.cache_geometry={int(i):dict(points_odom=np.array([
                            [points[i,0],0.],[points[i,0],4.],[points[i,0],-measured]]),
                            route_point_odom=points[i].tolist(),normal_odom=[0.,1.],
                            capture_timestamp_ns=round(clock[0]*1.e9)) for i in keys}
                        node.cache.committed={int(i):(0.,clock[0]) for i in keys}
                        result=copy.deepcopy(node.path)
                        active,count=node.stable_result(result,clock[0],.2)
                        node.publish_result(result,clock[0],'straight sequence',active,count)
                        self.assertNotEqual(guard.select(clock[0],round(clock[0]*1.e9))[2]['source'],'stop')
                self.assertEqual(getattr(node,'reference_version',0),expected)

    def adopted_reference(self):
        node,guard,clock,points,geometry=self.repair_node(2.4)
        arc=points[:,0];u=np.clip(arc/8.,0.,1.);v=np.clip((arc[-1]-arc)/8.,0.,1.)
        adopted=points.copy();adopted[:,1]=-.6*u**3*(10-15*u+6*u*u)*v**3*(10-15*v+6*v*v)
        self.assertTrue(n['shape_report'](adopted,points,**node.vehicle_limits)['ok'])
        node.frozen_reference_map=adopted;node.reference_version=1
        node.reference_transition_end_odom=np.array([8.,0.])
        return node,guard,clock,points,geometry,adopted

    def test_heading_threshold_chatter_does_not_exit_or_modify_adopted_reference(self):
        node,_,clock,_,_,adopted=self.adopted_reference()
        with self.repair_context(clock):
            for frame in range(20):
                clock[0]+=.2;node.last_seq=frame
                yaw=math.radians(19.5 if frame%2 else 20.5)
                node.cache_pose=lambda now,force=False:([5.,float(adopted[10,1]),0.],yaw)
                node.osm_exit_requested=False
                node.stable_result(copy.deepcopy(node.path),clock[0],.2)
                np.testing.assert_array_equal(node.frozen_reference_map,adopted)
        self.assertEqual(node.reference_version,1);self.assertFalse(getattr(node,'reference_exit_frozen',False))

    def test_large_temporary_yaw_holds_reference_and_new_entry(self):
        node,_,clock,points,_=self.repair_node(3.6)
        node.cache_pose=lambda now,force=False:([0.,0.,0.],math.radians(60))
        with self.repair_context(clock):
            for frame in range(6):
                clock[0]+=.2;node.last_seq=frame;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        np.testing.assert_array_equal(node.frozen_reference_map,points)
        self.assertEqual(node.final_path_diagnostics['reason'],'hold_reference_heading_deviation')
        self.assertEqual(node.reference_outside_count,0)

    def test_junction_exit_is_confirmed_once_then_frozen_across_straight_frames(self):
        node,_,clock,_,_,adopted=self.adopted_reference()
        with self.repair_context(clock):
            for frame in range(3):
                clock[0]+=.2;node.osm_junctions=np.array([[4.,0.]])
                node.stable_result(copy.deepcopy(node.path),clock[0],.2)
                if frame<2:np.testing.assert_array_equal(node.frozen_reference_map,adopted)
            self.assertTrue(node.reference_exit_frozen);self.assertEqual(node.reference_version,2)
            exiting=node.frozen_reference_map.copy()
            for frame in range(20):
                clock[0]+=.2
                node.osm_junctions=np.array([[4.,0.]]) if frame%2 else np.empty((0,2))
                node.stable_result(copy.deepcopy(node.path),clock[0],.2)
                np.testing.assert_array_equal(node.frozen_reference_map,exiting)
            self.assertEqual(node.reference_version,2)

    def test_single_frame_junction_does_not_exit(self):
        node,_,clock,_,_,adopted=self.adopted_reference()
        with self.repair_context(clock):
            node.osm_junctions=np.array([[4.,0.]])
            node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            clock[0]+=.2;node.osm_junctions=np.empty((0,2))
            node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        np.testing.assert_array_equal(node.frozen_reference_map,adopted)
        self.assertEqual(node.reference_version,1)

    def test_exit_rearms_only_after_traversal_and_three_straight_ticks(self):
        node,_,clock,_,_,_=self.adopted_reference()
        with self.repair_context(clock):
            node.osm_junctions=np.array([[4.,0.]])
            for frame in range(3):
                clock[0]+=.2;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            end=node.reference_exit_end_s;self.assertTrue(node.reference_exit_frozen)
            node.osm_junctions=np.empty((0,2))
            node.cache_pose=lambda now,force=False:([end+1.,0.,0.],0.)
            # Avoid manufacturing an old-progress motion jump in this fixture.
            node.reference_progress_s=end
            for frame in range(3):
                clock[0]+=.2;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
                self.assertEqual(node.reference_exit_frozen,frame<2)

    def test_yaw_debounce_does_not_suppress_real_boundary_hazard(self):
        node,_,clock,_,_=self.repair_node(.3)
        node.cache_pose=lambda now,force=False:([0.,0.,0.],math.radians(30))
        node.current_proof_geometry={}
        with self.repair_context(clock):node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        self.assertTrue(node.hazards.regions);self.assertTrue(node.corridor_stop_required)

    def test_same_task_republish_preserves_exit_state(self):
        node,_,_,_,_=self.repair_node()
        msg=node.path
        node.route_signature=(msg.header.frame_id,msg.header.stamp.to_nsec(),
            tuple((p.pose.position.x,p.pose.position.y) for p in msg.poses))
        node.reference_exit_frozen=True;node.reference_exit_end_s=20.
        node.path_cb(copy.deepcopy(msg))
        self.assertTrue(node.reference_exit_frozen);self.assertEqual(node.reference_exit_end_s,20.)

    def test_true_route_bend_still_exits_even_when_vehicle_yaw_matches(self):
        node,_,clock,_,_=self.repair_node()
        points=np.array([[0.,0.],[2.,0.],[4.,0.],[4.,2.],[4.,4.]])
        node.path.poses=[NS(pose=NS(position=NS(x=float(x),y=float(y)))) for x,y in points]
        node.cache.committed.clear();node.cache_geometry={}
        with self.repair_context(clock):
            for frame in range(3):
                clock[0]+=.2;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        self.assertEqual(node.guidance_diagnostics['context'],'osm_turn')
        self.assertTrue(node.reference_exit_frozen)
        np.testing.assert_array_equal(node.frozen_reference_map,points)

    def test_completion_expiry_does_not_latch_unaccepted_exit(self):
        node,_,clock,_,_,adopted=self.adopted_reference()
        node.osm_junctions=np.array([[4.,0.]])
        with self.repair_context(clock):
            for frame in range(2):
                clock[0]+=.2;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            node.cache.committed={i:(0.,clock[0]-29.9) for i in node.cache_geometry}
            original=n['shape_report']
            def delayed(*args,**kwargs):
                report=original(*args,**kwargs);clock[0]+=.2;return report
            with patch.dict(n,shape_report=delayed):
                clock[0]+=.01;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        self.assertFalse(getattr(node,'reference_exit_frozen',False))
        np.testing.assert_array_equal(node.frozen_reference_map,adopted)

    def test_heading_only_context_never_masquerades_as_a_road_turn(self):
        points,_=road()
        for angle in (20.5,60.,179.):
            self.assertEqual(n['osm_guidance_context'](points,[0.,0.],math.radians(angle),np.empty((0,2))),
                'heading_deviation')

    def repair_node(self,gap=.76):
        node,guard,clock=protocol.RecoveryTests().node();points,geometry=road(gap)
        node.path.poses=[NS(pose=NS(position=NS(x=float(x),y=float(y),z=0.),
            orientation=NS(x=0.,y=0.,z=0.,w=1.))) for x,y in points]
        guard.original_cb(node.path)
        node.strategy='segment_reference';node.clearance_min=1.;node.clearance_max=3.;node.max_shift=1.5
        node.last_seq=0;node.osm_exit_requested=False;node.cache_geometry=geometry
        node.lock=n['threading'].RLock()
        node.cache.committed={i:(.24,clock[0]) for i in geometry}
        node.cache_pose=lambda now,force=False:([0.,0.,0.],0.)
        return node,guard,clock,points,geometry

    def repair_context(self,clock):
        return patch.dict(n,time=NS(monotonic=lambda:clock[0]),
            rospy=NS(Time=NS(now=lambda:Stamp(clock[0]))))

    def test_production_counterexample_repairs_with_unchanged_limits(self):
        node,_,_,points,geometry=self.repair_node()
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,
            [0.,0.],node.vehicle_limits)
        self.assertTrue(report['changed']);self.assertTrue(report['shape']['ok']);self.assertTrue(report['corridor']['ok'])
        self.assertEqual(report['transition_start_m'],0.)
        self.assertLessEqual(report['shape']['max_spatial_steer_rate'],.080001)
        self.assertLessEqual(report['shape']['max_steering_rad'],.350001)
        self.assertGreaterEqual(report['corridor']['min_right_m'],.8)
        np.testing.assert_array_equal(candidate[0],points[0]);np.testing.assert_array_equal(candidate[-1],points[-1])

    def test_repair_preserves_executed_prefix_at_nonzero_progress(self):
        points,geometry=road(.76)
        geometry={i+20:dict(item,points_odom=item['points_odom']+[10.,0.],route_point_odom=[points[i,0]+10.,0.])
                  for i,item in geometry.items()}
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,
            [10.,0.],dict(speed=1.5,wheelbase=.67,max_steer=.35,max_steer_rate=.08),progress=10.,heading=0.)
        self.assertTrue(report['changed']);self.assertGreaterEqual(report['transition_start_m'],10.)
        np.testing.assert_array_equal(candidate[:21],points[:21])

    def test_short_remaining_route_does_not_force_unsafe_repair(self):
        points,geometry=road(.76);points=points[:21]
        with self.assertRaises(n['CorridorUnsafe']):
            n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[0.,0.])

    def test_single_right_repair_still_requires_measured_area(self):
        points,geometry=road(.76)
        for item in geometry.values():item['mode']='right_only'
        with self.assertRaises(n['CorridorUnsafe']):
            n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[0.,0.])
        with patch.dict(n,observed_area_contains=Mock(return_value=True)):
            _,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[0.,0.])
            self.assertTrue(report['changed']);self.assertGreater(report['corridor']['area_checks'],0)

    def test_search_budget_keeps_real_danger_held(self):
        points,geometry=road(.76);clock=iter((0.,1.))
        with patch.dict(n,time=NS(monotonic=lambda:next(clock))),self.assertRaises(n['CorridorUnsafe']):
            n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[0.,0.])

    def test_held_danger_accepts_corrected_reference_but_requires_prediction_ack(self):
        node,guard,clock,points,geometry=self.repair_node()
        with self.repair_context(clock):
            for seq in range(1,4):
                clock[0]+=.1;node.last_seq=seq
                for item in geometry.values():item['capture_timestamp_ns']=round(clock[0]*1.e9)
                node.current_proof_geometry=copy.deepcopy(geometry)
                result=copy.deepcopy(node.path)
                active,count=node.stable_result(result,clock[0],.2)
                node.publish_result(result,clock[0],'test repair',active,count)
                state=guard.select(clock[0],round(clock[0]*1.e9))[2]
                if seq<3:self.assertEqual(state['source'],'stop')
            self.assertGreater(node.reference_version,0)
            self.assertTrue(node.hazards.regions);self.assertTrue(guard.hazard_regions)
            self.assertEqual(state['source'],'camera');self.assertTrue(state['recovery_pending'])
            self.assertFalse(guard.commit_recovery(guard.authorization_generation-1,guard.safety_generation,round(clock[0]*1.e9)))
            self.assertTrue(guard.commit_recovery(guard.authorization_generation,guard.safety_generation,round(clock[0]*1.e9)))
            released=guard.select(clock[0],round(clock[0]*1.e9))[2]
            node.hazard_ack_cb(NS(data=json.dumps(released)))
            self.assertFalse(node.hazards.regions)
            frozen=node.frozen_reference_map.copy();version=node.reference_version
            node.last_seq=4;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            np.testing.assert_array_equal(node.frozen_reference_map,frozen)
            self.assertEqual(node.reference_version,version);self.assertIsNone(node.execution_contract)

    def test_generated_reference_does_not_clear_unobserved_original_danger(self):
        node,_,clock,_,geometry=self.repair_node()
        node.hazards.remember(geometry,geometry)
        node.current_proof_geometry={}
        with self.repair_context(clock):
            for seq in range(1,4):
                node.last_seq=seq;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        self.assertGreater(node.reference_version,0)
        self.assertTrue(node.hazards.regions);self.assertTrue(node.corridor_stop_required)
        self.assertIsNone(node.execution_contract)

    def test_stale_spatial_proof_does_not_release_generated_repair(self):
        node,_,clock,_,geometry=self.repair_node()
        for item in geometry.values():item['capture_timestamp_ns']=round((clock[0]-2.)*1.e9)
        node.hazards.remember(geometry,geometry)
        node.current_proof_geometry=copy.deepcopy(geometry)
        with self.repair_context(clock):
            for seq in range(1,4):
                node.last_seq=seq;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        self.assertTrue(node.corridor_stop_required);self.assertFalse(node.hazard_proof)

    def test_exit_request_does_not_start_a_new_right_entry(self):
        node,_,clock,points,_=self.repair_node(3.6);node.osm_exit_requested=True
        with self.repair_context(clock):
            node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        np.testing.assert_array_equal(node.frozen_reference_map,points)
        self.assertEqual(getattr(node,'reference_version',0),0)

    def test_junction_keeps_osm_priority(self):
        node,_,clock,points,_=self.repair_node(3.6);node.osm_junctions=np.array([[4.,0.]])
        with self.repair_context(clock):node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        np.testing.assert_array_equal(node.frozen_reference_map,points)
        self.assertEqual(node.guidance_diagnostics['context'],'osm_junction')

    def test_in_band_keeps_exact_geometry_not_preference_for_two(self):
        for gap in (1.5,2.4,2.8):
            points,geometry=road(gap)
            candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[0.,.4,0.])
            np.testing.assert_array_equal(candidate,points);self.assertFalse(report['changed'])

    def test_outside_band_uses_spatial_transition_and_original_goal(self):
        points,geometry=road(3.6)
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[0.,0.])
        self.assertTrue(report['changed']);self.assertTrue(report['shape']['ok'])
        np.testing.assert_array_equal(candidate[:4],points[:4]);np.testing.assert_array_equal(candidate[-1],points[-1])
        self.assertLess(candidate[70,1],-.4)

    def test_missing_support_is_advisory_hold_not_new_danger(self):
        points,_=road();current=points.copy();current[20:80,1]=.2
        candidate,report=n['stable_segment_reference'](points,current,{},([0.,0.],0.),{},[0.,0.])
        np.testing.assert_array_equal(candidate,current);self.assertFalse(report['changed'])

    def test_actual_boundary_conflict_is_not_ignored(self):
        points,geometry=road(.3)
        with self.assertRaises(n['CorridorUnsafe']):
            n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[0.,0.])

    def test_advisory_gap_never_creates_observation_gap_hazards(self):
        node,_,_=protocol.RecoveryTests().node();node.strategy='segment_reference'
        node.last_verified_scope={1:dict(points_odom=np.zeros((3,2)))}
        node.remember_observation_gap('temporary observation support lost')
        self.assertFalse(node.hazards.regions);self.assertFalse(getattr(node,'recovery_unlocated_required',False))

    def test_real_existing_danger_survives_observation_gap(self):
        node,_,_=protocol.RecoveryTests().node();node.strategy='segment_reference'
        node.hazards.regions={'real':dict(kind='boundary')}
        node.remember_observation_gap('cache expired')
        self.assertIn('real',node.hazards.regions)

    def test_node_freezes_entry_and_does_not_reselect_each_frame(self):
        node,_,clock=protocol.RecoveryTests().node();points,geometry=road(3.6)
        node.path.poses=[NS(pose=NS(position=NS(x=float(x),y=float(y)))) for x,y in points]
        node.strategy='segment_reference';node.clearance_min=1.;node.clearance_max=3.;node.max_shift=1.5
        node.last_seq=1;node.osm_exit_requested=False;node.cache_geometry=geometry
        node.cache.committed={i:(-.6,clock[0]) for i in geometry}
        node.cache_pose=lambda now,force=False:([0.,0.,0.],0.)
        with patch.dict(n,time=NS(monotonic=lambda:clock[0])):
            for seq in range(1,4):
                node.last_seq=seq;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            frozen=node.frozen_reference_map.copy();version=node.reference_version
            self.assertGreater(version,0)
            for seq in range(4,12):
                node.last_seq=seq;clock[0]+=.2;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            np.testing.assert_array_equal(node.frozen_reference_map,frozen)
            self.assertEqual(node.reference_version,version);self.assertIsNone(node.execution_contract)
            for item in geometry.values():item['points_odom'][2,1]-=.7
            node.last_seq=13;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            self.assertNotEqual(node.final_path_diagnostics['reason'],'hold_spatial_entry_until_traversed')
            for item in geometry.values():item['points_odom'][2,1]+=.7
            clock[0]+=31.;node.stable_result(copy.deepcopy(node.path),clock[0],.2)
            np.testing.assert_array_equal(node.frozen_reference_map,frozen)
            self.assertFalse(node.cache.committed);self.assertFalse(node.hazards.regions)

    def test_search_crossing_cache_ttl_does_not_commit_new_geometry(self):
        node,_,clock=protocol.RecoveryTests().node();points,geometry=road(3.6)
        node.path.poses=[NS(pose=NS(position=NS(x=float(x),y=float(y)))) for x,y in points]
        node.strategy='segment_reference';node.clearance_min=1.;node.clearance_max=3.;node.max_shift=1.5
        node.last_seq=3;node.osm_exit_requested=False;node.cache_geometry=geometry;node.reference_outside_count=2
        node.cache.committed={i:(-.6,clock[0]-29.9) for i in geometry}
        node.cache_pose=lambda now,force=False:([0.,0.,0.],0.)
        original=n['stable_segment_reference']
        def delayed(*args):
            result=original(*args);clock[0]+=.2;return result
        with patch.dict(n,time=NS(monotonic=lambda:clock[0]),stable_segment_reference=delayed):
            node.stable_result(copy.deepcopy(node.path),1000.,.2)
        np.testing.assert_array_equal(node.frozen_reference_map,points)
        self.assertEqual(getattr(node,'reference_version',0),0)
        self.assertFalse(node.hazards.regions);self.assertIsNone(node.execution_contract)


class AdvisoryConsumerTests(unittest.TestCase):
    def guard(self):
        route=protocol.path();guard=CameraPathGuard();guard.original_cb(route);guard.candidate_cb(route)
        status=dict(control_enabled=True,active=True,route_stamp_ns=123,route_frame_id='map',
            control_path_digest=path_digest(route),corridor_stop_required=False,corridor_verified=True,
            inference_diagnostics=dict(reference_role='advisory',reference_version=1))
        guard.status_cb(status,1000.);return guard,status

    def test_active_advisory_does_not_need_half_second_execution_lease(self):
        guard,_=self.guard();state=guard.select(1000.,1000000000000)[2]
        self.assertEqual(state['source'],'camera');self.assertIsNone(state['execution_contract'])

    def test_advisory_heartbeat_loss_holds_reference_not_vehicle_stop(self):
        guard,_=self.guard();self.assertEqual(guard.select(1020.,1020000000000)[2]['source'],'camera')

    def test_real_hazard_remains_held_even_for_advisory_and_after_timeout(self):
        guard,status=self.guard();status.update(corridor_stop_required=True,
            hazard_regions={'real':dict(region_odom=[4.,0.],normal_odom=[0.,1.],capture_timestamp_ns=1)})
        guard.status_cb(status,1000.)
        self.assertEqual(guard.select(1020.,1020000000000)[2]['source'],'stop')
        status['corridor_stop_required']=False;guard.status_cb(status,1020.)
        self.assertEqual(guard.select(1020.,1020000000000)[2]['source'],'stop')

    def test_new_route_does_not_reuse_old_segment(self):
        guard,_=self.guard();route=copy.deepcopy(guard.original);route.header.stamp=NS(to_nsec=lambda:124)
        guard.original_cb(route);self.assertEqual(guard.select(1000.,1000000000000)[2]['source'],'osm')

    def test_legacy_visual_execution_still_requires_contract(self):
        guard,status=self.guard();status['inference_diagnostics']={};guard.status_cb(status,1000.)
        self.assertEqual(guard.select(1000.,1000000000000)[2]['source'],'stop')

    def test_explicit_recovery_contract_is_not_ignored_by_advisory_role(self):
        guard,status=self.guard()
        status['inference_diagnostics']['execution_contract']=protocol.contract(guard.candidate)
        guard.status_cb(status,1000.)
        self.assertIsNotNone(guard.select(1000.,1000000000000)[2]['execution_contract'])

    def test_actual_run_advisory_active_can_execute_without_visual_lease(self):
        guard,_=self.guard()
        def hook(node,clock,ns):
            node.camera_guard=guard
            def apply():
                state=guard.select(clock.value,round(clock.value*1.e9))[2]
                node.camera_guard_selected_state=state;node.camera_execution_contract=state['execution_contract']
                node.camera_safety_generation=state['safety_generation'];return state['source']!='stop'
            node.apply_camera_path_guard=apply
        node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=hook)
        self.assertTrue(all(v>0 for v in commands));self.assertIsNone(getattr(node,'planner_fault',None))

    def test_identical_reference_heartbeats_do_not_call_path_reset_again(self):
        guard,_=self.guard()
        def hook(node,clock,ns):
            source=fixture.ROOT/'src/neupan_ros/src/neupan_core.py'
            cls=next(x for x in ast.parse(source.read_text()).body if isinstance(x,ast.ClassDef))
            method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='apply_camera_path_guard')
            tree=ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[]))
            ns.update(copy=copy,String=lambda **kw:NS(**kw))
            exec(compile(tree,str(source),'exec'),ns)
            node.apply_camera_path_guard=ns['apply_camera_path_guard'].__get__(node)
            node.camera_guard=guard;node.camera_guard_signature=None;node.camera_guard_last_state=None;node.camera_guard_last_publish=0.
            node.camera_guard_pub=NS(publish=Mock());node.path_callback=Mock(return_value=True)
        node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=hook)
        self.assertTrue(all(v>0 for v in commands));self.assertEqual(node.path_callback.call_count,1)


class AlignmentTests(unittest.TestCase):
    def node(self):
        node,_,_=protocol.RecoveryTests().node();points,geometry=road()
        node.path.poses=[NS(pose=NS(position=NS(x=float(x),y=float(y)))) for x,y in points]
        node.cache_geometry=geometry;node.cache.committed={i:(0.,999.) for i in geometry}
        node.strategy='segment_reference';node.safe=True;node.safe_received=1000.;node.distance_reference='camera_center'
        node.max_shift=1.5;node.map_anchor=([0.,0.],0.);node.anchor_time=998.;node.anchor_sample_sec=998.
        node.pose_tick_open=True;node.pose_tick_snapshot=None
        state=NS(value=1000.,shift=.036,calls=0)
        def lookup(a,b,stamp):
            state.calls+=1
            return ((state.shift,0.,0.) if b=='odom' else (0.,0.,0.)),(0.,0.,0.,1.)
        node.listener=NS(getLatestCommonTime=lambda *_:Stamp(state.value),lookupTransform=lookup)
        node.cache_pose=n['CameraRightPath'].cache_pose.__get__(node)
        return node,state

    def context(self,state):return patch.dict(n,rospy=NS(Time=NS(now=lambda:Stamp(state.value))),
        tf=NS(Exception=RuntimeError,transformations=NS(euler_from_quaternion=lambda _:(0.,0.,0.))))

    def test_two_second_gap_small_map_correction_does_not_create_stop(self):
        node,state=self.node()
        with self.context(state):node.cache_pose(state.value)
        self.assertTrue(node.cache.committed);self.assertTrue(node.cache_alignment_diagnostics['smooth_classification'])
        self.assertFalse(node.hazards.regions)

    def test_one_tick_reuses_same_pose_not_artificial_one_millisecond_rate(self):
        node,state=self.node()
        with self.context(state):
            node.cache_pose(state.value);state.shift=.07;node.cache_pose(state.value)
        self.assertEqual(state.calls,2);self.assertAlmostEqual(node.map_anchor[0][0],.036)

    def test_real_large_jump_still_invalidates_cache_not_real_hazard_memory(self):
        node,state=self.node();state.shift=1.
        node.hazards.regions={'real':dict(kind='boundary')}
        with self.context(state),self.assertRaises(ValueError):node.cache_pose(state.value)
        self.assertFalse(node.cache.committed);self.assertIn('real',node.hazards.regions)


class MotionTests(unittest.TestCase):
    def profile(self):
        robot=NS(kinematics='acker',L=.67,max_speed=np.array([1.5,.35]),max_acce=np.array([.8,.08]))
        info=dict(stop=False,arrive=False,vel_tensor=np.vstack((np.ones(50)*.8,np.ones(50)*.3)),
                  opt_state_list=[np.zeros((3,1))])
        return robot,info

    def test_first_command_and_horizon_slew_then_reintegration(self):
        robot,info=self.profile();policy=MotionContinuity();policy.record([.2,-.1],10.)
        action,result=continuous_solution(np.array([[.8],[.3]]),info,robot,.1,policy,10.1)
        self.assertLessEqual(action[0,0]-.2,.080001);self.assertLessEqual(action[1,0]+.1,.008001)
        validate_solution(action,result,robot,.1)

    def test_speed_restriction_is_retained_then_released_gradually(self):
        policy=MotionContinuity();policy.restrict(.2);policy.record([.2,0.],10.)
        for i in (1,2,3):policy.accepted(10.+i*.1,.8,.8,clearance=.06);policy.record([.2,0.],10.+i*.1)
        self.assertEqual(policy.cap,.2);self.assertEqual(policy.good,0)
        for i in (1,2):policy.accepted(11.+i*.1,.8,.8,clearance=.2);policy.record([.2,0.],11.+i*.1)
        self.assertEqual(policy.cap,.2);policy.accepted(11.3,.8,.8,clearance=.2)
        self.assertGreater(policy.cap,.2)
        self.assertLess(policy.cap,.3)

    def test_every_zero_branch_updates_command_history(self):
        policy=MotionContinuity();policy.record([.8,.3],10.)
        published=[];pub=ZeroAwarePublisher(NS(publish=published.append),policy,lambda:10.1)
        pub.publish(NS(linear=NS(x=0.),angular=NS(z=0.)))
        np.testing.assert_array_equal(policy.last,[0.,0.]);self.assertEqual(policy.stamp,10.1)

    def test_collision_details_capture_first_conflict(self):
        vertices=np.array([[-.14,.81,.81,-.14],[-.425,-.425,.425,.425]])
        with self.assertRaises(CandidateRejected) as caught:
            trajectory_clearance(np.array([[0.,.1],[0.,0.],[0.,0.]]),np.array([[.9],[0.]]),vertices)
        self.assertIn('point_body',caught.exception.details);self.assertIn('step',caught.exception.details)

    def test_actual_run_slow_candidate_does_not_jump_back_to_point_eight(self):
        def hook(node,clock,ns):
            node.motion_policy=MotionContinuity();node.obstacle_points=np.array([[1.03],[0.]])
            node.safety_scan=(node.obstacle_points.copy(),1000000000000,1000.)
        node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=hook)
        self.assertTrue(all(0<v<=.4 for v in commands));self.assertLessEqual(node.neupan_planner.used_reference_speeds[-1],.4)

    def test_rejected_all_profiles_really_resolves_and_keeps_stop(self):
        def hook(node,clock,ns):
            node.motion_policy=MotionContinuity();node.obstacle_points=np.array([[.9],[0.]])
            node.safety_scan=(node.obstacle_points.copy(),1000000000000,1000.)
        node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=hook)
        self.assertTrue(all(v==0 for v in commands));self.assertGreater(node.neupan_planner.calls,2)
        self.assertIsNone(getattr(node,'planner_fault',None))

    def test_calculation_expiry_with_fresh_live_inputs_does_not_reset_input_window(self):
        def hook(node,clock,ns):node.motion_policy=MotionContinuity()
        node,_=fixture.CommandBoundaryTests().run_loop(delay=.2,newer_scan=True,setup_hook=hook)
        records=[json.loads(call.args[0]) for call in node.runtime_pub.publish.call_args_list]
        self.assertTrue(any(r.get('command_source')=='calculation_retry' for r in records))
        self.assertNotIn('input',node.stop_registry.reasons)

    def test_true_live_expiry_still_uses_input_recovery(self):
        def hook(node,clock,ns):node.motion_policy=MotionContinuity()
        node,_=fixture.CommandBoundaryTests().run_loop(delay=.2,setup_hook=hook)
        self.assertIn('input',node.stop_registry.reasons)


class EvidenceTests(unittest.TestCase):
    def test_bounded_evidence_is_not_a_fault_and_saves_numeric_inputs(self):
        from neupan_diagnostics import CandidateEvidenceSink
        with tempfile.TemporaryDirectory() as folder:
            sink=CandidateEvidenceSink(folder,limit=1,interval=0.)
            node=NS(diagnostic_safety_scan=(np.zeros((2,3)),123,1.),diagnostic_solver_input=(np.zeros((3,1)),None),
                    planner_session_id='test',diagnostic_state_stamp_ns=123,diagnostic_cycle_id=1,diagnostic_context_cycle_id=1)
            info=dict(opt_state_list=[np.zeros((3,1)),np.zeros((3,1))],vel_tensor=np.zeros((2,1)))
            for _ in range(3):sink.capture(node,CandidateRejected('test',{'step':1}),info)
            sink.queue.join();self.assertEqual(sink.count,1);self.assertEqual(sink.errors,0)
            self.assertEqual(len(list(Path(folder).glob('*.npz'))),1)
            data=json.loads(next(Path(folder).glob('*.json')).read_text())
            self.assertEqual(data['kind'],'candidate_rejection_not_solver_crash')


if __name__=='__main__':unittest.main()
