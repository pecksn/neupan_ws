"""Oct27 policy regressions: actual source helpers, fake clocks/outputs only."""
import copy
import json
import math
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock,patch
import unittest
import numpy as np
import cv2
from test_camera_right_refinement import namespace as n, workspace
sys.path.insert(0,str(workspace/'src/neupan_ros/src'))
sys.path.insert(0,str(workspace/'src/osm_map_handler/src'))
from neupan_safety import (pose_snapshot, checked_input_ages, InputUnavailable, StopRegistry,
    cap_solution_speed, validate_solution,trajectory_clearance)
from osm_camera_path_guard import CameraPathGuard, path_digest, check_execution_contract


class EpochAndStops(unittest.TestCase):
    def stamp(self,value):return NS(to_nsec=lambda:round(value*1.e9))
    def test_transform_is_queried_at_the_selected_epoch_even_if_latest_changes(self):
        selected=self.stamp(999.4);latest=[selected]
        def lookup(target,source,stamp):
            self.assertIs(stamp,selected);latest[0]=self.stamp(999.8)
            return (1.,2.,0.),(0.,0.,0.,1.)
        listener=NS(getLatestCommonTime=Mock(side_effect=lambda *_:latest[0]),lookupTransform=lookup)
        _,_,epoch=pose_snapshot(listener,'map','base_link')
        self.assertEqual(epoch,999400000000)
        listener.getLatestCommonTime.assert_called_once()
        with self.assertRaises(InputUnavailable):checked_input_ages((None,999900000000,999.9),10**12,1000.,.5,epoch)
    def test_missing_history_does_not_fall_back_to_latest(self):
        listener=NS(getLatestCommonTime=lambda *_:self.stamp(999.9),lookupTransform=Mock(side_effect=LookupError('history missing')))
        with self.assertRaises(LookupError):pose_snapshot(listener,'map','base_link')
        listener.lookupTransform.assert_called_once()
    def test_zero_epoch_rejected(self):
        listener=NS(getLatestCommonTime=lambda *_:self.stamp(0.),lookupTransform=Mock())
        with self.assertRaises(InputUnavailable):pose_snapshot(listener,'map','base_link')
        listener.lookupTransform.assert_not_called()
    def test_one_recovery_cannot_clear_another_reason(self):
        registry=StopRegistry()
        registry.set('input',1.,'fresh stable input')
        registry.set('camera',1.,'original danger rechecked')
        registry.set('solver',1.,'inspect/restart',True)
        registry.clear('input');registry.clear('solver')
        self.assertEqual(set(registry.snapshot()),{'camera','solver'})
    def test_first_trigger_time_survives_repeat(self):
        registry=StopRegistry();registry.set('input',1.,'fresh');registry.set('input',4.,'fresh')
        self.assertEqual(registry.snapshot()['input']['first'],1.)
        self.assertEqual(registry.snapshot()['input']['last'],4.)


class PlanningPolicy(unittest.TestCase):
    def scene(self,right=-2.,left=2.,span=(4,29)):
        points=np.column_stack((np.arange(0,30.5,.5),np.zeros(61)))
        keys=list(range(*span))
        geometry={i:dict(points_odom=np.array([[points[i,0],right+2.],[points[i,0],left],[points[i,0],right]]),capture_timestamp_ns=100) for i in keys}
        return points,keys,geometry
    def solve(self,points,keys,geometry,**kw):
        return n['feasible_cached_path'](points,{i:0. for i in keys},geometry,([0.,0.],0.),None,.2,
            limits=dict(speed=1.5,wheelbase=.67,max_steer=.35,max_steer_rate=.08),
            clearance_range=(1.,2.),allow_speed_contract=True,**kw)
    def test_longer_support_reduces_slew_without_four_metre_ceiling(self):
        rates=[]
        for length in (8.,20.,40.):
            points=np.column_stack((np.arange(0,length+.25,.5),np.zeros(int(length/.5)+1)))
            values={i:1. for i in range(len(points))}
            offsets=n['smooth_supported_offsets'](points,values)
            rates.append(n['shape_report'](points+np.column_stack((np.zeros(len(points)),offsets)),points,speed=1.5)['max_spatial_steer_rate'])
        self.assertGreater(rates[0],rates[1]);self.assertGreater(rates[1],rates[2])
        self.assertLess(rates[-1],.08)
    def test_preferred_two_metres_with_real_lateral_offset(self):
        points,keys,geometry=self.scene(right=-3.)
        path,offsets,report=self.solve(points,keys,geometry)
        self.assertTrue(report['shape']['ok']);self.assertTrue(report['corridor']['ok'])
        self.assertEqual(report['selected_clearance_m'],2.)
        self.assertGreater(abs(offsets).max(),.009)
        self.assertLessEqual(report['speed_cap_mps'],1.5)
    def test_narrow_road_selects_one_point_five_instead_of_rejecting_two(self):
        points,keys,geometry=self.scene(right=-1.5,left=.9)
        _,offsets,report=self.solve(points,keys,geometry)
        self.assertGreaterEqual(report['selected_clearance_m'],1.5)
        self.assertLessEqual(report['selected_clearance_m'],1.575)
        self.assertTrue(report['target_plan_feasible'])
        self.assertTrue(report['corridor']['ok'])
    def test_camera_offset_is_applied_to_range_search(self):
        points,keys,geometry=self.scene(right=-1.4,left=.9)
        _,_,report=self.solve(points,keys,geometry,camera_y=.1)
        self.assertLessEqual(report['selected_clearance_m'],1.6)
        self.assertGreaterEqual(report['selected_clearance_m'],1.)
    def test_small_step_is_checked_even_if_full_target_infeasible(self):
        points,keys,geometry=self.scene(right=-3.,span=(4,13))
        _,offsets,report=self.solve(points,keys,geometry)
        self.assertGreaterEqual(abs(offsets).max(),.01)
        self.assertLessEqual(abs(offsets).max(),.030001)
        self.assertTrue(report['shape']['ok']);self.assertTrue(report['corridor']['ok'])
        self.assertIn('target_plan_feasible',report)
    def test_impossible_original_and_transition_stop(self):
        points,keys,geometry=self.scene(right=-.2,left=.5,span=(4,13))
        with self.assertRaises(n['CorridorUnsafe']):self.solve(points,keys,geometry)
    def test_no_millimetre_success_for_single_point(self):
        points,keys,geometry=self.scene(right=-3.,span=(8,9))
        _,offsets,report=self.solve(points,keys,geometry)
        self.assertFalse(offsets.any());self.assertFalse(report['target_plan_feasible'])
    def test_noisy_feasible_choice_has_hysteresis(self):
        points,keys,geometry=self.scene(right=-1.5,left=.92)
        _,_,report=self.solve(points,keys,geometry,selected=1.5)
        self.assertAlmostEqual(report['selected_clearance_m'],1.5)
    def test_reference_converges_over_multiple_checked_cycles(self):
        points,keys,geometry=self.scene(right=-2.3)
        previous=points.copy();offsets=None
        for _ in range(25):
            path,offsets,report=n['feasible_cached_path'](points,{i:-.3 for i in keys},geometry,([0.,0.],0.),offsets,.2,
                previous_path=previous,limits=dict(speed=1.5,wheelbase=.67,max_steer=.35,max_steer_rate=.08),
                clearance_range=(1.,2.),allow_speed_contract=True)
            self.assertTrue(report['shape']['ok']);self.assertTrue(report['corridor']['ok'])
            self.assertLessEqual(float(np.linalg.norm(path-previous,axis=1).max()),.030001)
            previous=path
        self.assertGreater(abs(offsets[keys[-1]]),.28)
    def test_far_unexecuted_osm_turn_does_not_reject_verified_local_prefix(self):
        original=np.vstack((np.column_stack((np.arange(0,25.5,.5),np.zeros(51))),
                            np.column_stack((np.full(20,25.),np.arange(.5,10.5,.5)))))
        path=original.copy();u=np.clip((path[:,0]-2.)/8.,0,1)
        path[:,1]+=.1*u**3*(10-15*u+6*u*u)
        self.assertFalse(n['shape_report'](path,original,speed=1.5)['ok'])
        self.assertTrue(n['shape_report'](path,original,speed=1.5,end_index=21)['ok'])
    def test_osm_exit_has_bounded_lateral_changes_and_preserves_goal(self):
        points,keys,geometry=self.scene(right=-3.,left=3.,span=(4,41))
        offsets=n['smooth_supported_offsets'](points,{i:.5 for i in keys})
        previous=points+np.column_stack((np.zeros(len(points)),offsets))
        for _ in range(25):
            path,offsets,report=n['feasible_osm_exit'](points,previous,geometry,([0.,0.],0.),keys,.2,
                dict(speed=.3,wheelbase=.67,max_steer=.35,max_steer_rate=.08,start_index=0,end_index=42))
            self.assertLessEqual(float(np.linalg.norm(path-previous,axis=1).max()),.030001)
            self.assertTrue(report['shape']['ok']);self.assertTrue(report['corridor']['ok'])
            np.testing.assert_array_equal(path[-1],points[-1]);previous=path
        self.assertLess(abs(offsets).max(),.001)
    def test_rate_lower_bound_rejects_impossible_cycle_without_full_search(self):
        points,keys,geometry=self.scene(right=-.55,left=3.2,span=(4,13))
        with patch.dict(n,smooth_supported_offsets=Mock(side_effect=AssertionError('unnecessary search'))):
            with self.assertRaises(n['CorridorUnsafe']):self.solve(points,keys,geometry)
    def test_single_side_requires_actual_area_and_full_footprint(self):
        points,keys,geometry=self.scene(span=(6,15))
        r=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
        matrix=np.array([[200.,0.,320.],[0.,200.,240.],[0.,0.,1.]])
        origin=np.array([0.,0.,1.])
        polygon_ground=np.array([[1.,-3.,0.],[9.,-3.,0.],[9.,3.,0.],[1.,3.,0.]])
        polygon=cv2.projectPoints((polygon_ground-origin)@r,np.zeros(3),np.zeros(3),matrix,np.zeros(5))[0].reshape(-1,2)
        area=dict(origin=origin.tolist(),rotation=r.tolist(),matrix=matrix.tolist(),distortion=[0.]*5,
            polygon=polygon.tolist(),footprint=[[-.14,-.425],[.81,-.425],[.81,.425],[-.14,.425]])
        for v in geometry.values():
            v['points_odom'][1]=np.nan;v.update(mode='right_only',normal_odom=[0.,1.])
        report=n['corridor_report'](points,geometry,([0.,0.],0.),keys)
        self.assertFalse(report['ok'])
        for v in geometry.values():v['area']=area
        self.assertTrue(n['corridor_report'](points,geometry,([0.,0.],0.),keys)['ok'])
        for i in (keys[0],keys[-1]):
            geometry[i]['points_odom'][1]=[points[i,0],2.]
            geometry[i]['mode']='dual'
        self.assertTrue(n['corridor_report'](points,geometry,([0.,0.],0.),keys)['ok'])
        for v in geometry.values():v['area']=dict(area,polygon=[[318,260],[322,260],[322,440],[318,440]])
        self.assertFalse(n['corridor_report'](points,geometry,([0.,0.],0.),keys)['ok'])

    def test_short_left_support_preserves_real_constraints_without_vetoing_right(self):
        left=np.array([[2.,2.],[2.5,2.],[3.,2.]])
        right=np.array([[1.,-2.],[4.,-2.],[8.,-2.]])
        x,ly,ry=n['boundary_samples'](left,right,1.)
        self.assertEqual((x[0],x[-1]),(2.,6.))
        np.testing.assert_allclose(ly[x<=3.],2.)
        self.assertTrue(np.isnan(ly[x>3.]).all())
        np.testing.assert_allclose(ry,-2.)
    def test_disjoint_left_is_unknown_not_extrapolated_into_right_support(self):
        left=np.array([[6.5,2.],[7.,2.],[8.,2.]])
        right=np.array([[1.,-2.],[4.,-2.],[8.,-2.]])
        _,ly,_=n['boundary_samples'](left,right,1.)
        self.assertTrue(np.isnan(ly).all())
    def test_short_but_known_narrow_left_cannot_be_discarded(self):
        left=np.array([[2.,-.8],[2.5,-.8],[3.,-.8]])
        right=np.array([[1.,-2.],[4.,-2.],[8.,-2.]])
        with self.assertRaisesRegex(ValueError,'narrow'):n['boundary_samples'](left,right,1.)

    def test_batched_area_test_agrees_with_opencv_including_duplicate_vertices(self):
        rng=np.random.RandomState(17)
        for polygon in ([[0,0],[10,0],[10,10],[0,10],[0,0]],[[0,0],[10,0],[10,3],[3,3],[3,10],[0,10]]):
            contour=np.asarray(polygon,np.float32).reshape(-1,1,2)
            for point in rng.uniform(-2,12,(200,2)):
                expected=cv2.pointPolygonTest(contour,tuple(map(float,point)),False)>=0
                self.assertEqual(n['polygon_contains_points']([point],polygon),expected)
            self.assertFalse(n['polygon_contains_points']([[1.,1.],[20.,20.]],polygon))
    def test_failed_area_short_circuits_but_success_checks_the_whole_sweep(self):
        points,keys,geometry=self.scene(span=(6,15))
        for evidence in geometry.values():evidence.update(mode='right_only',normal_odom=[0.,1.])
        for result in (False,True):
            check=Mock(return_value=result)
            with patch.dict(n,observed_area_contains=check):
                report=n['corridor_report'](points,geometry,([0.,0.],0.),keys)
            self.assertEqual(report['ok'],result)
            if result:self.assertGreater(check.call_count,len(keys))
            else:self.assertEqual(check.call_count,1)
    def test_folded_distortion_cannot_authorize_unseen_ground(self):
        rotation=[[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]]
        area=dict(origin=[0.,0.,1.],rotation=rotation,matrix=[[200.,0.,320.],[0.,200.,240.],[0.,0.,1.]],
            distortion=[-.2,0.,0.,0.,0.],polygon=[[0,0],[640,0],[640,480],[0,480]],
            footprint=[[-.14,-.425],[.81,-.425],[.81,.425],[-.14,.425]])
        path=np.array([[.6,-2.],[1.1,-2.],[1.6,-2.]])
        self.assertFalse(n['observed_area_contains'](path[1],path,1,{'area':area},([0.,0.],0.)))


class HazardPolicy(unittest.TestCase):
    def evidence(self,x,stamp):return dict(points_odom=np.array([[x,0.],[x,2.],[x,-2.]]),capture_timestamp_ns=stamp)
    def test_capacity_never_evicts_or_auto_releases_unresolved_danger(self):
        memory=n['HazardMemory'](max_regions=2)
        data={i:self.evidence(float(i),100) for i in (4,5,6)}
        memory.remember(data,data)
        self.assertTrue(memory.blocked);self.assertEqual(len(memory.regions),2)
        old=copy.deepcopy(memory.regions)
        memory.remember({7:self.evidence(7.,200)},[7])
        self.assertEqual(memory.regions,old)
        fresh={i:self.evidence(float(i),200) for i in (4,5,6)}
        self.assertFalse(memory.verification(fresh,fresh,200,500,{'ok':True}))
    def test_other_point_cache_rebuild_or_stale_evidence_cannot_clear_danger(self):
        memory=n['HazardMemory']();memory.remember({4:self.evidence(4.,100)},[4])
        for geometry,now in (({6:self.evidence(6.,200)},200),({4:self.evidence(4.,100)},200),({4:self.evidence(4.,200)},2000)):
            self.assertFalse(memory.verification(geometry,list(geometry),now,500,{'ok':True}))
        self.assertTrue(memory.regions)
    def test_full_fresh_recheck_is_required_for_every_region(self):
        memory=n['HazardMemory']();geometry={4:self.evidence(4.,100),5:self.evidence(4.5,100)}
        memory.remember(geometry,geometry)
        first=memory.verification({4:self.evidence(4.,200)},[4],200,500,{'ok':True})
        self.assertLess(len(first),len(memory.regions))
        geometry={4:self.evidence(4.,200),5:self.evidence(4.5,200)}
        self.assertEqual(len(memory.verification(geometry,geometry,200,500,{'ok':True})),len(memory.regions))
    def test_recheck_binds_road_region_not_a_persistent_detected_edge(self):
        memory=n['HazardMemory']()
        old=self.evidence(4.,100);old['points_odom'][2,1]=-.55
        old['route_point_odom']=[4.,0.]
        memory.remember({8:old},[8])
        corrected=self.evidence(4.,200);corrected['route_point_odom']=[4.,0.]
        # The old false edge disappeared. A fresh safe measurement of the
        # ORIGINAL road section must not remain locked to that old edge pixel.
        self.assertTrue(memory.verification({8:corrected},[8],200,500,{'ok':True}))
        other=self.evidence(6.,200);other['route_point_odom']=[6.,0.]
        self.assertFalse(memory.verification({12:other},[12],200,500,{'ok':True}))

    def test_actual_node_keeps_danger_until_spatial_recheck_and_consumer_ack(self):
        cls=n['CameraRightPath'];node=cls.__new__(cls)
        node.cache=n['SegmentCache']();node.cache.committed={12:(0.,1.)}
        node.cache_geometry={12:self.evidence(6.,200000000)}
        node.cache_pose=lambda now:([0.,0.,0.],0.)
        node.packet=None;node.received=0.;node.timeout=.8;node.max_capture_age_ms=800
        node.applied=None;node.map_anchor=([0.,0.],0.);node.osm_junctions=np.zeros((0,2));node.origin=np.zeros(3)
        node.hazards=n['HazardMemory']();node.hazards.remember({8:self.evidence(4.,100000000)},[8])
        path=NS(header=NS(stamp=NS(to_nsec=lambda:123)),poses=[NS(pose=NS(position=NS(x=x,y=0.))) for x in np.arange(0.,20.,.5)])
        node.path=path
        with patch.dict(n,rospy=NS(Time=NS(now=lambda:NS(to_nsec=lambda:200000000)))):
            with patch.dict(n,feasible_cached_path=Mock(side_effect=AssertionError('unseen danger cannot be searched away'))):
                node.cached_result(copy.deepcopy(path),1.,.2)
            self.assertTrue(node.corridor_stop_required)
            node.cache.committed={8:(0.,1.),9:(0.,1.)}
            node.cache_geometry={8:self.evidence(4.,200000000),9:self.evidence(4.5,200000000)}
            node.cached_result(copy.deepcopy(path),1.,.2)
            self.assertTrue(node.corridor_stop_required) # Raw refresh alone is not confirmed proof.
            node.current_proof_geometry=dict(node.cache_geometry)
            node.cached_result(copy.deepcopy(path),1.,.2)
            self.assertFalse(node.corridor_stop_required)
            self.assertTrue(node.hazards.regions) # Until the consumer confirms receipt.
        node.lock=__import__('threading').RLock();node.last_hazard_verified_digest='safe'
        node.hazard_ack_cb(NS(data=json.dumps(dict(source='camera',matched_path_digest='wrong',released_hazard_ids=list(node.hazards.regions)))))
        self.assertTrue(node.hazards.regions)
        node.corridor_stop_required=True
        node.hazard_ack_cb(NS(data=json.dumps(dict(source='camera',matched_path_digest='safe',released_hazard_ids=list(node.hazards.regions)))))
        self.assertTrue(node.hazards.regions) # Delayed ACK cannot clear a newly held STOP.
        node.corridor_stop_required=False
        node.hazard_ack_cb(NS(data=json.dumps(dict(source='camera',matched_path_digest='safe',released_hazard_ids=list(node.hazards.regions)))))
        self.assertFalse(node.hazards.regions)

    def test_stationary_braking_region_outside_target_window_can_be_rechecked(self):
        cls=n['CameraRightPath'];node=cls.__new__(cls)
        node.path=NS(header=NS(stamp=NS(to_nsec=lambda:123)),
            poses=[NS(pose=NS(position=NS(x=x,y=0.))) for x in np.arange(0.,10.5,.5)])
        node.listener=NS(lookupTransform=lambda *_:([0.,0.,0.],[0.,0.,0.,1.]))
        node.origin=np.array([0.,0.,1.]);node.rotation=np.eye(3)
        node.matrix=np.eye(3);node.distortion=np.zeros(5);node.ground_z=0.
        node.packet={'road_polygon_px':[[0,0],[1,0],[1,1],[0,1]]};node.received=1.3
        node.strategy='segment_cache';node.timeout=.8;node.max_capture_age_ms=800
        node.applied=None;node.map_anchor=([0.,0.],0.);node.osm_junctions=np.zeros((0,2))
        node.osm_exit_requested=True # Junction/exit must not suppress spatial proof.
        node.cache=n['SegmentCache']();node.cache.committed={4:(0.,1.)}
        old=self.evidence(2.,100000000);old['points_odom'][2,1]=-.55
        old['route_point_odom']=[2.,0.]
        node.cache_geometry={4:old};node.hazards=n['HazardMemory']()
        node.hazards.remember(node.cache_geometry,[4])
        node.cache_pose=lambda now:([.3,0.,0.],0.)
        curves=[np.column_stack((np.arange(1.,8.1,.25),np.full(29,value))) for value in (2.,-2.)]
        fake_time=Mock(side_effect=lambda sec,nsec:(sec,nsec))
        fake_time.now=lambda:NS(to_nsec=lambda:1300000000)
        with patch.dict(n,rospy=NS(Time=fake_time),capture_vehicle_pose=lambda *_:([.3,0.,0.],0.),
                        tf=NS(transformations=NS(euler_from_quaternion=lambda *_:(0.,0.,0.)))):
            node.last_seq=1;node.observe_verification(curves,1100000000,1.1)
            self.assertNotIn(4,node.current_proof_geometry)
            node.observe_verification(curves,1100000000,1.11) # Same frame is not confirmation.
            self.assertNotIn(4,node.current_proof_geometry)
            node.last_seq=2;node.observe_verification(curves,1200000000,1.2)
            self.assertNotIn(4,node.current_proof_geometry)
            node.last_seq=3;node.observe_verification(curves,1300000000,1.3)
            self.assertIn(4,node.current_proof_geometry)
            self.assertEqual(node.cache.committed[4][1],1.) # No static TTL extension.
            self.assertEqual(node.cache_geometry[4]['points_odom'][2,1],-2.)
            node.cached_result(copy.deepcopy(node.path),1.3,.2)
            self.assertFalse(node.corridor_stop_required)
            self.assertTrue(node.corridor_verified)
            self.assertEqual(len(node.hazard_proof),len(node.hazards.regions))
    def test_guard_keeps_original_danger_across_new_route_and_wrong_proof(self):
        def route(stamp):
            return NS(header=NS(frame_id='map',stamp=NS(to_nsec=lambda:stamp)),poses=[NS(pose=NS(position=NS(x=x,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))) for x in (0.,1.,8.)])
        guard=CameraPathGuard();path=route(1);guard.original_cb(path);guard.candidate_cb(path)
        status=dict(control_enabled=True,route_stamp_ns=1,route_frame_id='map',corridor_stop_required=True,
                    hazard_regions={'danger':dict(right_odom=[4.,-2.],capture_timestamp_ns=100)})
        guard.status_cb(status,1.);self.assertEqual(guard.select(1.,200)[2]['source'],'stop')
        path=route(2);guard.original_cb(path);guard.candidate_cb(path)
        verification=dict(path_digest=path_digest(path),coordinate_frame='odom',verified_at_ns=200,valid_until_ns=500)
        safe=dict(status,route_stamp_ns=2,corridor_stop_required=False,corridor_verified=True,control_path_digest=path_digest(path),
            inference_diagnostics=dict(hazard_verification=verification,hazard_proof={'danger':dict(right_odom=[6.,-2.],capture_timestamp_ns=200)}))
        guard.status_cb(safe,2.);self.assertEqual(guard.select(2.,200)[2]['source'],'stop')
        safe['inference_diagnostics']['hazard_proof']['danger']['right_odom']=[4.,-2.]
        lease=ExecutionPolicy().contract();lease['path_digest']=path_digest(path)
        safe['inference_diagnostics']['execution_contract']=lease
        guard.status_cb(safe,2.);self.assertEqual(guard.select(2.,600)[2]['source'],'stop')
        guard.status_cb(safe,2.);self.assertEqual(guard.select(2.,200)[2]['source'],'camera')
        self.assertTrue(guard.hazard_regions)
        self.assertTrue(guard.commit_recovery(guard.authorization_generation,guard.safety_generation,200))
        self.assertEqual(guard.select(2.,200)[2]['released_hazard_ids'],['danger'])
    def test_stop_callback_cannot_be_hidden_by_a_later_heartbeat_before_poll(self):
        path=NS(header=NS(frame_id='map',stamp=NS(to_nsec=lambda:1)),poses=[NS(pose=NS(position=NS(x=x,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))) for x in (0.,1.,8.)])
        guard=CameraPathGuard();guard.original_cb(path);guard.candidate_cb(path)
        status=dict(control_enabled=True,route_stamp_ns=1,route_frame_id='map',corridor_stop_required=True,
            hazard_regions={'danger':dict(right_odom=[4.,-2.],capture_timestamp_ns=100)})
        guard.status_cb(status,1.)
        self.assertEqual(guard.safety_generation,1)
        # No select() in between: a long solve must not miss the revocation.
        guard.status_cb(dict(status,corridor_stop_required=False,corridor_verified=True,
            control_path_digest=path_digest(path),inference_diagnostics={}),1.1)
        self.assertEqual(guard.select(1.1,200)[2]['source'],'stop')
        self.assertEqual(guard.select(1.1,200)[2]['safety_generation'],1)


class ExecutionPolicy(unittest.TestCase):
    def contract(self):
        return dict(version=1,path_digest='test',expires_ns=500000000,speed_cap_mps=.3,coordinate_frame='odom',
            end_odom=[8.,0.],direction_odom=[1.,0.],sections=[dict(route_point=[x,0.],normal=[0.,1.],right=[x,-2.],left=[x,2.]) for x in np.arange(1.,8.1,.5)])
    def test_hard_cap_reintegrates_and_validates_the_actual_prediction(self):
        robot=NS(kinematics='acker',L=.67,max_speed=np.array([1.5,.35]),max_acce=np.array([.8,.08]))
        states=[np.array([[i*.08],[0.],[0.]]) for i in range(5)]
        info=dict(opt_state_list=states,vel_tensor=np.array([[.8]*4,[0.]*4]),stop=False,arrive=False)
        action,result=cap_solution_speed(np.array([[.8],[0.]]),info,robot,.1,.3)
        validate_solution(action,result,robot,.1)
        self.assertAlmostEqual(action[0,0],.3)
        self.assertAlmostEqual(result['opt_state_list'][-1][0,0],.12)
        self.assertAlmostEqual(info['opt_state_list'][-1][0,0],.32)
    def test_real_planner_saved_inputs_with_reduced_reference_and_hard_cap(self):
        from neupan import neupan
        files=sorted((workspace/'tracker_logs').glob('neupan_failure_20261026_*.npz'))
        if not files:self.skipTest('recorded solver snapshots not available')
        for filename in files:
            data=np.load(filename);meta=json.loads(filename.with_suffix('.json').read_text())
            planner=neupan.init_from_yaml(meta['config_file'],pan={'dune_checkpoint':meta['dune_checkpoint']})
            planner.ipath.set_initial_path([p.reshape(4,1) for p in data['initial_path'].T])
            for key in ('point_index','curve_index'):setattr(planner.ipath,key,meta['path_progress'][key])
            planner.cur_vel_array=data['cur_vel_array'].copy()
            planner.set_reference_speed(.3)
            action,info=planner(data['solver_robot_state'],data['solver_obstacle_points'])
            validate_solution(action,info,planner.robot,planner.dt)
            action,info=cap_solution_speed(action,info,planner.robot,planner.dt,.3)
            validate_solution(action,info,planner.robot,planner.dt)
            trajectory_clearance(np.hstack(info['opt_state_list']),data['solver_obstacle_points'],planner.robot.vertices)
            self.assertLessEqual(abs(float(action[0,0])),.300001)
    def test_speed_expiry_endpoint_and_actual_swept_corridor_are_enforced(self):
        footprint=np.array([[-.14,.81,.81,-.14],[-.425,-.425,.425,.425]])
        states=np.array([[2.,0.,0.],[2.1,0.,0.]])
        check_execution_contract(self.contract(),states,100,.3,footprint)
        for contract,poses,now,speed in ((self.contract(),states,100,.4),(self.contract(),states,600000000,.3),
                (self.contract(),states+np.array([5.5,0.,0.]),100,.3),
                (self.contract(),states+np.array([0.,-1.7,0.]),100,.3)):
            with self.assertRaises(ValueError):check_execution_contract(contract,poses,now,speed,footprint)


if __name__=='__main__':unittest.main()
