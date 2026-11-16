"""Oct30: advisory failures, actual-pose suffix loading and timer diagnostics.

No ROS master or actuator publications; frozen bag inputs are not a new drive.
"""
import copy
import ast
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch
import numpy as np
import test_stable_reference_policy as stable
from test_camera_right_refinement import namespace as n
from reference_test_frames import stamp_reference_frame
import test_outdoor_start_connection as connection_tests
import test_runtime_position as localization_fixture
import test_global_alignment as alignment_fixture
from osm_camera_path_guard import path_digest

ROOT=Path(__file__).resolve().parents[3]


def connected_scene():
    node,guard,clock,points,_=stable.StableReferenceTests().repair_node(3.6)
    geometry={i:dict(points_odom=np.array([[points[i,0],0.],[points[i,0],0.],[points[i,0],-3.6]]),
        route_point_odom=points[i].tolist(),normal_odom=[0.,1.],capture_timestamp_ns=10**12) for i in range(24,37)}
    node.cache_geometry=geometry;node.cache.committed={i:(0.,clock[0]) for i in geometry}
    node.cache_pose=lambda now,force=False:([10.,-1.1,0.],0.)
    node.reference_progress_s=10.
    return node,guard,clock,points,geometry


class AdvisoryTests(unittest.TestCase):
    def test_original_digest_is_bound_before_densification_and_moved_endpoint_is_consumed(self):
        node,guard,clock,points,geometry=stable.StableReferenceTests().repair_node(4.)
        geometry=copy.deepcopy(geometry)
        original=copy.deepcopy(node.path);original.poses=[original.poses[i] for i in (0,60,200)]
        node.route_signature=None;node.path_cb(original);guard.original_cb(original)
        node.map_anchor=([0.,0.],0.)
        self.assertEqual(node.original_path_digest,path_digest(original))
        self.assertNotEqual(path_digest(node.path),path_digest(original))
        node.cache_geometry=geometry;node.cache.committed={i:(0.,clock[0]) for i in geometry}
        with stable.StableReferenceTests().repair_context(clock):
            for seq in (1,2,3):
                clock[0]+=.2;node.last_seq=seq;stamp_reference_frame(node,seq);result=copy.deepcopy(node.path)
                active,count=node.stable_result(result,clock[0],.2);node.publish_result(result,clock[0],'terminal',active,count)
                selected=guard.select(clock[0],round(clock[0]*1.e9))
            self.assertEqual(selected[2]['source'],'camera')
            np.testing.assert_allclose(selected[2]['reference_endpoint_map'],[100.,-1.])
            np.testing.assert_allclose(selected[2]['original_endpoint_map'],[100.,0.])
            # Missing/misbound ORIGINAL digest cannot nominate a new reference.
            status=copy.deepcopy(guard.status);status['original_path_digest']='different_original'
            guard.status_cb(status,clock[0]);self.assertEqual(guard.select(clock[0])[2]['source'],'osm')
        self.assertEqual(original.poses[-1].pose.position.y,0.)

    def test_new_segment_does_not_inherit_first_segment_offset(self):
        points,geometry=stable.road(4.)
        first,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,
            [0.,0.],segment_end=40.)
        self.assertTrue(report['changed']);self.assertLess(first[50,1],-.5)
        np.testing.assert_array_equal(first[80:],points[80:])
        tail={i:dict(points_odom=np.array([points[i],points[i]+[0,2],points[i]+[0,-.7]]),normal_odom=[0.,1.]) for i in range(124,137)}
        second,report=n['stable_segment_reference'](points,first,tail,([0.,0.],0.),tail,
            [60.,0.],progress=60.,heading=0.)
        self.assertTrue(report['changed']);self.assertAlmostEqual(second[-1,1],.3)
        np.testing.assert_array_equal(second[:120],first[:120])

    def test_twelve_meter_route_can_establish_terminal_offset(self):
        points,geometry=stable.road(4.);points=points[:25]
        result,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,[0.,0.])
        self.assertTrue(report['changed']);self.assertTrue(report['shape']['ok'])
        np.testing.assert_allclose(result[-1],[12.,-1.])

    def test_queued_late_reference_cannot_restart_completed_task(self):
        adapter,apply=connection_tests.AdapterTests().node()
        adapter.arrival_path_key=('map',123);adapter.goal_arrived_state=True;adapter.waiting_for_new_goal=True
        adapter.pending_route_update=('path',[np.array([[0.],[0.],[0.],[1.]]),np.array([[12.],[-1.],[0.],[1.]])],('map',123))
        apply(adapter)
        adapter.neupan_planner.set_initial_path.assert_not_called()
        self.assertTrue(adapter.goal_arrived_state);self.assertTrue(adapter.waiting_for_new_goal)

    def test_zero_stamp_corrected_end_keeps_original_task_identity(self):
        tree=ast.parse((ROOT/'src/neupan_ros/src/neupan_core.py').read_text())
        cls=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name=='neupan_core')
        method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='path_callback')
        ns=dict(np=np,atan2=math.atan2,rospy=NS(Time=lambda *a:0,Duration=lambda *a:0,
            loginfo_once=Mock(),loginfo=Mock(),logwarn=Mock()))
        exec(compile(ast.Module(body=[method],type_ignores=[]),'path_callback','exec'),ns)
        node=NS(map_frame='map',listener=NS(waitForTransform=Mock(),transformPose=lambda frame,p:p),
            include_initial_path_direction=False,verbose_runtime_log=False,neupan_planner=NS(initial_path=[1]),
            refresh_initial_path=True,waiting_for_new_goal=False,queue_route_update=Mock(),
            camera_guard_selected_state=dict(source='camera',reference_role='advisory',matched_route_identity=('map',0,8.,0.)))
        path=stable.protocol.path(stamp=0)
        for pose in path.poses:pose.header=NS(frame_id='map',stamp=0)
        for offset in (-1.,-1.2):
            path.poses[-1].pose.position.y=offset
            self.assertTrue(ns['path_callback'](node,copy.deepcopy(path)))
            self.assertEqual(node.queue_route_update.call_args[0][2],('map',0,8.,0.))

    def test_status_first_endpoint_replacement_waits_for_matching_path(self):
        node,guard,clock,_,_=stable.StableReferenceTests().repair_node(4.)
        with stable.StableReferenceTests().repair_context(clock):
            for seq in (1,2,3):
                node.last_seq=seq;stamp_reference_frame(node,seq);result=copy.deepcopy(node.path)
                active,count=node.stable_result(result,clock[0],.2);node.publish_result(result,clock[0],'terminal',active,count)
                before=guard.select(clock[0])
            replacement=copy.deepcopy(result);replacement.poses[-1].pose.position.y=-1.2
            status=copy.deepcopy(guard.status);status['control_path_digest']=path_digest(replacement)
            status['inference_diagnostics']['reference_version']+=1
            guard.status_cb(status,clock[0])
            held=guard.select(clock[0]);self.assertEqual(held[1],before[1])
            self.assertEqual(held[2]['reference_endpoint_map'],[100.,-1.])
            guard.candidate_cb(replacement)
            selected=guard.select(clock[0]);self.assertEqual(selected[2]['source'],'camera')
            self.assertEqual(selected[2]['reference_endpoint_map'],[100.,-1.2])

    def test_curved_offsets_follow_local_normals_through_corrected_goal(self):
        for sign in (-1.,1.):
            arc=np.arange(0.,80.5,.5);angle=arc/20.
            points=np.column_stack((20*np.sin(angle),sign*20*(1-np.cos(angle))))
            tangent=np.vstack((np.diff(points,axis=0),points[-1]-points[-2]))
            normals=np.column_stack((-tangent[:,1],tangent[:,0]))/np.linalg.norm(tangent,axis=1)[:,None]
            geometry={i:dict(points_odom=np.array([points[i],points[i]+normals[i]*.1,points[i]-normals[i]*3.6]),
                normal_odom=normals[i].tolist(),mode='right_only') for i in range(4,17)}
            candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,
                [0.,0.],dict(speed=1.5,wheelbase=.67,max_steer=.35,max_steer_rate=.08),progress=0.,heading=sign*.0125)
            self.assertTrue(report['changed']);self.assertTrue(report['shape']['ok'])
            displacement=candidate-points
            np.testing.assert_allclose(np.sum(displacement*tangent,axis=1),0.,atol=1.e-8)
            self.assertLess(float(displacement[60]@normals[60]),-.4)
            self.assertGreater(np.linalg.norm(displacement[60]-displacement[100]),.2)
            self.assertTrue(n['advisory_geometry_ok'](candidate,points))
            self.assertAlmostEqual(float((candidate[-1]-points[-1])@normals[-1]),-.6)

    def test_left_mask_and_past_cache_do_not_withdraw_accepted_reference(self):
        node,guard,clock,points,geometry,adopted=stable.StableReferenceTests().adopted_reference()
        version=node.reference_version;connection=getattr(node,'reference_connection',None)
        for item in geometry.values():item['points_odom'][1,1]=-3.
        geometry[2]=dict(points_odom=np.array([[1.,0.],[1.,-.2],[1.,-.1]]),normal_odom=[0.,1.])
        node.cache_geometry=geometry;node.cache.committed={i:(0.,clock[0]) for i in geometry}
        with stable.StableReferenceTests().repair_context(clock):
            for seq in range(10,16):
                node.last_seq=seq;stamp_reference_frame(node,seq);clock[0]+=.2
                node.stable_result(copy.deepcopy(node.path),clock[0],.2)
                np.testing.assert_array_equal(node.frozen_reference_map,adopted)
                self.assertEqual(node.reference_version,version)
                self.assertEqual(getattr(node,'reference_connection',None),connection)
            node.cache.committed.clear();node.cache_geometry={}
            node.advisory_fallback('occlusion',clock[0])
            selected=guard.select(clock[0],round(clock[0]*1.e9))
            self.assertEqual(selected[2]['source'],'camera')
            np.testing.assert_array_equal(node.frozen_reference_map,adopted)

    def test_consumer_keeps_last_accepted_pair_during_partial_replacement(self):
        node,guard,clock,_,_,adopted=stable.StableReferenceTests().adopted_reference()
        with stable.StableReferenceTests().repair_context(clock):
            result=copy.deepcopy(node.path)
            active,count=node.stable_result(result,clock[0],.2)
            node.publish_result(result,clock[0],'accepted',active,count)
            before=guard.select(clock[0],round(clock[0]*1.e9))
            replacement=copy.deepcopy(result);replacement.poses[40].pose.position.y-=.1
            guard.candidate_cb(replacement)
            after=guard.select(clock[0],round(clock[0]*1.e9))
            self.assertEqual(after[2]['source'],'camera');self.assertEqual(after[1],before[1])
            self.assertEqual(after[2]['reference_connection'],before[2]['reference_connection'])
            status=copy.deepcopy(guard.status)
            status['control_path_digest']=guard.original_digest
            status['inference_diagnostics']['reference_frame_invalidated']=True
            guard.status_cb(status,clock[0])
            self.assertIsNone(guard.accepted_advisory)
            self.assertNotEqual(guard.select(clock[0],round(clock[0]*1.e9))[2]['source'],'camera')
            # A new original task must never inherit this retained geometry.
            original=copy.deepcopy(node.path);original.header.stamp=stable.Stamp(clock[0]+10)
            guard.original_cb(original)
            self.assertNotEqual(guard.select(clock[0],round(clock[0]*1.e9))[2]['source'],'camera')

    def test_replacing_reference_commits_transition_and_connection_together(self):
        node,_,clock,points,geometry,adopted=stable.StableReferenceTests().adopted_reference()
        node.reference_transition_end_odom=np.array([-1.,0.])
        node.reference_exit_frozen=True;node.reference_exit_end_s=99.
        # A replacement requires fresh out-of-band road evidence, not merely
        # a mocked successful geometry search while already inside the band.
        for item in geometry.values():
            item['points_odom'][2,1] -= 1.5
        proposed=adopted.copy();proposed[40,1]-=.1
        report=dict(changed=True,reason='new_spatial_segment',transition_start_m=0.,transition_length_m=16.,reference_connection=None)
        with stable.StableReferenceTests().repair_context(clock),patch.dict(n,stable_segment_reference=Mock(side_effect=lambda *a,**kw:(proposed,copy.deepcopy(report)))):
            for seq in (50,51,52):
                node.last_seq=seq;stamp_reference_frame(node,seq);node.stable_result(copy.deepcopy(node.path),clock[0],.2)
        np.testing.assert_array_equal(node.frozen_reference_map,proposed)
        np.testing.assert_allclose(node.reference_transition_end_odom,[16.,0.])
        self.assertIsNone(node.reference_connection);self.assertFalse(node.reference_exit_frozen)
        self.assertIsNone(node.reference_exit_end_s)

    def test_deployed_planner_loads_new_connection_during_same_task(self):
        from neupan import neupan
        node,_,_,points,geometry=connected_scene()
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,
            [10.,-1.1,0.],node.vehicle_limits,progress=10.,heading=0.)
        config=ROOT/'src/neupan_ros/example/lxw_car/config/neupan_config_outdoor.yaml'
        checkpoint=ROOT/'src/neupan_ros/example/lxw_car/model/acker_robot_default_7/model_5000.pth'
        planner=neupan.init_from_yaml(str(config),pan={'dune_checkpoint':str(checkpoint)})
        planner.set_initial_path([np.array([[x],[y],[0.],[1.]]) for x,y in points]);planner.reset()
        planner.ipath.point_index=20
        adapter,apply=connection_tests.AdapterTests().node()
        adapter.neupan_planner=planner;adapter.outdoor_reference_spacing=.5
        adapter.camera_guard=object();adapter.arrival_path_key=('map',123)
        adapter.robot_state=np.array([[10.],[-1.1],[0.]])
        adapter.camera_guard_selected_state=dict(source='camera',reference_role='advisory',reference_version=1,
            reference_connection=report['reference_connection'],matched_path_digest='bound')
        data=[np.array([[x],[y],[0.],[1.]]) for x,y in candidate]
        adapter.pending_route_update=('path',data,('map',123));apply(adapter)
        np.testing.assert_allclose(planner.ipath.cur_curve[0][:2,0],[10.,-1.1],atol=1.e-5)
        self.assertEqual(planner.ipath.point_index,0)
        self.assertEqual(adapter.reference_receipt['selected_reference_version'],1)
        self.assertEqual(adapter.start_connector.connection_kind,'normal_route')
        actual_end=planner.ipath.cur_curve[-1][:2,0]
        np.testing.assert_allclose(actual_end,candidate[-1])
        np.testing.assert_allclose(adapter.reference_receipt['loaded_endpoint_map'],actual_end)
        planner.ipath.point_index=len(planner.ipath.cur_curve)-1
        old_goal=np.r_[points[-1],0.].reshape(3,1)
        corrected_goal=np.r_[actual_end,0.].reshape(3,1)
        self.assertAlmostEqual(planner.ipath.arrive_threshold,.5)
        self.assertFalse(planner.ipath.check_curve_arrive(old_goal,planner.ipath.arrive_threshold))
        self.assertTrue(planner.ipath.check_curve_arrive(corrected_goal,planner.ipath.arrive_threshold))
        # Returning to full OSM must not reuse the shortened suffix's arc index.
        adapter.camera_guard_selected_state=dict(source='osm',reference_role='advisory')
        adapter.pending_route_update=('path',[np.array([[x],[y],[0.],[1.]]) for x,y in points],('map',123))
        apply(adapter)
        self.assertGreaterEqual(planner.ipath.point_index,19)
        self.assertIsNone(adapter.reference_receipt['connection_kind'])

    def test_measured_pose_connects_where_unused_osm_prefix_cannot(self):
        node,guard,clock,points,geometry=connected_scene()
        with stable.StableReferenceTests().repair_context(clock):
            for seq in range(1,5):
                clock[0]+=.2;node.last_seq=seq
                stamp_reference_frame(node,seq)
                result=copy.deepcopy(node.path)
                active,count=node.stable_result(result,clock[0],.2)
                node.publish_result(result,clock[0],'pose connection',active,count)
                state=guard.select(clock[0],round(clock[0]*1.e9))[2]
                self.assertNotEqual(state['source'],'stop')
                self.assertFalse(node.hazards.regions);self.assertIsNone(state['execution_contract'])
                if seq==3:
                    self.assertEqual(node.reference_version,1);frozen=node.frozen_reference_map.copy()
            np.testing.assert_array_equal(frozen,node.frozen_reference_map)
            self.assertEqual(node.reference_version,1)
            connection=state['reference_connection'];self.assertEqual(connection['start_index'],20)
            np.testing.assert_allclose(frozen[20],[10.,-1.1])
            # Already 2.5m from the right edge: stay near the measured vehicle,
            # rather than moving 0.5m merely to minimize offset from bad OSM.
            self.assertAlmostEqual(frozen[-1,1],-1.1)
            self.assertTrue(n['advisory_geometry_ok'](frozen[20:],points[20:]))

    def test_connected_suffix_is_actually_loaded_by_existing_neupan_adapter(self):
        node,_,clock,points,geometry=connected_scene()
        candidate,report=n['stable_segment_reference'](points,points,geometry,([0.,0.],0.),geometry,
            [10.,-1.1,0.],node.vehicle_limits,progress=10.,heading=0.)
        adapter,apply=connection_tests.AdapterTests().node();adapter.start_connector=None
        adapter.outdoor_reference_spacing=.5;adapter.robot_state=np.array([[10.],[-1.1],[0.]])
        adapter.camera_guard_selected_state=dict(source='camera',reference_role='advisory',reference_version=1,
            reference_connection=report['reference_connection'],matched_path_digest='bound')
        data=[np.array([[x],[y],[0.],[1.]]) for x,y in candidate]
        class Planner:
            def set_initial_path(self,route):
                self.loaded=route;self.ipath=NS(cur_curve=route,point_index=0)
            def reset(self):pass
        adapter.neupan_planner=Planner();adapter.pending_route_update=('path',data,('map',123))
        apply(adapter)
        np.testing.assert_allclose(adapter.neupan_planner.loaded[0][:2,0],[10.,-1.1])
        self.assertEqual(adapter.neupan_planner.ipath.point_index,0)
        self.assertEqual(adapter.reference_receipt['connection_kind'],'measured_pose_c2')

    def test_frozen_field_conflicts_do_not_create_motion_hazards(self):
        directory=ROOT/'tracker_logs/review_20261030/field_run_review'
        files=sorted(directory.glob('*_conflict_geometry.json'))
        if not files:self.skipTest('frozen field review inputs unavailable')
        for filename in files:
            data=json.loads(filename.read_text())
            node,guard,clock,_,_=stable.StableReferenceTests().repair_node()
            points=np.asarray(data['points'],float)
            node.path.poses=[NS(pose=NS(position=NS(x=float(x),y=float(y),z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))) for x,y in points]
            guard.original_cb(node.path)
            node.cache_geometry={int(k):dict(v,points_odom=np.asarray(v['points_odom'],float)) for k,v in data['geometry'].items()}
            node.cache.committed={i:(0.,clock[0]) for i in node.cache_geometry};node.map_anchor=data['anchor']
            node.cache_pose=lambda now,force=False:(data['position'],data['heading'])
            node.reference_progress_s=data['progress']
            with stable.StableReferenceTests().repair_context(clock):
                for seq in range(1,5):
                    clock[0]+=.2;node.last_seq=seq;stamp_reference_frame(node,seq);result=copy.deepcopy(node.path)
                    active,count=node.stable_result(result,clock[0],.2)
                    node.publish_result(result,clock[0],'frozen conflict',active,count)
                    self.assertFalse(node.hazards.regions)
                    self.assertFalse(node.corridor_stop_required)
                    self.assertIsNone(node.execution_contract)
                    self.assertNotEqual(guard.select(clock[0],round(clock[0]*1.e9))[2]['source'],'stop')

    def test_independent_danger_is_not_cleared_by_reference_failure(self):
        node,guard,clock,_,geometry=stable.StableReferenceTests().repair_node(.3)
        node.hazards.remember(geometry,geometry);before=copy.deepcopy(node.hazards.regions)
        with stable.StableReferenceTests().repair_context(clock):
            result=copy.deepcopy(node.path);active,count=node.stable_result(result,clock[0],.2)
            node.publish_result(result,clock[0],'independent retained danger',active,count)
            self.assertEqual(node.hazards.regions,before)
            self.assertEqual(guard.select(clock[0],round(clock[0]*1.e9))[2]['source'],'stop')
            self.assertIsNone(node.execution_contract)


class ProcessingHealthTests(unittest.TestCase):
    def node(self):
        node,_,clock,_,_=stable.StableReferenceTests().repair_node()
        node.processing_health_pub=NS(publish=Mock());node.packet={'frame_seq':123}
        return node,clock

    def test_unexpected_exception_falls_back_and_next_callback_runs(self):
        node,clock=self.node();node._tick_locked=Mock(side_effect=[RuntimeError('bad geometry'),None])
        with stable.StableReferenceTests().repair_context(clock):
            node.tick(None)
            self.assertFalse(node.pose_tick_open);self.assertEqual(node.processing_health['stage'],'failed')
            self.assertIn('bad geometry',node.processing_health['last_exception']['traceback'])
            self.assertFalse(node.corridor_stop_required);self.assertFalse(node.hazards.regions)
            clock[0]+=.2;node.tick(None)
            self.assertEqual(node.processing_health['stage'],'idle')
            self.assertEqual(node.processing_health['last_completed_frame'],123)
            self.assertEqual(node._tick_locked.call_count,2)

    def test_exception_before_inner_processing_try_does_not_kill_real_tick(self):
        node,clock=self.node();node.last_tick=clock[0]-.2
        node.diagnose_observation=Mock(side_effect=[ValueError('diagnosis failed'),{}])
        node.refine=Mock(side_effect=ValueError('ordinary observation gap'))
        node.geometry_diagnostics={};node.confirmations=0
        with stable.StableReferenceTests().repair_context(clock):
            node.tick(None)
            self.assertEqual(node.processing_health['last_exception']['stage'],'observation_diagnostics')
            clock[0]+=.2;node.tick(None)
            self.assertEqual(node.processing_health['stage'],'idle')
            self.assertEqual(node.diagnose_observation.call_count,2)
            self.assertFalse(node.pose_tick_open)

    def test_publication_exception_is_observable_not_a_dead_timer(self):
        node,clock=self.node()
        def fail(_):
            node.processing_stage('status_publish');raise OSError('transport failed')
        node._tick_locked=fail;node.status=NS(publish=Mock(side_effect=OSError('publish failed')))
        with stable.StableReferenceTests().repair_context(clock):node.tick(None)
        self.assertEqual(node.processing_health['last_exception']['stage'],'status_publish')
        self.assertIn('fallback_exception',node.processing_health)
        self.assertFalse(node.pose_tick_open)
        node.processing_health_pub.publish.assert_called()

    def test_health_watchdog_does_not_wait_for_processing_lock(self):
        node,clock=self.node();node.lock.acquire()
        try:
            with stable.StableReferenceTests().repair_context(clock):
                node.processing_stage('reference_search',active=True,started_at_mono=clock[0])
                clock[0]+=3.;node.publish_processing_health()
            state=json.loads(node.processing_health_pub.publish.call_args.args[0].data)
            self.assertEqual(state['stage'],'reference_search');self.assertGreaterEqual(state['active_elapsed_ms'],3000.)
        finally:node.lock.release()


class AlignmentContinuityTests(unittest.TestCase):
    """Real bridge correction + real camera tick/consumer; no ROS master."""
    setUp=localization_fixture.RuntimePositionTests.setUp

    def scene(self):
        self.now=localization_fixture.rospy.Time(1000)
        bridge=localization_fixture.RuntimePositionTests.bridge(self)
        bridge.transform_x=bridge.transform_y=bridge.transform_yaw=0.
        bridge.mode=bridge.RECOVERING
        alignment_fixture.GlobalAlignmentTests.anchor(bridge,0.,0.)
        bridge.last_local_odom.header.stamp=self.now;bridge.last_local_recv=self.now
        bridge.dead_reckoning_budget.begin(1000.,(0.,0.))
        node,guard,clock,points,geometry,frozen=stable.StableReferenceTests().adopted_reference()
        node.cache_pose=n['CameraRightPath'].cache_pose.__get__(node)
        node.safe=True;node.safe_received=1000.;node.distance_reference='camera_center'
        node.last_tick=999.9;node.anchor_sample_sec=999.9
        node.processing_health={};node.processing_health_pub=NS(publish=Mock())
        node.diagnose_observation=lambda now:{};node.refine=Mock(side_effect=ValueError('occluded'))
        def lookup(a,b,stamp):
            angle=bridge.transform_yaw
            if b=='odom':return ([bridge.transform_x,bridge.transform_y,0.],[0.,0.,math.sin(angle/2),math.cos(angle/2)])
            return ([0.,0.,0.],[0.,0.,0.,1.])
        node.listener=NS(getLatestCommonTime=lambda *a:stable.Stamp(clock[0]),lookupTransform=lookup)
        class Time(stable.Stamp):
            @staticmethod
            def now():return stable.Stamp(clock[0])
        context=patch.dict(n,time=NS(monotonic=lambda:clock[0]),
            rospy=NS(Time=Time,logwarn_throttle=Mock(),logerr_throttle=Mock()),
            tf=NS(Exception=RuntimeError,transformations=NS(euler_from_quaternion=lambda q:(0.,0.,2*math.atan2(q[2],q[3])),
                quaternion_from_euler=lambda *a:(0.,0.,0.,1.))))
        return bridge,node,guard,clock,frozen,context

    def status(self,bridge,node,clock):
        self.now=localization_fixture.rospy.Time.from_sec(clock[0])
        bridge.publish_status(self.now)
        data=bridge.status_pub.publish.call_args[0][0]
        if hasattr(data,'data'):data=data.data
        node.alignment_status_cb(NS(data=data))
        return json.loads(data)

    def completed_connection_consumer(self,node,guard,clock,frozen):
        """Real native loaded path includes an already executed off-road prefix."""
        from neupan.blocks.initial_path import InitialPath
        ipath=InitialPath(10,.1,1.,NS(kinematics='acker',L=.67,max_speed=[1.,.35]))
        prefix=np.array([[-2.,1.],[-1.,.5]])
        ipath.set_initial_path([np.array([[x],[y],[0.],[1.]]) for x,y in np.vstack((prefix,frozen))])
        ipath.point_index=42;ipath.arrive_flag=False
        planner=NS(ipath=ipath,initial_path=ipath.initial_path,reset=Mock(),set_initial_path=Mock())
        adapter=NS(camera_guard=guard,refresh_camera_hazard_scope=Mock(),
            camera_guard_signature=None,camera_guard_last_state=None,camera_guard_last_publish=0.,
            camera_guard_pub=NS(publish=Mock()),path_callback=Mock(return_value=True),
            start_connector=NS(phase='following_route',prefix_count=2),neupan_planner=planner,
            goal_arrived_state=False,arrive=False,waiting_for_new_goal=False,
            pending_route_update=None,arrival_path_key=('map',node.path.header.stamp.to_nsec()),
            reference_receipt=dict(selected_reference_version=node.reference_version,
                loaded_endpoint_map=ipath.initial_path[-1][:2,0].tolist()))
        source=ROOT/'src/neupan_ros/src/neupan_core.py'
        cls=next(x for x in ast.parse(source.read_text()).body if isinstance(x,ast.ClassDef) and x.name=='neupan_core')
        method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='apply_camera_path_guard')
        ns=dict(np=np,copy=copy,json=json,atan2=math.atan2,cos=math.cos,sin=math.sin,
            time=NS(monotonic=lambda:clock[0]),String=lambda **kw:NS(**kw),
            rospy=NS(Time=NS(now=lambda:stable.Stamp(clock[0])),logwarn_throttle=Mock(),logdebug=Mock()))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
        apply=ns['apply_camera_path_guard'].__get__(adapter)
        self.assertTrue(apply());adapter.path_callback.reset_mock()
        return adapter,apply

    def test_gps_translation_corrects_vehicle_error_without_transporting_loaded_reference(self):
        b,node,guard,clock,frozen,context=self.scene()
        with context:
            self.status(b,node,clock);node.tick(None)
            adapter,apply=self.completed_connection_consumer(node,guard,clock,frozen)
            local_vehicle=frozen[40]-[0.,1.]
            loaded=adapter.neupan_planner.ipath.initial_path
            original=np.array([p.copy() for p in loaded]);version=node.reference_version
            entrance=n['planar_transform']([node.reference_transition_end_odom],node.map_anchor)[0]
            self.assertAlmostEqual(np.linalg.norm(local_vehicle-loaded[42][:2,0]),1.)
            # Existing gradual position-only recovery, without heading recovery.
            for step in range(10):
                clock[0]+=.1;node.safe_received=clock[0]
                b.move_transform_toward((0.,1.,0.),.1,1.,math.radians(8))
                self.status(b,node,clock);node.tick(None)
                self.assertTrue(apply())
                np.testing.assert_array_equal(np.array(loaded),original)
                np.testing.assert_array_equal(node.frozen_reference_map,frozen)
                np.testing.assert_allclose(n['planar_transform']([node.reference_transition_end_odom],node.map_anchor)[0],entrance)
                selected=guard.select(clock[0])[0]
                np.testing.assert_allclose([selected.poses[-1].pose.position.x,selected.poses[-1].pose.position.y],loaded[-1][:2,0])
                self.assertEqual(node.reference_version,version)
                self.assertEqual(adapter.neupan_planner.ipath.point_index,42)
            vehicle_map=n['planar_transform']([local_vehicle],node.map_anchor)[0]
            self.assertAlmostEqual(np.linalg.norm(vehicle_map-loaded[42][:2,0]),0.,places=9)
            adapter.path_callback.assert_not_called();adapter.neupan_planner.reset.assert_not_called()
            adapter.neupan_planner.set_initial_path.assert_not_called()
            self.assertIsNone(adapter.pending_route_update)

    def test_completed_connection_prefix_does_not_wait_during_position_and_heading_recovery(self):
        b,node,guard,clock,frozen,context=self.scene()
        with context:
            self.status(b,node,clock);node.tick(None)
            adapter,apply=self.completed_connection_consumer(node,guard,clock,frozen)
            ipath=adapter.neupan_planner.ipath;loaded=np.array([p.copy() for p in ipath.initial_path])
            adapter.goal_arrived_state=True;adapter.arrive=True;adapter.waiting_for_new_goal=True;ipath.arrive_flag=True
            for step in range(20):
                clock[0]+=.1;node.safe_received=clock[0]
                b.move_transform_toward((1.,0.,math.radians(15)),.1,.5,math.radians(8))
                self.status(b,node,clock);node.tick(None)
                self.assertTrue(apply())
                np.testing.assert_array_equal(np.array(ipath.initial_path),loaded)
                self.assertEqual((ipath.curve_index,ipath.point_index),(0,42))
                self.assertTrue(adapter.goal_arrived_state and adapter.arrive and adapter.waiting_for_new_goal and ipath.arrive_flag)
            adapter.path_callback.assert_not_called();adapter.neupan_planner.reset.assert_not_called()
            self.assertEqual(node.reference_version,1)

    def test_legal_point_eight_degree_recovery_keeps_reference_and_consumer_version(self):
        b,node,guard,clock,frozen,context=self.scene()
        with context:
            self.status(b,node,clock)
            node.anchor_sample_sec=clock[0]
            clock[0]+=.1;self.now=localization_fixture.rospy.Time.from_sec(clock[0])
            self.assertTrue(b.move_transform_toward((0.,0.,math.radians(10)),.1,.5,math.radians(8)))
            self.status(b,node,clock);node.tick(None)
            self.assertAlmostEqual(b.correction_diagnostics['actual_yaw_step_deg'],.8)
            self.assertFalse(node.cache_alignment_diagnostics['smooth_classification'])
            self.assertEqual(node.cache_alignment_diagnostics['correction_source'],'bridge_alignment_correction')
            np.testing.assert_array_equal(node.frozen_reference_map,frozen)
            self.assertEqual(node.reference_version,1);self.assertFalse(getattr(node,'reference_frame_invalidated',False))
            end_map=n['planar_transform']([node.reference_transition_end_odom],node.map_anchor)[0]
            np.testing.assert_allclose(end_map,[8.,0.],atol=1.e-10)
            selected=guard.select(clock[0],round(clock[0]*1.e9))[2]
            self.assertEqual(selected['source'],'camera');self.assertEqual(selected['reference_version'],1)
            self.assertIsNone(node.processing_health.get('last_exception'))

    def test_accumulated_normal_correction_exceeds_old_window_without_reset(self):
        b,node,guard,clock,frozen,context=self.scene()
        with context:
            self.status(b,node,clock)
            for step in range(20):
                clock[0]+=.1;self.now=localization_fixture.rospy.Time.from_sec(clock[0]);node.safe_received=clock[0]
                b.move_transform_toward((1.,0.,math.radians(25)),.1,.5,math.radians(8))
                self.status(b,node,clock);node.tick(None)
                np.testing.assert_array_equal(node.frozen_reference_map,frozen)
                self.assertEqual(node.reference_version,1)
            self.assertGreater(node.cache_alignment_diagnostics['window_yaw_deg'],10.)
            self.assertEqual(guard.select(clock[0])[2]['source'],'camera')

    def test_real_reanchor_generation_invalidates_once_and_retains_independent_danger(self):
        b,node,guard,clock,frozen,context=self.scene()
        with context:
            self.status(b,node,clock);node.hazards.regions={'real':dict(kind='boundary')}
            node.proof_cache=n['SegmentCache']();node.proof_cache.committed={4:(.1,clock[0])}
            clock[0]+=.1;self.now=localization_fixture.rospy.Time.from_sec(clock[0])
            odom=localization_fixture.Odometry();odom.header.stamp=self.now;odom.pose.pose.position.x=100.;odom.pose.pose.orientation.w=1.
            b.handle_local_fault(self.now,odom,'position_jump_100m')
            payload=self.status(b,node,clock)
            self.assertEqual(payload['reference_frame']['odom_epoch'],1)
            self.assertIsNone(node.frozen_reference_map);self.assertTrue(node.reference_frame_invalidated)
            self.assertFalse(node.proof_cache.committed)
            version=node.reference_version;self.assertIn('real',node.hazards.regions)
            node.alignment_fault_cb(NS(data=json.dumps(dict(stamp=clock[0],accepted=True,fault_triggered=True,fault='position_jump_100m'))))
            self.assertEqual(node.reference_version,version)
            node.advisory_fallback('coordinate reset',clock[0])
            self.assertIsNone(guard.accepted_advisory);self.assertEqual(guard.select(clock[0])[2]['source'],'stop')

    def test_bridge_restart_not_gnss_session_controls_coordinate_generation(self):
        b,node,guard,clock,frozen,context=self.scene()
        with context:
            payload=self.status(b,node,clock);clock[0]+=.1
            payload.update(stamp=clock[0],timebase_contract=dict(session='different_gnss_session'))
            node.alignment_status_cb(NS(data=json.dumps(payload)))
            np.testing.assert_array_equal(node.frozen_reference_map,frozen)
            payload['stamp']=clock[0]+.1;clock[0]+=.1
            payload['reference_frame']['instance_id']='restarted_localization_bridge'
            node.alignment_status_cb(NS(data=json.dumps(payload)))
            self.assertIsNone(node.frozen_reference_map);self.assertTrue(node.reference_frame_invalidated)

    def test_stale_reset_status_cannot_invalidate_current_reference(self):
        b,node,guard,clock,frozen,context=self.scene()
        with context:
            payload=self.status(b,node,clock)
            payload['stamp']=clock[0]-2.;payload['reference_frame']['odom_epoch']=99
            node.alignment_status_cb(NS(data=json.dumps(payload)))
            np.testing.assert_array_equal(node.frozen_reference_map,frozen)
            self.assertEqual(node.reference_version,1)

    def test_fault_before_first_status_is_invalidated_only_once(self):
        b,node,guard,clock,frozen,context=self.scene()
        with context:
            node.reference_accepted_stamp=clock[0];clock[0]+=.1
            self.now=localization_fixture.rospy.Time.from_sec(clock[0])
            event=dict(stamp=clock[0],accepted=True,fault_triggered=True,fault='local_origin_reset')
            node.alignment_fault_cb(NS(data=json.dumps(event)))
            version=node.reference_version
            odom=localization_fixture.Odometry();odom.header.stamp=self.now;odom.pose.pose.orientation.w=1.
            b.handle_local_fault(self.now,odom,'local_origin_reset')
            self.status(b,node,clock)
            self.assertEqual(node.reference_version,version)
            self.assertIsNone(node.frozen_reference_map)


if __name__=='__main__':unittest.main()
