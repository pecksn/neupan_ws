"""P3-I/P2-E/P3-J: actual production methods, fake clocks and publishers only."""
import copy
import json
from types import SimpleNamespace as NS
import threading
import unittest
from unittest.mock import Mock, patch
import numpy as np
import test_followup_safety as fixture
from test_camera_right_refinement import namespace as n
from neupan_safety import reference_offsets
from osm_camera_path_guard import CameraPathGuard,path_digest


def path(stamp=123,middle=0.):
    return NS(header=NS(frame_id='map',seq=0,stamp=NS(to_nsec=lambda:stamp)),poses=[
        NS(pose=NS(position=NS(x=x,y=y,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.)))
        for x,y in ((0.,0.),(1.,middle),(8.,0.))])


def contract(route,cap=.3):
    return dict(version=1,path_digest=path_digest(route),route_stamp_ns=123,expires_ns=1000500000000,
        speed_cap_mps=cap,coordinate_frame='odom',end_odom=[8.,0.],direction_odom=[1.,0.],
        sections=[dict(route_point=[x,0.],normal=[0.,1.],right=[x,-2.],left=[x,2.]) for x in np.arange(1.,8.1,.5)])


def status(route,lease):
    return dict(control_enabled=True,route_stamp_ns=123,route_frame_id='map',active=True,
        corridor_stop_required=False,corridor_verified=True,control_path_digest=path_digest(route),
        inference_diagnostics=dict(execution_contract=lease))


class PublicationTests(unittest.TestCase):
    def run_case(self,kind):
        old_path=path();new_path=path(middle=.1)
        old=contract(old_path);new=contract(new_path,.12 if kind in ('lower','during_check') else .3)
        context={}
        def hook(node,clock,ns):
            ns['reference_offsets']=reference_offsets
            class VersionDelayGuard(CameraPathGuard):
                def __getattribute__(self,key):
                    if key=='authorization_generation' and object.__getattribute__(self,'__dict__').get('delay_read_once',False):
                        self.delay_read_once=False;clock.value+=.005
                    return super().__getattribute__(key)
            guard=VersionDelayGuard() if kind=='late_version' else CameraPathGuard()
            guard.original_cb(old_path);guard.candidate_cb(old_path)
            if kind=='lease_lock':old['expires_ns']=1000003000000
            guard.status_cb(status(old_path,old),clock.value)
            if kind in ('recovery','recovery_late'):
                hold=status(old_path,old)
                hold.update(corridor_stop_required=True,corridor_verified=False,
                    hazard_regions={'original':dict(region_odom=[4.,0.],normal_odom=[0.,1.],right_odom=[4.,-2.],capture_timestamp_ns=999000000000)})
                guard.status_cb(hold,clock.value)
                proposal=status(old_path,old)
                proposal['inference_diagnostics'].update(hazard_proof={'original':dict(region_odom=[4.,0.],
                    normal_odom=[0.,1.],right_odom=[4.,-2.],capture_timestamp_ns=1000000000000)},
                    hazard_verification=dict(path_digest=path_digest(old_path),coordinate_frame='odom',
                        verified_at_ns=1000000000000,valid_until_ns=1000500000000))
                guard.status_cb(proposal,clock.value)
            node.camera_guard=guard
            def apply():
                _,_,state=guard.select(clock.value,round(clock.value*1.e9))
                node.camera_guard_selected_state=state
                node.camera_execution_contract=state['execution_contract']
                node.camera_safety_generation=state['safety_generation']
                return state['source']!='stop'
            node.apply_camera_path_guard=apply
            changed=[False]
            def update():
                changed[0]=True
                if kind=='heartbeat':
                    lease=copy.deepcopy(old);lease['expires_ns']+=100000000
                    before=guard.authorization_generation
                    guard.status_cb(status(old_path,lease),clock.value)
                    self.assertEqual(guard.authorization_generation,before)
                else:
                    guard.candidate_cb(new_path);guard.status_cb(status(new_path,new),clock.value)
            validate=ns['validate_solution']
            def checked(*args):
                if kind=='road' and not args[1].get('_shifted'):
                    for pose in args[1]['opt_state_list']:pose[:2,0]+=[2.,-.7]
                    args[1]['_shifted']=True
                validate(*args)
                if kind not in ('late_select','late_lock','late_version','lease_lock','during_check','recovery','recovery_late') and not changed[0]:update()
            ns['validate_solution']=checked
            if kind=='road':
                for item in new['sections']:item['right'][1]=-1.
                node.listener.lookupTransform=lambda a,b,t:((2.,-.7,0.),(0.,0.,0.,1.)) if b=='base_link' else ((0.,0.,0.),(0.,0.,0.,1.))
            if kind=='late_select':
                select=guard.select;calls=[0]
                def delayed(*args):
                    calls[0]+=1
                    if calls[0]==2:clock.value+=.005
                    return select(*args)
                guard.select=delayed
            if kind in ('late_lock','lease_lock','recovery_late'):
                class DelayedLock:
                    pending=False
                    lock=threading.RLock()
                    def __enter__(self):
                        self.lock.acquire()
                        if self.pending:clock.value+=.005;self.pending=False
                        return self
                    def __exit__(self,*_):self.lock.release()
                guard.lock=DelayedLock();check=node.check_camera_execution
                def check_then_delay(*args):
                    check(*args);guard.lock.pending=True
                node.check_camera_execution=check_then_delay
            if kind=='late_version':
                check=node.check_camera_execution
                def check_then_delay_version(*args):
                    check(*args);guard.delay_read_once=True
                node.check_camera_execution=check_then_delay_version
            if kind=='during_check':
                check=ns['check_execution_contract']
                def replace_after_check(*args):
                    check(*args)
                    if not changed[0]:update()
                ns['check_execution_contract']=replace_after_check
            context['guard']=guard
        node,commands=fixture.CommandBoundaryTests().run_loop(contract=old,setup_hook=hook,
            delay=.099 if kind in ('late_select','late_lock','late_version','recovery_late') else 0.,delay_at='solver')
        return node,commands,context

    def test_latest_lower_speed_never_leaks_old_speed_and_does_not_wait_for_sensors(self):
        node,commands,_=self.run_case('lower')
        self.assertEqual(commands[0],0.)
        self.assertIn(.12,commands)
        self.assertNotIn(.3,commands)
        self.assertNotIn('input',node.stop_registry.reasons)
        self.assertIsNone(getattr(node,'planner_fault',None))

    def test_latest_road_region_rechecks_the_actual_old_prediction(self):
        node,commands,_=self.run_case('road')
        self.assertTrue(all(value==0 for value in commands))
        self.assertIsNone(getattr(node,'planner_fault',None))

    def test_compatible_new_path_does_not_starve_planning(self):
        node,commands,context=self.run_case('compatible')
        self.assertEqual(commands,[.3,.3])
        self.assertEqual(node.camera_checked_authorization_id,context['guard'].authorization_id)

    def test_pure_heartbeat_extension_does_not_replace_authority(self):
        _,commands,_=self.run_case('heartbeat')
        self.assertEqual(commands,[.3,.3])

    def test_update_during_swept_check_cannot_publish(self):
        _,commands,_=self.run_case('during_check')
        self.assertEqual(commands[0],0.)
        self.assertIn(.12,commands)

    def test_final_select_delay_cannot_leak_expired_inputs(self):
        node,commands,_=self.run_case('late_select')
        self.assertTrue(all(value==0 for value in commands))
        self.assertIn('input',node.stop_registry.reasons)
        self.assertIsNone(getattr(node,'planner_fault',None))

    def test_final_lock_wait_cannot_leak_expired_inputs(self):
        node,commands,_=self.run_case('late_lock')
        self.assertTrue(all(value==0 for value in commands))
        self.assertIn('input',node.stop_registry.reasons)

    def test_version_check_delay_is_before_final_input_age_check(self):
        node,commands,_=self.run_case('late_version')
        self.assertTrue(all(value==0 for value in commands))
        self.assertIn('input',node.stop_registry.reasons)

    def test_lease_expiration_during_final_lock_wait_is_rejected(self):
        node,commands,_=self.run_case('lease_lock')
        self.assertTrue(all(value==0 for value in commands))
        self.assertNotIn('input',node.stop_registry.reasons)

    def test_actual_run_commits_recovery_only_after_safe_prediction_and_publication_gate(self):
        node,commands,context=self.run_case('recovery')
        self.assertIn(.3,commands)
        self.assertIsNone(context['guard'].corridor_hold_key)
        self.assertEqual(context['guard'].released_hazard_ids,['original'])
        self.assertNotIn('camera',node.stop_registry.reasons)

    def test_expired_publication_cannot_commit_an_otherwise_valid_recovery(self):
        _,commands,context=self.run_case('recovery_late')
        self.assertTrue(all(value==0 for value in commands))
        self.assertIsNotNone(context['guard'].corridor_hold_key)
        self.assertTrue(context['guard'].hazard_regions)
        self.assertFalse(context['guard'].released_hazard_ids)


class RecoveryTests(unittest.TestCase):
    def node(self):
        clock=[1000.]
        poses=[NS(pose=NS(position=NS(x=float(x),y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))) for x in np.arange(0.,20.5,.5)]
        route=NS(header=NS(frame_id='map',seq=0,stamp=NS(to_nsec=lambda:123)),poses=poses)
        guard=CameraPathGuard();guard.original_cb(route)
        cls=n['CameraRightPath'];node=cls.__new__(cls)
        node.path=route;node.cache=n['SegmentCache']();node.cache_geometry={}
        node.applied=None;node.delivered_points=None;node.last_verified_scope={}
        node.cache_pose=lambda _:([0.,0.,0.],0.)
        node.map_anchor=([0.,0.],0.);node.origin=np.zeros(3);node.osm_junctions=np.zeros((0,2))
        node.vehicle_limits=dict(speed=1.5,wheelbase=.67,max_steer=.35,max_steer_rate=.08)
        node.packet=None;node.received=0.;node.timeout=.8;node.max_capture_age_ms=800
        node.control=True;node.previous=None;node.hazards=n['HazardMemory']();node.strategy='segment_cache'
        node.preview=NS(publish=Mock());node.output=NS(publish=guard.candidate_cb)
        node.status=NS(publish=lambda msg:guard.status_cb(json.loads(msg.data),clock[0]))
        node.delivery_dt=.2;node.shift=0.
        return node,guard,clock

    def observe(self,node,clock,keys=range(4,13)):
        node.cache.committed={i:(.1,clock[0]) for i in keys}
        node.cache_geometry={i:dict(points_odom=np.array([[i*.5,.1],[i*.5,3.],[i*.5,-1.9]]),
            route_point_odom=[i*.5,0.],normal_odom=[0.,1.],capture_timestamp_ns=round(clock[0]*1.e9)) for i in keys}
        node.current_proof_geometry=dict(node.cache_geometry)

    def publish(self,node,clock):
        result=copy.deepcopy(node.path)
        active,count=node.cached_result(result,clock[0],.2)
        node.publish_result(result,clock[0],'test',active,count)

    def test_expired_cache_retains_scope_and_recovers_on_fresh_full_reobservation(self):
        node,guard,clock=self.node()
        with patch.dict(n,rospy=NS(Time=NS(now=lambda:NS(to_nsec=lambda:round(clock[0]*1.e9))))):
            self.observe(node,clock);self.publish(node,clock)
            self.assertTrue(node.last_verified_scope)
            clock[0]+=31.
            result=copy.deepcopy(node.path)
            with self.assertRaises(n['CorridorUnsafe']):node.cached_result(result,clock[0],.2)
            node.publish_result(result,clock[0],'cache expired',False,0)
            held=guard.select(clock[0],round(clock[0]*1.e9))[2]
            self.recovery_trace=[copy.deepcopy(held)]
            self.assertEqual(held['source'],'stop');self.assertTrue(held['hazard_regions'])
            self.assertTrue(node.recovery_wait_active)
            clock[0]+=.1
            self.observe(node,clock,keys=range(20,29));self.publish(node,clock)
            self.recovery_trace.append(copy.deepcopy(guard.select(clock[0],round(clock[0]*1.e9))[2]))
            self.assertEqual(guard.select(clock[0],round(clock[0]*1.e9))[2]['source'],'stop')
            clock[0]+=.1
            self.observe(node,clock);self.publish(node,clock)
            selected=guard.select(clock[0],round(clock[0]*1.e9))[2]
            self.recovery_trace.append(copy.deepcopy(selected))
            self.assertEqual(selected['source'],'camera')
            self.assertTrue(selected['recovery_pending'])
            self.assertTrue(guard.hazard_regions) # Planning is not movement permission.
            self.assertFalse(guard.commit_recovery(guard.authorization_generation-1,guard.safety_generation,round(clock[0]*1.e9)))
            self.assertTrue(guard.commit_recovery(guard.authorization_generation,guard.safety_generation,round(clock[0]*1.e9)))
            selected=guard.select(clock[0],round(clock[0]*1.e9))[2]
            self.recovery_trace.append(copy.deepcopy(selected))
            self.assertTrue(selected['released_hazard_ids'])
            self.assertTrue(node.corridor_verified)

    def test_observation_gap_timestamp_does_not_move_forward_on_each_retry(self):
        node,_,clock=self.node()
        with patch.dict(n,rospy=NS(Time=NS(now=lambda:NS(to_nsec=lambda:round(clock[0]*1.e9))))):
            self.observe(node,clock);self.publish(node,clock)
            node.remember_observation_gap('cache lost')
            before=copy.deepcopy(node.hazards.regions)
            clock[0]+=.2;node.remember_observation_gap('still waiting')
            self.assertEqual(node.hazards.regions,before)

    def test_queued_pre_gap_frames_do_not_release_recovery_wait(self):
        node,guard,clock=self.node()
        with patch.dict(n,rospy=NS(Time=NS(now=lambda:NS(to_nsec=lambda:round(clock[0]*1.e9))))):
            self.observe(node,clock);self.publish(node,clock)
            node.remember_observation_gap('cache lost')
            clock[0]+=.1;self.observe(node,clock)
            for row in node.cache_geometry.values():row['capture_timestamp_ns']=999990000000
            self.publish(node,clock)
            self.assertTrue(node.corridor_stop_required)
            self.assertEqual(guard.select(clock[0],round(clock[0]*1.e9))[2]['source'],'stop')

    def test_tf_cache_clear_preserves_scope_and_can_recover(self):
        node,guard,clock=self.node()
        node.lock=threading.RLock();node.last_tick=clock[0]-.2
        with patch.dict(n,rospy=NS(Time=NS(now=lambda:NS(to_nsec=lambda:round(clock[0]*1.e9)))),
                time=NS(monotonic=lambda:clock[0])):
            self.observe(node,clock);self.publish(node,clock)
            node.cache_pose=Mock(side_effect=RuntimeError('history TF missing'))
            node.tick(None)
            self.assertFalse(node.cache.committed)
            self.assertTrue(node.hazards.regions)
            self.assertEqual(guard.select(clock[0],round(clock[0]*1.e9))[2]['source'],'stop')
            clock[0]+=.1;node.cache_pose=lambda _:([0.,0.,0.],0.)
            self.observe(node,clock);self.publish(node,clock)
            self.assertTrue(guard.select(clock[0],round(clock[0]*1.e9))[2]['recovery_pending'])

    def test_truly_unlocated_legacy_stop_requires_explicit_manual_check(self):
        node,guard,clock=self.node()
        node.applied=np.full(len(node.path.poses),.1)
        node.delivered_points=np.array([[p.pose.position.x,.1] for p in node.path.poses])
        with patch.dict(n,rospy=NS(Time=NS(now=lambda:NS(to_nsec=lambda:round(clock[0]*1.e9))))):
            result=copy.deepcopy(node.path)
            with self.assertRaises(n['CorridorUnsafe']):node.cached_result(result,clock[0],.2)
            node.publish_result(result,clock[0],'no stored scope',False,0)
            selected=guard.select(clock[0],round(clock[0]*1.e9))[2]
            self.assertEqual(selected['reason'],'camera_recovery_scope_unknown_manual_check')


if __name__=='__main__':unittest.main()
