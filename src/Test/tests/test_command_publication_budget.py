"""Publication age vs current input outage: actual control loop, fake I/O."""
import unittest
import json
from pathlib import Path
import test_followup_safety as fixture

FIXTURE=Path(__file__).resolve().parents[1]/'fixtures/command_publication_nov10.json'


def recorded_case(case, limit, fault=None):
    """Real loop/checks with recorded timing and synthetic valid solver output.

    Fresh callbacks continue during computation; the solved tuple remains old.
    No ROS master, no vehicle, no claim about a new closed-loop trajectory.
    """
    input_age=case['input_ages'];prep=case['preparation_ms']/1000
    solver=case['solver_ms']/1000
    post=case['publish_scan_age_sec']-input_age['scan_measurement_age_ms']/1000-solver
    state={}
    def configure(node,clock,namespace):
        base=clock.value;bound=base+prep
        # Immutable arrivals, not timestamps regenerated at final inspection.
        # Verification can supply the actual recorded scan/LIO schedule.
        scan_arrivals=case.get('current_scan_arrivals',[(i*.1,i*.1-.01) for i in range(31)])
        lio_arrivals=case.get('current_lio_arrivals',[(i*.1,i*.1-.005) for i in range(31)])
        node.result_publish_timeout=limit;node.result_compute_timeout=.50
        node.last_command_monotonic=base-.01;node.last_command_was_nonzero=True
        node.safety_scan=(node.obstacle_points.copy(),round((bound-input_age['scan_measurement_age_ms']/1000)*1.e9),
            bound-input_age['scan_receive_age_ms']/1000)
        class Stamp:
            def __init__(self,value):self.value=value
            def to_nsec(self):return round(self.value*1.e9)
            def to_sec(self):return self.value
        def pose(*args):
            if fault=='tf_missing' and clock.value>bound:raise LookupError('test TF stream missing')
            if clock.value<=bound+1.e-7:value=min(clock.value,bound-input_age['state_age_ms']/1000)
            else:value=base+max((p for p in lio_arrivals if p[0]<=clock.value-base),key=lambda p:p[0])[1]
            if fault=='lio_stale' and clock.value>bound:value=clock.value-.51
            return Stamp(value)
        node.listener.getLatestCommonTime=pose
        def refresh():
            if clock.value<=bound+1.e-7:return
            if fault=='scan_stale':return
            # Captured arrival stamps, never a rewrite of checked_scan.
            arrived,source=max((p for p in scan_arrivals if p[0]<=clock.value-base),key=lambda p:p[0])
            node.safety_scan=(node.obstacle_points.copy(),round((base+source)*1.e9),base+arrived)
        node.refresh_scan_snapshot=refresh
        node.scan_is_healthy=lambda now:(not (fault=='scan_stale' and clock.value>bound),'test scan stream')
        node.update_adaptive_adjust_parameters=lambda:setattr(clock,'value',clock.value+prep)
        original=namespace['validate_solution']
        def validate(*args):
            original(*args);clock.value+=max(0.,post)
            if fault=='manual':node.control_mode=1;node.command_route_version+=1
            if fault=='route':node.command_route_version+=1
            if fault=='localization_unsafe':node.localization_startup_status=(clock.value,dict(localization_safe=False))
            if fault=='generation':
                node.localization_startup_status=(clock.value,dict(localization_safe=True,
                    reference_frame=dict(instance_id='bridge',odom_epoch=1)))
        namespace['validate_solution']=validate
        if fault=='generation':node.localization_startup_status=(clock.value,dict(localization_safe=True,
            reference_frame=dict(instance_id='bridge',odom_epoch=0)))
        original_call=node.neupan_planner.__class__.__call__
        def native(self,*args):
            action,info=original_call(self,*args)
            action[1,0]=.12;info['vel_tensor'][1,:]=.12
            return action,info
        node.neupan_planner.__class__.__call__=native
        events=[];messages=[]
        node.runtime_pub.publish=lambda payload:events.append(json.loads(payload))
        original_publish=node.vel_pub.publish
        def publish(message):
            messages.append(dict(mono=clock.value,speed=message.linear.x,steering=message.angular.z))
            original_publish(message)
        node.vel_pub.publish=publish
        state.update(events=events,commands=messages)
    node,commands=fixture.CommandBoundaryTests().run_loop(delay=solver,delay_at='solver',setup_hook=configure)
    state.update(node=node,values=commands)
    return state


class PublicationBudgetTests(unittest.TestCase):
    def test_healthy_stream_does_not_zero_and_resolve_at_505ms_snapshot_age(self):
        def configure(node,clock,namespace):
            node.result_publish_timeout=.60
            node.result_compute_timeout=.50
        node,commands=fixture.CommandBoundaryTests().run_loop(delay=.105,delay_at='solver',
            newer_scan=True,setup_hook=configure)
        self.assertTrue(commands)
        self.assertNotIn(0.,commands,'Healthy fresh stream: 505 ms snapshot should publish bounded native result, not STOP/recompute')

    def test_all_twelve_recorded_critical_ages_keep_native_action_without_retry(self):
        events=json.loads(FIXTURE.read_text())['events'];self.assertEqual(len(events),12)
        for case in events:
            with self.subTest(event=case['event']):
                old=recorded_case(case,.5);new=recorded_case(case,.6)
                self.assertTrue(any(e['command_source']=='calculation_retry' for e in old['events']))
                self.assertTrue(new['commands'])
                self.assertTrue(all(c['speed']==.8 and c['steering']==.12 for c in new['commands']))
                self.assertFalse(any(e['command_source']=='calculation_retry' for e in new['events']))
                self.assertEqual(new['node'].neupan_planner.calls,len(new['commands']))
                self.assertGreater(new['events'][0]['publish_input_ages']['scan_measurement_age_ms'],500.)
                self.assertLessEqual(new['events'][0]['timing']['publication_current_input_ages']['scan_measurement_age_ms'],500.)

    def test_outages_authority_and_generation_still_reject_candidate(self):
        case=json.loads(FIXTURE.read_text())['events'][0]
        for fault in ('scan_stale','lio_stale','tf_missing','manual','route','localization_unsafe','generation'):
            with self.subTest(fault=fault):
                result=recorded_case(case,.6,fault)
                self.assertTrue(result['commands'])
                rejected=result['commands'][:1] if fault=='generation' else result['commands']
                self.assertTrue(all(c['speed']==0. and c['steering']==0. for c in rejected))
                self.assertIsNone(getattr(result['node'],'planner_fault',None))

    def test_overlong_635ms_source_582ms_calculation_remains_rejected(self):
        case=dict(preparation_ms=5.,solver_ms=560.,publish_scan_age_sec=.635,
            input_ages=dict(scan_measurement_age_ms=58.,scan_receive_age_ms=40.,state_age_ms=20.))
        result=recorded_case(case,.6)
        self.assertTrue(result['commands'])
        self.assertTrue(all(c['speed']==0. for c in result['commands']))
        self.assertTrue(any('calculation budget expired' in e.get('reason','') for e in result['events']))

    def test_diagnostic_path_work_occurs_after_native_publication(self):
        context={}
        def configure(node,clock,namespace):
            node.result_publish_timeout=.6;node.result_compute_timeout=.5
            original=node.generate_path_msg
            count=[0]
            def path(*args):
                count[0]+=1
                if count[0]==2:clock.value+=.25
                return original(*args)
            node.generate_path_msg=path
            node.runtime_pub.publish=lambda payload:context.update(last=json.loads(payload))
        node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=configure)
        self.assertEqual(commands[0],.8)
        self.assertFalse(getattr(node,'planner_fault',None))

    def test_only_one_immediate_retry_never_renews_previous_command(self):
        context={}
        def configure(node,clock,namespace):
            node.result_publish_timeout=.6;node.result_compute_timeout=.05
            node.last_command_monotonic=clock.value-.01;node.last_command_was_nonzero=True
            context['original_stamp']=node.last_command_monotonic
            context['events']=[]
            node.runtime_pub.publish=lambda payload:context['events'].append(json.loads(payload))
        node,commands=fixture.CommandBoundaryTests().run_loop(delay=.08,delay_at='solver',
            newer_scan=True,setup_hook=configure)
        events=[e for e in context['events'] if e['command_source']=='calculation_retry']
        self.assertEqual(len(events),2)
        self.assertFalse(events[0]['command_published']);self.assertTrue(events[0]['retry_scheduled'])
        self.assertTrue(events[1]['command_published']);self.assertTrue(events[1]['retry_exhausted'])
        self.assertEqual(commands,[0.])
        self.assertEqual(node.last_command_monotonic,context['original_stamp'])
        self.assertNotIn('input',node.stop_registry.reasons)


class LaunchBudgetTests(unittest.TestCase):
    def test_default_and_candidate_do_not_change_current_input_or_mux_deadlines(self):
        import roslaunch
        root=Path(__file__).resolve().parents[3]
        path=root/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch'
        for camera in (False,True):
            for age in (None,.6):
                args=['outdoor_mode:=true','enable_camera_right_path:='+str(camera).lower(),
                    'enable_camera_right_control:='+str(camera).lower()]
                if age:args+=['neupan_result_publish_timeout:=0.60','neupan_result_compute_timeout:=0.50']
                config=roslaunch.config.load_config_default([(str(path),args)],None,verbose=False)
                params={k:v.value for k,v in config.params.items()}
                values=lambda suffix:[v for k,v in params.items() if k.endswith(suffix)]
                self.assertEqual(values('/result_publish_timeout'),[age or .5])
                self.assertEqual(values('/result_compute_timeout'),[.5])
                self.assertEqual(values('/auto_timeout'),[.5])
                self.assertEqual(values('/auto_command_timeout'),[.5])
                self.assertEqual(values('/localization_safety_timeout'),[1.])
                self.assertEqual(values('/scan_timeout'),[.5])

