"""P0-D/P1-E: real library empty-input contract and actual run+scan binding."""
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np
import test_followup_safety as fixture
from test_startup_recovery import scan_node,message,Stamp
from neupan_safety import planner_obstacle_input,validate_solution,trajectory_clearance

ROOT=Path(__file__).resolve().parents[3]


def binding_case(kind,preparation_sec=.35,recovering=False):
    context={}
    def hook(node,clock,ns):
        scan,_,_=scan_node()
        # Use exact production scan methods alongside the exact run() fixture.
        scan.refresh_scan_snapshot.__func__.__globals__.update(time=ns['time'],rospy=ns['rospy'])
        for name,value in vars(scan).items():setattr(node,name,value)
        for name in ('scan_callback','refresh_scan_snapshot','convert_scan_snapshot','bind_scan_snapshot',
                     'mark_scan_fault','mark_scan_valid','scan_is_healthy'):
            setattr(node,name,getattr(type(scan),name).__get__(node))
        node.listener=NS(lookupTransform=lambda *_:((0.,0.,0.),(0.,0.,0.,1.)),getLatestCommonTime=lambda *_:ns['rospy'].Time.now())
        old=message(clock.value-.01,(math.inf,)*3 if kind=='empty' else (3.,4.,5.))
        node.scan_callback(old)
        prepared=[False];generate=node.generate_path_msg
        if recovering:
            node.scan_recovery_stable_sec=.5
            def restore():
                if 'input' not in node.stop_registry.reasons:
                    node.stop_registry.set('input',clock.value-1.,'fresh inputs stable')
                    node.input_good_since=clock.value-.6
            node.apply_pending_goal_request=restore
        def prepare(states):
            if not prepared[0]:
                prepared[0]=True
                if kind!='empty':clock.value+=preparation_sec
                if kind not in ('no_new','empty'):
                    node.scan_callback(message(clock.value-.01,(6.,7.,8.)))
                    if kind=='out_of_order':node.scan_callback(message(clock.value-.10,(1.,2.,3.)))
                if kind=='tf_delay':
                    lookup=node.listener.lookupTransform
                    def wait(target,source,stamp):
                        if source=='laser' and stamp.to_nsec()==round((clock.value-.01)*1.e9):raise LookupError('pending same-epoch TF')
                        return lookup(target,source,stamp)
                    node.listener.lookupTransform=wait
            return generate(states)
        node.generate_path_msg=prepare
        solve=node.neupan_planner.__class__.__call__;inputs=[]
        class DelayedPlanner(type(node.neupan_planner)):
            def __call__(self,state,points):
                inputs.append(dict(points=None if points is None else points.copy(),stamp=node.safety_scan[1]))
                if kind!='empty':clock.value+=.60 if kind=='during_solve_expired' else .20
                if kind.startswith('during_solve'):node.scan_callback(message(clock.value-.01,(9.,10.,11.)))
                return solve(self,state,points)
        node.neupan_planner.__class__=DelayedPlanner
        context.update(inputs=inputs,old_stamp=old.header.stamp.to_nsec())
    node,commands=fixture.CommandBoundaryTests().run_loop(setup_hook=hook)
    return node,commands,context


def real_library_sequence():
    from neupan import neupan
    config=json.loads((ROOT/'tracker_logs/review_20261027/field_startup_review/rosbag20261027_162947_config.json').read_text())['node_parameters']
    planner=neupan.init_from_yaml(config['config_file'],pan={'dune_checkpoint':config['dune_checkpoint']})
    planner.set_initial_path([np.array([[x],[0.],[0.],[1.]]) for x in np.arange(0,20.5,.5)])
    rows=[]
    for name,points in [('finite_before',np.array([[10.,11.],[4.,5.]])),('empty_after_obstacle',np.empty((2,0))),
                        ('empty_continuous',np.empty((2,0))),('finite_after',np.array([[10.,11.],[4.,5.]]))]:
        if name=='empty_after_obstacle':
            # Simulate the previous DUNE frame's obstacle-stop cache explicitly.
            planner.pan.dune_layer.min_distance=-.1
        adapted=planner_obstacle_input(planner,points)
        action,info=planner(np.zeros((3,1)),adapted)
        validate_solution(action,info,planner.robot,planner.dt)
        clearance=trajectory_clearance(np.hstack(info['opt_state_list']),points,planner.robot.vertices)
        rows.append(dict(name=name,input_none=adapted is None,action=np.asarray(action).reshape(-1).tolist(),
            stop=bool(info['stop']),minimum_is_infinite=math.isinf(float(planner.min_distance)),
            no_obs=planner.pan.no_obs,clearance=clearance))
    return rows


class EmptyScanTests(unittest.TestCase):
    def test_actual_run_repeated_valid_empty_does_not_fault(self):
        node,commands,context=binding_case('empty')
        self.assertEqual(node.neupan_planner.calls,2);self.assertTrue(all(v>0 for v in commands))
        self.assertTrue(all(row['points'] is None for row in context['inputs']))
        self.assertEqual(node.safety_scan[0].shape,(2,0));self.assertIsNone(getattr(node,'planner_fault',None))

    def test_self_filter_empty_retains_safety_epoch_and_adapts_to_none(self):
        node,_,_=scan_node();node.scan_self_filter=True
        node.scan_callback(message(ranges=(.2,.25,.3)));node.refresh_scan_snapshot()
        full,points,_=node.bind_scan_snapshot()
        self.assertEqual(full[0].shape,(2,0));self.assertEqual(full[1],message().header.stamp.to_nsec())
        self.assertIsNone(planner_obstacle_input(NS(),points))

    def test_nonfinite_points_are_not_adapted_to_no_obstacles(self):
        for points in (np.array([[math.nan],[0.]]),np.array([[math.inf],[0.]]),np.zeros((3,0))):
            with self.assertRaises(ValueError):planner_obstacle_input(NS(),points)

    def test_real_library_nonempty_empty_nonempty_and_old_stop_cache(self):
        rows=real_library_sequence()
        self.assertTrue(all(not row['stop'] and row['action'][0]>0 for row in rows))
        self.assertTrue(all(not row['no_obs'] for row in rows))
        self.assertTrue(rows[1]['minimum_is_infinite']);self.assertTrue(rows[2]['input_none'])
        self.assertFalse(rows[3]['input_none']);self.assertFalse(rows[3]['minimum_is_infinite'])


class PreparationBindingTests(unittest.TestCase):
    def test_550ms_preparation_refreshes_scan_and_pose_before_health(self):
        node,commands,context=binding_case('new',.55)
        # Preparation alone exhausted the new explicit 0.50s total budget.
        # Rebind without an input-outage recovery, then solve the fresh cycle.
        self.assertEqual(commands[0],0.);self.assertIn(.8,commands)
        self.assertGreater(context['inputs'][0]['stamp'],context['old_stamp'])
        self.assertLessEqual(node.publish_input_ages['scan_measurement_age_ms'],500.)
        self.assertNotIn('input',node.stop_registry.reasons)

    def test_550ms_preparation_does_not_use_old_pose_in_recovery_gate(self):
        node,commands,_=binding_case('new',.55,recovering=True)
        self.assertEqual(commands[0],0.);self.assertIn(.8,commands)
        self.assertNotIn('input',node.stop_registry.reasons)

    def test_550ms_without_new_or_convertible_frame_still_stops(self):
        for kind in ('no_new','tf_delay'):
            with self.subTest(kind=kind):
                node,commands,_=binding_case(kind,.55)
                self.assertTrue(commands);self.assertTrue(all(v==0 for v in commands))
                self.assertIn('input',node.stop_registry.reasons)
                self.assertIsNone(getattr(node,'planner_fault',None))

    def test_550ms_new_frame_waiting_tf_still_respects_existing_recovery(self):
        node,commands,_=binding_case('tf_delay',.55,recovering=True)
        self.assertTrue(all(v==0 for v in commands));self.assertIn('input',node.stop_registry.reasons)

    def test_prepare_new_scan_is_consumed_before_solve_and_final_check(self):
        node,commands,context=binding_case('new',.25)
        self.assertGreater(commands[0],0)
        self.assertGreater(context['inputs'][0]['stamp'],context['old_stamp'])
        self.assertLess(node.cycle_timing['solve_input_ages']['scan_measurement_age_ms'],500.)
        self.assertEqual(node.diagnostic_safety_scan[1],context['inputs'][-1]['stamp'])

    def test_no_new_scan_does_not_renew_the_old_one(self):
        node,commands,_=binding_case('no_new')
        self.assertTrue(all(v==0 for v in commands));self.assertIn('input',node.stop_registry.reasons)

    def test_new_scan_waiting_tf_does_not_claim_a_fresh_converted_frame(self):
        node,commands,context=binding_case('tf_delay')
        self.assertEqual(commands[0],0.)
        self.assertEqual(context['inputs'][0]['stamp'],context['old_stamp'])

    def test_out_of_order_does_not_replace_newest_snapshot(self):
        _,commands,context=binding_case('out_of_order',.25)
        self.assertGreater(commands[0],0)
        self.assertGreater(context['inputs'][0]['points'][0,0],5.)

    def test_scan_arriving_during_solve_checks_current_stream_without_renewing_result(self):
        node,commands,context=binding_case('during_solve',.25)
        self.assertGreater(commands[0],0)
        self.assertEqual(node.diagnostic_safety_scan[1],context['inputs'][-1]['stamp'])
        self.assertGreater(node.safety_scan[1],node.diagnostic_safety_scan[1])
        self.assertEqual(node.publish_input_ages['scan_stamp_ns'],node.diagnostic_safety_scan[1])

    def test_fresh_scan_during_slow_solve_does_not_rescue_expired_result(self):
        node,commands,_=binding_case('during_solve_expired')
        self.assertTrue(all(v==0 for v in commands))
        self.assertIn('calculation_retry',node.stop_registry.reasons)
        self.assertNotIn('input',node.stop_registry.reasons)
        self.assertIsNone(getattr(node,'planner_fault',None))


def report(directory):
    rows={}
    for kind in ('empty','new','no_new','tf_delay','out_of_order','during_solve','during_solve_expired'):
        node,commands,context=binding_case(kind)
        rows[kind]=dict(commands_mps=commands,planner_fault=getattr(node,'planner_fault',None),
            bound_stamps_ns=[r['stamp'] for r in context['inputs']],
            timing=node.cycle_timing,publish_input_ages=getattr(node,'publish_input_ages',None))
    for kind,recovering in (('new',False),('new',True),('no_new',False),('tf_delay',False),('tf_delay',True)):
        node,commands,_=binding_case(kind,.55,recovering=recovering)
        rows['prepare550_'+kind+('_recovery' if recovering else '')]=dict(commands_mps=commands,
            planner_fault=getattr(node,'planner_fault',None),timing=node.cycle_timing,
            stop_reasons=node.stop_registry.snapshot(),publish_input_ages=getattr(node,'publish_input_ages',None))
    source=[ROOT/'src/neupan_ros/src'/name for name in ('neupan_core.py','neupan_safety.py')]
    result=dict(scope='Actual production run+scan methods with fake clock/publishers; real external library static scene sequence. No ROS nodes or vehicle outputs; not physical closed-loop validation.',
        cases=rows,real_library=real_library_sequence(),
        implementation_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source})
    directory.mkdir(parents=True,exist_ok=True)
    (directory/'scan_binding.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    lines=['# 空扫描接口与准备后扫描绑定验证','',result['scope'],'',
        '| 场景 | 模拟发布速度m/s | 内部故障 |','| --- | --- | --- |']
    for name,row in rows.items():lines.append(f"| {name} | {row['commands_mps']} | {bool(row['planner_fault'])} |")
    lines+=['','new为350ms准备；prepare550为550ms准备及已有输入恢复分支，均再求解200ms。扫描/位姿刷新位于健康及恢复判断前，no_new/tf_delay仍保留真实过期停车。',
        '真实库依次测试非空→空→空→非空，并在第一空帧前注入旧障碍最小距离缓存；均无内部异常，空帧不沿用旧障碍停车，后续非空仍启用障碍处理。',
        '不改变0.5s时效、不改外部库源码、不绕过独立危险/故障保持。完整回归结果见unit_regression.json。需重启原规划节点加载，实车动态表现待验收。']
    (directory/'report.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    if len(sys.argv)==3 and sys.argv[1]=='--report-dir':report(Path(sys.argv[2]))
    else:unittest.main()
