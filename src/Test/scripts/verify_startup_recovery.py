#!/usr/bin/env python3
"""Offline Oct27 full-part TF queue replay + real run() rejection probes.

Never starts a ROS node/master or publishes control. This is NOT a closed-loop
drive: recorded pose is unchanged by the new commands and the bag TF arrival
order only approximates the vehicle listener schedule.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/Test/tests'))
from test_startup_recovery import scan_node,RecoveryTests,Stamp
from osm_outdoor_start_connection import OutdoorTrackingGuard,resample_forward_reference


def distribution(values):
    return dict(count=len(values),max=float(max(values)),p95=float(np.percentile(values,95))) if values else None


def probe_solver(route,pose,scan,cfg):
    """Static recorded scene, actual optimizer; not a vehicle closed loop."""
    import time
    from neupan import neupan
    from neupan_safety import validate_solution,trajectory_clearance
    planner=neupan.init_from_yaml(cfg['config_file'],pan={'dune_checkpoint':cfg['dune_checkpoint']})
    planner.set_initial_path([p.reshape(4,1) for p in route])
    rows=[]
    for attempt in range(3):
        begin=time.monotonic()
        try:
            action,info=planner(np.asarray(pose).reshape(3,1),scan)
            validate_solution(action,info,planner.robot,planner.dt)
            checked=trajectory_clearance(np.hstack(info['opt_state_list']),scan,planner.robot.vertices)
            rows.append(dict(attempt=attempt,action=np.asarray(action).reshape(-1).tolist(),
                outcome='checked_candidate',clearance=checked,elapsed_ms=(time.monotonic()-begin)*1000))
        except Exception as error:
            rows.append(dict(attempt=attempt,outcome=type(error).__name__,reason=str(error),elapsed_ms=(time.monotonic()-begin)*1000))
    return dict(scope='Three real optimizer calls at ONE recorded static scene, native action; geometric check is diagnostic only; no pose integration or camera authority/publication emulation. Elapsed time is offline only.',
        input_pose=pose,scan_points=scan.shape[1],cases=rows)


def replay(session,solve=False):
    import rosbag,rospy,tf2_py
    node,clock,_=scan_node();buffer=tf2_py.BufferCore(rospy.Duration(20.))
    def lookup(target,source,stamp):
        try:t=buffer.lookup_transform_core(target,source,rospy.Time(stamp.to_nsec()//10**9,stamp.to_nsec()%10**9))
        except Exception as error:raise LookupError(str(error))
        p,q=t.transform.translation,t.transform.rotation
        return (p.x,p.y,p.z),(q.x,q.y,q.z,q.w)
    node.listener.lookupTransform=lookup
    node.listener.getLatestCommonTime=lambda *_:Stamp(0.) # unavailable in this adapter, not a false epoch
    node.quat_to_yaw_list=lambda q:math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    cfg=json.loads((ROOT/'tracker_logs/review_20261027/field_startup_review'/f'{session}_config.json').read_text())['node_parameters']
    for name in ('map_frame','base_frame','lidar_frame','scan_downsample','scan_timeout','scan_transform_timeout'):
        if name in cfg:setattr(node,name,cfg[name])
    for name in ('scan_range','scan_angle_range'):
        setattr(node,name,[float(v) for v in cfg[name].split()])
    node.scan_self_filter=bool(cfg.get('scan_self_filter',False));node.flip_angle=bool(cfg.get('flip_angle',False))
    statuses=Counter();waits=[];ages=[];chosen=set();immediate_misses=0;scans=0;post_goal_scans=0;unavailable=[]
    goal=False;next_tick=None;route=None;pose=None;tracking=OutdoorTrackingGuard();tracking_reports=[];solve_input=[]
    def tick():
        node.refresh_scan_snapshot()
        if not goal:return
        healthy=node.scan_is_healthy(Stamp(clock.value))[0]
        statuses['healthy_ticks' if healthy else 'unavailable_ticks']+=1
        if not healthy:
            unavailable.append(dict(stamp_sec=clock.value,reason=node.scan_is_healthy(Stamp(clock.value))[1],
                tf_state=node.scan_tf_diagnostics['state'],last_capture_age_ms=node.scan_tf_diagnostics['selected_age_ms']))
        if healthy:
            snapshot=node.safety_scan
            ages.append((clock.value-snapshot[1]/1.e9)*1000)
            if snapshot[1] not in chosen:
                chosen.add(snapshot[1]);waits.append((clock.value-snapshot[2])*1000)
            if route is not None and pose is not None and not solve_input:
                solve_input.extend([route.copy(),list(pose),snapshot[0].copy()])
        if route is not None and pose is not None:
            report=tracking.check(route,pose,0,clock.value)
            tracking_reports.append(report)
    files=sorted((ROOT/'rosbags'/session).glob('*.bag'))
    if not files:raise FileNotFoundError(session)
    for file in files:
        with rosbag.Bag(str(file)) as bag:
            for topic,msg,stamp in bag.read_messages(topics=['/tf','/tf_static','/scan','/move_base_simple/goal','/osm_global_path','/odometry/global']):
                t=stamp.to_sec()
                if next_tick is None:next_tick=t
                while next_tick<t:
                    clock.value=next_tick;tick();next_tick+=.02
                clock.value=t
                if topic in ('/tf','/tf_static'):
                    for transform in msg.transforms:
                        if topic=='/tf_static':buffer.set_transform_static(transform,'recorded')
                        else:buffer.set_transform(transform,'recorded')
                elif topic=='/move_base_simple/goal':goal=True
                elif topic=='/scan':
                    scans+=1
                    if goal:post_goal_scans+=1
                    try:lookup(node.map_frame,msg.header.frame_id,msg.header.stamp)
                    except LookupError:
                        if goal:immediate_misses+=1
                    node.scan_callback(msg)
                elif topic=='/osm_global_path':
                    route=resample_forward_reference([[p.pose.position.x,p.pose.position.y,
                        node.quat_to_yaw_list([p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w]),1.]
                        for p in msg.poses],.5)
                elif topic=='/odometry/global':
                    p,q=msg.pose.pose.position,msg.pose.pose.orientation
                    pose=[p.x,p.y,node.quat_to_yaw_list([q.x,q.y,q.z,q.w])]
    tick()
    result=dict(session=session,files=[p.name for p in files],scans_all_parts=scans,
        post_goal_scans=post_goal_scans,post_goal_immediate_tf_misses=immediate_misses,
        planning_ticks=dict(statuses),selected_unique_frames=len(chosen),selected_age_ms=distribution(ages),
        unavailable_ticks_detail=unavailable,
        selected_receive_to_tick_ms=distribution(waits),tracking_latched_ticks=sum(r['latched'] for r in tracking_reports),
        tracking_initial=tracking_reports[0] if tracking_reports else None,
        tracking_final=tracking_reports[-1] if tracking_reports else None)
    if solve and solve_input:result['static_real_solver']=probe_solver(*solve_input,cfg)
    print(json.dumps(result,ensure_ascii=False),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--bags',action='store_true');parser.add_argument('--solve',action='store_true');args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    cases={}
    for kind in ('blocked','removed','slow','startup','nan'):
        node,commands=RecoveryTests().run_case(kind)
        cases[kind]=dict(commands_mps=commands,solver_calls=node.neupan_planner.calls,
            planner_fault=getattr(node,'planner_fault',None),stop_reasons=node.stop_registry.snapshot(),
            tracking=getattr(node,'outdoor_tracking_status',None))
    files=[ROOT/'src/neupan_ros/src'/p for p in ('neupan_core.py','neupan_safety.py')]
    files+=[ROOT/'src/osm_map_handler/src'/p for p in ('osm_outdoor_start_connection.py','osm_camera_path_refinement.py')]
    result=dict(scope=__doc__,cases=cases,
        bags=[replay('rosbag20261027_'+tag,args.solve) for tag in ('162947','163253')] if args.bags else [],
        implementation_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files})
    (args.output_dir/'startup_recovery.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    lines=['# 起步与候选拒绝恢复验证','',
        '真实生产方法配假时钟/发布器；录包使用全部分片、记录的TF到达顺序和原测量时刻，以50 Hz离线调用生产扫描队列。无ROS节点或车辆控制。',
        '不是完整NeuPAN求解/动态闭环，记录位姿不随新命令移动；不能证明实车收敛、避障和制动已验收。','',
        '| 场景 | 发布速度 m/s | 求解次数 | 内部故障保持 |','| --- | --- | ---: | --- |']
    for name,row in cases.items():lines.append(f"| {name} | {row['commands_mps']} | {row['solver_calls']} | {bool(row['planner_fault'])} |")
    lines+=['','blocked=持续近车碰撞；removed=首轮冲突后移走；slow=完整复查合格的降速方案；startup=1.54 m起步偏差；nan=真实数值错误。','',
        '## 两趟故障bag的扫描与跟踪规则复核','']
    for row in result['bags']:
        lines.append(f"- {row['session']}：{len(row['files'])}分片，目标后{row['post_goal_scans']}扫描，即时TF失败{row['post_goal_immediate_tf_misses']}次；新版50 Hz检查{row['planning_ticks']}，仅偏差触发的锁存{row['tracking_latched_ticks']}次。扫描最大年龄{row['selected_age_ms']}。")
        if 'static_real_solver' in row:
            lines.append('  单个记录场景的3次真实优化器调用（不是闭环）：'+json.dumps(row['static_real_solver']['cases'],ensure_ascii=False))
    lines+=['','详细输入范围、源码指纹和结果见startup_recovery.json；完整回归见unit_regression.json。',
        'GPU/外部NeuPAN优化器、定位预算、雷达配置未更改。需重启原规划/靠右节点加载，再封闭场地低速验收。']
    (args.output_dir/'report.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':main()
