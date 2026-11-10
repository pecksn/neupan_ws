#!/usr/bin/env python3
"""Oct28 read-only recorded-input checks. No ROS master/node/control outputs.

Camera replay calls actual producer+consumer with original TF/message arrival
order. Obstacle probes are static re-solves, NOT exact rejected-solution replay:
the original rejected controls/warm starts were not recorded. Neither changes
recorded pose using new commands or proves dynamic closed-loop performance.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/osm_map_handler/src'))
sys.path.insert(0,str(ROOT/'src/neupan_ros/src'))


def ordered_bag_files(directory):
    """Use recorded start times: lexical _10 before _2 corrupts TF history."""
    import rosbag
    files=[]
    for file in Path(directory).glob('*.bag'):
        with rosbag.Bag(str(file)) as bag:files.append((bag.get_start_time(),file.name,file))
    return [file for _,_,file in sorted(files)]


def tf_environment(clock):
    import rospy,tf,tf2_py
    buffer=tf2_py.BufferCore(rospy.Duration(30.));stamps={}
    class Listener:
        def lookupTransform(self,target,source,stamp):
            try:t=buffer.lookup_transform_core(target,source,stamp)
            except Exception as error:raise tf.Exception(str(error))
            p,q=t.transform.translation,t.transform.rotation
            return (p.x,p.y,p.z),(q.x,q.y,q.z,q.w)
        def getLatestCommonTime(self,target,source):
            needed=[stamps.get(('map','odom')),stamps.get(('odom','base_link'))]
            if any(s is None for s in needed):raise tf.Exception('recorded dynamic chain not ready')
            return min(needed,key=lambda s:s.to_nsec())
    def feed(message,static=False):
        for t in message.transforms:
            if static:buffer.set_transform_static(t,'recorded')
            else:
                buffer.set_transform(t,'recorded');stamps[(t.header.frame_id,t.child_frame_id)]=t.header.stamp
    class Time:
        def __new__(cls,sec=0,nsec=0):return rospy.Time(sec,nsec)
        @staticmethod
        def now():return rospy.Time(clock.ns//10**9,clock.ns%10**9)
    return Listener(),feed,Time


def camera_replay(directory=None):
    import rosbag,rospy,tf
    from std_msgs.msg import String
    import osm_camera_path_refinement as module
    from osm_camera_path_guard import CameraPathGuard,path_digest
    directory=Path(directory) if directory is not None else ROOT/'rosbags/rosbag20261028_093624'
    if not directory.is_absolute():directory=ROOT/'rosbags'/directory
    files=ordered_bag_files(directory)
    if not files:raise FileNotFoundError('Oct28 camera bag missing')
    with rosbag.Bag(str(files[0])) as bag:start=bag.get_start_time()
    clock=NS(ns=round(start*1.e9));listener,feed,Time=tf_environment(clock)
    guard=CameraPathGuard(require_initial_reference=True);rows=[];health_rows=[];route_changes=[];last_digest=[None];recorded_runtime=[None,None]
    params={'~strategy':'segment_reference','~control_enabled':True,
        '~camera_info_file':str(ROOT/'src/usb_camera/config/usb_camera.yaml'),
        '~extrinsics_file':str(ROOT/'src/usb_camera/config/livox_camera_extrinsics.yaml'),
        '~vehicle_config_file':str(ROOT/'src/neupan_ros/example/lxw_car/config/neupan_config_outdoor.yaml')}
    def output(path):
        guard.candidate_cb(path)
        digest=path_digest(path)
        if digest!=last_digest[0]:route_changes.append(clock.ns);last_digest[0]=digest
    def status(message):
        payload=json.loads(message.data);guard.status_cb(payload,clock.ns/1.e9)
        state=guard.select(clock.ns/1.e9,clock.ns)[2]
        detail=payload.get('inference_diagnostics') or {};final=detail.get('final_path',{})
        original_digest=guard.original_digest
        changed_geometry=bool(payload.get('control_path_digest') and original_digest
                              and payload.get('control_path_digest')!=original_digest)
        matched=state['source']=='camera' and state.get('matched_path_digest')==payload.get('control_path_digest')
        rows.append(dict(stamp_ns=clock.ns,active=payload['active'],producer_reason=payload['reason'],
            reference_shift_m=payload.get('shift_m'),
            source=state['source'],consumer_reason=state['reason'],
            version=detail.get('reference_version'),producer_decision=final,
            reference_accepted=detail.get('reference_accepted'),reference_accepted_stamp=detail.get('reference_accepted_stamp'),
            pipeline_stage=('new_reference_consumer_selected_tracking_not_replayed' if matched else 'new_reference_consumer_not_selected')
                if changed_geometry and (detail.get('reference_version') or 0)>0
                else 'held_or_reprojected_geometry_consumer_selected' if changed_geometry and matched
                else 'held_or_reprojected_geometry_consumer_not_selected' if changed_geometry else 'producer_kept_reference',
            original_digest=original_digest,producer_digest=payload.get('control_path_digest'),
            producer_original_digest=payload.get('original_path_digest'),
            original_endpoint_map=payload.get('original_endpoint_map'),
            producer_endpoint_map=payload.get('reference_endpoint_map'),
            consumer_endpoint_map=state.get('reference_endpoint_map'),
            consumer_digest=state.get('matched_path_digest'),consumer_matches_producer=matched,
            observation={key:detail.get(key) for key in ('confidence','source_age_ms','capture_age_ms_clock_dependent',
                'producer_session_id','camera_time_epoch','last_json_rejection','observation_geometry','osm_guidance_context')},
            recorded_runtime_age_ms=None if recorded_runtime[1] is None else (clock.ns-recorded_runtime[1])/1.e6,
            recorded_tracking=recorded_runtime[0],cached_points=payload.get('cached_points'),hazards=len(state['hazard_regions'])))
    class Publisher:
        def __init__(self,topic,*_,**kwargs):self.topic=topic
        def publish(self,message):
            if self.topic=='/camera_right/control_path':output(message)
            elif self.topic=='/camera_right/status':status(message)
            elif self.topic=='/camera_right/processing_health':health_rows.append(json.loads(message.data))
    original=(module.rospy,module.tf,module.time)
    module.rospy=NS(get_param=lambda key,default=None:params.get(key,default),Publisher=Publisher,
        Subscriber=lambda *a,**kw:None,Timer=lambda *a,**kw:None,Duration=rospy.Duration,Time=Time,
        logwarn=lambda *a,**kw:None,logwarn_throttle=lambda *a,**kw:None)
    module.tf=NS(TransformListener=lambda:listener,Exception=tf.Exception,transformations=tf.transformations)
    module.time=NS(monotonic=lambda:clock.ns/1.e9)
    try:
        node=module.CameraRightPath();next_tick=clock.ns
        for file in files:
            with rosbag.Bag(str(file)) as bag:
                for topic,msg,stamp in bag.read_messages(topics=['/tf','/tf_static','/osm_global_path',
                        '/hybridnets/result_json','/hybridnets/observations_json','/hybridnets/valid',
                        '/hybridnets/timing_diagnostics','/localization_safe','/neupan/runtime','/gps_guard/status','/gps_guard/local_fault_event']):
                    while next_tick<stamp.to_nsec():
                        clock.ns=next_tick;node.tick(None);next_tick+=200000000
                    clock.ns=stamp.to_nsec()
                    if topic in ('/tf','/tf_static'):feed(msg,topic=='/tf_static')
                    elif topic=='/osm_global_path':guard.original_cb(msg);node.path_cb(msg)
                    elif topic=='/hybridnets/result_json':node.json_cb(msg)
                    elif topic=='/hybridnets/observations_json':node.observation_cb(msg)
                    elif topic=='/hybridnets/valid':node.valid_cb(msg)
                    elif topic=='/hybridnets/timing_diagnostics':node.receiver_health_cb(msg)
                    elif topic=='/localization_safe':node.safe_cb(msg)
                    elif topic=='/gps_guard/status':node.alignment_status_cb(msg)
                    elif topic=='/gps_guard/local_fault_event':node.alignment_fault_cb(msg)
                    elif topic=='/neupan/runtime':
                        runtime=json.loads(msg.data)
                        recorded_runtime[:]=[{key:runtime.get(key) for key in ('cycle_id','command_source','linear_x',
                            'reference_receipt','selected_reference_version','tracking_guard','reference_visual',
                            'path_point_index','robot_x','robot_y','robot_yaw_deg')},clock.ns]
        node.tick(None)
        return dict(bag=directory.name,files=[p.name for p in files],ticks=len(set(r['stamp_ns'] for r in rows)),
            status_events=len(rows),sources=dict(Counter(r['source'] for r in rows)),
            pipeline_stages=dict(Counter(r['pipeline_stage'] for r in rows)),
            consumer_reasons=dict(Counter(r['consumer_reason'] for r in rows)),
            producer_reasons=dict(Counter(r['producer_reason'] for r in rows)),
            reference_geometry_changes=len(route_changes),reference_versions=sorted(set(r['version'] for r in rows if r['version'] is not None)),
            max_hazards=max((r['hazards'] for r in rows),default=0),timeline=rows,
            processing_health_events=len(health_rows),processing_health=health_rows,
            limitation='Actual producer and consumer selection only; no planner, no simulated vehicle or recovery ACK without prediction verification.')
    finally:module.rospy,module.tf,module.time=original


def obstacle_probes(output=None,directory=None,limit=4):
    import rosbag,rospy
    import torch
    torch.set_num_threads(1)
    from neupan import neupan
    from neupan_safety import planner_obstacle_input,validate_solution,trajectory_clearance,CandidateRejected,candidate_snapshot
    sys.path.insert(0,str(ROOT/'src/Test/tests'))
    from test_startup_recovery import scan_node
    directory=Path(directory) if directory is not None else ROOT/'rosbags/rosbag20261028_093827'
    files=ordered_bag_files(directory)
    if not files:raise FileNotFoundError('obstacle bag missing: '+str(directory))
    clock=NS(ns=0);listener,feed,Time=tf_environment(clock)
    converter,_,_=scan_node();converter.listener=listener;converter.lidar_frame='base_link'
    converter.quat_to_yaw_list=lambda q:math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
    config=json.loads((ROOT/'tracker_logs/review_20261028/field_run_review/system_run_full_20261028_093818_config.json').read_text())[0]
    params=config['node_parameters']
    native=neupan.init_from_yaml(params['config_file'],pan={'dune_checkpoint':params['dune_checkpoint']})
    scans={};route=None;rows=[];layers={};last_command=None
    for file in files:
        with rosbag.Bag(str(file)) as bag:
            for topic,msg,stamp in bag.read_messages(topics=['/tf','/tf_static','/scan','/osm_global_path','/neupan/runtime',
                                                           '/cmd_vel/auto','/neupan_cmd_vel']):
                clock.ns=stamp.to_nsec()
                if topic in ('/tf','/tf_static'):feed(msg,topic=='/tf_static');continue
                if topic in ('/cmd_vel/auto','/neupan_cmd_vel'):
                    last_command=([msg.linear.x,msg.angular.z],clock.ns/1.e9);continue
                if topic=='/scan':
                    scans[msg.header.stamp.to_nsec()]=msg
                    if len(scans)>32:scans.pop(next(iter(scans)))
                    continue
                if topic=='/osm_global_path':
                    route=[np.array([[p.pose.position.x],[p.pose.position.y],[converter.quat_to_yaw_list(
                        [p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z,p.pose.orientation.w])],[1.]]) for p in msg.poses]
                    native.set_initial_path(route);continue
                recorded=json.loads(msg.data)
                if (len(rows)>=limit or not scans or route is None
                        or recorded.get('solver_ms') is None):continue
                row=dict(recorded_stamp_ns=clock.ns,recorded_reason=recorded.get('reason'))
                try:
                    timing=recorded.get('timing',{})
                    scan_stamp=timing.get('solve_input_ages',{}).get('scan_stamp_ns') or timing.get('scan_tf',{}).get('selected_stamp_ns')
                    selected=scans.get(scan_stamp,next(reversed(scans.values())))
                    snapshot=converter.convert_scan_snapshot(selected,clock.ns/1.e9)
                    state_ns=timing.get('state_stamp_ns') or selected.header.stamp.to_nsec()
                    p,q=listener.lookupTransform('map','base_link',rospy.Time(state_ns//10**9,state_ns%10**9))
                    state=np.array([[p[0]],[p[1]],[converter.quat_to_yaw_list(q)]])
                    # Identical state/full scan/reference, explicit COMMON cold
                    # warm start and nearest progress, not reconstructed flight history.
                    variants=[];prefix='event_%04d_'%len(rows)
                    layers[prefix+'scan']=snapshot['safety'][0];layers[prefix+'pose']=state
                    for label,instance in (('native',native),):
                        instance.set_initial_path([point.copy() for point in route]);instance.reset()
                        xy=np.array([p[:2,0] for p in instance.ipath.cur_curve])
                        instance.ipath.point_index=int(np.linalg.norm(xy-state[:2,0],axis=1).argmin())
                        instance.set_reference_speed(.8)
                        action,info=instance(state,planner_obstacle_input(instance,snapshot['safety'][0]))
                        info=candidate_snapshot(info)
                        layers[prefix+label+'_states']=np.hstack(info['opt_state_list'])
                        layers[prefix+label+'_controls']=info['vel_tensor']
                        layers[prefix+label+'_reference']=np.hstack(info['ref_state_list'])
                        entry=dict(stage=label,native_stop=bool(info.get('stop')),action=np.asarray(action).reshape(-1).tolist())
                        try:entry['geometry_diagnostic']=trajectory_clearance(np.hstack(info['opt_state_list']),snapshot['safety'][0],instance.robot.vertices)
                        except CandidateRejected as error:entry.update(geometry_diagnostic_rejected=True,geometry_diagnostic=error.details)
                        variants.append(entry)
                    row['same_input_layers']=variants;row['common_warm_start']='zeros; not original recorded warm start'
                    validate_solution(action,info,native.robot,native.dt)
                    row.update(accepted=not bool(info.get('stop') or info.get('arrive')),
                        command_mps=float(action[0,0]),scan_stamp_ns=snapshot['safety'][1],state_stamp_ns=state_ns,
                        native_action=np.asarray(action).reshape(2).tolist(),command_source='native_static_probe')
                except Exception as error:row.update(error=type(error).__name__,message=str(error))
                rows.append(row);print(json.dumps({'obstacle_probe':len(rows),'accepted':row.get('accepted'),'error':row.get('error')}),flush=True)
    if output is not None:np.savez_compressed(str(output/'obstacle_layers.npz'),**layers)
    return dict(files=[p.name for p in files],probes=rows,
        limitation='Static real optimizer at recorded rejection epochs; original rejected solutions/warm starts absent. Does not emulate intervening controls, full run retry/publication timing or prove detour success. No ROS retiming, geometric selection adapter or secondary dispatch is applied.')


def feedback_replay():
    """Actual wheel-odom samples: averages are not instantaneous feedback."""
    import rosbag
    from neupan_navigation import MotionFeedback
    result=[]
    for name in ('rosbag20261028_093624','rosbag20261028_093827'):
        feedback=MotionFeedback(timeout=1.5,sample_max_interval=1.5)
        files=ordered_bag_files(ROOT/'rosbags'/name);rows=[]
        if not files:raise FileNotFoundError(name)
        for file in files:
            with rosbag.Bag(str(file)) as bag:
                for _,msg,stamp in bag.read_messages(topics=['/odom']):
                    p,q=msg.pose.pose.position,msg.pose.pose.orientation
                    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                    feedback.odom([p.x,p.y,yaw],msg.header.stamp.to_nsec(),stamp.to_sec(),
                        msg.header.frame_id,stamp.to_nsec())
                    rows.append(feedback.snapshot(stamp.to_nsec(),stamp.to_sec()))
        result.append(dict(session=name,messages=len(rows),
            average_available=sum(row['average_speed_mps'] is not None for row in rows),
            instantaneous_available=sum(row['speed_mps'] is not None for row in rows),
            sources=dict(Counter(row['speed_source'] for row in rows)),details=rows))
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--camera',action='store_true');parser.add_argument('--obstacles',action='store_true')
    parser.add_argument('--feedback',action='store_true')
    parser.add_argument('--camera-bag',action='append',help='repeat for independent recorded-input camera sessions')
    parser.add_argument('--obstacle-bag',type=Path,help='native static probes from this recorded navigation session')
    parser.add_argument('--json-only',action='store_true',help='write evidence only; maintain conclusions in PROJECT_ISSUES_CN.md')
    args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    result=dict(scope=__doc__)
    def checkpoint():
        (args.output_dir/'recorded_input_checks.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    if args.camera:
        if args.camera_bag:
            result['camera_sessions']=[]
            for directory in args.camera_bag:
                row=camera_replay(directory);result['camera_sessions'].append(row)
                checkpoint()
                print(json.dumps({k:v for k,v in row.items() if k not in ('timeline','processing_health')}),flush=True)
        else:
            result['camera']=camera_replay()
            print(json.dumps({k:v for k,v in result['camera'].items() if k not in ('timeline','processing_health')}),flush=True)
    if args.obstacles:result['obstacles']=obstacle_probes(args.output_dir,args.obstacle_bag)
    if args.feedback:
        result['feedback']=feedback_replay()
        print(json.dumps({'feedback':[{k:v for k,v in row.items() if k!='details'} for row in result['feedback']]}),flush=True)
    files=[ROOT/'src/neupan_ros/src'/p for p in ('neupan_core.py','neupan_safety.py','neupan_diagnostics.py','neupan_navigation.py')]
    files+=[ROOT/'src/osm_map_handler/src'/p for p in ('osm_camera_path_refinement.py','osm_camera_path_guard.py')]
    result['implementation_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    (args.output_dir/'recorded_input_checks.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    if args.json_only:return
    lines=['# 稳定参考与原生NeuPAN静态验证','',__doc__,'']
    if args.camera and 'camera' in result:
        r=result['camera'];lines+=['## 原相机bag生产/消费重算','',
            f"分片{len(r['files'])}，检查{r['ticks']}轮；消费者来源{r['sources']}；参考几何变化{r['reference_geometry_changes']}次；最大危险记录{r['max_hazards']}。",r['limitation'],'']
    if args.obstacles:
        r=result['obstacles'];rows=r['probes'];lines+=['## 原普通导航bag遇障碍静态重算','',
            f"分片{len(r['files'])}，记录拒绝时刻{len(rows)}个；原生非停止动作{sum(row.get('accepted',False) for row in rows)}个；检查/求解错误{sum('error' in row for row in rows)}个。",
            r['limitation'],'']
    if args.feedback:
        lines+=['## 原轮里程计回放','']
        for row in result['feedback']:
            lines.append('%s：%d条，区间均速有效%d条，瞬时首步反馈%d条（低频数据不得冒充瞬时反馈）。'%(
                row['session'],row['messages'],row['average_available'],row['instantaneous_available']))
    lines+=['完整回归及可选跳过原因见unit_regression.json；详细时间线、原始分片名单及源码指纹见recorded_input_checks.json。',
        '仅重算原生求解及记录只读几何诊断，不改写动作、选点/排序或热启动，不模拟真实车辆闭环。实车绕行和净距仍需验证。']
    (args.output_dir/'report.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':main()
