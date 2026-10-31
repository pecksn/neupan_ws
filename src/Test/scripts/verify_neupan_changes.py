#!/usr/bin/env python3
"""Read-only snapshot and bag A/B verification. Fake ROS publishers only.

Does not replay ROS topics, start a ROS master, or command hardware. Recorded
poses remain unchanged; replay is not a counterfactual drive or safety proof.
"""
import argparse
from collections import Counter
import copy
import json
import hashlib
from pathlib import Path
import subprocess
import sys
import time
from datetime import datetime, timedelta, timezone
from types import SimpleNamespace as NS, ModuleType
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/osm_map_handler/src'))


def diagnostic_videos(output):
    """Recorded images + recorded GPU/old-controller diagnostics, NOT new drive.

    RTP has no shared application frame ID. Overlays use latest received JSON,
    explicitly approximate, not a precise per-image correspondence claim.
    """
    import cv2,rosbag,rospy
    rows=[];output.mkdir(parents=True,exist_ok=True)
    for suffix in ('153335','155048','155614'):
        directory=ROOT/'rosbags'/('rosbag20261026_'+suffix)
        files=[];statuses=[]
        for file in directory.glob('*.bag'):
            with rosbag.Bag(str(file)) as bag:
                files.append((bag.get_start_time(),bag.get_end_time(),file))
                for _,msg,stamp in bag.read_messages(topics=['/camera_right/status']):
                    statuses.append((stamp.to_sec(),json.loads(msg.data)))
        files.sort();statuses.sort(key=lambda r:r[0])
        desired=('active' if suffix=='153335' else 'shared' if suffix=='155048' else 'alignment')
        candidates=[t for t,s in statuses if (s.get('active') if desired=='active' else desired in s.get('reason',''))]
        center=candidates[len(candidates)//2] if candidates else (files[0][0]+files[-1][1])/2
        begin=max(files[0][0],center-15);end=min(files[-1][1],begin+30)
        target=output/('recorded_'+suffix+'.mp4')
        encoder=subprocess.Popen(['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-y',
            '-f','rawvideo','-pix_fmt','bgr24','-s','960x720','-r','5','-i','pipe:0',
            '-an','-c:v','libx264','-preset','veryfast','-crf','23','-pix_fmt','yuv420p',
            '-movflags','+faststart',str(target)],stdin=subprocess.PIPE)
        image=None;packet=None;status=None;image_time=None;packet_time=None;status_time=None
        next_frame=begin;count=0
        def text(canvas,message,y,color=(235,235,235)):
            cv2.putText(canvas,message[:112],(12,y),cv2.FONT_HERSHEY_SIMPLEX,.55,color,1,cv2.LINE_AA)
        def emit(t):
            nonlocal count
            canvas=np.zeros((720,960,3),np.uint8)
            if image is not None and t-image_time<1.:
                canvas[:540]=cv2.resize(image,(960,540))
            else: text(canvas,'NO FRESH RECORDED IMAGE',280)
            overlay=packet if packet is not None and 0<=t-packet_time<.8 else {}
            for side,color in (('left',(255,0,255)),('right',(0,255,255))):
                for segment in overlay.get(side+'_valid_segments_px',[]):
                    p=np.asarray(segment,float)*np.array([960/max(1,overlay.get('image_width',1920)),540/max(1,overlay.get('image_height',1080))])
                    if len(p)>=2:cv2.polylines(canvas,[np.rint(p).astype(np.int32).reshape(-1,1,2)],False,color,2)
            local=datetime.fromtimestamp(t,timezone(timedelta(hours=8))).strftime('%H:%M:%S.%f')[:-3]
            text(canvas,'RECORDED DATA / NOT NEW CONTROLLER VIDEO | '+suffix+' | '+local,565,(0,255,255))
            text(canvas,'GPU: L=%s R=%s confidence=%s source_age_ms=%s' % (
                overlay.get('left_boundary_valid','--'),overlay.get('right_boundary_valid','--'),
                overlay.get('confidence','--'),overlay.get('source_age_ms','--')),595)
            s=status if status and 0<=t-status_time<1.5 else {}
            text(canvas,'OLD status: active=%s reason=%s' % (s.get('active','--'),s.get('reason','missing/stale')),625)
            text(canvas,'GPU overlay pairing APPROXIMATE (RTP has no common frame ID)',655,(0,200,255))
            text(canvas,'Image receive age=%.0fms  JSON receive age=%sms' % (
                (t-image_time)*1000 if image_time is not None else -1,
                '%.0f'%((t-packet_time)*1000) if packet_time is not None else '--'),685)
            encoder.stdin.write(canvas.tobytes());count+=1
            if count==60:cv2.imwrite(str(output/('recorded_'+suffix+'_preview.jpg')),canvas)
        try:
            topics=['/camera/remote_image_raw','/hybridnets/observations_json','/camera_right/status']
            for start,finish,file in files:
                if finish<begin-1 or start>end:continue
                with rosbag.Bag(str(file)) as bag:
                    for topic,msg,stamp in bag.read_messages(topics=topics,
                            start_time=rospy.Time.from_sec(begin-1),end_time=rospy.Time.from_sec(end)):
                        t=stamp.to_sec()
                        while next_frame<t and next_frame<end:
                            emit(next_frame);next_frame+=.2
                        if topic=='/camera/remote_image_raw' and msg.encoding in ('bgr8','rgb8'):
                            data=np.frombuffer(msg.data,np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3]
                            image=data.reshape(msg.height,msg.width,3)
                            if msg.encoding=='rgb8':image=cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                            image_time=t
                        elif topic=='/hybridnets/observations_json':packet=json.loads(msg.data);packet_time=t
                        elif topic=='/camera_right/status':status=json.loads(msg.data);status_time=t
            while next_frame<end:emit(next_frame);next_frame+=.2
        finally:
            encoder.stdin.close()
            if encoder.wait(timeout=30):raise RuntimeError('video encoding failed')
        rows.append(dict(bag=directory.name,video=str(target),frames=count,begin=begin,end=end,
            meaning='Recorded GPU observations and OLD controller status; approximate RTP overlay; NOT new algorithm driving or exact frame synchronization'))
        print(json.dumps(rows[-1]),flush=True)
    (output/'recorded_video_manifest.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2))
    return rows


def snapshots():
    import torch
    from neupan.blocks.initial_path import InitialPath
    from neupan.blocks.nrmp import NRMP
    from neupan.robot import robot
    torch.set_num_threads(1)
    rows=[]
    for file in sorted((ROOT/'tracker_logs').glob('neupan_failure_20261026_*.npz')):
        a=np.load(file); meta=json.loads(file.with_suffix('.json').read_text())
        bot=robot(receding=50,step_time=.1,kinematics='acker',wheelbase=.67,
                  max_speed=[1.5,.35],max_acce=[.8,.08],length=.95,width=.85)
        ipath=InitialPath(50,.1,.8,bot)
        ipath.set_initial_path([v.reshape(4,1) for v in a['initial_path'].T])
        for key in ('point_index','curve_index'): setattr(ipath,key,meta['path_progress'][key])
        _,_,ref,_=ipath.generate_nom_ref_state(a['solver_robot_state'],a['cur_vel_array'],.8)
        row=dict(snapshot=file.name,max_reference_step_m=float(np.linalg.norm(np.diff(ref[:2],axis=1),axis=0).max()))
        layer=NRMP(50,.1,bot,nrmp_max_num=10,q_s=1.2,p_u=21.,eta=1.,d_max=.8,d_min=.2)
        keys=sorted([k for k in a.files if '_parameter' in k],key=lambda k:int(k.split('_parameter')[1]))
        params=[a[k].copy() for k in keys]
        params[1]=(params[253]*ref.astype('float32')).astype('float32')
        try:
            solved=layer.nrmp_layer(*[torch.from_numpy(v) for v in params],solver_args={'solve_method':'ECOS'})
            row['ecos']='solved'
            row['max_returned_control']=np.max(np.abs(solved[1].detach().numpy()),axis=1).tolist()
            sys.path.insert(0,str(ROOT/'src/neupan_ros/src'))
            from neupan_safety import validate_solution
            try:
                states=solved[0].detach().numpy()
                validate_solution(solved[1].detach().numpy()[:,:1],
                    dict(opt_state_list=[s.reshape(3,1) for s in states.T],vel_tensor=solved[1]),bot,.1)
                row['returned_control_checks']='passed'
            except ValueError as error: row['returned_control_checks']=str(error)
        except Exception as error: row['ecos']=str(error)
        # Fixed-parameter ablation is not the complete PAN iteration. Exercise
        # the deployed model and iterative planner too, with recorded warm start.
        try:
            from neupan import neupan
            planner=neupan.init_from_yaml(meta['config_file'],pan={'dune_checkpoint':meta['dune_checkpoint']})
            planner.ipath.set_initial_path([v.reshape(4,1) for v in a['initial_path'].T])
            for key in ('point_index','curve_index'): setattr(planner.ipath,key,meta['path_progress'][key])
            planner.cur_vel_array=a['cur_vel_array'].copy()
            action,info=planner(a['solver_robot_state'],a['solver_obstacle_points'])
            validate_solution(action,info,planner.robot,planner.dt)
            row['complete_planner']='solved_and_return_checks_passed'
            row['complete_action']=action[:,0].tolist()
            try:
                from neupan_safety import trajectory_clearance
                row['complete_trajectory_check']=trajectory_clearance(np.hstack(info['opt_state_list']),
                    a['solver_obstacle_points'],planner.robot.vertices)
            except ValueError as error: row['complete_trajectory_check']='blocked: '+str(error)
        except Exception as error: row['complete_planner']=str(error)
        rows.append(row)
    return rows


def event_cache_files(directory, files, cache_root):
    """Temporary, lossless subset for repeat offline checks; originals untouched."""
    import rosbag
    destination=cache_root/directory.name
    destination.mkdir(parents=True,exist_ok=True)
    target=destination/'events.bag';manifest=destination/'sources.json'
    sources=[dict(path=str(f),size=f.stat().st_size,mtime_ns=f.stat().st_mtime_ns) for f in files]
    if target.exists() and manifest.exists() and json.loads(manifest.read_text())==sources:return [target]
    topics=['/tf','/tf_static','/osm_global_path','/hybridnets/result_json','/hybridnets/valid','/localization_safe']
    previous=-1
    with rosbag.Bag(str(target),'w',compression='none') as out:
        for file in files:
            with rosbag.Bag(str(file)) as bag:
                for topic,message,stamp in bag.read_messages(topics=topics):
                    if stamp.to_nsec()<previous:raise ValueError('event-cache input time moved backwards')
                    previous=stamp.to_nsec();out.write(topic,message,stamp)
    manifest.write_text(json.dumps(sources,indent=2))
    return [target]


def replay(directory, baseline_file=None, baseline_hz=2., tick_limit=None, event_cache=None):
    import rospy,tf,tf2_ros,rosbag
    from geometry_msgs.msg import TransformStamped
    import osm_camera_path_refinement as updated
    from osm_camera_path_guard import CameraPathGuard
    source=baseline_file.read_text() if baseline_file else subprocess.check_output(['git','show','fa35795:src/osm_map_handler/src/osm_camera_path_refinement.py'],cwd=str(ROOT),text=True)
    baseline=ModuleType('offline_baseline');exec(compile(source,'baseline_camera.py','exec'),baseline.__dict__)
    buffer=tf2_ros.Buffer(cache_time=rospy.Duration(3600),debug=False)
    def start(file):
        with rosbag.Bag(str(file)) as bag: return bag.get_start_time()
    files=sorted(directory.glob('*.bag'),key=start)
    if event_cache:files=event_cache_files(directory,files,event_cache)
    for file in files:
        with rosbag.Bag(str(file)) as bag:
            for topic,msg,stamp in bag.read_messages(topics=['/tf','/tf_static']):
                for transform in msg.transforms:
                    converted=TransformStamped()
                    converted.header.stamp=transform.header.stamp
                    converted.header.frame_id=transform.header.frame_id
                    converted.child_frame_id=transform.child_frame_id
                    for name in ('x','y','z'):
                        setattr(converted.transform.translation,name,getattr(transform.transform.translation,name))
                        setattr(converted.transform.rotation,name,getattr(transform.transform.rotation,name))
                    converted.transform.rotation.w=transform.transform.rotation.w
                    if topic=='/tf_static': buffer.set_transform_static(converted,'recorded')
                    else: buffer.set_transform(converted,'recorded')
    def lookup(target,source,stamp):
        try: t=buffer.lookup_transform(target,source,stamp)
        except Exception as error: raise tf.Exception(str(error))
        p,q=t.transform.translation,t.transform.rotation
        return (p.x,p.y,p.z),(q.x,q.y,q.z,q.w)
    listener=NS(lookupTransform=lookup,getLatestCommonTime=lambda a,b:buffer.lookup_transform(a,b,rospy.Time(0)).header.stamp)
    # A preloaded buffer contains future poses. Latest means latest AT replay time.
    clock=NS(ns=0)
    def latest(a,b):
        try:
            buffer.lookup_transform(a,b,rospy.Time.from_sec(clock.ns/1e9))
            return rospy.Time.from_sec(clock.ns/1e9)
        except Exception:
            return buffer.lookup_transform(a,b,rospy.Time(0)).header.stamp
    listener.getLatestCommonTime=latest
    raw_lookup=listener.lookupTransform
    listener.lookupTransform=lambda a,b,t:raw_lookup(a,b,rospy.Time.from_sec(clock.ns/1e9) if t.to_nsec()==0 else t)
    params={'~control_enabled':True,'~strategy':'segment_cache','~distance_reference':'camera_center',
            '~camera_clearance_m':2.,'~base_link_height_m':.2,
            '~camera_info_file':str(ROOT/'src/usb_camera/config/usb_camera.yaml'),
            '~extrinsics_file':str(ROOT/'src/usb_camera/config/livox_camera_extrinsics.yaml')}
    class Publisher:
        def __init__(self,topic,*args,**kw): self.topic=topic;self.last=None
        def publish(self,msg): self.last=msg
    controls=[]
    with patch.object(rospy,'get_param',side_effect=lambda name,default=None:params.get(name,default)), \
         patch.object(rospy,'Publisher',Publisher),patch.object(rospy,'Subscriber'),patch.object(rospy,'Timer'), \
         patch.object(tf,'TransformListener',return_value=listener),patch.object(rospy.Time,'now',side_effect=lambda:rospy.Time.from_sec(clock.ns/1e9)), \
         patch.object(rospy,'logwarn_throttle'),patch.object(rospy,'logwarn'):
        for module,hz in ((baseline,baseline_hz),(updated,5.)):
            module.time=NS(monotonic=lambda:clock.ns/1e9)
            node=module.CameraRightPath();node.last_tick=0.
            controls.append(dict(node=node,guard=CameraPathGuard() if module is updated else None,hz=hz,next=None,reasons=Counter(),ticks=0,active=0,max_shift=0.,cpu_tick_ms=[],max_coverage_m=0.,
                                 visual_authorized_ticks=0,selected_clearances=Counter(),speed_caps=[],
                                 final_gates_instrumented=hasattr(module,'shape_report'),
                                 corridor_stop_ticks=0,corridor_verified_ticks=0,active_shape_violations=0,active_corridor_violations=0))
        for file in files:
            if tick_limit and all(run['ticks']>=tick_limit for run in controls):break
            with rosbag.Bag(str(file)) as bag:
                topics=['/osm_global_path','/hybridnets/result_json','/hybridnets/valid','/localization_safe']
                for topic,msg,stamp in bag.read_messages(topics=topics):
                    if tick_limit and all(run['ticks']>=tick_limit for run in controls):break
                    if stamp.to_nsec()<clock.ns:
                        raise ValueError('bag message time moved backwards; refusing misleading replay')
                    clock.ns=stamp.to_nsec();now=clock.ns/1e9
                    for run in controls:
                        node=run['node']
                        {'/osm_global_path':node.path_cb,'/hybridnets/result_json':node.json_cb,
                         '/hybridnets/valid':node.valid_cb,'/localization_safe':node.safe_cb}[topic](msg)
                        if topic=='/osm_global_path' and run['guard'] is not None:run['guard'].original_cb(msg)
                        if run['next'] is None: run['next']=now
                        if now>=run['next']:
                            started=time.perf_counter()
                            node.tick(None);run['next']=now+1/run['hz']
                            run['cpu_tick_ms'].append((time.perf_counter()-started)*1000)
                            if node.status.last:
                                state=json.loads(node.status.last.data)
                                detail=state.get('inference_diagnostics') or {}
                                contract=detail.get('execution_contract')
                                guard=run['guard']
                                if guard is not None:
                                    if node.output.last:guard.candidate_cb(node.output.last)
                                    guard.status_cb(state,now)
                                    _,_,selected=guard.select(now,clock.ns)
                                    authorized=(bool(state['active']) and selected['source']=='camera' and bool(contract)
                                        and not selected.get('recovery_pending')
                                        and clock.ns<=contract['expires_ns'] and contract['speed_cap_mps']>0)
                                    run['visual_authorized_ticks']+=authorized
                                    if hasattr(node,'hazard_ack_cb'):node.hazard_ack_cb(NS(data=json.dumps(selected)))
                                target=detail.get('selected_clearance_m')
                                if target is not None:run['selected_clearances'][str(round(target,3))]+=1
                                if contract:run['speed_caps'].append(contract['speed_cap_mps'])
                                run['ticks']+=1;run['active']+=bool(state['active'])
                                if run['guard'] is not None and run['ticks']%100==0:
                                    print(json.dumps(dict(progress_bag=directory.name,ticks=run['ticks'],
                                        active=run['active'],authorized=run['visual_authorized_ticks'],
                                        last_tick_ms=round(run['cpu_tick_ms'][-1],2))),flush=True)
                                run['reasons'][state['reason']]+=1
                                run['max_shift']=max(run['max_shift'],abs(state['shift_m']))
                                run['max_coverage_m']=max(run['max_coverage_m'],state.get('continuous_committed_coverage_m',0.))
                                run['corridor_stop_ticks']+=bool(state.get('corridor_stop_required'))
                                run['corridor_verified_ticks']+=bool(state.get('corridor_verified'))
                                final=getattr(node,'final_path_diagnostics',{})
                                if state['active']:
                                    run['active_shape_violations']+=final.get('shape',{}).get('ok') is False
                                    run['active_corridor_violations']+=final.get('corridor',{}).get('ok') is False
    for run in controls:
        values=run.pop('cpu_tick_ms')
        run['tick_ms_p50_p95']=np.percentile(values,[50,95]).tolist() if values else []
        run['tick_over_500ms']=sum(value>500 for value in values)
        run['active_fraction']=run['active']/max(1,run['ticks'])
        if run['hz']==2.: run['max_coverage_m']=None # Old producer did not expose this metric.
        caps=run.pop('speed_caps');run['speed_cap_min_max_mps']=[min(caps),max(caps)] if caps else None
    return dict(bag=directory.name,tick_limit=tick_limit,variants=[{k:v for k,v in r.items() if k not in ('node','next','guard')} for r in controls],
                limitation='Recorded poses/TF, no counterfactual drive; tick schedule approximated by available input arrivals. No real ROS publishers.')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--baseline-file',type=Path)
    parser.add_argument('--baseline-hz',type=float,default=2.)
    parser.add_argument('--tick-limit',type=int,help='Profiling only: stop after this many ticks per variant; not a full-bag validation')
    parser.add_argument('--event-cache-dir',type=Path,help='Temporary lossless topic subset; does not alter source bags')
    parser.add_argument('--bag-suffix',action='append',choices=['153335','155048','155614'])
    parser.add_argument('--update-snapshots',action='store_true')
    parser.add_argument('--recorded-videos',action='store_true')
    parser.add_argument('--unit-tests',action='store_true')
    parser.add_argument('--test-pattern',default='test_*.py',help='unit-test discovery pattern; full suite by default')
    parser.add_argument('--publication-regression',action='store_true')
    parser.add_argument('--bags',action='store_true');args=parser.parse_args()
    if args.publication_regression:
        sys.path.insert(0,str(ROOT/'src/Test/tests'))
        from test_camera_publication_protocol import PublicationTests,RecoveryTests
        rows={}
        for kind in ('lower','road','compatible','heartbeat','during_check','late_select','late_lock',
                     'late_version','lease_lock','recovery','recovery_late'):
            node,commands,context=PublicationTests().run_case(kind)
            rows[kind]=dict(commands_mps=commands,planner_fault=getattr(node,'planner_fault',None),
                stop_reasons=node.stop_registry.snapshot(),
                remaining_hazards=len(context['guard'].hazard_regions),
                released_ids=context['guard'].released_hazard_ids,
                publish_input_ages=getattr(node,'publish_input_ages',None))
        recovery=RecoveryTests()
        recovery.test_expired_cache_retains_scope_and_recovers_on_fresh_full_reobservation()
        files=[ROOT/'src/neupan_ros/src'/p for p in ('neupan_core.py','neupan_safety.py')]
        files += [ROOT/'src/osm_map_handler/src'/p for p in ('osm_camera_path_guard.py','osm_camera_path_refinement.py')]
        report=dict(cases=rows,recovery_trace=recovery.recovery_trace,
            implementation_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
            scope='Actual production methods with fake clocks/ROS publishers. No vehicle, no ROS master or actuator topics. Not physical validation.')
        args.output_dir.mkdir(parents=True,exist_ok=True)
        (args.output_dir/'publication_regression.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
        lines=['# 发布边界与恢复协议整改验证','',
            '只运行真实生产方法配假时钟/发布器；无实车控制，也不是完整录包闭环。',
            '完整回归及跳过原因另见unit_regression.json，复现数据和源码指纹见publication_regression.json。','',
            '| 场景 | 实际模拟发布速度 m/s | 剩余危险/复查记录 |','| --- | --- | ---: |']
        for name,row in rows.items():lines.append('| %s | %s | %d |'%(name,row['commands_mps'],row['remaining_hazards']))
        lines+=['','lower：0.30降到0.12；road：新道路区域冲突；compatible/heartbeat：兼容更新和续期。',
            'during_check：轨迹检查中替换授权；late_select/late_lock/late_version：最后阶段延迟；lease_lock：锁等待跨越授权到期。',
            'recovery/recovery_late：实际run提交恢复/输入过期不得提交恢复。','',
            '缓存联合链：正常发布并保存范围 → 过期停车（有固定odom身份）→ 其他安全区域仍停车 → 原范围复查后仅允许规划 → 模拟发布提交后解除。',
            '包含缓存TTL和TF清缓存、故障前排队观测拒绝、未知范围要求人工确认的专项回归；不证明实车传感器/制动已验收。',
            '没有改GPU、外部NeuPAN优化器、定位预算或车辆物理限制；需重启原规划和靠右节点加载。']
        (args.output_dir/'report.md').write_text('\n'.join(lines)+'\n')
        print(json.dumps(rows,ensure_ascii=False),flush=True)
        return
    if args.unit_tests:
        import unittest
        sys.path.append('/usr/lib/python3/dist-packages')
        result=unittest.TextTestRunner(verbosity=0).run(unittest.defaultTestLoader.discover(str(ROOT/'src/Test/tests'),pattern=args.test_pattern))
        files=[ROOT/'src/neupan_ros/src'/name for name in ('neupan_core.py','neupan_safety.py','neupan_diagnostics.py','neupan_navigation.py')]
        files += [ROOT/'src/osm_map_handler/src'/name for name in ('osm_camera_path_refinement.py','osm_camera_path_guard.py')]
        files += [ROOT/'src/robot_base_driver/scripts/system_run_logger.py']
        files += [ROOT/'src/robot_base_driver/scripts/global_odom_bridge.py']
        files += [ROOT/'src/hybridnets_receiver/scripts/hybridnets_json_receiver.py']
        files += [ROOT/'src/vehicle_bag_recorder/config'/name for name in ('camera_right_test.yaml','hybridnets_data.yaml','vehicle_data.yaml')]
        record=dict(tests=result.testsRun,passed=result.testsRun-len(result.skipped)-len(result.failures)-len(result.errors),
            test_pattern=args.test_pattern,
            environment=dict(executable=sys.executable,python=sys.version,ros_distro=__import__('os').environ.get('ROS_DISTRO')),
            successful=result.wasSuccessful(),skipped=[dict(test=str(test),reason=reason) for test,reason in result.skipped],
            failures=[str(test) for test,_ in result.failures],errors=[str(test) for test,_ in result.errors],
            implementation_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
            scope='Offline unit tests/fake publishers; no vehicle, ROS master or real actuator topics.')
        args.output_dir.mkdir(parents=True,exist_ok=True)
        (args.output_dir/'unit_regression.json').write_text(json.dumps(record,ensure_ascii=False,indent=2))
        print(json.dumps({k:v for k,v in record.items() if k!='implementation_sha256'}),flush=True)
        sys.exit(not result.wasSuccessful())
    if args.recorded_videos:
        diagnostic_videos(args.output_dir)
        return
    result={'snapshots':snapshots()}
    result['baseline']={'file':str(args.baseline_file),'hz':args.baseline_hz,
        'sha256':hashlib.sha256(args.baseline_file.read_bytes()).hexdigest() if args.baseline_file else None}
    result['implementation_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (
        ROOT.parent/'NeuPAN/neupan/blocks/initial_path.py',
        ROOT/'src/neupan_ros/src/neupan_safety.py',ROOT/'src/neupan_ros/src/neupan_core.py',
        ROOT/'src/osm_map_handler/src/osm_camera_path_guard.py',ROOT/'src/osm_map_handler/src/osm_camera_path_refinement.py')}
    if args.bags:
        result['bag_replay']=[]
        for suffix in (args.bag_suffix or ('153335','155048','155614')):
            row=replay(ROOT/'rosbags'/('rosbag20261026_'+suffix),args.baseline_file,args.baseline_hz,args.tick_limit,args.event_cache_dir)
            result['bag_replay'].append(row)
            print(json.dumps(dict(completed_bag=row['bag'],variants=[{key:v[key] for key in
                ('ticks','active','visual_authorized_ticks','corridor_stop_ticks','tick_ms_p50_p95','tick_over_500ms')}
                for v in row['variants']])),flush=True)
            result['complete']=False
            args.output_dir.mkdir(parents=True,exist_ok=True)
            (args.output_dir/'verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    args.output_dir.mkdir(parents=True,exist_ok=True)
    target=args.output_dir/'verification.json'
    if (args.bag_suffix or args.update_snapshots) and target.exists():
        previous=json.loads(target.read_text())
        merged={r['bag']:r for r in previous.get('bag_replay',[])}
        merged.update({r['bag']:r for r in result.get('bag_replay',[])})
        result['bag_replay']=list(merged.values())
    result['complete']=not bool(args.tick_limit)
    target.write_text(json.dumps(result,ensure_ascii=False,indent=2))
    summary=['# NeuPAN 离线整改验证','',
        '范围：'+('仅前%d个处理周期，非完整录包。' % args.tick_limit if args.tick_limit else '指定组的完整录包。'),
        '不启动车辆、不发布ROS话题；录包沿用原位姿和TF，定时调度按输入到达近似，不能当作修改后的实车行驶。',
        '只读基线：%s，%.1f Hz；新版：5 Hz。基线源码SHA256见verification.json。置信度门限不变；单右边界改用真实可行驶区域，不伪造左边界。' % (args.baseline_file.name if args.baseline_file else 'fa35795',args.baseline_hz),'',
        '## 失败快照','']
    for row in result['snapshots']:
        summary.append('- `%s`：最大参考步长 %.9f m；完整规划：%s；保存点云净距检查：%s。' % (
            row['snapshot'],row['max_reference_step_m'],row.get('complete_planner'),row.get('complete_trajectory_check')))
    summary+=['','## 靠右录包对照','', '| bag | 旧版激活/处理 | 新版激活/处理 | 最大偏移旧→新 |',
              '| --- | --- | --- | --- |']
    for row in result.get('bag_replay',[]):
        a,b=row['variants']
        summary.append('| %s | %d/%d（%.2f%%） | %d/%d（%.2f%%） | %.3f→%.3f m |' % (
            row['bag'],a['active'],a['ticks'],100*a['active_fraction'],b['active'],b['ticks'],100*b['active_fraction'],a['max_shift'],b['max_shift']))
    summary+=['','统计不是靠右里程比例；新版本增加最终形状/走廊检查，拒绝时可能回退或保持停车，不能只比较激活率。',
              '第13片组按bag实际起始时间排序，发现时间倒退会停止验证。',
              '控制/缓存对照未输入完整观测诊断通道，耗时不能代表该通道启用后的整机CPU预算。',
              '激活仅表示参考候选；visual_authorized_ticks另外核对消费心跳、路径摘要和速度授权，不代表已有非零实车指令。',
              '回放时钟不计入真实计算耗时，且TF缓冲预装全包历史；授权数不证明实车时效或因果可用性。高于500ms的处理周期尤其不能据此声称获得0.5s授权。',
              '录包位置在新版要求停车后仍继续前进，不代表新版允许前进；后续危险记忆累积不能当成停稳后一定无法恢复。静止原危险区重观测另由回归测试覆盖。',
              '模型几何、平地假设和坐标/安装需实测；动态障碍、感知/执行延迟和实车制动未验收。']
    summary+=['','## 新版最终门控','',
              '| bag | 消费端授权周期 | STOP状态周期 | 走廊已核验周期 | 最大连续覆盖 | 处理p50/p95 | 超500ms周期 |',
              '| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for row in result.get('bag_replay',[]):
        b=row['variants'][1]
        summary.append('| %s | %d | %d/%d | %d | %.2f m | %.1f/%.1f ms | %d |' % (
            row['bag'],b['visual_authorized_ticks'],b['corridor_stop_ticks'],b['ticks'],b['corridor_verified_ticks'],
            b['max_coverage_m'],*b['tick_ms_p50_p95'],b['tick_over_500ms']))
    summary+=['','STOP为生产端保持状态周期，非独立事件数或真实停车次数；回放沿用原车位，不模拟停车后的新观测。',
              '旧基线埋点能力见final_gates_instrumented；相关计数0不代表旧路径检查合格；新版若没有激活，违规数0亦不能证明靠右效果。',
              '区分原速度上限1.5 m/s与带路径版本/有效期的实际硬限速；参考候选之外的路段不获执行授权。']
    (args.output_dir/'report.md').write_text('\n'.join(summary)+'\n')
    print(json.dumps(result['snapshots']),flush=True)


if __name__=='__main__': main()
