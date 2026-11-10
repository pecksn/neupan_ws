#!/usr/bin/env python3
"""All eight afternoon bag parts; recorded poses only, no real control outputs."""
import argparse
import json
import math
import runpy
from collections import deque
from pathlib import Path
import sys
from types import SimpleNamespace
import roslaunch
import verify_stable_navigation as harness

ROOT=Path(__file__).resolve().parents[3]


def gnss_checks(output):
    evidence=ROOT/'tracker_logs/review_20261110/afternoon_diagnosis'
    timing=runpy.run_path(str(evidence/'probe_afternoon_timing.py'))
    rows=[timing['run'](tag,limit) for tag in ('151531','154713','154933') for limit in (0.,4.)]
    for row in rows:
        assert row['fault'] is None and not row['auto_failed'],row
        if row['session'] in ('154713','154933'):
            assert row['final_ready']==bool(row['limit_sec']),row
    (output/'timing.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n')
    sys.path.insert(0,str(ROOT/'src/Test/tests'))
    import test_runtime_position as fixture
    import rospy
    from geometry_msgs.msg import QuaternionStamped
    def yaw(q):return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),1-2*(q['y']**2+q['z']**2))
    headings=[];poses=[];guards=[]
    with (evidence/'bag_rosbag20261110_154322_critical.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line);data=row['data']
            if row['topic']=='/heading':headings.append(row)
            elif row['topic']=='/gps_guard/status':guards.append(row)
            elif row['topic']=='/odometry/local':
                p=data['pose']['pose']['position'];q=data['pose']['pose']['orientation']
                poses.append((data['header']['stamp'],(p['x'],p['y'],yaw(q))))
    old,new=next((a,b) for a,b in zip(headings,headings[1:]) if b['t']-a['t']>100)
    prior=next(row['data'] for row in reversed(guards) if row['t']<new['t'])
    test=fixture.RuntimePositionTests(methodName='runTest');test.setUp()
    try:
        test.now=rospy.Time.from_sec(new['t']);test.params['~pvt_period_sec']=1.
        bridge=test.bridge();bridge.transform_yaw=math.radians(prior['transform_yaw_deg'])
        bridge.reference_frame_epoch=bridge.heading_alignment_epoch=prior['reference_frame']['odom_epoch']
        old_ns=old['data']['header']['stamp'];new_ns=new['data']['header']['stamp']
        bridge.last_heading_yaw=yaw(old['data']['quaternion'])
        bridge.last_heading_stamp=rospy.Time(old_ns//10**9,old_ns%10**9)
        bridge.local_pose_history=deque((p for p in poses if new_ns-10*10**9<=p[0]<=new_ns+500000000),maxlen=10000)
        message=QuaternionStamped();message.header.stamp=rospy.Time(new_ns//10**9,new_ns%10**9)
        for key,value in new['data']['quaternion'].items():setattr(message.quaternion,key,value)
        local,_=bridge.local_pose_at(new_ns)
        residual=bridge.normalize_angle(yaw(new['data']['quaternion'])-local[2]-bridge.transform_yaw)
        bridge.heading_cb(message)
        result=dict(receive_gap_sec=new['t']-old['t'],old_history_available=bridge.local_pose_at(old_ns)[0] is not None,
            residual_deg=math.degrees(residual),accepted_alignment_deg=prior['transform_yaw_deg'],
            false_jump=bridge.heading_bad_until>test.now,
            scope='Actual heading callback with bounded history and recorded same-generation accepted alignment; not full localization replay')
        (output/'heading_gap.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
        assert not result['old_history_available'] and not result['false_jump'],result
    finally:test.doCleanups()
    print(json.dumps(dict(timing=rows,heading=result),ensure_ascii=False),flush=True)


def run(output):
    evidence=ROOT/'tracker_logs/review_20261110/afternoon_diagnosis'
    config=roslaunch.config.load_config_default([(str(ROOT/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch'),
        ['outdoor_mode:=true','low_precision_startup:=true','gps_transport_limit_sec:=4.0','gps_common_utc_verified:=true',
         'enable_camera_right_path:=true','enable_camera_right_control:=true',
         'camera_right_strategy:=segment_reference','camera_right_base_link_height_m:=0.20'])],None,verbose=False)
    params={k[len('/camera_right_path/'):]:v.value for k,v in config.params.items() if k.startswith('/camera_right_path/')}
    def namespace(**values):
        if 'get_param' in values:
            original=values['get_param']
            values['get_param']=lambda k,default=None:params.get(k[1:],original(k,default)) if k.startswith('~') else original(k,default)
            for key in ('logdebug_throttle','logdebug','loginfo','logerr','logerr_throttle'):values.setdefault(key,lambda *a,**kw:None)
        return SimpleNamespace(**values)
    harness.NS=namespace;results=[]
    for profile in ('recorded_008','current_005'):
        for tag,parts in (('151533',2),('154322',6)):
            params['vehicle_config_file']=str(evidence/('recorded_vehicle_config_'+tag+'.yaml') if profile=='recorded_008'
                else ROOT/'src/neupan_ros/example/lxw_car/config/neupan_config_outdoor.yaml')
            result=harness.camera_replay(ROOT/'rosbags'/('rosbag20261110_'+tag))
            result['profile']=profile
            accepted=[r for r in result['timeline'] if r.get('reference_accepted')]
            result['first_accepted']=accepted[0] if accepted else None
            (output/(profile+'_'+tag+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            summary={k:v for k,v in result.items() if k not in ('timeline','processing_health','first_accepted')}
            summary['first_accepted']=None if not accepted else {k:accepted[0][k] for k in ('stamp_ns','version','reference_shift_m','source','reference_accepted_stamp','consumer_endpoint_map')}
            print(json.dumps(summary,ensure_ascii=False),flush=True);results.append(summary)
            assert len(result['files'])==parts,result['files']
            assert accepted,'No accepted reference: '+tag
            assert not any(r['source']=='camera' and not r['reference_accepted'] for r in result['timeline'])
            assert all(r['consumer_matches_producer'] for r in accepted if r['source']=='camera')
            assert max(result['reference_versions'])>0,'No near-vehicle correction: '+tag
    (output/'summary.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',type=Path,required=True);args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True);gnss_checks(args.output_dir);run(args.output_dir)
