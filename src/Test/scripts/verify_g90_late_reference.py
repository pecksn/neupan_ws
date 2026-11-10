#!/usr/bin/env python3
"""Nov10 recorded inputs, no ROS master/devices/control. UTC is an explicit
OFFLINE candidate assumption, not inferred hardware synchronization evidence.
Original field_diagnosis files are read-only; write only to --output-dir.
"""
import argparse
import json
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace
from collections import Counter

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'src/robot_base_driver/scripts'),str(ROOT/'src/wheeltec_gps_driver/src')]
from gnss_timebase import GnssTimebase
from libnmea_navsat_driver.observation_time import ObservationTiming


def timing_replays():
    helper=runpy.run_path(str(ROOT/'tracker_logs/review_20261106/startup_diagnosis/replay_clock.py'))
    results=[]
    for tag in ('103150','103558','104156'):
        rows=json.loads((ROOT/'tracker_logs/review_20261110/field_diagnosis'/
                         ('system_run_full_20261110_'+tag+'_time_events.json')).read_text())
        for limit in (0.,4.):
            obs=ObservationTiming('measurement','auto',lambda:True,pvt_period_sec=1.,gga_period_sec=.05,
                transport_limit_sec=limit,common_utc_verified=limit>0)
            gate=GnssTimebase('measurement','auto');accepted=Counter()
            for row in rows:
                obs.clock_probe=lambda row=row:row.get('host_clock_synchronized') is True
                event=obs.observe(helper['sentence'](row),row['host_read_ros_ns'],row['host_read_mono_ns'])
                gate.receive(event,row['host_read_ros_ns'],row['host_read_mono_ns'])
                if event['clock_verified'] and event['epoch_order']=='new':accepted[event['family']]+=1
            result=dict(session=tag,transport_candidate_sec=limit,clock_ready=gate.clock_ready,
                fault=gate.fault,driver_failed=obs.auto_failed,accepted=dict(accepted),
                independent_utc='offline assumption only' if limit else 'legacy auto consistency')
            if limit:assert gate.clock_ready and gate.fault is None,result
            assert not obs.auto_failed and gate.fault is None,result
            results.append(result)
    return results


def camera_replay():
    import roslaunch
    import verify_stable_navigation as harness
    config=roslaunch.config.load_config_default([(str(ROOT/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch'),
        ['outdoor_mode:=true','low_precision_startup:=true','enable_camera_right_path:=true',
         'enable_camera_right_control:=true','camera_right_strategy:=segment_reference',
         'camera_right_base_link_height_m:=0.20'])],None,verbose=False)
    params={k[len('/camera_right_path/'):]:v.value for k,v in config.params.items() if k.startswith('/camera_right_path/')}
    def namespace(**values):
        if 'get_param' in values:
            original=values['get_param']
            values['get_param']=lambda k,default=None:params.get(k[1:],original(k,default)) if k.startswith('~') else original(k,default)
            for key in ('logdebug_throttle','logdebug','loginfo','logerr','logerr_throttle'):
                values.setdefault(key,lambda *a,**kw:None)
        return SimpleNamespace(**values)
    harness.NS=namespace
    result=harness.camera_replay(ROOT/'rosbags/rosbag20261110_103207')
    assert len(result['files'])==3,result['files']
    assert max(result['reference_versions'],default=0)>0,'No actual corrected reference established'
    assert any((r.get('reference_shift_m') or 0)>2.49 and r['consumer_matches_producer']
               for r in result['timeline']),'No measured map correction selected by consumer'
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--timing-only',action='store_true');args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    result=dict(scope=__doc__,timing=timing_replays())
    if not args.timing_only:result['camera']=camera_replay()
    (args.output_dir/'recorded_input_results.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(timing=result['timing'],camera={k:v for k,v in result.get('camera',{}).items()
        if k not in ('timeline','processing_health')}),ensure_ascii=False))
