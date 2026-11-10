#!/usr/bin/env python3
"""Recorded timing drives actual control branches, not recorded vehicle motion.

Native actions are valid synthetic solver outputs: rejected original actions
were not recorded. Historical JSON is read only and still contains 12 stops.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import runpy
import sys

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src/Test/tests'))
from test_command_publication_budget import recorded_case,FIXTURE
import test_control_mode_mux as mux_fixture


def mux_gap(gap):
    """Exercise unchanged mux timeout; no command refresh during the gap."""
    test=mux_fixture.SafetyHeartbeatTests(methodName='runTest');test.setUp()
    try:
        mux=mux_fixture.ControlModeMux();test.auto(mux)
        original=mux.last_auto_time.to_sec()
        test.monotonic+=gap;test.now=mux_fixture.rospy.Time.from_sec(100.+gap)
        mux.localization_safety_callback(mux_fixture.Bool(data=True))
        command,reason=mux.selected_command(test.now)
        assert mux.last_auto_time.to_sec()==original
        return dict(gap_sec=gap,reason=reason,speed=command.linear.x,
            original_command_time_not_renewed=True,auto_timeout_sec=mux.auto_timeout)
    finally:test.doCleanups()


def compact(result):
    native=[r for r in result['events'] if r['command_source']=='neupan_native']
    return dict(published=result['commands'],solver_calls=result['node'].neupan_planner.calls,
        runtime_sources=dict(Counter(r['command_source'] for r in result['events'])),
        first_publication_ages=native[0]['publish_input_ages'] if native else None,
        first_timing=native[0]['timing'] if native else None)


def interval_stats(values):
    values=sorted(float(v) for v in values if v is not None)
    return None if not values else dict(count=len(values),min=min(values),max=max(values),
        median=values[len(values)//2],over_500ms=sum(v>500. for v in values))


def recorded_sessions():
    base=ROOT/'tracker_logs/review_20261110/afternoon_diagnosis'
    sessions={}
    for tag in ('151533','154322'):
        commands=[];runtime=[];scans=[];lio=[]
        with (base/('bag_rosbag20261110_'+tag+'_critical.jsonl')).open() as stream:
            for line in stream:
                if line.startswith('{"topic": "/cmd_vel/auto"'):commands.append(json.loads(line))
                elif line.startswith('{"topic": "/neupan/runtime"'):runtime.append(json.loads(line))
                elif line.startswith('{"topic": "/scan"'):
                    r=json.loads(line);scans.append((r['bag_ns'],r['data']['header']['stamp']))
                elif line.startswith('{"topic": "/odometry/local"'):
                    r=json.loads(line);lio.append((r['bag_ns'],r['data']['header']['stamp']))
        sessions[tag]=dict(commands=commands,runtime=runtime,scans=scans,lio=lio)
    return sessions


def pure_recording():
    file=ROOT/'tracker_logs/system_run_full_20261110_150622.jsonl'
    native={};audits=[];retry=[]
    with file.open() as stream:
        for line in stream:
            r=json.loads(line);kind=r.get('record_type');data=r.get('data') or {}
            if kind=='sample':
                n=data.get('neupan_runtime') or {}
                if n.get('command_source')=='neupan_native':native[n.get('cycle_id',n['stamp'])]=n
            elif kind=='command_audit_event':audits.append(data)
            elif kind=='neupan_failure_event' and data.get('command_source')=='calculation_retry':retry.append(data)
    moving=[v for v in native.values() if v.get('linear_x',0)>.2]
    start=min(v['stamp'] for v in moving);end=max(v['stamp'] for v in moving)
    written=[v for v in audits if start<=v['stamp']<=end and v.get('write_status')=='written']
    retries=[v for v in retry if start<=v['stamp']<=end]
    return dict(session='150622',scope='Original sampled runtime plus full serial-write events, not modified replay',
        native_sampled_cycles=len(native),moving_written_commands=len(written),
        moving_written_zeros=sum(abs(v['command_speed_mps'])<.001 for v in written),
        retries_during_motion=len(retries),speed_range=[min(v['linear_x'] for v in moving),max(v['linear_x'] for v in moving)],
        native_command_interval_ms=interval_stats([v.get('command_interval_ms') for v in native.values()]),
        steering_deg_range=[min(v['steering_deg'] for v in moving),max(v['steering_deg'] for v in moving)])


def main():
    p=argparse.ArgumentParser();p.add_argument('--output-dir',type=Path,required=True);args=p.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=True)
    sessions=recorded_sessions();cases=json.loads(FIXTURE.read_text())['events'];results=[]
    for case in cases:
        stream=sessions[case['session']]
        started=(case['input_ages']['scan_stamp_ns']+round(case['input_ages']['scan_measurement_age_ms']*1.e6)
                 -round(case['preparation_ms']*1.e6))
        for key,topic in (('current_scan_arrivals','scans'),('current_lio_arrivals','lio')):
            case[key]=[((received-started)*1.e-9,(source-started)*1.e-9) for received,source in stream[topic]
                       if started-10**9<=received<=started+3*10**9]
            assert len(case[key])>=10,case['event']
        old=recorded_case(case,.50);new=recorded_case(case,.60)
        assert any(v['command_source']=='calculation_retry' for v in old['events'])
        assert new['commands'] and all(v['speed']==.8 and v['steering']==.12 for v in new['commands'])
        assert not any(v['command_source']=='calculation_retry' for v in new['events'])
        rejection=next(r for r in stream['runtime'] if r['data'].get('reason')==case['reason'])
        stamp=rejection['t']
        before=max(r['t'] for r in stream['commands'] if r['t']<stamp and r['data']['linear']['x']>.2)
        after=min(r['t'] for r in stream['commands'] if r['t']>stamp and r['data']['linear']['x']>.2)
        result=dict(session=case['session'],event=case['event'],baseline_age_050=compact(old),candidate_age_060=compact(new),
            current_stream_scope='Actual recorded /scan and /odometry/local capture/arrival epochs, transformed into fixture clock; not regenerated as now()',
            recorded_nonzero_gap_sec=after-before,
            single_cycle_mux_timing_estimate=mux_gap(stamp-before),
            interval_scope='Timing-only estimate: publish at historical rejection time instead of retrying; not counterfactual trajectory or measured new command interval')
        results.append(result)
    # Re-run the historical detector without its main() writing old evidence.
    probe=runpy.run_path(str(ROOT/'tracker_logs/review_20261110/afternoon_diagnosis/probe_motion_stutter.py'))
    historical=[probe['summary'](tag) for tag in ('151533','154322')]
    assert sum(len(s['interruptions']) for s in historical)==12
    pure=pure_recording();assert pure['moving_written_zeros']==0 and pure['retries_during_motion']==0
    summary=dict(scope=__doc__,critical_cases=len(results),candidate_no_zero_no_retry=len(results),
        source_age_candidate_sec=.60,total_compute_budget_sec=.50,current_scan_timeout_sec=.50,
        unchanged_mux=mux_gap(.501),historical_original_interruptions=12,
        single_cycle_mux_estimate_timeout_count=sum(v['single_cycle_mux_timing_estimate']['reason']=='auto_timeout' for v in results),
        pure_neupan_original=pure,
        original_camera_sessions=[dict(
            **{k:v for k,v in r.items() if k in ('session','native_cycles','retry_during_motion','native_speed_range')},
            native_steering_deg_range=[min(x['data']['steering_deg'] for x in sessions[r['session']]['runtime']
                if x['data'].get('command_source')=='neupan_native'),
                max(x['data']['steering_deg'] for x in sessions[r['session']]['runtime']
                if x['data'].get('command_source')=='neupan_native')],
            recorded_nonzero_auto_interval_ms=interval_stats([
                (b-a)*1000 for a,b in zip(
                    [v['t'] for v in sessions[r['session']]['commands'] if v['data']['linear']['x']>.2],
                    [v['t'] for v in sessions[r['session']]['commands'] if v['data']['linear']['x']>.2][1:])])) for r in historical],
        source_sha256={str(file.relative_to(ROOT)):hashlib.sha256(file.read_bytes()).hexdigest() for file in
            [ROOT/'src/neupan_ros/src/neupan_core.py',ROOT/'src/robot_base_driver/scripts/control_mode_mux.py']})
    (args.output_dir/'timing_comparison.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n')
    (args.output_dir/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
