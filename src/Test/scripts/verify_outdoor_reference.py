#!/usr/bin/env python3
"""Read-only vehicle-log replay. No ROS master, publishers or hardware access.

Replays recorded poses, NOT a counterfactual drive. Cannot reproduce obstacle
avoidance without the original scans; a geometric improvement is not a safety
certification. Run in the lxw/ROS1 environment that contains NeuPAN.
"""
import argparse
import copy
from collections import Counter
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src/osm_map_handler/src'))
from osm_outdoor_start_connection import OutdoorTrackingGuard, resample_forward_reference


def review(filename):
    samples, references = [], []
    receiver = Counter()
    with filename.open() as stream:
        for line in stream:
            row = json.loads(line)
            if row['record_type'] == 'sample':
                samples.append(row['data'])
            elif row['record_type'] == 'neupan_reference_path_event':
                references.append(row['data'])
            elif row['record_type'] == 'hybridnets_timing_event':
                d = row['data']
                receiver[str((d.get('event'), d.get('receiver_reason')))] += 1
    result = {'source': str(filename), 'samples': len(samples),
              'receiver_events': dict(receiver),
              'reference_updates': len(references),
              'camera_status_reasons': dict(Counter(
                  s['camera_right'].get('reason') for s in samples if s.get('camera_right'))),
              'camera_active_samples': sum(bool((s.get('camera_right') or {}).get('active')) for s in samples)}
    if samples:
        result['duration_sec'] = samples[-1]['stamp']-samples[0]['stamp']
    if len(references) != 1:
        result['replay_note'] = 'Single-reference pose replay skipped: route was absent or changed.'
        return result
    from neupan.blocks.initial_path import InitialPath
    reference = references[0]
    source = np.array([[p['x'], p['y'], math.radians(p['yaw_deg']), 1.]
                       for p in reference['poses']])
    result['pose_replay'] = {}
    for name, points in (('original', source), ('resampled_10cm', resample_forward_reference(source))):
        ipath = InitialPath(50, .1, .8, SimpleNamespace(
            kinematics='acker', L=.67, max_speed=np.array([1.5, .35])),
            interval=.09, close_threshold=.3, arrive_threshold=.5, arrive_index_threshold=2)
        ipath.set_initial_path([p.reshape(4,1) for p in points])
        guard, first_stop, forward = OutdoorTrackingGuard(), None, []
        for s in samples:
            odom = s.get('global_odom')
            if not odom or s['stamp'] < reference['msg_stamp']:
                continue
            state = np.array([odom['x'], odom['y'], math.radians(odom['yaw_deg'])]).reshape(3,1)
            if ipath.check_arrive(state):
                continue
            d = ipath.cur_point[:2,0]-state[:2,0]
            report = guard.check(ipath.cur_curve, state, ipath.point_index, s['stamp'])
            if first_stop is None:
                forward.append(float(d @ np.array([math.cos(state[2,0]), math.sin(state[2,0])])))
            if report['latched'] and first_stop is None:
                first_stop = {'time': s['local_time'], **report}
        result['pose_replay'][name] = {
            'points': len(points), 'average_interval_m': ipath.interval,
            'worst_reference_forward_before_guard_m': min(forward) if forward else None,
            'first_guard_stop': first_stop}
    return result


def no_obstacle_ablation(filename, start, end):
    """Diagnostic re-solves, NOT reproduction: original scans/horizon absent.

    Pose is approximated from logged odometry at solve start. Two guessed
    warm-start horizons expose seed sensitivity; neither is the saved solver
    horizon. Never use these actions to command a vehicle.
    """
    snapshot, references, odom, runtime = None, [], [], []
    with filename.open() as stream:
        for line in stream:
            row = json.loads(line)
            data = row.get('data') or {}
            if row['record_type'] == 'neupan_config':
                snapshot = data
            elif row['record_type'] == 'neupan_reference_path_event':
                references.append(data)
            elif row['record_type'] == 'sample' and data.get('global_odom'):
                p = data['global_odom']
                odom.append((p['msg_stamp'], p['x'], p['y']))
            elif row['record_type'] == 'neupan_runtime_event':
                runtime.append(data)
    if snapshot is None:
        raise ValueError('missing planner snapshot')
    checkpoint = Path(snapshot['dune_checkpoint'])
    if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != snapshot['checkpoint_sha256']:
        raise ValueError('recorded checkpoint hash differs; refusing ablation')
    import torch
    from neupan import neupan
    torch.set_num_threads(1)
    config = copy.deepcopy(snapshot['planner_config'])
    for key in ('robot', 'ipath', 'pan', 'adjust', 'train'):
        config[key+'_kwargs'] = config.pop(key, {})
    config['pan_kwargs']['dune_checkpoint'] = str(checkpoint)
    planner = neupan(**config)
    history = np.unique(np.asarray(odom, float), axis=0)
    history = history[np.argsort(history[:, 0])]
    results, previous, last_sample = [], None, -float('inf')
    for actual in runtime:
        stamp = actual.get('stamp', 0.)
        hms = datetime.fromtimestamp(stamp, timezone(timedelta(hours=8))).strftime('%H:%M:%S')
        prior = previous
        previous = actual
        if not start <= hms <= end or 'solver_ms' not in actual or stamp-last_sample < .8:
            continue
        if 'robot_yaw_deg' not in actual or 'path_point_index' not in actual:
            continue
        available = [r for r in references if r['msg_stamp'] <= stamp]
        if not available:
            continue
        ref = available[-1]
        path = [np.array([p['x'], p['y'], math.radians(p['yaw_deg']), 1.]).reshape(4,1)
                for p in ref['poses']]
        source_time = stamp-actual['solver_ms']/1000.
        if np.min(abs(history[:,0]-source_time)) > .2:
            continue
        state = np.array([np.interp(source_time, history[:,0], history[:,1]),
                          np.interp(source_time, history[:,0], history[:,2]),
                          math.radians(actual['robot_yaw_deg'])]).reshape(3,1)
        row = {'time': hms, 'stamp': stamp, 'estimated_pose': state[:,0].tolist(),
               'actual_steering_deg': actual.get('steering_deg'),
               'actual_tracking_guard': actual.get('tracking_guard'),
               'actual_reference_forward_m': actual.get('reference_forward_m'), 'without_obstacles': {}}
        for label, steer in (('straight_seed', 0.),
                             ('previous_action_repeated_seed', math.radians((prior or {}).get('steering_deg', 0.)))):
            planner.set_initial_path(copy.deepcopy(path))
            planner.reset()
            planner.ipath.point_index = int(actual['path_point_index'])
            planner.cur_vel_array[0,:] = float((prior or {}).get('linear_x', .8))
            planner.cur_vel_array[1,:] = steer
            action, info = planner(state, None)
            row['without_obstacles'][label] = {'linear_x': float(action[0,0]),
                'steering_deg': math.degrees(float(action[1,0])), 'arrive': bool(info['arrive'])}
        results.append(row)
        last_sample = stamp
    return {'source': str(filename), 'checkpoint_sha256': snapshot['checkpoint_sha256'],
            'limitation': 'Approximate pose and guessed warm-start horizons, no original scans. '
                          'These re-solves cannot prove the original obstacle geometry or sole cause.',
            'results': results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', nargs='+', type=Path)
    parser.add_argument('--bag', type=Path)
    parser.add_argument('--no-obstacle-window', nargs=2, metavar=('START_HMS','END_HMS'),
                        help='Optional diagnostic solver ablation, HH:MM:SS in Asia/Shanghai, first log only')
    parser.add_argument('--output', type=Path, required=True,
                        help='New report JSON; refuses to overwrite an existing report')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; choose a new report filename')
    result = {'limitation': 'Recorded-pose replay only; no obstacle solver or real vehicle validation.',
              'logs': [review(p) for p in args.logs]}
    if args.no_obstacle_window:
        result['no_obstacle_ablation'] = no_obstacle_ablation(args.logs[0], *args.no_obstacle_window)
    if args.bag:
        import rosbag
        with rosbag.Bag(str(args.bag)) as bag:
            topics = bag.get_type_and_topic_info().topics
            result['bag'] = {'source': str(args.bag), 'topics': {
                name: info.message_count for name, info in topics.items()
                if name.startswith(('/hybridnets/', '/camera_right/', '/camera/remote_'))}}
            result['bag']['result_json_messages'] = topics['/hybridnets/result_json'].message_count \
                if '/hybridnets/result_json' in topics else 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print('Report:', args.output)


if __name__ == '__main__':
    main()
