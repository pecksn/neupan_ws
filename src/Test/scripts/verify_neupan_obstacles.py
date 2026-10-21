#!/usr/bin/env python3
"""Read-only NeuPAN bag audit. No ROS node, publisher, master or serial port.

Pairs the last scan received before each recorded plan. This does not recreate
the original callback scheduling or optimiser warm-start. Pairing errors are
reported explicitly; offline re-solves are diagnostic, not original commands.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rosbag
import rospy
import tf
import torch


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def rotation(a):
    return np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])


def poses(msg):
    return np.array([[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)]
                     for p in msg.poses])


def signed_rectangle(points, front, rear, half_width):
    x, y = points[:, 0], points[:, 1]
    distance = np.hypot(np.maximum(np.maximum(rear-x, x-front), 0),
                        np.maximum(abs(y)-half_width, 0))
    inside = (x >= rear) & (x <= front) & (abs(y) <= half_width)
    distance[inside] = -np.minimum.reduce([x[inside]-rear, front-x[inside],
                                         half_width-abs(y[inside])])
    return distance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--log', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=35.)
    parser.add_argument('--replay-count', type=int, default=3)
    args = parser.parse_args()
    torch.set_num_threads(1)
    from neupan import neupan
    from neupan.configuration import np_to_tensor, tensor_to_np

    snapshot = None
    with open(args.log) as stream:
        for line in stream:
            row = json.loads(line)
            if row.get('record_type') == 'neupan_config':
                snapshot = row['data']
                break
    if snapshot is None:
        raise ValueError('No recorded planner configuration')
    checkpoint = Path(snapshot['dune_checkpoint'])
    digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    if digest != snapshot['checkpoint_sha256']:
        raise ValueError('Checkpoint differs from recorded SHA256; refusing model comparison')
    config = copy.deepcopy(snapshot['planner_config'])
    for key in ('robot', 'ipath', 'pan', 'adjust', 'train'):
        config[key+'_kwargs'] = config.pop(key, {})
    config['pan_kwargs']['dune_checkpoint'] = str(checkpoint)
    planner = neupan(**config)
    front = float(planner.robot.vertices[0].max())
    rear = float(planner.robot.vertices[0].min())
    half_width = float(abs(planner.robot.vertices[1]).max())
    params = snapshot['node_parameters']
    if params.get('scan_self_filter', False) or params.get('flip_angle', False):
        raise ValueError('This audit does not implement body self-filter or flipped scans')
    low, high = map(float, params['scan_range'].replace(',', ' ').split())
    amin, amax = map(float, params['scan_angle_range'].split())
    tf_buffer = tf.Transformer(True, rospy.Duration(180.))
    with rosbag.Bag(args.bag) as bag:
        end = bag.get_end_time()
        start = max(bag.get_start_time(), end-args.seconds)
        # Load transforms ahead of the scan pass so interpolation can use both
        # sides; no live TF listener or ROS master is involved.
        for _, msg, _ in bag.read_messages(topics=['/tf', '/tf_static'],
                start_time=rospy.Time.from_sec(max(bag.get_start_time(), start-2))):
            for transform in msg.transforms:
                tf_buffer.setTransform(transform, 'offline_bag')
        latest_scan = latest_ref = None
        selected = []
        previous_second = None
        for topic, msg, received in bag.read_messages(
                topics=['/scan', '/neupan_plan', '/neupan_ref_state'],
                start_time=rospy.Time.from_sec(start)):
            if topic == '/neupan_ref_state':
                latest_ref = (received.to_sec(), poses(msg))
                continue
            if topic == '/scan':
                r = np.asarray(msg.ranges)
                a = msg.angle_min+np.arange(len(r))*msg.angle_increment
                good = np.isfinite(r) & (r >= low) & (r <= high) & (a > amin) & (a < amax)
                good &= np.arange(len(r)) % int(params['scan_downsample']) == 0
                local = np.c_[r[good]*np.cos(a[good]), r[good]*np.sin(a[good])]
                if len(local) == 0:
                    latest_scan = None
                    continue
                try:
                    trans, quat = tf_buffer.lookupTransform('map', msg.header.frame_id, msg.header.stamp)
                except tf.Exception:
                    latest_scan = None
                    continue
                heading = tf.transformations.euler_from_quaternion(quat)[2]
                world = local @ rotation(heading).T+np.asarray(trans[:2])
                latest_scan = (received.to_sec(), world, np.array([*trans[:2], heading]))
                continue
            if latest_scan is None or not msg.poses or int(received.to_sec()) == previous_second:
                continue
            previous_second = int(received.to_sec())
            path = poses(msg)
            scan_time, world, scan_pose = latest_scan
            gap = received.to_sec()-scan_time
            if gap > .2 or len(path) != planner.T+1:
                continue
            nominal = np_to_tensor(path.T)
            obstacles = np_to_tensor(world.T)
            flow, rotations, world_list = planner.pan.generate_point_flow(nominal, obstacles)
            with torch.no_grad():
                mus, lams, sorted_points = planner.pan.dune_layer(flow, rotations, world_list)
            sampled = tensor_to_np(world_list[0]).T
            full_clear, sample_clear, learned_clear = [], [], []
            for state in path:
                local_full = (world-state[:2]) @ rotation(state[2])
                local_sample = (sampled-state[:2]) @ rotation(state[2])
                full_clear.append(float(signed_rectangle(local_full, front, rear, half_width).min()))
                sample_clear.append(float(signed_rectangle(local_sample, front, rear, half_width).min()))
                with torch.no_grad():
                    points = np_to_tensor(local_sample.T)
                    mu = planner.pan.dune_layer.model(points.T).T
                    distances = planner.pan.dune_layer.cal_objective_distance(mu, points)
                    learned_clear.append(float(distances.min()))
            row = {
                'stamp': received.to_sec(), 'scan_to_plan_receive_ms': gap*1000,
                'scan_pose_to_plan_start_m': float(np.linalg.norm(scan_pose[:2]-path[0,:2])),
                'scan_pose_to_plan_start_yaw_deg': float(math.degrees(math.atan2(
                    math.sin(scan_pose[2]-path[0,2]), math.cos(scan_pose[2]-path[0,2])))),
                'input_points': len(world), 'dune_points': len(sampled),
                'current_exact_clearance_m': full_clear[0],
                'current_sampled_clearance_m': sample_clear[0],
                'current_learned_clearance_m': learned_clear[0],
                'future_exact_min_m': min(full_clear[1:]),
                'future_sampled_min_m': min(sample_clear[1:]),
                'future_learned_min_m': min(learned_clear[1:]),
                'first_full_overlap_step': next((i for i, d in enumerate(full_clear) if d < 0), None),
                'stop_under_recorded_threshold': learned_clear[0] < planner.collision_threshold,
            }
            reference = latest_ref[1] if latest_ref and abs(received.to_sec()-latest_ref[0]) < .3 else None
            selected.append((row, path, world, sampled, reference))

    # Prefer nearly stationary / well-aligned pairs over the visually worst
    # pairs: large pose mismatch could itself explain apparent penetration.
    reliable = [entry for entry in selected
                if entry[0]['scan_pose_to_plan_start_m'] < .02
                and abs(entry[0]['scan_pose_to_plan_start_yaw_deg']) < 1.]
    replays = []
    recorded_point_limit = planner.pan.dune_max_num
    candidates = sorted((entry for entry in reliable if entry[4] is not None),
                        key=lambda e: e[0]['future_exact_min_m'])[:args.replay_count]
    for row, path, world, sampled, reference in candidates:
        # Infer nominal controls from the recorded trajectory, NOT from the
        # unrecorded solver warm-start. Use this only as a controlled re-solve.
        delta = np.diff(path[:,:2], axis=0)
        speeds = np.sum(delta*np.c_[np.cos(path[:-1,2]), np.sin(path[:-1,2])], axis=1)/planner.dt
        dyaw = np.arctan2(np.sin(np.diff(path[:,2])), np.cos(np.diff(path[:,2])))
        steer = np.arctan(planner.robot.L*dyaw/np.maximum(abs(speeds)*planner.dt, 1e-5))
        controls = np.vstack([speeds, steer])
        for variant, limit in (('recorded_point_limit', recorded_point_limit),
                               ('all_scan_points', len(world))):
            try:
                # Local diagnostic instance only; no config/checkpoint edits.
                planner.pan.dune_max_num = limit
                planner.pan.current_nom_values = [None, None, None, None]
                state, action, _ = planner.pan(np_to_tensor(path.T), np_to_tensor(controls),
                    np_to_tensor(reference.T), np_to_tensor(np.full(planner.T, planner.ref_speed)),
                    np_to_tensor(world.T))
                result = tensor_to_np(state).T
                clearance = [float(signed_rectangle((world-p[:2]) @ rotation(p[2]),
                            front, rear, half_width).min()) for p in result]
                replays.append({'stamp': row['stamp'], 'variant': variant, 'point_limit': limit,
                                'action': tensor_to_np(action)[:,0].tolist(),
                                'future_exact_min_m': min(clearance[1:]),
                                'planner_current_min_m': float(planner.min_distance),
                                'planner_check_stop': bool(planner.check_stop())})
            except Exception as exc:
                replays.append({'stamp': row['stamp'], 'variant': variant, 'error': str(exc)})
    planner.pan.dune_max_num = recorded_point_limit

    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    report = {'bag': args.bag, 'checkpoint': str(checkpoint), 'checkpoint_sha256': digest,
              'geometry': {'rear_x': rear, 'front_x': front, 'half_width': half_width},
              'collision_threshold': planner.collision_threshold,
              'summary': {'examined_pairs': len(selected),
                  'close_pose_pairs': len(reliable),
                  'close_pose_pairs_with_future_overlap': sum(e[0]['future_exact_min_m'] < 0 for e in reliable),
                  'close_pose_overlap_lost_after_sampling': sum(
                      e[0]['future_exact_min_m'] < 0 <= e[0]['future_sampled_min_m'] for e in reliable)},
              'limitation': 'Nearest preceding scan paired by bag receive order; TF at scan source time. Original latest-TF callback scheduling and warm-start are not recorded. Horizon geometry assumes stationary obstacles and the recorded model footprint; it does not establish actual wall contact or measured vehicle geometry.',
              'frames': [entry[0] for entry in selected], 'conditional_resolves': replays}
    (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
    if selected:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        from datetime import datetime, timedelta, timezone
        for ax, entry in zip(axes, sorted(reliable or selected, key=lambda e: e[0]['future_exact_min_m'])[:2]):
            row, path, world, sampled, reference = entry
            origin, heading = path[0,:2], path[0,2]
            points = (world-origin) @ rotation(heading)
            points_sample = (sampled-origin) @ rotation(heading)
            trajectory = (path[:,:2]-origin) @ rotation(heading)
            ax.scatter(points[:,0], points[:,1], s=9, c='gray', label='All scan points')
            ax.scatter(points_sample[:,0], points_sample[:,1], s=10, c='orange', label='DUNE sampled points')
            ax.plot(trajectory[:,0], trajectory[:,1], 'b-', label='Recorded plan')
            corners = np.array([[rear,-half_width],[front,-half_width],[front,half_width],[rear,half_width],[rear,-half_width]])
            for i in range(0,len(path),5):
                polygon = (corners @ rotation(path[i,2]).T+path[i,:2]-origin) @ rotation(heading)
                ax.plot(polygon[:,0],polygon[:,1],color='red',alpha=.4)
            ax.set(xlim=(-2,6),ylim=(-4,4),aspect='equal',xlabel='Forward m',ylabel='Left m',
                   title='%s (+08), future clearance %.3fm' % (
                       datetime.fromtimestamp(row['stamp'], timezone(timedelta(hours=8))).strftime('%H:%M:%S'),
                       row['future_exact_min_m']))
            ax.grid();ax.legend(fontsize=7)
        fig.tight_layout();fig.savefig(output/'trajectory_scan.png',dpi=150);plt.close(fig)
    print(json.dumps({'summary': report['summary'], 'conditional_resolves': replays,
                      'output': str(output)}, ensure_ascii=False))


if __name__ == '__main__':
    main()
