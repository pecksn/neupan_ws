#!/usr/bin/env python3
"""Independently check comparison clocks, sensors and geometry from C++ rosbags."""
import argparse
import importlib.util
import json
import math
from bisect import bisect_right
from pathlib import Path


def audit_trial(folder):
    import rosbag
    spec=importlib.util.spec_from_file_location('comparison',Path(__file__).with_name('planner_comparison.py'))
    sim=importlib.util.module_from_spec(spec);spec.loader.exec_module(sim)
    manifest=json.loads((folder/'manifest.json').read_text())
    result=json.loads((folder/'result.json').read_text())
    topics={name:[] for name in ('/clock','/scan','/odom','/planner_sim/model_states')}
    start=finish=None
    with rosbag.Bag(str(folder/'telemetry.bag')) as bag:
        for topic,msg,stamp in bag.read_messages():
            t=stamp.to_sec()
            if topic=='/planner_sim/episode_start':start=float(msg.data)
            elif topic=='/planner_sim/episode_finished' and msg.data:finish=t
            elif topic in topics:topics[topic].append((t,msg))
    if start is None or finish is None:
        raise ValueError('Missing episode boundary in '+str(folder))
    checks={}
    for topic,rows in topics.items():
        rows=[(t,m) for t,m in rows if start<=t<=finish]
        topics[topic]=rows
        stamps=[m.clock.to_sec() if topic=='/clock' else m.header.stamp.to_sec()
                if topic in ('/scan','/odom') else t for t,m in rows]
        checks[topic]={'messages':len(rows),'maximum_gap_s':max([b-a for a,b in zip(stamps,stamps[1:])] or [math.inf]),
                       'maximum_record_age_s':max([t-s for (t,_),s in zip(rows,stamps)] or [math.inf])}
    def pose(msg):
        q=msg.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return [msg.position.x,msg.position.y,yaw]
    gaps=[];last_pose=None;pose_errors=[]
    odom_times=[t for t,_ in topics['/odom']]
    for t,msg in topics['/planner_sim/model_states']:
        positions=dict(zip(msg.name,msg.pose))
        if 'lxwcar' not in positions:continue
        last_pose=pose(positions['lxwcar'])
        obstacles=[(positions[name].position.x,positions[name].position.y,radius)
                   for name,_,_,radius in manifest['actor_specs']]
        gaps.append(sim.clearance(last_pose,obstacles))
        if odom_times:
            index=max(0,bisect_right(odom_times,t)-1)
            odom_pose=topics['/odom'][index][1].pose.pose.position
            pose_errors.append(math.hypot(last_pose[0]-odom_pose.x,last_pose[1]-odom_pose.y))
    terminal_error=math.dist(last_pose[:2],sim.GOAL[:2]) if last_pose else math.inf
    speed=topics['/odom'][-1][1].twist.twist.linear.x if topics['/odom'] else math.inf
    continuity=(checks['/clock']['maximum_gap_s']<=.05 and checks['/scan']['maximum_gap_s']<=.2
                and checks['/odom']['maximum_gap_s']<=.1 and checks['/planner_sim/model_states']['maximum_gap_s']<=.1)
    terminal=(last_pose is not None and terminal_error<=sim.GOAL_POSITION_TOLERANCE
              and abs(last_pose[2])<=.35 and abs(speed)<=.1)
    no_overlap=bool(gaps) and min(gaps)>=-.02
    return {'folder':str(folder),'planner':result['planner'],'scenario':result['scenario'],
            'episode_start_s':start,'episode_finish_s':finish,'topics':checks,
            'continuous_transport':continuity,'minimum_bag_body_clearance_m':min(gaps) if gaps else None,
            'maximum_bag_gazebo_odometry_error_m':max(pose_errors) if pose_errors else None,
            'no_sampled_body_overlap':no_overlap,'terminal_pose':last_pose,'terminal_goal_error_m':terminal_error,
            'terminal_speed_mps':speed,'terminal_geometry_verified':terminal,'reported_reason':result['reason'],
            'completed_and_verified':bool(result['passed'] and continuity and no_overlap and terminal)}


def audit(root):
    data=json.loads((root/'comparison.json').read_text())['results']
    rows=[audit_trial(root/r['folder']) for r in data]
    output={'trials':rows,'all_completed_and_verified':all(r['completed_and_verified'] for r in rows),
            'all_transport_continuous':all(r['continuous_transport'] for r in rows),
            'scope':'Independent C++ rosbag observation; finite model-state samples, kinematic simulation, not a physical collision-safety proof'}
    (root/'transport_audit.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'trials':len(rows),'all_transport_continuous':output['all_transport_continuous'],
                      'all_completed_and_verified':output['all_completed_and_verified'],
                      'not_verified':[r['folder'] for r in rows if not r['completed_and_verified']]},indent=2))
    return output


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path)
    audit(parser.parse_args().folder.resolve())
