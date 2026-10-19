#!/usr/bin/env python3
"""Isolated, repeatable Gazebo comparison; no hardware launches or parameters."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone, timedelta

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[3]
DYNAMIC_SCENARIOS = ('lead_slow', 'lead_stop', 'crossing_once')
SCENARIOS = ('open', 'static') + DYNAMIC_SCENARIOS
AVAILABLE_SCENARIOS = SCENARIOS + ('crossing', 'crossing_stress')
GOAL = (8.0, 0.0, 0.0)
GOAL_POSITION_TOLERANCE = .2
LENGTH, WIDTH, CENTRE = .95, .75, .335
ACTOR_SPEED = .7
ACTOR_YIELD_GAP = .60


def reference_points(start, target, scenario):
    """Common reference; the fixed island is bypassed before local planning.

    The scripted comparison goal has a smooth, kinematically feasible bypass.
    Arbitrary manual goals retain the original straight reference behavior.
    """
    dx, dy = target[0]-start[0], target[1]-start[1]
    length = math.hypot(dx, dy)
    bypass = scenario == 'crossing' and math.hypot(*start) < .05 and math.hypot(target[0]-8, target[1]) < .05
    points = []
    count = max(2, math.ceil(length/.06)+1)
    for i in range(count):
        t = i/(count-1)
        offset = 1.35*math.sin(math.pi*t)**2 if bypass else 0.
        derivative = 1.35*math.pi*math.sin(2*math.pi*t) if bypass else 0.
        points.append((start[0]+t*dx, start[1]+t*dy+offset, math.atan2(dy+derivative, dx)))
    return points


def obstacle_specs(scenario):
    if scenario == 'open':
        return []
    if scenario == 'static':
        return [('blocker', 3.8, 0., .4), ('side_obstacle', 5.6, -1.5, .35)]
    if scenario == 'crossing':
        return [('traffic_island', 3.8, 0., .4),
                ('crossing_a', 5.2, 3.0, .35), ('crossing_b', 6.6, -3.0, .35)]
    if scenario == 'crossing_stress':
        return [('crossing_a', 3.0, 2.5, .35), ('crossing_b', 5.2, -2.5, .35)]
    if scenario == 'lead_slow':
        return [('lead_vehicle', 2.4, 0., .35)]
    if scenario == 'lead_stop':
        return [('lead_vehicle', 2.8, 0., .35)]
    if scenario == 'crossing_once':
        return [('crossing_once_actor', 4.2, 3., .35)]
    raise ValueError('Unknown scenario: '+scenario)


def obstacle_position(spec, scenario, elapsed):
    name, x, y, radius = spec
    if scenario == 'crossing_stress':
        period = 20. if name == 'crossing_a' else 26.
        y *= math.cos(2.*math.pi*elapsed/period)
    elif scenario == 'crossing' and name != 'traffic_island':
        delay = 4.0 if name == 'crossing_a' else 6.0
        progress = min(2*abs(y), ACTOR_SPEED*max(0., elapsed-delay))
        y -= math.copysign(progress, y)
    elif scenario == 'lead_slow':
        x = min(10., x+.18*max(0., elapsed))
    elif scenario == 'lead_stop':
        x = min(10., x+.22*(min(6., max(0., elapsed))+max(0., elapsed-10.)))
    elif scenario == 'crossing_once':
        y = max(-3., y-.6*max(0.,elapsed))
    return x, y, radius


def moving_actor(scenario, name):
    return scenario in DYNAMIC_SCENARIOS + ('crossing', 'crossing_stress') and name != 'traffic_island'


def dynamic_feasibility(scenario):
    """Offline geometric/kinematic witness, separate from either planner.

    A 0.42 m/s x progression around a smooth bypass overtakes each lead actor.
    This witnesses a feasible option; it does not command the tested robot.
    """
    if scenario not in DYNAMIC_SCENARIOS:
        raise ValueError(scenario)
    if scenario == 'crossing_once':
        poses=[];gaps=[]
        for i in range(3001):
            t=30*i/3000
            x=min(8.,max(0.,t-9.)*.42)
            pose=(x,0.,0.)
            poses.append((t,*pose));gaps.append(clearance(pose,[obstacle_position(s,scenario,t) for s in obstacle_specs(scenario)]))
        return {'scenario':scenario,'verification':'offline witness: wait at spawn until t=9 s, then proceed; not provided to planners',
                'minimum_body_clearance_m':min(gaps),'maximum_speed_mps':.42,'maximum_abs_steering_rad':0.,
                'arrival_time_s':9.+8/.42,'passed':min(gaps)>.2,'witness_poses':poses}
    poses, gaps, speeds, steers = [], [], [], []
    for i in range(2001):
        x = 8*i/2000
        t = x/.42
        y = 1.35*math.sin(math.pi*x/8)**2
        dy = 1.35*math.pi/8*math.sin(2*math.pi*x/8)
        ddy = 2*1.35*(math.pi/8)**2*math.cos(2*math.pi*x/8)
        pose = (x,y,math.atan(dy))
        gap = clearance(pose, [obstacle_position(s,scenario,t) for s in obstacle_specs(scenario)])
        poses.append((t,*pose)); gaps.append(gap)
        speeds.append(.42*math.hypot(1.,dy)); steers.append(math.atan(.67*ddy/(1+dy*dy)**1.5))
    return {'scenario':scenario,'verification':'offline geometric/kinematic witness, not a planner or a Gazebo trial',
            'minimum_body_clearance_m':min(gaps),'maximum_speed_mps':max(speeds),
            'maximum_abs_steering_rad':max(abs(a) for a in steers),'arrival_time_s':poses[-1][0],
            'passed':min(gaps)>.2 and max(speeds)<=.5 and max(abs(a) for a in steers)<=.35,
            'witness_poses':poses}


def hesitation_metrics(records):
    """Stops >=0.3 s after moving, >0.4 m from goal; resume above 0.08 m/s."""
    armed, pending, intervals = False, None, []
    first_movement = None
    for row in records:
        t, speed = row['t'], abs(row['speed'])
        far = math.hypot(row['pose'][0]-GOAL[0],row['pose'][1]-GOAL[1]) > .4
        if speed > .08:
            armed = True
            if first_movement is None: first_movement = t
            if pending is not None:
                if t-pending >= .3:
                    intervals.append({'start_s':pending,'end_s':t,'duration_s':round(t-pending,3),'resumed':True})
                pending = None
        elif armed and far and speed < .03 and pending is None:
            pending = t
        elif not far and pending is not None:
            pending = None
    if pending is not None and records[-1]['t']-pending >= .3:
        intervals.append({'start_s':pending,'end_s':records[-1]['t'],
                          'duration_s':round(records[-1]['t']-pending,3),'resumed':False})
    return {'stop_definition':'abs(v)<0.03, >=0.3 s, after first movement, >0.4 m from goal; resume abs(v)>0.08',
            'stop_intervals':intervals,'stop_count':len(intervals),
            'stop_total_s':round(sum(r['duration_s'] for r in intervals),3),
            'longest_stop_s':max((r['duration_s'] for r in intervals),default=0.),
            'initial_wait_s':round(first_movement if first_movement is not None else records[-1]['t'],3),
            'invalid_control_samples':sum(r.get('controller_runtime',{}).get('valid') is False and
                r.get('controller_runtime',{}).get('attempted',False) and
                0 <= r.get('controller_runtime_age_s',float('inf')) <= .5 for r in records),
            'command_watchdog_samples':sum(r.get('gate_health',{}).get('command_ok') is False for r in records),
            'scan_watchdog_samples':sum(r.get('gate_health',{}).get('scan_ok') is False for r in records)}


def predict_ego(ego, dt):
    x, y, yaw, speed, yaw_rate = ego
    if abs(yaw_rate) < 1e-6:
        return x+speed*math.cos(yaw)*dt, y+speed*math.sin(yaw)*dt, yaw
    end = yaw+yaw_rate*dt
    return (x+speed/yaw_rate*(math.sin(end)-math.sin(yaw)),
            y+speed/yaw_rate*(math.cos(yaw)-math.cos(end)), end)


class CrossingActors:
    """Shared actor policy, independent of which robot planner is running.

    Actors traverse once, pause before a predicted encounter, then continue
    from the held path phase. They never catch up by teleporting after a hold.
    Robot ground truth is used only by simulated actors, never by planners.
    """
    def __init__(self):
        self.specs = [s for s in obstacle_specs('crossing') if s[0] != 'traffic_island']
        self.states = {s[0]: {'phase_s': 0., 'yielding': False, 'yield_events': 0,
                              'yield_duration_s': 0.} for s in self.specs}
        self.last_elapsed = 0.

    def update(self, elapsed, ego):
        dt = max(0., min(.1, elapsed-self.last_elapsed))
        self.last_elapsed = elapsed
        output = []
        for spec in self.specs:
            state = self.states[spec[0]]
            phase = state['phase_s']
            # Don't withhold the spawn/start delay. Only moving actors yield.
            before = obstacle_position(spec, 'crossing', phase)
            after = obstacle_position(spec, 'crossing', phase+dt)
            moving = math.hypot(after[0]-before[0], after[1]-before[1]) > 1e-8
            predicted = [clearance(predict_ego(ego, horizon),
                         [obstacle_position(spec, 'crossing', phase+dt+horizon)])
                         for horizon in (0., .25, .5, .75, 1., 1.25, 1.5)]
            yielding = moving and min(predicted) < ACTOR_YIELD_GAP
            if yielding:
                state['yield_duration_s'] += dt
                if not state['yielding']: state['yield_events'] += 1
            else:
                state['phase_s'] += dt
            state['yielding'] = yielding
            output.append(obstacle_position(spec, 'crossing', state['phase_s']))
        return output


def clearance(pose, obstacles):
    """Signed circle-to-oriented-body distance, using the actual rear axle."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(x+c*dx-s*dy, y+s*dx+c*dy)
               for dx in (-.14, .81) for dy in (-WIDTH/2, WIDTH/2)]
    distances = [min(px+3.95, 11.95-px, py+4.95, 4.95-py) for px, py in corners]
    for ox, oy, radius in obstacles:
        dx, dy = ox-x, oy-y
        local_x, local_y = c*dx+s*dy-CENTRE, -s*dx+c*dy
        qx, qy = abs(local_x)-LENGTH/2, abs(local_y)-WIDTH/2
        signed_box = math.hypot(max(qx, 0.), max(qy, 0.)) + min(max(qx, qy), 0.)
        distances.append(signed_box-radius)
    return min(distances)


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')


def scene():
    import rospy
    import tf
    import numpy as np
    from gazebo_msgs.msg import ModelState
    from gazebo_msgs.srv import SpawnModel
    from geometry_msgs.msg import Pose, PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Float64, String

    rospy.init_node('comparison_scene')
    scenario = rospy.get_param('~scenario', 'open')
    specs = obstacle_specs(scenario)
    listener = tf.TransformListener()
    pending_scan, raw_command, command_stamp = [None], [Twist()], [None]
    episode_start, odom, finished, scan_enabled = [None], [None], [False], [True]
    actors = [CrossingActors()]
    scan_stamp = [None]
    scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
    drive_pub = rospy.Publisher('/planner_sim/cmd', Twist, queue_size=1)
    state_pub = rospy.Publisher(rospy.get_param('~model_state_command_topic','/gazebo/set_model_state'), ModelState, queue_size=10)
    path_pub = rospy.Publisher('/initial_path', RosPath, queue_size=1, latch=True)
    ready_pub = rospy.Publisher('/planner_sim/scene_ready', Bool, queue_size=1, latch=True)
    health_pub = rospy.Publisher('/planner_sim/gate_health', String, queue_size=1)
    actor_pub = rospy.Publisher('/planner_sim/actor_states', String, queue_size=1, latch=True)

    def command(msg):
        raw_command[0], command_stamp[0] = msg, rospy.get_time()

    def on_goal(msg):
        if msg.header.frame_id.lstrip('/') != 'odom' or odom[0] is None:
            rospy.logerr('Simulation goals must be in odom, after odometry is ready')
            return
        start = odom[0].pose.pose.position
        finished[0] = False
        target = msg.pose.position
        path = RosPath()
        path.header.frame_id, path.header.stamp = 'odom', msg.header.stamp
        for x, y, yaw in reference_points((start.x, start.y), (target.x, target.y), scenario):
            point = PoseStamped()
            point.header = copy.deepcopy(path.header)
            point.pose.position.x, point.pose.position.y = x, y
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            point.pose.orientation.x, point.pose.orientation.y, point.pose.orientation.z, point.pose.orientation.w = q
            path.poses.append(point)
        path.poses[-1].pose.orientation = copy.deepcopy(msg.pose.orientation)
        path_pub.publish(path)
        episode_start[0] = rospy.get_time()
        actors[0] = CrossingActors()

    rospy.Subscriber('/planner_sim/raw_cmd', Twist, command, queue_size=1)
    rospy.Subscriber('/planner_sim/scan_raw', LaserScan, lambda m: pending_scan.__setitem__(0, m), queue_size=1)
    rospy.Subscriber('/odom', Odometry, lambda m: odom.__setitem__(0, m), queue_size=1)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, on_goal, queue_size=1)
    rospy.Subscriber('/planner_sim/episode_start', Float64, lambda m: episode_start.__setitem__(0, m.data), queue_size=1)
    rospy.Subscriber('/planner_sim/episode_finished', Bool, lambda m: finished.__setitem__(0, m.data), queue_size=1)
    rospy.Subscriber('/planner_sim/scan_enabled', Bool, lambda m: scan_enabled.__setitem__(0, m.data), queue_size=1)
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    for name, x, y, radius in specs:
        # Gazebo publishes static-model visual poses much less reliably when
        # moved continuously. Moving actors are gravity-free kinematic links.
        actor = ('<static>false</static><link name="body"><gravity>false</gravity><kinematic>true</kinematic>'
                 '<inertial><mass>1000</mass><inertia><ixx>100</ixx><iyy>100</iyy><izz>100</izz></inertia></inertial>'
                 if moving_actor(scenario, name)
                 else '<static>true</static><link name="body">')
        sdf = ('<sdf version="1.6"><model name="'+name+'">'+actor+
               '<collision name="collision"><pose>0 0 .8 0 0 0</pose><geometry><cylinder><radius>'+str(radius)+
               '</radius><length>1.6</length></cylinder></geometry></collision>'
               '<visual name="visual"><pose>0 0 .8 0 0 0</pose><geometry><cylinder><radius>'+str(radius)+
               '</radius><length>1.6</length></cylinder></geometry><material>'
               '<ambient>0.9 0.3 0.1 1</ambient><diffuse>0.9 0.3 0.1 1</diffuse>'
               '</material></visual></link></model></sdf>')
        pose = Pose(); pose.position.x, pose.position.y, pose.orientation.w = x, y, 1.
        response = spawn(name, sdf, '', pose, 'world')
        if not response.success:
            raise RuntimeError(response.status_message)
    ready_pub.publish(Bool(data=True))
    rate = rospy.Rate(50)
    next_actor_update = 0.
    while not rospy.is_shutdown():
        now = rospy.get_time()
        message = pending_scan[0]
        if not scan_enabled[0]:
            pending_scan[0] = None
            message = None
        if message is not None:
            try:
                trans, rotation = listener.lookupTransform('base_link', message.header.frame_id, message.header.stamp)
                yaw = tf.transformations.euler_from_quaternion(rotation)[2]
                ranges = np.asarray(message.ranges)
                angles = message.angle_min+np.arange(len(ranges))*message.angle_increment+yaw
                xs, ys = ranges*np.cos(angles)+trans[0], ranges*np.sin(angles)+trans[1]
                inside = np.isfinite(ranges) & (np.abs(xs) <= LENGTH/2+.03) & (np.abs(ys) <= WIDTH/2+.03)
                filtered = copy.deepcopy(message)
                filtered.ranges = np.where(inside, np.inf, ranges).tolist()
                scan_pub.publish(filtered)
                scan_stamp[0] = message.header.stamp.to_sec()
                if pending_scan[0] is message:
                    pending_scan[0] = None
            except tf.Exception:
                if now-message.header.stamp.to_sec() > .5:
                    pending_scan[0] = None
        scan_ok = scan_stamp[0] is not None and 0 <= now-scan_stamp[0] < .5
        command_ok = command_stamp[0] is not None and 0 <= now-command_stamp[0] < .5
        cmd = Twist()
        if scan_ok and command_ok and not finished[0]:
            source = raw_command[0]
            if math.isfinite(source.linear.x) and math.isfinite(source.angular.z):
                cmd.linear.x = max(-.5, min(.5, source.linear.x))
                cmd.angular.z = max(-.35, min(.35, source.angular.z))
        drive_pub.publish(cmd)
        health_pub.publish(String(data=json.dumps({'scan_ok': scan_ok, 'command_ok': command_ok})))
        if (scenario in DYNAMIC_SCENARIOS + ('crossing', 'crossing_stress') and episode_start[0] is not None
                and not finished[0] and now >= next_actor_update):
            elapsed = max(0., now-episode_start[0])
            if scenario == 'crossing':
                if odom[0] is None:
                    continue
                pose = odom[0].pose.pose
                q = pose.orientation
                yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                ego = (pose.position.x, pose.position.y, yaw, odom[0].twist.twist.linear.x,
                       odom[0].twist.twist.angular.z)
                moving = dict(zip((s[0] for s in actors[0].specs), actors[0].update(elapsed, ego)))
                positions = [moving.get(s[0], obstacle_position(s, scenario, elapsed)) for s in specs]
                actor_pub.publish(String(data=json.dumps(actors[0].states)))
            else:
                positions = [obstacle_position(spec, scenario, elapsed) for spec in specs]
            for spec, (x, y, _) in zip(specs, positions):
                state = ModelState(model_name=spec[0], reference_frame='world')
                state.pose.position.x, state.pose.position.y, state.pose.orientation.w = x, y, 1.
                state_pub.publish(state)
            next_actor_update = now+.05
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


def render_frame(record, planner, scenario, result=None):
    import cv2
    import numpy as np
    image = np.full((720, 960, 3), (35, 30, 28), dtype=np.uint8)
    def pixel(x, y):
        return int(130+70*x), int(365-52*y)
    for x in range(-1, 12):
        cv2.line(image, pixel(x, -5), pixel(x, 5), (60, 55, 50), 1)
    for y in range(-5, 6):
        cv2.line(image, pixel(-1, y), pixel(11, y), (60, 55, 50), 1)
    reference = record.get('reference') or [(0.,0.), GOAL[:2]]
    cv2.polylines(image, [np.array([pixel(x,y) for x,y in reference], dtype=np.int32)],
                  False, (80,200,60), 2)
    cv2.circle(image, pixel(*GOAL[:2]), 13, (220, 210, 20), 3)
    for ox, oy, radius in record['obstacles']:
        cv2.ellipse(image, pixel(ox, oy), (int(radius*70), int(radius*52)), 0, 0, 360, (50, 110, 245), -1)
    trace = record.get('trace', [])
    if len(trace) > 1:
        cv2.polylines(image, [np.array([pixel(x, y) for x, y in trace], dtype=np.int32)], False, (0, 220, 255), 2)
    plan = record.get('plan', [])
    if len(plan) > 1:
        cv2.polylines(image, [np.array([pixel(x, y) for x, y in plan], dtype=np.int32)], False, (200, 100, 230), 1)
    x, y, yaw = record['pose']
    c, s = math.cos(yaw), math.sin(yaw)
    vertices = [pixel(x+c*dx-s*dy, y+s*dx+c*dy) for dx, dy in
                [(-.14, -.375), (.81, -.375), (.81, .375), (-.14, .375)]]
    cv2.fillPoly(image, [np.array(vertices, dtype=np.int32)], (190, 190, 190))
    cv2.arrowedLine(image, pixel(x, y), pixel(x+.9*c, y+.9*s), (255, 255, 255), 2)
    lines = [planner.upper()+' | '+scenario+' | t={:.1f}s'.format(record['t']),
             'Gazebo telemetry replay (not GUI capture)',
             'Goal (8, 0) | Rear axle | 0.95 x 0.75 m | Limit 0.5 m/s',
             'Clearance {:.3f} m | Speed {:.2f} m/s'.format(record['clearance'], record['speed'])]
    if result:
        lines.append('Result: '+result['reason'])
    for i, line in enumerate(lines):
        cv2.putText(image, line, (20, 30+i*27), cv2.FONT_HERSHEY_SIMPLEX, .58, (240, 240, 240), 1, cv2.LINE_AA)
    cv2.putText(image, 'Green: common reference | Yellow: travelled | Pink: local plan', (20, 695),
                cv2.FONT_HERSHEY_SIMPLEX, .58, (240, 240, 240), 1, cv2.LINE_AA)
    return image


def encode_video(records, folder, planner, scenario, result):
    if not records:
        return
    import cv2
    writer = cv2.VideoWriter(str(folder/'overview.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 10, (960, 720))
    index = 0
    for t in (i/10 for i in range(int(records[-1]['t']*10)+1)):
        while index+1 < len(records) and records[index+1]['t'] <= t:
            index += 1
        writer.write(render_frame(records[index], planner, scenario, result))
    writer.release()
    proc = subprocess.run(['ffmpeg', '-v', 'error', '-y', '-i', str(folder/'overview.avi'),
        '-vf', 'setsar=1', '-c:v', 'libx264', '-preset', 'fast', '-crf', '23', '-pix_fmt', 'yuv420p',
        '-movflags', '+faststart', str(folder/'overview.mp4')])
    if proc.returncode == 0:
        (folder/'overview.avi').unlink()


def episode():
    import rospy
    import tf
    import numpy as np
    from gazebo_msgs.msg import ModelStates
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Float64, String
    rospy.init_node('comparison_episode')
    planner, scenario = rospy.get_param('~planner'), rospy.get_param('~scenario')
    timeout = float(rospy.get_param('~timeout', 75))
    folder = Path(rospy.get_param('~output_dir')); folder.mkdir(parents=True, exist_ok=True)
    values = {'ready': False, 'controller': False, 'scan': None, 'odom': None, 'models': None,
              'arrived': False, 'plan': [], 'health': {}, 'reference': None,
              'actor_states': {},
              'drive_command': Twist(),
              'runtime': {},
              'gate_health': {},
              'recording': not rospy.get_param('~recording_required', False)}
    solver, records, trace = [], [], []
    def runtime(msg):
        data = json.loads(msg.data)
        values['runtime'] = dict(data,received_sim_s=rospy.get_time(),
                                 valid=data.get('valid',not data.get('planner_stop',False)),
                                 attempted=data.get('attempted',True))
        # Include attempted optimisations (also failed ones), not idle ticks.
        if data.get('solver_ms', 0) > 0 and data.get('attempted', data.get('valid', True)):
            solver.append(float(data['solver_ms']))
        if planner == 'neupan': values['controller'] = True
    def health(msg):
        values['health'] = json.loads(msg.data)
        if planner == 'neupan' and values['health'].get('healthy'):
            values['controller'] = True
    def path(msg):
        values['plan'] = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
    rospy.Subscriber('/planner_sim/scene_ready', Bool, lambda m: values.__setitem__('ready', m.data), queue_size=1)
    rospy.Subscriber('/planner_sim/recording_ready', Bool, lambda m: values.__setitem__('recording', m.data), queue_size=1)
    rospy.Subscriber('/planner_sim/controller_ready', Bool, lambda m: values.__setitem__('controller', m.data), queue_size=1)
    rospy.Subscriber('/planner_sim/actor_states', String,
                     lambda m: values.__setitem__('actor_states', json.loads(m.data)), queue_size=1)
    rospy.Subscriber('/planner_sim/cmd', Twist, lambda m: values.__setitem__('drive_command',m), queue_size=1)
    rospy.Subscriber('/planner_sim/gate_health', String,
                     lambda m: values.__setitem__('gate_health',json.loads(m.data)), queue_size=1)
    rospy.Subscriber('/scan', LaserScan, lambda m: values.__setitem__('scan', m), queue_size=1)
    rospy.Subscriber('/odom', Odometry, lambda m: values.__setitem__('odom', m), queue_size=1)
    rospy.Subscriber('/planner_sim/model_states', ModelStates, lambda m: values.__setitem__('models', m), queue_size=1)
    rospy.Subscriber('/goal_arrived', String, lambda m: values.__setitem__('arrived', m.data == '1'), queue_size=1)
    rospy.Subscriber('/neupan/scan_health', String, health, queue_size=1)
    rospy.Subscriber('/neupan/runtime' if planner == 'neupan' else '/planner_sim/runtime', String, runtime, queue_size=100)
    rospy.Subscriber('/planner_sim/local_plan', RosPath, path, queue_size=1)
    rospy.Subscriber('/initial_path', RosPath, lambda m: values.__setitem__('reference',
        [(round(p.pose.position.x, 6), round(p.pose.position.y, 6)) for p in m.poses]), queue_size=1)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
    start_pub = rospy.Publisher('/planner_sim/episode_start', Float64, queue_size=1, latch=True)
    finish_pub = rospy.Publisher('/planner_sim/episode_finished', Bool, queue_size=1, latch=True)
    deadline = time.monotonic()+50
    while time.monotonic() < deadline and not rospy.is_shutdown():
        if (all(values[k] is not None for k in ('scan', 'odom', 'models')) and values['ready']
                and values['controller'] and values['recording']):
            break
        time.sleep(.1)
    else:
        write_json(folder/'result.json', {'planner': planner, 'scenario': scenario, 'passed': False, 'reason': 'startup_not_ready'})
        return
    rospy.sleep(1.0)
    start = rospy.get_time()
    start_pub.publish(Float64(data=start))
    goal = PoseStamped(); goal.header.frame_id, goal.header.stamp = 'odom', rospy.Time.now()
    goal.pose.position.x, goal.pose.position.y, goal.pose.orientation.w = GOAL[0], GOAL[1], 1.
    goal_pub.publish(goal)
    stable_since = None
    reason = 'timeout'
    min_clearance, max_pose_error, last_t, path_length, stopped_time = math.inf, 0., 0., 0., 0.
    obstacle_travel, previous_obstacles = {}, {}
    log = (folder/'trajectory.jsonl').open('w')
    rate = rospy.Rate(20)
    wall_deadline = time.monotonic()+timeout*3+30
    while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
        t = rospy.get_time()-start
        telemetry_gap = max(0.,t-last_t)
        odom = values['odom']; models = values['models']
        p = odom.pose.pose
        yaw = tf.transformations.euler_from_quaternion([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])[2]
        pose = [p.position.x, p.position.y, yaw]
        actual_pose = models.pose[models.name.index('lxwcar')]
        actual = actual_pose.position
        max_pose_error = max(max_pose_error, math.hypot(actual.x-pose[0], actual.y-pose[1]))
        q = actual_pose.orientation
        yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        pose = [actual.x, actual.y, yaw]
        obstacles = []
        for name, _, _, radius in obstacle_specs(scenario):
            op = models.pose[models.name.index(name)].position
            obstacles.append((op.x, op.y, radius))
            if name in previous_obstacles:
                obstacle_travel[name] += math.hypot(op.x-previous_obstacles[name][0], op.y-previous_obstacles[name][1])
            else:
                obstacle_travel[name] = 0.
            previous_obstacles[name] = (op.x, op.y)
        gap = clearance(pose, obstacles)
        min_clearance = min(min_clearance, gap)
        speed = odom.twist.twist.linear.x
        if trace:
            path_length += math.hypot(pose[0]-trace[-1][0], pose[1]-trace[-1][1])
        if abs(speed) < .03: stopped_time += max(0., t-last_t)
        trace.append(pose[:2]); last_t = t
        distance = math.hypot(pose[0]-GOAL[0], pose[1]-GOAL[1])
        record = {'t': round(t, 4), 'wall_time_unix_s': time.time(),
                  'pose': pose, 'speed': speed, 'clearance': gap,
                  'steer_rad': values['drive_command'].angular.z,
                  'command_speed_mps': values['drive_command'].linear.x,
                  'controller_runtime': values['runtime'],
                  'controller_runtime_age_s': rospy.get_time()-values['runtime'].get('received_sim_s',-1e9),
                  'scan_age_s': rospy.get_time()-values['scan'].header.stamp.to_sec(),
                  'odom_age_s': rospy.get_time()-odom.header.stamp.to_sec(),
                  'telemetry_gap_s': telemetry_gap,
                  'gate_health': values['gate_health'],
                  'actor_states': values['actor_states'],
                  'obstacles': obstacles, 'arrived': values['arrived'], 'plan': values['plan']}
        log.write(json.dumps(record)+'\n')
        video_record = dict(record, trace=list(trace), reference=values['reference']); records.append(video_record)
        fresh_scan = 0 <= rospy.get_time()-values['scan'].header.stamp.to_sec() < .5
        if gap < -.02:
            reason = 'body_overlap'; break
        if max_pose_error > .25:
            reason = 'gazebo_odometry_mismatch'; break
        if not fresh_scan and t > 1.:
            reason = 'stale_scan'; break
        if values['arrived'] and distance <= GOAL_POSITION_TOLERANCE and abs(yaw) <= .35 and abs(speed) <= .1:
            if stable_since is None: stable_since = t
            if t-stable_since >= .5:
                reason = 'arrived'; break
        else:
            stable_since = None
        if t >= timeout: break
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    log.close()
    finish_pub.publish(Bool(data=True))
    reference = values['reference']
    write_json(folder/'reference.json', reference)
    tracking_rmse = None
    if reference and len(reference) > 1:
        ref = np.asarray(reference, dtype=float)
        segments = np.diff(ref, axis=0)
        delta = np.asarray(trace)[:,None,:]-ref[:-1][None,:,:]
        projection = np.clip(np.einsum('ijk,jk->ij',delta,segments)/
                             np.maximum((segments*segments).sum(1),1e-12),0,1)
        residual = delta-projection[:,:,None]*segments[None,:,:]
        tracking_rmse = round(float(np.sqrt((residual*residual).sum(2).min(1).mean())),3)
    overtake_time = None
    if scenario in DYNAMIC_SCENARIOS:
        for r in records:
            x,y,heading = r['pose']; c,s = math.cos(heading),math.sin(heading)
            rear_bound = min(x+c*dx-s*dy for dx in (-.14,.81) for dy in (-.375,.375))
            if rear_bound > r['obstacles'][0][0]+r['obstacles'][0][2]+.2:
                overtake_time = round(r['t'],3); break
    result = {'planner': planner, 'scenario': scenario, 'passed': reason == 'arrived', 'reason': reason,
        'elapsed_sim_s': round(last_t, 3), 'path_length_m': round(path_length, 3),
        'goal_error_m': round(distance, 3), 'yaw_error_rad': round(abs(yaw), 3),
        'minimum_body_clearance_m': round(min_clearance, 3), 'stopped_time_s': round(stopped_time, 3),
        'tracking_rmse_m': tracking_rmse,
        'tracking_metric_version': 2,
        'straight_axis_deviation_rmse_m': round(float(np.sqrt(np.mean([y*y+max(-x, x-GOAL[0], 0.)**2 for x, y in trace]))), 3),
        'solver_samples': len(solver), 'solver_median_ms': round(float(np.median(solver)), 3) if solver else None,
        'solver_p95_ms': round(float(np.percentile(solver, 95)), 3) if solver else None,
        'max_gazebo_odom_error_m': round(max_pose_error, 4), 'obstacle_path_lengths_m': obstacle_travel,
        'maximum_telemetry_gap_s': round(max(r['telemetry_gap_s'] for r in records),4),
        'maximum_scan_age_s': round(max(r['scan_age_s'] for r in records),4),
        'maximum_odometry_age_s': round(max(r['odom_age_s'] for r in records),4),
        'actor_yield_events': {n: s['yield_events'] for n, s in values['actor_states'].items()},
        'actor_yield_duration_s': {n: round(s['yield_duration_s'], 3) for n, s in values['actor_states'].items()},
        'actor_policy': 'one-way traversal with robot-aware yielding' if scenario == 'crossing' else 'prescribed trajectory',
        'overtake_time_s': overtake_time,
        'maximum_lateral_deviation_m': round(max(abs(p[1]) for p in trace),3),
        'steering_total_variation_rad': round(sum(abs(b['steer_rad']-a['steer_rad']) for a,b in zip(records,records[1:])),3),
        'acceptance_position_tolerance_m': GOAL_POSITION_TOLERANCE,
        'arrival_includes_actor_yield': any(s['yield_events'] for s in values['actor_states'].values()),
        'reference_sha256': hashlib.sha256(json.dumps(reference).encode()).hexdigest(),
        'goal': GOAL, 'footprint': {'length': LENGTH, 'width': WIDTH, 'centre_from_rear_axle': CENTRE},
        'dynamic_input': 'same scan sensor pipeline; no ground-truth obstacle velocity supplied',
        'simulation_model': 'kinematic pose driver; overlap evaluated geometrically, no tyre/contact dynamics'}
    result.update(hesitation_metrics(records))
    # Export before publishing result, so the orchestrator does not stop encoding.
    encode_video(records, folder, planner, scenario, result)
    write_json(folder/'result.json', result)
    print(json.dumps(result), flush=True)


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def cleanup(log_dir):
    """Only descendants whose ROS __log explicitly names this run directory."""
    marker = '__log:='+str(log_dir)+'/'
    for item in Path('/proc').iterdir():
        if not item.name.isdigit(): continue
        try:
            cmd = (item/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if marker in cmd:
                os.kill(int(item.name), signal.SIGTERM)
        except (OSError, ProcessLookupError):
            pass


def recorder_start(folder, env):
    import ctypes
    import re
    deadline = time.monotonic()+35
    match = None
    while time.monotonic() < deadline:
        output = subprocess.check_output(['xwininfo', '-root', '-tree'], env=env, text=True)
        match = re.search(r'(0x[0-9a-f]+) "Gazebo"', output)
        if match: break
        time.sleep(.2)
    if match is None: raise RuntimeError('Gazebo GUI window did not open')
    x = ctypes.CDLL('libX11.so.6')
    x.XOpenDisplay.argtypes = [ctypes.c_char_p]; x.XOpenDisplay.restype = ctypes.c_void_p
    x.XMoveResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
    x.XFlush.argtypes = [ctypes.c_void_p]; x.XCloseDisplay.argtypes = [ctypes.c_void_p]
    display = x.XOpenDisplay(env['DISPLAY'].encode())
    if not display: raise RuntimeError('Cannot open recording display')
    # Qt can restore its saved geometry after its first window appears. Keep
    # applying the recording geometry through that startup period.
    for _ in range(8):
        x.XMoveResizeWindow(display, int(match.group(1), 16), 0, 0, 1600, 900)
        x.XFlush(display)
        time.sleep(.4)
    x.XCloseDisplay(display)
    with (folder/'recorder.log').open('w') as log:
        # X11 input PTS are Unix microseconds. Retain the first timestamp in the
        # log so an exported GUI pair can follow the measured simulation clock.
        return subprocess.Popen(['ffmpeg', '-v', 'info', '-debug_ts', '-y', '-f', 'x11grab', '-framerate', '15',
            '-video_size', '1600x900', '-i', env['DISPLAY'], '-vf', 'scale=1280:720,setsar=1',
            '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '23', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(folder/'gazebo.mp4')], env=env,
            stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def compare_cases(args):
    if not (WORKSPACE/'devel/lib/liblxw_sim_pose.so').is_file():
        raise RuntimeError('Build the simulation pose plugin first: catkin_make --pkg neupan_ros')
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    root = Path(args.output_dir).resolve() if args.output_dir else WORKSPACE/'videos/planner_comparison'/stamp
    root.mkdir(parents=True, exist_ok=True)
    planners = ('neupan', 'teb') if args.planner == 'both' else (args.planner,)
    scenarios = SCENARIOS if args.scenario == 'all' else DYNAMIC_SCENARIOS if args.scenario == 'dynamics' else (args.scenario,)
    feasibility = [dynamic_feasibility(s) for s in scenarios if s in DYNAMIC_SCENARIOS]
    if feasibility:
        write_json(root/'feasibility.json',feasibility)
        if not all(f['passed'] for f in feasibility):
            raise ValueError('Dynamic scenario lacks the required safe kinematic witness')
    results = []
    print('Comparison results: '+str(root), flush=True)
    for repeat in range(1, args.repeats+1):
        for scenario in scenarios:
            for planner in planners:
                folder = root/(scenario+'_'+planner+'_'+str(repeat))
                if folder.exists() and any(folder.iterdir()):
                    raise FileExistsError('Trial already contains data; use a new --output-dir: '+str(folder))
                folder.mkdir(parents=True, exist_ok=True)
                with socket.socket() as a, socket.socket() as b:
                    a.bind(('127.0.0.1', 0)); b.bind(('127.0.0.1', 0))
                    port, gzport = a.getsockname()[1], b.getsockname()[1]
                env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:'+str(port), ROS_HOSTNAME='127.0.0.1',
                    GAZEBO_MASTER_URI='http://127.0.0.1:'+str(gzport), ROS_LOG_DIR=str(folder/'ros_logs'),
                    ROS_NAMESPACE='', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
                env.pop('ROS_IP', None)
                if not args.gui:
                    env.pop('DISPLAY', None)
                if args.record_gazebo:
                    env.update(DISPLAY=args.display, LIBGL_ALWAYS_SOFTWARE='1', QT_QPA_PLATFORM='xcb')
                launch = ['roslaunch', '-p', str(port), 'neupan_ros', 'planner_comparison.launch',
                    'planner:='+planner, 'scenario:='+scenario, 'gui:='+str(args.gui).lower(),
                    'automated:=true', 'output_dir:='+str(folder), 'timeout:='+str(args.timeout),
                    'recording_required:='+str(args.record_gazebo).lower()]
                teb_name = 'comparison_teb_field.yaml' if args.teb_profile == 'field' else 'comparison_teb.yaml'
                launch.append('teb_config_file:='+str(ROOT/'config'/teb_name))
                launch.append('neupan_profile:='+args.neupan_profile)
                write_json(folder/'manifest.json', {'command': launch, 'environment': {k: env[k] for k in
                    ('ROS_MASTER_URI', 'GAZEBO_MASTER_URI', 'ROS_LOG_DIR')}, 'hardware_started': False,
                    'repeat': repeat, 'planner_input': 'same reference and scan pipeline; observations depend on robot motion', 'controller_hz': 10,
                    'scenario_version': 5,
                    'teb_profile': args.teb_profile,
                    'neupan_profile': args.neupan_profile,
                    'controller_settings': {'neupan_profile': args.neupan_profile,
                        'physics_clock': 'shared half wall-time rate; all planner and actor periods use simulation time'},
                    'neupan_adapter_sha256': hashlib.sha256((ROOT/'scripts/neupan_sim_controller.py').read_bytes()).hexdigest(),
                    'teb_parameter_source': 'src/teb_test/config/teb_local_planner.yaml' if args.teb_profile == 'field' else 'simulation baseline',
                    'teb_field_adaptations': ['odom/map frames','actual simulated rear-axle footprint',
                        'same +/-0.5 m/s speed bound','0.15 m native goal tolerance'] if args.teb_profile == 'field' else [],
                    'acceptance_position_tolerance_m': GOAL_POSITION_TOLERANCE,
                    'reference_policy': 'common smooth fixed-island bypass' if scenario == 'crossing' else 'common straight reference',
                    'actor_specs': obstacle_specs(scenario),
                    'dynamic_trajectory': ('one-way, max 0.7 m/s, delays 4/6 s; robot-aware actor yielding; static traffic island forces a detour'
                                           if scenario == 'crossing' else 'cosine pressure test' if scenario == 'crossing_stress' else
                                           'x=min(10,2.4+0.18*t), y=0; no robot reaction' if scenario == 'lead_slow' else
                                           'x=min(10,2.8+0.22*(min(t,6)+max(t-10,0))), y=0; no robot reaction' if scenario == 'lead_stop' else
                                           'x=4.2, y=max(-3,3-0.6*t); one-way crossing, no robot reaction' if scenario == 'crossing_once' else 'static'),
                    'actor_yield_gap_m': ACTOR_YIELD_GAP if scenario == 'crossing' else None,
                    'actor_model': 'gravity-free kinematic link' if any(moving_actor(scenario,s[0]) for s in obstacle_specs(scenario)) else 'static',
                    'actor_types': {s[0]: ('kinematic' if moving_actor(scenario,s[0]) else 'static') for s in obstacle_specs(scenario)},
                    'physics_sha256': hashlib.sha256((ROOT/'worlds/planner_comparison.world').read_bytes()).hexdigest(),
                    'requested_real_time_factor': .5,
                    'ros_clock_publish_hz': 100,
                    'pose_transport': 'Gazebo world-update plugin; no per-command pause/unpause',
                    'pose_plugin_source_sha256': hashlib.sha256((ROOT/'src/sim_pose_world_plugin.cpp').read_bytes()).hexdigest(),
                    'driver_source_sha256': hashlib.sha256((ROOT/'scripts/ackermann_sim_driver.py').read_bytes()).hexdigest(),
                    'measurement_transport': 'Gazebo model states relayed at 30 Hz; scans and control rates unchanged',
                    'comparison_launch_sha256': hashlib.sha256((ROOT/'launch/planner_comparison.launch').read_bytes()).hexdigest(),
                    'config_sha256': {
                        name: hashlib.sha256((ROOT/'config'/name).read_bytes()).hexdigest()
                        for name in ('comparison_neupan.yaml', 'comparison_neupan_adapter.yaml', teb_name)},
                    'neupan_checkpoint': 'acker_robot_default_2/model_5000.pth',
                    'teb_plugin': 'teb_local_planner/TebLocalPlannerROS'})
                process = recorder = None
                print('Running {} / {} / repeat {}'.format(scenario, planner, repeat), flush=True)
                try:
                    with (folder/'console.log').open('w') as console:
                        process = subprocess.Popen(launch, env=env, stdout=console, stderr=subprocess.STDOUT, start_new_session=True)
                        if args.record_gazebo:
                            recorder = recorder_start(folder, env)
                            subprocess.run(['rostopic', 'pub', '-1', '/planner_sim/recording_ready', 'std_msgs/Bool', 'data: true'],
                                env=env, stdout=console, stderr=subprocess.STDOUT, timeout=15, check=True)
                        deadline, update = time.monotonic()+args.timeout*3+100, time.monotonic()
                        while process.poll() is None and not (folder/'result.json').exists() and time.monotonic() < deadline:
                            if time.monotonic()-update > 25:
                                print('  running '+str(folder), flush=True); update = time.monotonic()
                            time.sleep(.25)
                finally:
                    stop(recorder)
                    if recorder is not None and (folder/'recorder.log').exists():
                        import re
                        match = re.search(r'demuxer -> .*?pkt_pts:(\d+) ', (folder/'recorder.log').read_text())
                        if match:
                            write_json(folder/'recording_timing.json', {
                                'first_frame_wall_unix_s': int(match.group(1))/1_000_000.,
                                'input_time_base': '1/1000000', 'capture_fps': 15,
                                'alignment': 'X11 input PTS and telemetry wall clock; GUI render latency remains'})
                    stop(process); cleanup(folder/'ros_logs')
                if (folder/'result.json').exists():
                    result = json.loads((folder/'result.json').read_text())
                else:
                    result = {'planner': planner, 'scenario': scenario, 'passed': False, 'reason': 'launch_failure_or_wall_timeout'}
                    write_json(folder/'result.json', result)
                result.update(repeat=repeat, folder=folder.name)
                results.append(result)
                print('  '+result['reason'], flush=True)
    write_json(root/'comparison.json', {'results': results, 'scope': 'simulation only'})
    lines = ['# NeuPAN 与 TEB Gazebo 局部规划对比', '',
        '同一后轴参考、0.95×0.75 m 外形、0.67 m 轴距、0.5 m/s 限速、扫描和参考线；每轮独立重启 Gazebo。',
        '交互横穿采用相同路径、速度和主动让行规则，实际轨迹会随车辆行为变化；主动让行次数单独记录。强制横穿压测不让行。',
        '不向任何规划器提供障碍真值速度。TEB 使用扫描代价地图，不启用额外动态目标跟踪。', '',
        '| 场景 | 规划器 | 次数 | 结果 | 仿真时间 s | 路长 m | 最小车体间距 m | 求解中位数 ms |',
        '|---|---|---|---|---|---|---|---|']
    for r in results:
        lines.append('| {} | {} | {} | {} | {} | {} | {} | {} |'.format(r['scenario'], r['planner'], r['repeat'],
            r['reason'], r.get('elapsed_sim_s', '-'), r.get('path_length_m', '-'),
            r.get('minimum_body_clearance_m', '-'), r.get('solver_median_ms', '-')))
    lines += ['', '到达需规划器报告完成、位置误差≤0.5 m、航向误差≤0.35 rad、速度≤0.1 m/s 并稳定0.5 s。',
        '车体侵入障碍、越界、扫描超时或里程计与 Gazebo 误差>0.25 m 均失败。超时不等于安全通过。',
        'overview.mp4 是实际 Gazebo 位姿/扫描实验的俯视轨迹回放，不是 Gazebo GUI 录屏。',
        '使用 --record-gazebo 时，gazebo.mp4 为同轮真实 Gazebo 窗口录屏，独立 Xvfb 显示并保存正确像素比例。',
        '运动学虚拟底盘不模拟轮胎、打滑或接触动力学；碰撞以真实车体包络与障碍几何重叠判定。',
        '有限场景和次数不能推断算法在实车或所有场景上的优劣。']
    (root/'comparison.md').write_text('\n'.join(lines)+'\n')
    subprocess.run([sys.executable, str(Path(__file__).with_name('report_comparison.py')), str(root)], check=True)
    print('Report: '+str(root/'comparison.md'), flush=True)
    return 0


def compare(args):
    virtual_display = None
    try:
        if args.record_gazebo:
            args.gui = True
            for number in range(90, 110):
                if not Path('/tmp/.X11-unix/X'+str(number)).exists() and not Path('/tmp/.X'+str(number)+'-lock').exists():
                    args.display = ':'+str(number)
                    break
            else: raise RuntimeError('No free virtual display')
            virtual_display = subprocess.Popen(['Xvfb', args.display, '-screen', '0', '1600x900x24', '-ac', '-nolisten', 'tcp'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
            deadline = time.monotonic()+5
            while not Path('/tmp/.X11-unix/X'+args.display[1:]).exists() and time.monotonic() < deadline:
                if virtual_display.poll() is not None: raise RuntimeError('Xvfb failed')
                time.sleep(.1)
        return compare_cases(args)
    finally:
        stop(virtual_display)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    run = sub.add_parser('run')
    run.add_argument('--planner', choices=('both', 'neupan', 'teb'), default='both')
    run.add_argument('--scenario', choices=('all','dynamics')+AVAILABLE_SCENARIOS, default='all')
    run.add_argument('--repeats', type=int, default=2)
    run.add_argument('--timeout', type=float, default=75)
    run.add_argument('--gui', action='store_true')
    run.add_argument('--record-gazebo', action='store_true', help='record real GUI in a private Xvfb display')
    run.add_argument('--teb-profile', choices=('field','baseline'), default='field')
    run.add_argument('--neupan-profile', choices=('fixed','adaptive'), default='adaptive')
    run.add_argument('--output-dir')
    sub.add_parser('scene'); sub.add_parser('episode')
    args = parser.parse_args(rospy_args())
    if args.command == 'scene': return scene()
    if args.command == 'episode': return episode()
    if args.repeats < 1 or args.timeout <= 0: parser.error('positive repeats and timeout required')
    return compare(args)


def rospy_args():
    # roslaunch appends __name/__log; standalone orchestration does not init ROS.
    return [arg for arg in sys.argv[1:] if ':=' not in arg]


if __name__ == '__main__':
    sys.exit(main() or 0)
