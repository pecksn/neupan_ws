#!/usr/bin/env python3
"""Private-master Gazebo outdoor regression, sensor bridge and result recorder.

No serial ports, real base driver, real GNSS, Livox or GPU receiver are started.
The real OSM executable, outdoor connection helper, model and solver are used.
"""
import argparse
import copy
from datetime import datetime, timedelta, timezone
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
import xmlrpc.client


SIMULATION = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS = SIMULATION.parents[3] / 'videos' / 'outdoor_simulation'
PREFIX = '/outdoor_sim/'


def save_json(path, data):
    temporary = path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2))
    os.replace(str(temporary), str(path))


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def run_cases(args):
    scenarios = ('wide', 'narrow', 'unknown') if args.scenario == 'all' else (args.scenario,)
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    root = Path(args.output_dir).resolve() if args.output_dir else DEFAULT_RESULTS/stamp
    root.mkdir(parents=True, exist_ok=True)
    results = []
    print('Results: '+str(root), flush=True)
    for scenario in scenarios:
        folder = root/scenario
        folder.mkdir(parents=True, exist_ok=True)
        with socket.socket() as ros_port, socket.socket() as gazebo_port:
            ros_port.bind(('127.0.0.1', 0))
            gazebo_port.bind(('127.0.0.1', 0))
            port, gzport = ros_port.getsockname()[1], gazebo_port.getsockname()[1]
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:'+str(port),
                   ROS_HOSTNAME='127.0.0.1', GAZEBO_MASTER_URI='http://127.0.0.1:'+str(gzport),
                   ROS_LOG_DIR=str(folder/'ros_logs'), ROS_NAMESPACE='',
                   OMP_NUM_THREADS='1', MKL_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
        env.pop('ROS_IP', None)
        save_json(folder/'run_manifest.json', {'scenario': scenario,
            'ros_master_uri': env['ROS_MASTER_URI'], 'gazebo_master_uri': env['GAZEBO_MASTER_URI'],
            'world': str(SIMULATION/'worlds/outdoor_roads.world'),
            'model': 'acker_robot_default_6/model_5000.pth', 'production_config': 'neupan_config_outdoor.yaml',
            'ground_truth_localization': True, 'record_bag': args.record_bag,
            'real_hardware_started': False})
        launch = ['roslaunch', '-p', str(port), 'neupan_ros', 'outdoor_simulation.launch',
                  'scenario:='+scenario, 'output_dir:='+str(folder),
                  'gui:='+str(args.gui).lower(), 'rviz:='+str(args.rviz).lower()]
        process = bag = None
        print('Starting '+scenario+' on private ROS master '+env['ROS_MASTER_URI'], flush=True)
        try:
            with (folder/'console.log').open('w') as console:
                process = subprocess.Popen(launch, env=env, stdout=console, stderr=subprocess.STDOUT,
                                           start_new_session=True)
                if args.record_bag:
                    ready_deadline = time.monotonic()+20.
                    while process.poll() is None and time.monotonic() < ready_deadline:
                        try:
                            if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/outdoor_sim_recorder')[0] == 1:
                                break
                        except (OSError, xmlrpc.client.Error):
                            pass
                        time.sleep(.1)
                    topics = [PREFIX+t for t in ('odom', 'odometry/global', 'gps/fix', 'scan_raw', 'scan',
                        'goal', 'cmd_vel', 'goal_arrived', 'osm_global_path', 'neupan_initial_path',
                        'neupan_ref_state', 'neupan_plan', 'neupan/runtime', 'neupan/route_connection',
                        'neupan/scan_health', 'neupan/failure', 'tf', 'tf_static', 'scan_mode')]+['/clock']
                    bag = subprocess.Popen(['rosbag', 'record', '--lz4', '-O', str(folder/'simulation.bag')]+topics,
                        env=env, stdout=console, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic()+args.timeout
                last_report = 0.
                while process.poll() is None and not (folder/'result.json').exists() and time.monotonic() < deadline:
                    if time.monotonic()-last_report >= 20:
                        print('  '+scenario+': running; '+str(folder/'console.log'), flush=True)
                        last_report = time.monotonic()
                    time.sleep(.25)
        finally:
            # Finalise the bag while the private master is still alive.
            stop_process(bag)
            stop_process(process)
        if (folder/'result.json').exists():
            result = json.loads((folder/'result.json').read_text())
        else:
            result = {'scenario': scenario, 'passed': False, 'reason': 'launch_exited_or_wall_timeout',
                      'console': str(folder/'console.log')}
            save_json(folder/'result.json', result)
        results.append(result)
        print(scenario+': '+('PASS' if result['passed'] else 'FAIL')+' / '+result['reason'], flush=True)
    save_json(root/'summary.json', {'results': results, 'all_passed': all(r['passed'] for r in results)})
    lines = ['# 室外仿真结果', '', '| 场景 | 结果 | 说明 |', '|---|---|---|']
    for result in results:
        lines.append('| {} | {} | {} |'.format(result['scenario'], 'PASS' if result['passed'] else 'FAIL', result['reason']))
    lines += ['', '使用生产 OSM A*、室外配置、模型 6 与 NeuPAN 求解器；定位为仿真真值，不代表实车 RTK/FAST-LIO 验收。',
              '每个场景目录保存 overview.mp4（俯视轨迹，不是相机视频）、navigation.jsonl、console.log 和 result.json；可选 simulation.bag。']
    (root/'summary.md').write_text('\n'.join(lines)+'\n')
    print('Summary: '+str(root/'summary.md'), flush=True)
    return 0 if all(r['passed'] for r in results) else 1


def sensor_bridge():
    import rospy
    import tf
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan, NavSatFix, NavSatStatus
    from std_msgs.msg import String
    rospy.init_node('outdoor_sim_sensors')
    mode = ['normal']
    odom_pub = rospy.Publisher(PREFIX+'odometry/global', Odometry, queue_size=5)
    gps_pub = rospy.Publisher(PREFIX+'gps/fix', NavSatFix, queue_size=5)
    scan_pub = rospy.Publisher(PREFIX+'scan', LaserScan, queue_size=1)
    listener = tf.TransformListener()
    previous_scan = [None]

    def odometry(message):
        global_odom = copy.deepcopy(message)
        global_odom.header.frame_id = 'map'  # map->odom is identity in this test.
        global_odom.child_frame_id = 'base_link'
        odom_pub.publish(global_odom)
        fix = NavSatFix()
        fix.header = global_odom.header
        fix.header.frame_id = 'gps_link'
        fix.status.status, fix.status.service = NavSatStatus.STATUS_GBAS_FIX, NavSatStatus.SERVICE_GPS
        scale = 6378137.*math.pi/180.
        fix.latitude = 23.0398+message.pose.pose.position.y/scale
        fix.longitude = 113.39002+message.pose.pose.position.x/(scale*math.cos(math.radians(23.0398)))
        fix.position_covariance = [.01, 0, 0, 0, .01, 0, 0, 0, .02]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps_pub.publish(fix)

    def laser(message):
        # Gazebo can emit the current scan before the virtual-base TF callback
        # reaches subscribers. Release the previous (100 ms old) scan only once
        # its exact capture-time transform is available; never restamp it.
        old = previous_scan[0]
        previous_scan[0] = message
        if old is None:
            return
        message = old
        try:
            listener.lookupTransform('map', message.header.frame_id, message.header.stamp)
        except tf.Exception:
            return
        if mode[0] == 'dropout':
            return
        if mode[0] == 'unknown':
            message = copy.deepcopy(message)
            message.ranges = [float('inf')]*len(message.ranges)
        scan_pub.publish(message)

    rospy.Subscriber(PREFIX+'odom', Odometry, odometry, queue_size=5)
    rospy.Subscriber(PREFIX+'scan_raw', LaserScan, laser, queue_size=1)
    rospy.Subscriber(PREFIX+'scan_mode', String, lambda msg: mode.__setitem__(0, msg.data), queue_size=1)
    rospy.spin()


class ScenarioTest:
    def __init__(self):
        import cv2
        import numpy as np
        import rospy
        from geometry_msgs.msg import PoseStamped, Twist
        from gazebo_msgs.msg import ModelStates
        from nav_msgs.msg import Odometry, Path as RosPath
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String
        self.cv2, self.np, self.rospy = cv2, np, rospy
        self.PoseStamped, self.Twist, self.String = PoseStamped, Twist, String
        rospy.init_node('outdoor_sim_test')
        self.scenario = rospy.get_param('~scenario', 'wide')
        self.center = float(rospy.get_param('~start_y', 0.))
        self.width = 3. if self.scenario == 'narrow' else 12.
        self.folder = Path(rospy.get_param('~output_dir'))
        self.folder.mkdir(parents=True, exist_ok=True)
        self.log = (self.folder/'navigation.jsonl').open('w', buffering=1)
        self.lock = threading.RLock()
        self.pose = self.command = self.connection = self.runtime = None
        self.gazebo_pose = None
        self.max_gazebo_error = 0.
        self.arrived = False
        self.scan_received = None
        self.stage, self.stage_started = 'waiting', None
        self.started_wall = time.monotonic()
        self.outbound_arrived = False
        self.seen_connecting = self.seen_completed = False
        self.blocked_since = self.blocked_pose = None
        self.zero_samples = 0
        self.min_clearance = float('inf')
        self.trace, self.paths = [], {}
        self.last_path_signatures = {}
        self.frames, self.done = 0, False
        self.writer = cv2.VideoWriter(str(self.folder/'overview_raw.mp4'), cv2.VideoWriter_fourcc(*'mp4v'),
                                      10., (1100, 650))
        if not self.writer.isOpened():
            raise RuntimeError('cannot open simulation overview video')
        self.goal_pub = rospy.Publisher(PREFIX+'goal', PoseStamped, queue_size=1, latch=True)
        self.mode_pub = rospy.Publisher(PREFIX+'scan_mode', String, queue_size=1, latch=True)
        self.mode_pub.publish(String(data='normal'))
        rospy.Subscriber(PREFIX+'odometry/global', Odometry, self.odometry, queue_size=5)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_state, queue_size=1)
        rospy.Subscriber(PREFIX+'cmd_vel', Twist, self.commands, queue_size=10)
        rospy.Subscriber(PREFIX+'goal_arrived', String, self.arrival, queue_size=1)
        rospy.Subscriber(PREFIX+'scan', LaserScan, self.scan, queue_size=1)
        rospy.Subscriber(PREFIX+'neupan/route_connection', String, self.connection_status, queue_size=10)
        rospy.Subscriber(PREFIX+'neupan/runtime', String, self.runtime_status, queue_size=10)
        rospy.Subscriber(PREFIX+'neupan/failure', String, self.failure, queue_size=1)
        for name in ('osm_global_path', 'neupan_initial_path', 'neupan_plan'):
            rospy.Subscriber(PREFIX+name, RosPath, lambda msg, name=name: self.path(name, msg), queue_size=1)
        rospy.Timer(rospy.Duration(.1), self.tick)
        rospy.on_shutdown(self.close)

    def event(self, kind, data):
        self.log.write(json.dumps({'record_type': kind, 'sim_time': self.rospy.get_time(),
                                   'wall_time': time.time(), 'data': data})+'\n')

    def odometry(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            self.pose = (p.x, p.y, yaw, message.twist.twist.linear.x)

    def commands(self, message):
        with self.lock:
            self.command = (message.linear.x, message.angular.z)
            if self.blocked_since is not None and self.rospy.get_time()-self.blocked_since > .2:
                if abs(message.linear.x) < 1.e-8:
                    self.zero_samples += 1
                else:
                    self.finish(False, 'nonzero_command_after_blocked_route')

    def gazebo_state(self, message):
        if 'outdoor_lxwcar' in message.name:
            p = message.pose[message.name.index('outdoor_lxwcar')].position
            with self.lock:
                self.gazebo_pose = (p.x, p.y)

    def arrival(self, message):
        with self.lock:
            self.arrived = message.data == '1'

    def scan(self, message):
        with self.lock:
            self.scan_received = self.rospy.get_time()

    def connection_status(self, message):
        status = json.loads(message.data)
        with self.lock:
            if status != self.connection:
                self.event('route_connection', status)
            self.connection = status
            self.seen_connecting |= status.get('phase') == 'connecting'
            self.seen_completed |= status.get('reason') == 'connection_completed'

    def runtime_status(self, message):
        with self.lock:
            self.runtime = json.loads(message.data)

    def failure(self, message):
        with self.lock:
            self.event('neupan_failure', json.loads(message.data))
            self.finish(False, 'neupan_exception')

    def path(self, name, message):
        points = [(p.pose.position.x, p.pose.position.y) for p in message.poses]
        with self.lock:
            self.paths[name] = points
            signature = tuple((round(x, 3), round(y, 3)) for x, y in points)
            if name != 'neupan_plan' and signature != self.last_path_signatures.get(name):
                self.last_path_signatures[name] = signature
                self.event(name, {'points': points, 'stamp_ns': message.header.stamp.to_nsec()})

    def send_goal(self, x, yaw):
        goal = self.PoseStamped()
        goal.header.frame_id, goal.header.stamp = 'map', self.rospy.Time.now()
        goal.pose.position.x, goal.pose.position.y = x, self.center
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.arrived = False
        self.goal_pub.publish(goal)
        self.event('goal', {'x': x, 'y': self.center, 'yaw': yaw})

    def clearance(self):
        x, y, yaw, _ = self.pose
        corners = self.np.array([[-.14, -.425], [.81, -.425], [.81, .425], [-.14, .425]])
        c, s = math.cos(yaw), math.sin(yaw)
        world = corners @ self.np.array([[c, s], [-s, c]])+[x, y]
        return float(min(self.np.min(world[:, 0]+10), self.np.min(22-world[:, 0]),
                         self.np.min(world[:, 1]-self.center+self.width/2),
                         self.np.min(self.center+self.width/2-world[:, 1])))

    def tick(self, _event):
        with self.lock:
            if self.done:
                return
            now = self.rospy.get_time()
            if time.monotonic()-self.started_wall > 180:
                self.finish(False, 'scenario_wall_timeout')
                return
            if self.pose is not None:
                self.trace.append(self.pose[:2])
                self.min_clearance = min(self.min_clearance, self.clearance())
                if self.min_clearance < -.02:
                    self.finish(False, 'vehicle_footprint_crossed_road_wall')
                    return
                if self.gazebo_pose is not None:
                    self.max_gazebo_error = max(self.max_gazebo_error, math.dist(self.pose[:2], self.gazebo_pose))
            self.event('sample', {'stage': self.stage, 'pose': self.pose, 'command': self.command,
                                  'connection': self.connection, 'runtime': self.runtime})
            self.writer.write(self.render(now))
            self.frames += 1
            if self.stage == 'waiting':
                if (self.pose is not None and self.gazebo_pose is not None and self.scan_received is not None and
                        self.goal_pub.get_num_connections() >= 2):
                    self.send_goal(10., 0.)
                    self.stage, self.stage_started = 'outbound', now
            elif self.stage == 'outbound' and self.arrived and math.dist(self.pose[:2], (10., self.center)) < .8:
                self.outbound_arrived = True
                self.stage, self.stage_started = 'settling', now
                self.event('outbound_arrived', {'pose': self.pose})
                if self.scenario == 'unknown':
                    self.mode_pub.publish(self.String(data='unknown'))
            elif self.stage == 'settling' and now-self.stage_started > 1.:
                self.send_goal(0., math.pi)
                self.stage, self.stage_started = 'return', now
            elif self.stage == 'return':
                phase = (self.connection or {}).get('phase')
                if self.scenario == 'wide':
                    if phase == 'blocked':
                        self.finish(False, 'unexpected_block: '+self.connection.get('reason', ''))
                    elif self.arrived and math.dist(self.pose[:2], (0., self.center)) < .8:
                        self.finish(self.seen_connecting and self.seen_completed, 'round_trip_completed')
                elif phase == 'blocked':
                    if self.connection.get('reason') != 'no_safe_forward_connection':
                        self.finish(False, 'unexpected_stop_reason: '+self.connection.get('reason', ''))
                        return
                    if self.blocked_since is None:
                        self.blocked_since, self.blocked_pose = now, self.pose[:2]
                    elif now-self.blocked_since > 2.:
                        drift = math.dist(self.pose[:2], self.blocked_pose)
                        self.finish(self.zero_samples >= 5 and drift < .05, 'safe_stop_verified')

    def render(self, now):
        cv2, np = self.cv2, self.np
        frame = np.full((650, 1100, 3), (35, 43, 36), np.uint8)
        def pixel(x, y):
            return int(55+(x+12)*27), int(155+(9-(y-self.center))*25)
        a, b = pixel(-10, self.center+self.width/2), pixel(22, self.center-self.width/2)
        cv2.rectangle(frame, a, b, (65, 65, 65), -1)
        cv2.rectangle(frame, a, b, (210, 210, 210), 3)
        cv2.line(frame, pixel(-10, self.center), pixel(22, self.center), (100, 100, 100), 1)
        for name, color in (('osm_global_path', (240, 150, 40)), ('neupan_initial_path', (90, 230, 100)),
                            ('neupan_plan', (180, 140, 255))):
            points = self.paths.get(name, [])
            if len(points) >= 2:
                cv2.polylines(frame, [np.array([pixel(x, y) for x, y in points], np.int32)], False, color, 2)
        if len(self.trace) >= 2:
            cv2.polylines(frame, [np.array([pixel(x, y) for x, y in self.trace], np.int32)], False, (50, 220, 255), 2)
        if self.pose:
            x, y, yaw, _ = self.pose
            vertices = np.array([[-.14, -.425], [.81, -.425], [.81, .425], [-.14, .425]])
            c, s = math.cos(yaw), math.sin(yaw)
            corners = vertices @ np.array([[c, s], [-s, c]])+[x, y]
            cv2.fillPoly(frame, [np.array([pixel(*p) for p in corners], np.int32)], (230, 210, 90))
            cv2.arrowedLine(frame, pixel(x, y), pixel(x+1.2*c, y+1.2*s), (40, 50, 255), 2)
        for x, label in ((0., 'START / RETURN'), (10., 'OUTBOUND GOAL')):
            cv2.circle(frame, pixel(x, self.center), 7, (60, 90, 255), 2)
            px, py = pixel(x, self.center)
            cv2.putText(frame, label, (px-40, py+35), cv2.FONT_HERSHEY_SIMPLEX, .45, (240, 240, 240), 1)
        phase, reason = (self.connection or {}).get('phase', 'waiting'), (self.connection or {}).get('reason', '')
        speed = self.command[0] if self.command else 0.
        for y, text in ((30, 'Outdoor simulation: '+self.scenario+' | stage='+self.stage),
                        (62, 'sim={:.1f}s  v={:.2f}m/s  road_width={:.1f}m'.format(now, speed, self.width)),
                        (94, 'connection='+phase+' | '+reason),
                        (625, 'Blue: OSM | Green: accepted reference | Pink: local plan | Yellow: actual trajectory')):
            cv2.putText(frame, text, (22, y), cv2.FONT_HERSHEY_SIMPLEX, .65, (230, 235, 235), 1, cv2.LINE_AA)
        return frame

    def finish(self, passed, reason):
        if self.done:
            return
        if passed and self.max_gazebo_error > .25:
            passed, reason = False, 'gazebo_model_not_following_virtual_base'
        self.done = True
        finished_sim, finished_wall = self.rospy.get_time(), time.monotonic()
        self.event('result', {'passed': passed, 'reason': reason})
        self.writer.release()
        raw, final = self.folder/'overview_raw.mp4', self.folder/'overview.mp4'
        conversion = subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-i', str(raw),
            '-c:v', 'libx264', '-threads', '2', '-preset', 'veryfast', '-pix_fmt', 'yuv420p',
            '-movflags', '+faststart', str(final)], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        video = final if conversion.returncode == 0 else raw
        if video == final:
            raw.unlink()
        result = {'scenario': self.scenario, 'passed': bool(passed), 'reason': reason,
            'outbound_arrived': self.outbound_arrived, 'seen_connection': self.seen_connecting,
            'seen_connection_completed': self.seen_completed, 'final_pose': self.pose,
            'min_wall_clearance_m': self.min_clearance if math.isfinite(self.min_clearance) else None,
            'zero_commands_after_block': self.zero_samples, 'connection': self.connection,
            'sim_time': finished_sim, 'wall_duration_sec': finished_wall-self.started_wall,
            'video_frames': self.frames, 'video': str(video), 'video_fps': 10,
            'max_gazebo_to_odometry_error_m': self.max_gazebo_error,
            'return_goal_error_m': math.dist(self.pose[:2], (0., self.center)) if self.pose else None,
            'ground_truth_localization': True, 'real_hardware_started': False}
        self.log.flush()
        os.fsync(self.log.fileno())
        save_json(self.folder/'result.json', result)
        self.rospy.loginfo('OUTDOOR TEST %s: %s', 'PASS' if passed else 'FAIL', reason)

    def close(self):
        with self.lock:
            if not self.done:
                self.writer.release()
            if not self.log.closed:
                self.log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=('run', 'bridge', 'scenario'))
    parser.add_argument('--scenario', choices=('all', 'wide', 'narrow', 'unknown'), default='all')
    parser.add_argument('--output-dir')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--rviz', action='store_true')
    parser.add_argument('--record-bag', action='store_true')
    parser.add_argument('--timeout', type=float, default=240.)
    args, unknown = parser.parse_known_args()
    if args.mode == 'run':
        if unknown:
            parser.error('unknown arguments: '+' '.join(unknown))
        return run_cases(args)
    if args.mode == 'bridge':
        sensor_bridge()
    else:
        import rospy
        ScenarioTest()
        rospy.spin()
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
