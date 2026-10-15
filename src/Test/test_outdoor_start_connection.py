"""Regression tests for outdoor return-route admission; never drives hardware."""
import ast
import math
import os
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/neupan_ros/src'))
from outdoor_start_connection import (RoadCorridor, ScanEnvelope, StartRouteConnector,
                                      clean_route, dubins_candidates, wrap)

VERTICES = np.array([[-.14, .81, .81, -.14], [-.425, -.425, .425, .425]])


def route(yaw=0., origin=(0., 0.)):
    distance = np.linspace(0., 20., 41)
    return np.column_stack((origin[0]+distance*math.cos(yaw), origin[1]+distance*math.sin(yaw),
                            np.full(len(distance), yaw), np.ones(len(distance))))


def scan(now=1., distance=15., pose=(0., 0., 0.)):
    return ScanEnvelope(np.full(1441, distance), -math.pi, math.pi/720,
                        .05, 30., pose, now, now)


def connector(width=20.):
    return StartRouteConnector(VERTICES, 1.85, RoadCorridor([[-30, 0, 30, 0, width/2, 0]]))


class ConnectionTests(unittest.TestCase):
    def test_clean_duplicate_positions_without_changing_gear(self):
        result = clean_route([[0, 0, 0, 1], [0, 0, 0, 1], [1, 0, 0, 1]])
        self.assertEqual(len(result), 2)
        self.assertTrue(np.all(result[:, 3] == 1))

    def test_degenerate_or_nonfinite_reference_is_rejected(self):
        for path in ([[0, 0, 0, 1]]*2, [[0, 0, 0, 1], [np.nan, 0, 0, 1]],
                     [[0, 0, 0, 1], [1, 0, 0, -1]]):
            with self.assertRaises(ValueError):
                clean_route(path)

    def test_forward_route_remains_unchanged(self):
        state = connector()
        source = route()
        decision, result = state.prepare(source, ('map', 1), (0, 0, 0), 1, 1)
        self.assertEqual(decision, 'apply')
        np.testing.assert_array_equal(result, source)
        self.assertEqual(state.reason, 'start_aligned')

    def test_nonfinite_vehicle_pose_is_held_before_solver(self):
        state = connector()
        self.assertEqual(state.prepare(route(), ('map', 1), (np.nan, 0, 0), 1, 1), ('reject', None))
        self.assertEqual(state.phase, 'blocked')

    def test_reverse_heading_is_not_admitted_without_scan(self):
        state = connector()
        self.assertEqual(state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 1, 1)[0], 'wait')
        self.assertEqual(state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 7, 7)[0], 'reject')
        self.assertEqual(state.phase, 'blocked')

    def test_missing_corridor_never_falls_back_to_opposite_reference(self):
        state = connector()
        state.corridor = None
        state.scan = scan()
        self.assertEqual(state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 1, 1), ('reject', None))
        self.assertEqual(state.reason, 'connection_road_corridor_unavailable')

    def test_stale_or_future_scan_is_not_accepted(self):
        self.assertTrue(scan().fresh(1.1, 1.1))
        self.assertFalse(scan().fresh(1.6, 1.6))
        self.assertFalse(scan().fresh(1., .8))

    def test_synthetic_inf_and_nan_are_unknown(self):
        for distance in (np.inf, np.nan):
            self.assertFalse(scan(distance=distance).contains([[1, 0, 0]], VERTICES, (0, 0, 0)))

    def test_occupied_space_blocks_the_inflated_body(self):
        self.assertFalse(scan(distance=.5).contains([[1, 0, 0]], VERTICES, (0, 0, 0)))

    def test_partial_fov_is_not_wrapped_into_rear_coverage(self):
        evidence = ScanEnvelope(np.full(721, 15.), -math.pi/2, math.pi/720,
                                .05, 30., (0, 0, 0), 1, 1)
        self.assertFalse(evidence.contains([[-1.5, 0, 0]], VERTICES, (0, 0, 0)))

    def test_scan_rotation_and_translation(self):
        evidence = scan(pose=(10, 20, math.pi/2))
        self.assertTrue(evidence.contains([[10, 21, math.pi/2]], VERTICES, (10, 20, math.pi/2)))

    def test_road_width_includes_body_not_just_center(self):
        corridor = RoadCorridor([[-20, 0, 20, 0, .6, 0]])
        self.assertTrue(corridor.contains([[0, 0, 0]], VERTICES))
        self.assertFalse(corridor.contains([[0, .4, 0]], VERTICES))

    def test_wrong_way_oneway_connection_is_rejected(self):
        corridor = RoadCorridor([[-20, 0, 20, 0, 10, 1]])
        self.assertTrue(corridor.contains([[0, 0, 0]], VERTICES))
        self.assertFalse(corridor.contains([[0, 0, math.pi]], VERTICES))

    def test_blocked_goal_remains_blocked_until_new_goal(self):
        state = connector()
        state.corridor = None
        state.scan = scan()
        state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 1, 1)
        self.assertEqual(state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 2, 2)[0], 'reject')
        self.assertEqual(state.prepare(route(), ('map', 2), (0, 0, 0), 2, 2)[0], 'apply')

    def test_camera_or_fallback_cannot_erase_active_connection(self):
        state = connector()
        state.key, state.phase, state.prefix_count = ('map', 1), 'connecting', 50
        for path in (route(math.pi), route(math.pi)+np.array([0, .2, 0, 0])):
            self.assertEqual(state.prepare(path, ('map', 1), (0, 0, 0), 2, 2)[0], 'keep')
        self.assertFalse(state.update_progress(48))
        self.assertTrue(state.update_progress(49))
        self.assertTrue(state.reseed)
        self.assertEqual(state.phase, 'following_route')

    def test_indoor_launch_and_hybrid_branch_do_not_enable_connector(self):
        tree = ET.parse(ROOT/'src/neupan_ros/example/lxw_car/launch/full_system_run.launch')
        for group in tree.getroot().iter('group'):
            if (group.get('unless') == '$(arg outdoor_mode)' or
                    group.get('if') == '$(arg outdoor_use_hybrid_astar)'):
                self.assertFalse(any(p.get('name') == 'outdoor_start_connector_enabled' for p in group.iter('param')))

    def test_real_forward_uturn_geometry(self):
        state = connector()
        state.scan = scan()
        source = route(math.pi)
        try:
            decision, result = state.prepare(source, ('map', 1), (0, 0, 0), 1, 1)
        except ImportError:
            self.skipTest('gctl is available in the ROS1 lxw environment')
        self.assertEqual(decision, 'apply')
        self.assertEqual(state.phase, 'connecting')
        np.testing.assert_allclose(result[0], [0, 0, 0, 1], atol=1.e-8)
        np.testing.assert_allclose(result[-1], source[-1])
        self.assertTrue(np.all(result[:, 3] == 1))
        prefix = result[:state.prefix_count]
        delta = np.diff(prefix[:, :2], axis=0)
        ds = np.linalg.norm(delta, axis=1)
        yaw_delta = np.array([wrap(b-a) for a, b in zip(prefix[:-1, 2], prefix[1:, 2])])
        self.assertLessEqual(np.max(np.abs(yaw_delta)/ds), 1/1.85+.002)
        tangent = np.column_stack((np.cos(prefix[:-1, 2]), np.sin(prefix[:-1, 2])))
        self.assertTrue(np.all(np.sum(delta*tangent, axis=1) > 0))

    def test_logged_175_degree_start_mismatch(self):
        pose = np.array([123.375, 332.447, math.radians(-85.257)])
        yaw = math.radians(89.726)
        source = route(yaw, pose[:2])
        direction = np.array([math.cos(yaw), math.sin(yaw)])
        a, b = pose[:2]-30*direction, pose[:2]+30*direction
        state = StartRouteConnector(VERTICES, 1.85, RoadCorridor([[*a, *b, 10., 0]]))
        state.scan = scan(pose=pose)
        try:
            decision, result = state.prepare(source, ('map', 2), pose, 1, 1)
        except ImportError:
            self.skipTest('gctl is available in the ROS1 lxw environment')
        self.assertAlmostEqual(state.heading_error_deg, 174.983, places=2)
        self.assertEqual(decision, 'apply')
        np.testing.assert_allclose(result[0, :3], pose, atol=1.e-8)
        self.assertLess(abs(wrap(result[0, 2]-pose[2])), 1.e-8)

    def test_no_room_and_unknown_scan_both_reject_without_crash(self):
        for width, distance in ((1., 15.), (20., np.inf), (20., 1.)):
            state = connector(width)
            state.scan = scan(distance=distance)
            try:
                decision, result = state.prepare(route(math.pi), ('map', 1), (0, 0, 0), 1, 1)
            except ImportError:
                self.skipTest('gctl is available in the ROS1 lxw environment')
            self.assertEqual((decision, result), ('reject', None))
            self.assertEqual(state.reason, 'no_safe_forward_connection')


class AdapterTests(unittest.TestCase):
    """Run the real ROS adapter method with fake publishers/planner, no master."""
    def node(self):
        filename = ROOT/'src/neupan_ros/src/neupan_core.py'
        tree = ast.parse(filename.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'neupan_core')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'apply_pending_route')
        ros = SimpleNamespace(loginfo=Mock(), logwarn_throttle=Mock(), Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 1.)))
        namespace = {'np': np, 'time': SimpleNamespace(monotonic=lambda: 1.),
                     'rospy': ros, 'Twist': object, 'yellow_terminal_text': lambda x: x}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(filename), 'exec'), namespace)
        node = SimpleNamespace(route_update_lock=threading.Lock(), pending_route_update=None,
            minimum_path_stamp=0, diagnostic_route_update=None, start_connector=connector(),
            robot_state=np.zeros((3, 1)), publish_connection_status=Mock(), vel_pub=Mock(),
            waiting_for_new_goal=False, camera_guard=None, arrival_path_key=('map', 0),
            goal_arrived_timer_callback=Mock(), neupan_planner=Mock())
        return node, namespace['apply_pending_route']

    def test_rejected_return_route_does_not_reach_solver_or_resume_old_route(self):
        node, apply = self.node()
        node.start_connector.scan = scan()
        node.start_connector.corridor = None
        node.pending_route_update = ('path', [p.reshape(4, 1) for p in route(math.pi)], ('map', 1))
        apply(node)
        node.neupan_planner.set_initial_path.assert_not_called()
        node.neupan_planner.reset.assert_not_called()
        node.vel_pub.publish.assert_called_once()
        self.assertTrue(node.waiting_for_new_goal)

    def test_valid_forward_route_still_resets_arrival_for_second_goal(self):
        node, apply = self.node()
        node.pending_route_update = ('path', [p.reshape(4, 1) for p in route()], ('map', 1))
        apply(node)
        node.neupan_planner.set_initial_path.assert_called_once()
        self.assertFalse(node.waiting_for_new_goal)
        self.assertFalse(node.arrive)
        self.assertEqual(node.arrival_path_key, ('map', 1))

    def test_missing_scan_requeues_new_goal_and_holds_zero(self):
        node, apply = self.node()
        node.pending_route_update = ('path', [p.reshape(4, 1) for p in route(math.pi)], ('map', 1))
        apply(node)
        node.neupan_planner.set_initial_path.assert_not_called()
        self.assertIsNotNone(node.pending_route_update)
        self.assertTrue(node.waiting_for_new_goal)


@unittest.skipUnless(os.environ.get('NEUPAN_ROS_SMOKE') == '1', 'optional isolated ROS1 process test')
class RosProcessTest(unittest.TestCase):
    def test_blocked_second_goal_holds_zero_and_process_stays_alive(self):
        import json
        import signal
        import socket
        import subprocess
        import tempfile
        import time
        import rospy
        import tf
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Path as RosPath
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String

        with socket.socket() as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:'+str(port), ROS_HOSTNAME='127.0.0.1')
        env['ROS_LOG_DIR'] = tempfile.mkdtemp(prefix='outdoor_start_connection_ros_')
        os.environ.update({k: env[k] for k in ('ROS_MASTER_URI', 'ROS_HOSTNAME', 'ROS_LOG_DIR')})
        master = subprocess.Popen(['roscore', '-p', str(port)], env=env,
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        process = None
        running = threading.Event()
        running.set()
        worker = None
        try:
            rospy.init_node('outdoor_start_connection_test', anonymous=True, disable_signals=True)
            base = ROOT/'src/neupan_ros/example/lxw_car'
            command = [sys.executable, str(ROOT/'src/neupan_ros/src/neupan_node.py'), '__name:=connection_test_planner',
                '_config_file:='+str(base/'config/neupan_config_outdoor.yaml'),
                '_dune_checkpoint:='+str(base/'model/acker_robot_default_6/model_5000.pth'),
                '_refresh_initial_path:=true', '_include_initial_path_direction:=true',
                '_enable_adaptive_adjust:=false', '_outdoor_start_connector_enabled:=true',
                '_osm_file:=/home/nuc/lxw/ros_maps/map.osm', '_map_origin_lat:=23.0398',
                '_map_origin_lon:=113.39002', '/neupan_cmd_vel:=/connection_test/cmd_vel']
            # Only planner + this fake-sensor process are launched. The private
            # master has no base driver or control mux, and commands are remapped.
            process = subprocess.Popen(command, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            scans = rospy.Publisher('/scan', LaserScan, queue_size=1)
            paths = rospy.Publisher('/initial_path', RosPath, queue_size=1, latch=True)
            goals = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
            commands, statuses = [], []
            subscribers = [rospy.Subscriber('/connection_test/cmd_vel', Twist,
                lambda msg: commands.append((time.monotonic(), msg.linear.x)), queue_size=100),
                rospy.Subscriber('/neupan/route_connection', String,
                lambda msg: statuses.append(json.loads(msg.data)), queue_size=100)]
            broadcaster = tf.TransformBroadcaster()

            def sensors():
                while running.is_set() and not rospy.is_shutdown():
                    stamp = rospy.Time.now()
                    broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), stamp, 'base_link', 'map')
                    message = LaserScan()
                    message.header.stamp, message.header.frame_id = stamp, 'base_link'
                    message.angle_min, message.angle_max, message.angle_increment = -math.pi, math.pi, math.pi/720
                    message.range_min, message.range_max = .4, 30.
                    message.ranges = [float('inf')]*1441
                    scans.publish(message)
                    time.sleep(.04)

            worker = threading.Thread(target=sensors, daemon=True)
            worker.start()

            def wait_for(predicate, seconds=20):
                deadline = time.monotonic()+seconds
                while time.monotonic() < deadline:
                    self.assertIsNone(process.poll(), 'planner process exited unexpectedly')
                    if predicate():
                        return
                    time.sleep(.05)
                self.fail('ROS smoke timeout; statuses='+str(statuses[-3:]))

            wait_for(lambda: paths.get_num_connections() > 0 and goals.get_num_connections() > 0)

            def publish_goal_and_path(yaw):
                goal = PoseStamped()
                goal.header.frame_id, goal.header.stamp = 'map', rospy.Time.now()
                goal.pose.position.x = 20*math.cos(yaw)
                goal.pose.position.y = 20*math.sin(yaw)
                goal.pose.orientation.w = 1.
                path = RosPath()
                path.header.frame_id, path.header.stamp = 'map', goal.header.stamp
                for point in route(yaw):
                    pose = PoseStamped()
                    pose.header = path.header
                    pose.pose.position.x, pose.pose.position.y = point[:2]
                    pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                    path.poses.append(pose)
                goals.publish(goal)
                paths.publish(path)

            publish_goal_and_path(0.)
            wait_for(lambda: any(v > .05 for _, v in commands))
            publish_goal_and_path(math.pi)
            wait_for(lambda: statuses and statuses[-1]['phase'] == 'blocked')
            blocked_time = time.monotonic()
            time.sleep(.7)
            tail = [v for stamp, v in commands if stamp > blocked_time+.2]
            self.assertTrue(tail)
            self.assertTrue(all(v == 0. for v in tail), tail)
            self.assertIsNone(process.poll())
        finally:
            running.clear()
            if worker:
                worker.join(timeout=2)
            for child in (process, master):
                if child is not None and child.poll() is None:
                    child.send_signal(signal.SIGINT)
                    try:
                        child.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
