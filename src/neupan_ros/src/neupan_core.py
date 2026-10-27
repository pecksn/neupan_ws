#!/usr/bin/env python3

"""
neupan_core is the main class for the neupan_ros package. It is used to run the NeuPAN algorithm in the ROS framework, which subscribes to the laser scan and localization information, and publishes the velocity command to the robot.

Developed by Ruihua Han
Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.
"""

from neupan import neupan
from neupan_safety import (validate_solution, trajectory_clearance, reference_offsets,
                           checked_input_ages, pose_snapshot, InputUnavailable, AuthorizationChanged, StopRegistry, cap_solution_speed)
from osm_camera_path_guard import CameraPathGuard, check_execution_contract
from osm_outdoor_start_connection import RoadCorridor, ScanEnvelope, StartRouteConnector, resample_forward_reference, OutdoorTrackingGuard
import copy
from collections import deque
import json
import threading
import time
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray, Marker
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String
from math import sin, cos, atan2, degrees, radians
import numpy as np
from neupan.util import get_transform
import tf
import sensor_msgs.point_cloud2 as pc2


ANSI_YELLOW = "\033[33m"
ANSI_RED = "\033[31m"
ANSI_RESET = "\033[0m"


def yellow_terminal_text(message):
    """Highlight lifecycle messages without changing their ROS log severity."""
    return "{}{}{}".format(ANSI_YELLOW, message, ANSI_RESET)


def red_terminal_text(message):
    """Highlight missing TF messages without changing their ROS log severity."""
    return "{}{}{}".format(ANSI_RED, message, ANSI_RESET)


class neupan_core:
    def __init__(self) -> None:

        rospy.init_node("neupan_node", anonymous=True)
        self.diagnostic_stage = "initialization"
        self.diagnostic_solver_input = None
        self.diagnostic_route_update = None
        self.failure_pub = rospy.Publisher('/neupan/failure', String, queue_size=1, latch=True)

        # ros parameters
        self.planner_config_file = rospy.get_param("~config_file", None)
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.lidar_frame = rospy.get_param("~lidar_frame", "laser_link")
        self.marker_size = float(rospy.get_param("~marker_size", "0.05"))
        self.marker_z = float(rospy.get_param("~marker_z", "1.0"))

        scan_angle_range_para = rospy.get_param("~scan_angle_range", "-3.14 3.14")
        self.scan_angle_range = np.fromstring(
            scan_angle_range_para, dtype=np.float32, sep=" "
        )

        self.scan_downsample = int(rospy.get_param("~scan_downsample", "1"))

        scan_range_para = rospy.get_param("~scan_range", "0.0 5.0")
        scan_range_para = scan_range_para.replace(",", " ")
        self.scan_range = np.fromstring(scan_range_para, dtype=np.float32, sep=" ")
        self.scan_self_filter = bool(rospy.get_param("~scan_self_filter", False))
        self.scan_self_filter_length = float(
            rospy.get_param("~scan_self_filter_length", "0.0")
        )
        self.scan_self_filter_width = float(
            rospy.get_param("~scan_self_filter_width", "0.0")
        )
        self.scan_self_filter_margin = max(
            0.0, float(rospy.get_param("~scan_self_filter_margin", "0.03"))
        )
        self.stop_on_scan_fault = bool(rospy.get_param("~stop_on_scan_fault", True))
        self.scan_timeout = max(0.1, float(rospy.get_param("~scan_timeout", 0.5)))
        self.scan_transform_timeout = max(
            0.1, float(rospy.get_param("~scan_transform_timeout", 0.5))
        )
        self.scan_recovery_stable_sec = max(
            0.0, float(rospy.get_param("~scan_recovery_stable_sec", 0.5))
        )
        self.scan_filtered_empty_timeout = max(
            0.1, float(rospy.get_param("~scan_filtered_empty_timeout", 2.0))
        )
        self.dune_checkpoint = rospy.get_param("~dune_checkpoint", None)
        self.refresh_initial_path = rospy.get_param("~refresh_initial_path", False)
        self.flip_angle = rospy.get_param("~flip_angle", False)
        self.include_initial_path_direction = rospy.get_param("~include_initial_path_direction", False)
        # Enabled only in the outdoor OSM launch; indoor references are untouched.
        self.outdoor_reference_spacing = float(rospy.get_param('~outdoor_reference_spacing_m', 0.))
        if self.outdoor_reference_spacing != 0 and not (
                np.isfinite(self.outdoor_reference_spacing) and self.outdoor_reference_spacing >= .02):
            raise ValueError('outdoor_reference_spacing_m must be zero or finite and at least 0.02 m')
        self.outdoor_tracking_guard = OutdoorTrackingGuard() if self.outdoor_reference_spacing else None
        self.outdoor_tracking_status = None
        self.camera_guard = CameraPathGuard(float(rospy.get_param('~camera_right_guard_timeout', 3.0))) \
            if rospy.get_param('~camera_right_guard_enabled', False) else None
        self.camera_guard_signature = None
        self.camera_guard_last_state = None
        self.camera_guard_last_publish = 0.
        self.verbose_runtime_log = bool(rospy.get_param("~verbose_runtime_log", False))
        self.enable_adaptive_adjust = rospy.get_param("~enable_adaptive_adjust", True)
        self.adaptive_far_distance = float(rospy.get_param("~adaptive_far_distance", "4.0"))
        self.adaptive_near_distance = float(rospy.get_param("~adaptive_near_distance", "1.8"))
        self.adaptive_front_half_angle_deg = float(rospy.get_param("~adaptive_front_half_angle_deg", "60.0"))
        self.clear_adjust_profile = self.parse_adjust_profile(
            rospy.get_param("~clear_adjust_profile", "1.6 36.0 0.15 0.25 0.2"),
            [1.6, 36.0, 0.15, 0.25, 0.2],
        )
        self.avoid_adjust_profile = self.parse_adjust_profile(
            rospy.get_param("~avoid_adjust_profile", "2.8 18.0 2.6 1.2 0.2"),
            [2.8, 18.0, 2.6, 1.2, 0.2],
        )
        
        if self.planner_config_file is None:
            raise ValueError(
                "No planner config file provided! Please set the parameter ~config_file"
            )

        pan = {'dune_checkpoint': self.dune_checkpoint}
        self.neupan_planner = neupan.init_from_yaml(
            self.planner_config_file, pan=pan
        )
        # Outdoor U-turn connection only. Neither the optimiser nor its
        # existing reverse capability or the indoor search profile is changed.
        self.start_connector = None
        self.last_connection_status = None
        self.last_connection_publish = 0.
        self.connection_scan_lock = threading.Lock()
        self.connection_scan_queue = deque(maxlen=8)
        if rospy.get_param('~outdoor_start_connector_enabled', False):
            planner = self.neupan_planner
            if planner.robot.kinematics != 'acker':
                raise ValueError('Outdoor start connection requires Ackermann kinematics')
            radius = max(float(planner.ipath.min_radius),
                         float(planner.robot.L)/np.tan(float(planner.robot.max_speed[1, 0])))
            try:
                corridor = RoadCorridor.load(rospy.get_param('~osm_file'),
                    float(rospy.get_param('~map_origin_lat')), float(rospy.get_param('~map_origin_lon')),
                    float(rospy.get_param('~default_road_width', 6.)))
            except Exception as error:
                corridor = None
                rospy.logwarn('Outdoor start connection: OSM corridor unavailable: %s', error)
            method = rospy.get_param('~outdoor_connection_planner', 'hybrid_astar')
            search = None
            if method == 'hybrid_astar':
                from osm_hybrid_search import CppHybridSearch
                search = CppHybridSearch(
                    float(rospy.get_param('~outdoor_connection_budget_sec', 1.0)),
                    int(rospy.get_param('~outdoor_connection_max_iterations', 12000)))
                rospy.on_shutdown(search.close)
            elif method != 'dubins':
                raise ValueError('outdoor_connection_planner must be hybrid_astar or dubins')
            self.start_connector = StartRouteConnector(planner.robot.vertices, radius, corridor, search=search)
        # print()

        # data
        self.obstacle_points = None  # (2, n)  n number of points
        self.robot_state = None  # (3, 1) [x, y, theta]
        self.stop = False
        self.arrive = False
        self.goal_arrived_state = False
        self.arrival_path_key = None
        self.waiting_for_new_goal = False
        self.route_update_lock = threading.Lock()
        self.pending_route_update = None
        self.pending_goal_request_stamp = None
        self.minimum_path_stamp = 0
        self.current_adjust_profile_name = "init"
        self.current_adjust_values = None
        self.last_scan_received_time = None
        self.last_scan_valid_time = None
        self.scan_valid_since = None
        self.filtered_empty_since = None
        self.scan_fault_reason = "waiting_for_scan"
        self.scan_mode = "WAITING"
        self.last_scan_status_publish = rospy.Time(0)

        # publisher
        self.vel_pub = rospy.Publisher("/neupan_cmd_vel", Twist, queue_size=10)
        self.shutdown_requested = False
        self.planner_fault = None
        self.planner_session_id = '%s-%s' % (__import__('os').getpid(), time.time_ns())
        rospy.on_shutdown(self.shutdown_stop)
        self.runtime_pub = rospy.Publisher("/neupan/runtime", String, queue_size=10)
        self.camera_guard_pub = rospy.Publisher('/neupan/camera_path_guard', String, queue_size=2, latch=True)
        self.connection_status_pub = rospy.Publisher('/neupan/route_connection', String, queue_size=2, latch=True)
        self.last_command_monotonic = None
        self.goal_arrived_pub = rospy.Publisher("/goal_arrived", String, queue_size=1, latch=True)
        self.plan_pub = rospy.Publisher("/neupan_plan", Path, queue_size=10)
        self.ref_state_pub = rospy.Publisher(
            "/neupan_ref_state", Path, queue_size=10
        )  # current reference state
        self.ref_path_pub = rospy.Publisher(
            "/neupan_initial_path", Path, queue_size=10
        )  # initial path
        self.scan_health_pub = rospy.Publisher(
            "/neupan/scan_health", String, queue_size=2, latch=True
        )

        ## for rviz visualization
        self.point_markers_pub_dune = rospy.Publisher(
            "/dune_point_markers", MarkerArray, queue_size=10
        )
        self.robot_marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
        self.point_markers_pub_nrmp = rospy.Publisher(
            "/nrmp_point_markers", MarkerArray, queue_size=10
        )

        self.listener = tf.TransformListener()

        # subscriber
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # three types of initial path:
        # 1. from given path
        # 2. from waypoints
        # 3. from goal position
        rospy.Subscriber("/initial_path", Path,
                         self.camera_guard.candidate_cb if self.camera_guard else self.path_callback)
        if self.camera_guard:
            rospy.Subscriber(rospy.get_param('~camera_right_fallback_topic', '/osm_global_path'),
                             Path, self.camera_guard.original_cb, queue_size=1)
            rospy.Subscriber('/camera_right/status', String, self.camera_guard_status_cb, queue_size=1)
        rospy.Subscriber("/neupan_waypoints", Path, self.waypoints_callback)
        rospy.Subscriber("/neupan_goal", PoseStamped, self.goal_callback)
        rospy.Subscriber(
            rospy.get_param("~goal_request_topic", "/move_base_simple/goal"),
            PoseStamped, self.goal_request_callback
        )
        goal_arrived_rate = max(0.1, float(rospy.get_param("~goal_arrived_publish_rate", 2.0)))
        self.goal_arrived_timer = rospy.Timer(
            rospy.Duration(1.0 / goal_arrived_rate), self.goal_arrived_timer_callback
        )
        

    def shutdown_stop(self):
        self.shutdown_requested = True
        self.shutdown_requested_monotonic = time.monotonic()
        try:
            self.vel_pub.publish(Twist())
        except Exception:
            pass

    def run(self):

        r = rospy.Rate(50)
        self.stop_registry=StopRegistry()
        self.input_good_since=None

        while not rospy.is_shutdown():

            if getattr(self, 'planner_fault', None) is not None:
                self.stop_registry.set('planner_fault',time.monotonic(),'inspect evidence and restart planner',True)
                self.publish_stop_snapshot('planner_fault')
                self.vel_pub.publish(Twist())
                rospy.logerr_throttle(2., 'NeuPAN fault held STOP; verify input and restart: %s', self.planner_fault)
                r.sleep()
                continue

            self.diagnostic_stage = "goal_update"
            self.diagnostic_solver_input = None
            self.diagnostic_command_action=None
            self.diagnostic_checked_trajectory=None
            self.diagnostic_checked_controls=None
            self.camera_publication_contract=None
            self.camera_checked_authorization_id=None
            self.camera_publication_generation=None
            self.apply_pending_goal_request()
            if not self.apply_camera_path_guard():
                self.publish_stop_snapshot('camera_guard')
                self.vel_pub.publish(Twist())
                r.sleep()
                continue
            with self.route_update_lock:
                route_pending = self.pending_route_update is not None
            self.stop_registry.clear('authorization')
            if self.waiting_for_new_goal and not route_pending:
                self.stop_registry.set('goal',time.monotonic(),'new explicit goal/path')
                self.publish_stop_snapshot('waiting_for_goal')
                self.vel_pub.publish(Twist())
                message = ("Goal reached; waiting for a new goal/path" if self.goal_arrived_state
                           else "Waiting for planned path for the new goal")
                rospy.loginfo_throttle(2.0, yellow_terminal_text(message))
                r.sleep()
                continue
            self.stop_registry.clear('goal')

            self.diagnostic_stage = "vehicle_tf"
            try:
                t0 = rospy.Time.now()
                frame_started_mono = time.monotonic()
                trans,rot,self.diagnostic_state_stamp_ns=pose_snapshot(
                    self.listener,self.map_frame,self.base_frame)
                t1 = rospy.Time.now()

                yaw = self.quat_to_yaw_list(rot)
                x, y = trans[0], trans[1]
                self.robot_state = np.array([x, y, yaw]).reshape(3, 1)

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException,
                InputUnavailable,
            ):
                rospy.loginfo_throttle(
                    1,
                    red_terminal_text(
                        "waiting for tf for the transform from {} to {}".format(
                            self.base_frame, self.map_frame
                        )
                    ),
                )
                self.vel_pub.publish(Twist())
                self.temporary_input_stop('vehicle TF unavailable')
                self.publish_scan_health(rospy.Time.now())
                r.sleep()
                continue

            if self.robot_state is None:
                rospy.logwarn_throttle(1, "waiting for robot state")
                self.vel_pub.publish(Twist())
                self.publish_scan_health(rospy.Time.now())
                r.sleep()
                continue

            self.diagnostic_stage = "route_update"
            self.refresh_connection_scan()
            self.apply_pending_route()
            if self.waiting_for_new_goal:
                self.vel_pub.publish(Twist())
                r.sleep()
                continue

            rospy.loginfo_once(
                "robot state received {}".format(self.robot_state.tolist())
            )

            if (
                len(self.neupan_planner.waypoints) >= 1
                and self.neupan_planner.initial_path is None
            ):
                self.neupan_planner.set_initial_path_from_state(self.robot_state)
                # print('set initial path', self.neupan_planner.initial_path)

            if self.neupan_planner.initial_path is None:
                rospy.logwarn_throttle(1, "waiting for neupan initial path")
                self.vel_pub.publish(Twist())
                self.publish_scan_health(rospy.Time.now())
                r.sleep()
                continue

            rospy.loginfo_once("initial Path Received")
            self.ref_path_pub.publish(
                self.generate_path_msg(self.neupan_planner.initial_path)
            )

            scan_healthy, scan_reason = self.scan_is_healthy(rospy.Time.now())
            self.publish_scan_health(rospy.Time.now())
            if self.stop_on_scan_fault and not scan_healthy:
                rospy.logerr_throttle(1.0, "NeuPAN scan safety stop: %s", scan_reason)
                self.vel_pub.publish(Twist())
                self.temporary_input_stop(scan_reason)
                r.sleep()
                continue

            if 'input' in self.stop_registry.reasons:
                try:
                    checked_input_ages(getattr(self,'safety_scan',None),rospy.Time.now().to_nsec(),
                        time.monotonic(),self.scan_timeout,self.diagnostic_state_stamp_ns)
                except InputUnavailable as error:
                    self.temporary_input_stop(str(error));r.sleep();continue
                if self.input_good_since is None:self.input_good_since=time.monotonic()
                if time.monotonic()-self.input_good_since<.3:
                    self.vel_pub.publish(Twist());r.sleep();continue
                self.stop_registry.clear('input')

            self.diagnostic_stage = "outdoor_tracking_guard"
            if self.outdoor_tracking_guard is not None:
                ipath = self.neupan_planner.ipath
                self.outdoor_tracking_status = self.outdoor_tracking_guard.check(
                    ipath.cur_curve, self.robot_state, ipath.point_index, time.monotonic())
                if self.outdoor_tracking_status['latched']:
                    self.stop_registry.set('tracking',time.monotonic(),'new goal after verifying vehicle and route')
                    self.publish_stop_snapshot('tracking')
                    self.vel_pub.publish(Twist())
                    self.runtime_pub.publish(json.dumps({
                        'stamp': rospy.Time.now().to_sec(), 'linear_x': 0., 'steering_deg': 0.,
                        'command_source': 'outdoor_tracking_guard',
                        'tracking_guard': self.outdoor_tracking_status}, separators=(',', ':')))
                    rospy.logwarn_throttle(2., yellow_terminal_text(
                        'Outdoor tracking lost: holding STOP; verify vehicle/route, then send a NEW goal'))
                    r.sleep()
                    continue
                self.stop_registry.clear('tracking')
            self.diagnostic_stage = "adjust_parameters"
            self.update_adaptive_adjust_parameters()

            t2 = rospy.Time.now()
            solver_started = time.monotonic()
            self.diagnostic_stage = "solve"
            self.diagnostic_solver_input = (self.robot_state, self.obstacle_points)
            checked_scan = None
            try:
                contract=getattr(self,'camera_execution_contract',None)
                reference_speed=getattr(self.neupan_planner,'ref_speed',None)
                try:
                    if contract and reference_speed is not None:
                        self.neupan_planner.set_reference_speed(min(reference_speed,contract['speed_cap_mps']))
                    action, info = self.neupan_planner(*self.diagnostic_solver_input)
                finally:
                    if contract and reference_speed is not None:
                        self.neupan_planner.set_reference_speed(reference_speed)
                solver_finished = time.monotonic()
                safety_started = solver_finished
                self.diagnostic_stage = 'validate_solution'
                validate_solution(action, info, self.neupan_planner.robot, self.neupan_planner.dt)
                contract=getattr(self,'camera_execution_contract',None)
                if contract and not info.get('arrive'):
                    action,info=cap_solution_speed(action,info,self.neupan_planner.robot,
                                                  self.neupan_planner.dt,contract['speed_cap_mps'])
                    validate_solution(action,info,self.neupan_planner.robot,self.neupan_planner.dt)
                self.diagnostic_command_action=np.asarray(action,float).copy()
                self.diagnostic_checked_trajectory=np.hstack(info['opt_state_list']) if info.get('opt_state_list') else None
                self.diagnostic_checked_controls=info.get('vel_tensor')
                if not info.get('arrive'):
                    snapshot = getattr(self, 'safety_scan', None)
                    if snapshot is None:
                        raise InputUnavailable('full scan safety snapshot unavailable')
                    scan_points, stamp_ns, received_mono = snapshot
                    self.diagnostic_safety_scan = snapshot
                    checked_scan = snapshot
                    age = (rospy.Time.now().to_nsec()-stamp_ns)/1.e9
                    if not 0 <= age <= self.scan_timeout or time.monotonic()-received_mono > self.scan_timeout:
                        raise InputUnavailable('full scan expired before command publication')
                    self.diagnostic_stage = 'trajectory_safety'
                    self.trajectory_safety = trajectory_clearance(np.hstack(info['opt_state_list']),
                        scan_points, self.neupan_planner.robot.vertices)
                self.safety_check_ms = (time.monotonic()-safety_started)*1000
            except InputUnavailable as error:
                self.temporary_input_stop(str(error));r.sleep();continue
            except Exception as error:
                import sys
                self.hold_planner_fault(error, sys.exc_info())
                r.sleep()
                continue
            solver_ms = (solver_finished - solver_started) * 1000.0
            t3 = rospy.Time.now()
            if self.start_connector:
                if self.start_connector.update_progress(self.neupan_planner.ipath.point_index):
                    self.camera_guard_signature = None
                self.publish_connection_status()

            # 计算各个阶段的耗时
            dt_tf = (t1 - t0).to_sec() * 1000
            dt_infer = (t3 - t2).to_sec() * 1000
            
            if hasattr(self, 'last_plan_time_debug'):
                dt_loop = (t0 - self.last_plan_time_debug).to_sec()
                if dt_loop > 0 and self.verbose_runtime_log:
                    rospy.loginfo_throttle(1.0, f"Hz: {1.0/dt_loop:.1f} | TF: {dt_tf:.1f}ms | Infer: {dt_infer:.1f}ms")
            
            self.last_plan_time_debug = t0

            self.stop = info["stop"]
            if self.stop:self.stop_registry.set('planner_obstacle',time.monotonic(),'new valid collision-free solution')
            else:self.stop_registry.clear('planner_obstacle')
            self.arrive = info["arrive"]

            ref_state = None
            if info.get("ref_state_list") is not None and len(info["ref_state_list"]) > 0:
                ref_state = info["ref_state_list"][0]
            if ref_state is not None and self.verbose_runtime_log:
                robot_yaw = self.robot_state[2, 0]
                ref_yaw = ref_state[2, 0]
                yaw_error = atan2(sin(ref_yaw - robot_yaw), cos(ref_yaw - robot_yaw))
                rospy.loginfo_throttle(
                    1.0,
                    "neupan yaw debug: robot=%.1fdeg ref=%.1fdeg err=%.1fdeg cmd_v=%.2f cmd_steer=%.1fdeg",
                    degrees(robot_yaw),
                    degrees(ref_yaw),
                    degrees(yaw_error),
                    action[0, 0],
                    degrees(action[1, 0]),
                )

            self.diagnostic_stage = "publish_plan_and_command"
            reference_visual = None
            if self.camera_guard is not None and info.get('ref_state_list'):
                original=self.camera_guard.original
                if original is not None:
                    reference_visual=reference_offsets(np.hstack(info['ref_state_list']),
                        [[p.pose.position.x,p.pose.position.y] for p in original.poses])
                    status=getattr(self,'camera_guard_selected_state',{})
                    reference_visual['camera_channel_accepted']=status.get('camera_channel_accepted',False)
                    reference_visual['producer_active']=status.get('producer_active',False)
                    reference_visual['guidance_source']=status.get('guidance_source','osm')
            # publish the path and velocity
            self.plan_pub.publish(self.generate_path_msg(info["opt_state_list"]))
            self.ref_state_pub.publish(self.generate_path_msg(info["ref_state_list"]))
            command = self.generate_twist_msg(action)
            try:
                # No expensive diagnostic or path construction may follow this
                # gate before publishing. Never renew checked_scan using a new scan.
                self.diagnostic_stage = 'final_command_freshness'
                self.publish_input_ages = None
                if abs(command.linear.x)>1.e-9 or abs(command.angular.z)>1.e-9:
                    self.check_camera_execution(info,command)
                # All geometric/select work is finished. Lock waits and short
                # version checks precede the FINAL exact-input age check.
                publication_lock=self.camera_guard.lock if self.camera_guard is not None else threading.RLock()
                with publication_lock:
                    if abs(command.linear.x)>1.e-9 or abs(command.angular.z)>1.e-9:
                        if self.camera_guard is not None:
                            if (self.camera_guard.authorization_generation!=self.camera_publication_generation
                                    or self.camera_guard.safety_generation!=self.camera_publication_safety_generation):
                                raise AuthorizationChanged('camera authorization changed after swept check')
                            if (self.camera_publication_source=='camera' and
                                    not 0<=time.monotonic()-self.camera_guard.received<=self.camera_guard.timeout):
                                raise AuthorizationChanged('camera heartbeat expired at publication')
                        self.publish_input_ages=checked_input_ages(checked_scan,rospy.Time.now().to_nsec(),
                            time.monotonic(),self.scan_timeout,self.diagnostic_state_stamp_ns)
                        contract=getattr(self,'camera_publication_contract',None)
                        if contract and rospy.Time.now().to_nsec()>contract['expires_ns']:
                            raise AuthorizationChanged('camera lease expired at publication')
                        if self.camera_guard is not None and not self.camera_guard.commit_recovery(
                                self.camera_publication_generation,self.camera_publication_safety_generation,rospy.Time.now().to_nsec()):
                            raise AuthorizationChanged('camera recovery proof expired or changed before publication')
                        if self.camera_guard is not None:self.stop_registry.clear('camera')
                    self.vel_pub.publish(command)
            except AuthorizationChanged as error:
                self.vel_pub.publish(Twist())
                self.stop_registry.set('authorization',time.monotonic(),'adopt latest authority and solve again; no sensor recovery delay')
                self.runtime_pub.publish(json.dumps(dict(stamp=rospy.Time.now().to_sec(),linear_x=0.,
                    command_source='authorization_retry',reason=str(error),stop_reasons=self.stop_registry.snapshot())))
                rospy.logwarn_throttle(2.,yellow_terminal_text('Camera authority updated; discard old command and replan: %s'),str(error))
                r.sleep();continue
            except InputUnavailable as error:
                self.temporary_input_stop(str(error));r.sleep();continue
            except Exception as error:
                import sys
                self.hold_planner_fault(error,sys.exc_info())
                r.sleep()
                continue
            if info.get('speed_retimed'):
                self.neupan_planner.cur_vel_array=np.asarray(info['vel_tensor']).copy()
            command_time = time.monotonic()
            self.runtime_pub.publish(json.dumps({
                "stamp": rospy.Time.now().to_sec(),
                "command_source":"neupan_retimed_camera" if info.get('speed_retimed') else "neupan",
                "solver_ms": solver_ms,
                "safety_check_ms": getattr(self,'safety_check_ms',None),
                "publish_input_ages": self.publish_input_ages,
                "camera_checked_authorization_id":getattr(self,'camera_checked_authorization_id',None),
                "camera_checked_authorization_generation":getattr(self,'camera_publication_generation',None),
                "camera_checked_execution_contract": {key:self.camera_publication_contract.get(key) for key in
                    ('version','path_digest','route_stamp_ns','expires_ns','speed_cap_mps','end_odom','coordinate_frame')}
                    if getattr(self,'camera_publication_contract',None) else None,
                "stop_reasons": self.stop_registry.snapshot(),
                "camera_execution_contract": {key:self.camera_execution_contract.get(key) for key in
                    ('version','path_digest','route_stamp_ns','expires_ns','speed_cap_mps','end_odom','coordinate_frame')}
                    if getattr(self,'camera_execution_contract',None) else None,
                "speed_retimed":bool(info.get('speed_retimed',False)),
                "optimizer_action":info.get('optimizer_action'),
                "total_processing_ms": (command_time-frame_started_mono)*1000,
                "tf_lookup_ms": dt_tf,
                "command_interval_ms": None if self.last_command_monotonic is None
                else (command_time - self.last_command_monotonic) * 1000.0,
                "linear_x": float(command.linear.x),
                "steering_deg": degrees(float(command.angular.z)),
                "planner_stop": bool(self.stop),
                "planner_arrive": bool(self.arrive),
                "tracking_guard": self.outdoor_tracking_status,
                "trajectory_safety": getattr(self, 'trajectory_safety', None),
                "reference_visual": reference_visual,
                "path_point_index": int(self.neupan_planner.ipath.point_index),
                "path_interval_m": float(self.neupan_planner.ipath.interval),
                "robot_yaw_deg": degrees(float(self.robot_state[2, 0])),
                "reference_yaw_deg": None if ref_state is None else degrees(float(ref_state[2, 0])),
                "reference_forward_m": None if ref_state is None else float(
                    (ref_state[:2, 0]-self.robot_state[:2, 0]) @
                    np.array([cos(self.robot_state[2, 0]), sin(self.robot_state[2, 0])])),
            }, separators=(",", ":")))
            self.last_command_monotonic = command_time
            self.update_goal_arrival()

            # 计算并打印距离目的地的距离
            if (
                self.verbose_runtime_log
                and self.neupan_planner.waypoints is not None
                and len(self.neupan_planner.waypoints) > 0
            ):
                target = self.neupan_planner.waypoints[-1]
                dist = np.linalg.norm(self.robot_state[:2] - target[:2])
                rospy.loginfo_throttle(1.0, f"Distance to destination: {dist:.2f} m")

            self.diagnostic_stage = "publish_markers"
            self.point_markers_pub_dune.publish(self.generate_dune_points_markers_msg())
            self.point_markers_pub_nrmp.publish(self.generate_nrmp_points_markers_msg())
            self.robot_marker_pub.publish(self.generate_robot_marker_msg())

            if info["stop"]:
                rospy.logwarn_throttle(
                    0.5,
                    "neupan stop with the min distance "
                    + str(self.neupan_planner.min_distance.detach().item())
                    + " threshold "
                    + str(self.neupan_planner.collision_threshold),
                )

            r.sleep()

    def hold_planner_fault(self, error, exc_info):
        """Non-None fault identity is independent of error message and stop flag."""
        if getattr(self,'planner_fault',None) is None:
            self.planner_fault = {'type':type(error).__name__,'message':str(error),
                                  'stage':getattr(self,'diagnostic_stage','unknown')}
        self.stop=True
        if hasattr(self,'stop_registry'):
            self.stop_registry.set('planner_fault',time.monotonic(),'inspect evidence and restart planner',True)
        self.vel_pub.publish(Twist())  # Must precede any diagnostic IO.
        try:
            self.runtime_pub.publish(json.dumps({'stamp':rospy.Time.now().to_sec(),
                'linear_x':0.,'steering_deg':0.,'command_source':'planner_fault',
                'fault':self.planner_fault,'stage':self.diagnostic_stage},separators=(',',':')))
            from neupan_diagnostics import report_failure
            report_failure(self,exc_info)
        except Exception as diagnostic_error:
            rospy.logerr('NeuPAN fault diagnostics failed; STOP remains latched: %s', diagnostic_error)

    def temporary_input_stop(self, reason):
        self.vel_pub.publish(Twist())
        self.input_good_since=None
        self.stop_registry.set('input',time.monotonic(),'fresh bound pose/scan stable for 0.3s, then solve and recheck')
        self.stop_registry.reasons['input'].update(detail=reason,stage=getattr(self,'diagnostic_stage','input'))
        self.runtime_pub.publish(json.dumps(dict(stamp=rospy.Time.now().to_sec(),linear_x=0.,
            command_source='temporary_input_stop',reason=reason,stop_reasons=self.stop_registry.snapshot())))
        rospy.logwarn_throttle(2.,yellow_terminal_text('NeuPAN input STOP: %s; waiting for fresh stable inputs'),reason)

    def publish_stop_snapshot(self, source):
        now=time.monotonic();signature=(source,tuple(sorted(self.stop_registry.reasons)))
        if signature!=getattr(self,'last_stop_snapshot_signature',None) or now-getattr(self,'last_stop_snapshot_time',-1.e30)>=.5:
            self.runtime_pub.publish(json.dumps(dict(stamp=rospy.Time.now().to_sec(),linear_x=0.,
                command_source=source,stop_reasons=self.stop_registry.snapshot())))
            self.last_stop_snapshot_signature=signature;self.last_stop_snapshot_time=now

    def check_camera_execution(self, info, command):
        contract=getattr(self,'camera_execution_contract',None)
        self.camera_publication_contract=contract
        self.camera_checked_authorization_id=None
        if self.camera_guard is not None:
            _,_,state=self.camera_guard.select(time.monotonic(),rospy.Time.now().to_nsec())
            current=state.get('execution_contract')
            self.camera_publication_generation=state['authorization_generation']
            self.camera_publication_safety_generation=state['safety_generation']
            self.camera_publication_source=state['source']
            self.camera_checked_authorization_id=state.get('authorization_id')
            solved=getattr(self,'camera_guard_selected_state',{})
            if solved and any(solved.get(key)!=state.get(key) for key in ('matched_route_identity','original_path_digest')):
                raise AuthorizationChanged('original route changed during solve')
            if state['source']=='stop' or state['safety_generation']!=getattr(self,'camera_safety_generation',None):
                raise AuthorizationChanged('camera authority revoked during solve')
            if contract is None:
                if current or state.get('producer_active'):
                    raise AuthorizationChanged('camera activated during OSM solve')
            elif (state['source']!='camera' or not current
                    or current.get('route_stamp_ns')!=contract.get('route_stamp_ns')):
                raise AuthorizationChanged('camera authority replaced during solve')
            else:
                # A changed path is not re-labelled as the solved path: only
                # the actual prediction's compatibility with NEW limits permits
                # continuation. Tighter/conflicting limits cause a new solve.
                contract=current
                self.camera_publication_contract=contract
        if not contract:return
        if rospy.Time.now().to_nsec()>contract['expires_ns']:
            raise AuthorizationChanged('camera execution authorization expired')
        stamp=rospy.Time(self.diagnostic_state_stamp_ns//1000000000,self.diagnostic_state_stamp_ns%1000000000)
        try:trans,rot=self.listener.lookupTransform('map','odom',stamp)
        except (tf.LookupException,tf.ConnectivityException,tf.ExtrapolationException) as error:
            raise InputUnavailable('camera execution epoch TF missing: '+str(error))
        angle=self.quat_to_yaw_list(rot);c,s=cos(angle),sin(angle)
        poses=np.hstack(info['opt_state_list']).T.copy()
        poses[:,:2]=(poses[:,:2]-np.asarray(trans[:2]))@np.array([[c,-s],[s,c]])
        poses[:,2]-=angle
        try:
            check_execution_contract(contract,poses,rospy.Time.now().to_nsec(),command.linear.x,
                                     self.neupan_planner.robot.vertices)
        except ValueError as error:
            if self.camera_guard is not None:raise AuthorizationChanged('latest camera limits reject prediction: '+str(error))
            if 'expired' in str(error) or 'endpoint' in str(error):raise InputUnavailable(str(error))
            raise

    def parse_adjust_profile(self, profile_param, fallback):
        profile = np.fromstring(str(profile_param).replace(",", " "), dtype=np.float32, sep=" ")
        if profile.size != 5:
            rospy.logwarn(
                "Invalid adaptive adjust profile '%s', expected 5 values: q_s p_u eta d_max d_min. Falling back to %s",
                profile_param,
                fallback,
            )
            profile = np.array(fallback, dtype=np.float32)
        return profile

    def mark_scan_valid(self, now, mode):
        if self.scan_valid_since is None:
            self.scan_valid_since = now
        self.last_scan_valid_time = now
        self.scan_fault_reason = "ok"
        self.scan_mode = mode

    def mark_scan_fault(self, reason):
        self.scan_valid_since = None
        self.scan_fault_reason = reason
        self.scan_mode = "FAULT"

    def scan_is_healthy(self, now):
        if not self.stop_on_scan_fault:
            return True, "guard_disabled"
        if self.last_scan_received_time is None:
            return False, "scan_missing"
        scan_age = (now - self.last_scan_received_time).to_sec()
        if scan_age > self.scan_timeout:
            return False, "scan_stale_{:.2f}s".format(scan_age)
        if self.last_scan_valid_time is None:
            return False, self.scan_fault_reason
        valid_age = (now - self.last_scan_valid_time).to_sec()
        if valid_age > self.scan_transform_timeout:
            return False, "scan_transform_stale_{:.2f}s".format(valid_age)
        if self.scan_valid_since is None:
            return False, self.scan_fault_reason
        stable_sec = (now - self.scan_valid_since).to_sec()
        if stable_sec < self.scan_recovery_stable_sec:
            return False, "scan_recovering_{:.2f}s".format(stable_sec)
        return True, "ok"

    def publish_scan_health(self, now):
        if (now - self.last_scan_status_publish).to_sec() < 0.5:
            return
        self.last_scan_status_publish = now
        healthy, reason = self.scan_is_healthy(now)
        front_min_distance = self.compute_min_obstacle_distance()
        planner_min_distance = None
        if getattr(self.neupan_planner, "min_distance", None) is not None:
            try:
                planner_min_distance = float(
                    self.neupan_planner.min_distance.detach().item()
                )
            except (AttributeError, RuntimeError, TypeError, ValueError):
                planner_min_distance = None
        payload = {
            "stamp": now.to_sec(),
            "healthy": healthy,
            "reason": reason,
            "mode": self.scan_mode,
            "scan_age_sec": None
            if self.last_scan_received_time is None
            else max(0.0, (now - self.last_scan_received_time).to_sec()),
            "valid_age_sec": None
            if self.last_scan_valid_time is None
            else max(0.0, (now - self.last_scan_valid_time).to_sec()),
            "recovery_stable_sec": None
            if self.scan_valid_since is None
            else max(0.0, (now - self.scan_valid_since).to_sec()),
            "obstacle_points": 0
            if self.obstacle_points is None
            else int(self.obstacle_points.shape[1]),
            "front_min_obstacle_distance_m": front_min_distance,
            "planner_min_distance_m": planner_min_distance,
            "planner_stop": bool(self.stop),
            "planner_arrive": bool(self.arrive),
            "adjust_profile": self.current_adjust_profile_name,
            "adjust": self.current_adjust_values,
        }
        self.scan_health_pub.publish(
            String(data=json.dumps(payload, separators=(",", ":")))
        )

    def compute_min_obstacle_distance(self):
        if self.robot_state is None or self.obstacle_points is None or self.obstacle_points.size == 0:
            return None

        robot_xy = self.robot_state[:2, 0:1]
        robot_yaw = float(self.robot_state[2, 0])
        rel_points = self.obstacle_points - robot_xy
        distances = np.linalg.norm(rel_points, axis=0)
        if distances.size == 0:
            return None

        bearings = np.arctan2(rel_points[1, :], rel_points[0, :]) - robot_yaw
        bearings = np.arctan2(np.sin(bearings), np.cos(bearings))
        front_half_angle_rad = radians(self.adaptive_front_half_angle_deg)
        front_mask = np.abs(bearings) <= front_half_angle_rad

        if not np.any(front_mask):
            return None

        front_distances = distances[front_mask]
        if front_distances.size == 0:
            return None
        return float(np.min(front_distances))

    def update_adaptive_adjust_parameters(self):
        if not self.enable_adaptive_adjust:
            return

        min_distance = self.compute_min_obstacle_distance()

        if min_distance is None:
            alpha = 0.0
            profile_name = "clear_no_scan"
        elif min_distance <= self.adaptive_near_distance:
            alpha = 1.0
            profile_name = "avoid"
        elif min_distance >= self.adaptive_far_distance:
            alpha = 0.0
            profile_name = "clear"
        else:
            alpha = (
                self.adaptive_far_distance - min_distance
            ) / max(self.adaptive_far_distance - self.adaptive_near_distance, 1e-6)
            profile_name = "blend"

        adjust_values = (1.0 - alpha) * self.clear_adjust_profile + alpha * self.avoid_adjust_profile
        q_s, p_u, eta, d_max, d_min = [float(v) for v in adjust_values]
        d_max = max(d_max, d_min + 1e-3)

        self.neupan_planner.update_adjust_parameters(
            q_s=q_s,
            p_u=p_u,
            eta=eta,
            d_max=d_max,
            d_min=d_min,
        )

        self.current_adjust_profile_name = profile_name
        self.current_adjust_values = {
            "q_s": q_s,
            "p_u": p_u,
            "eta": eta,
            "d_max": d_max,
            "d_min": d_min,
            "alpha": float(alpha),
            "min_distance": min_distance,
        }

        if self.verbose_runtime_log:
            rospy.loginfo_throttle(
                1.0,
                "neupan adaptive adjust: mode=%s alpha=%.2f min_obs=%s q_s=%.2f p_u=%.2f eta=%.2f d_max=%.2f d_min=%.2f",
                profile_name,
                alpha,
                "none" if min_distance is None else f"{min_distance:.2f}m",
                q_s,
                p_u,
                eta,
                d_max,
                d_min,
            )

    # scan callback
    def refresh_connection_scan(self):
        """Retry capture-time TF off the subscriber thread; never restamp scans.

        A scan can arrive before its TF. Keep the last fresh, aligned snapshot
        while retrying a bounded queue rather than erasing it on one TF miss.
        This only feeds the outdoor admission helper, not normal scan handling.
        """
        if self.start_connector is None:
            return
        now, ros_now = time.monotonic(), rospy.Time.now().to_sec()
        with self.connection_scan_lock:
            queued = list(self.connection_scan_queue)
        pending = [(msg, received) for msg, received in queued if 0 <= now-received <= .5]
        state = 'waiting_for_capture_tf' if pending else 'no_fresh_scan'
        for scan_msg, received in reversed(pending):
            try:
                position, rotation = self.listener.lookupTransform(
                    self.map_frame, scan_msg.header.frame_id, scan_msg.header.stamp)
            except tf.Exception:
                continue
            evidence = ScanEnvelope(scan_msg.ranges, scan_msg.angle_min,
                scan_msg.angle_increment, scan_msg.range_min, scan_msg.range_max,
                (position[0], position[1], self.quat_to_yaw_list(rotation)),
                scan_msg.header.stamp.to_sec(), received)
            if not evidence.fresh(now, ros_now):
                continue
            self.start_connector.scan = evidence
            with self.connection_scan_lock:
                while self.connection_scan_queue and self.connection_scan_queue[0][1] <= received:
                    self.connection_scan_queue.popleft()
            state = 'ready'
            break
        evidence = self.start_connector.scan
        if state != 'ready' and evidence is not None and evidence.fresh(now, ros_now):
            state = 'using_previous_fresh_capture' if pending else 'ready'
        self.start_connector.scan_transform_state = state

    def scan_callback(self, scan_msg):

        now = rospy.Time.now()
        self.last_scan_received_time = now
        if self.start_connector:
            with self.connection_scan_lock:
                self.connection_scan_queue.append((scan_msg, time.monotonic()))

        if self.robot_state is None:
            self.mark_scan_fault("robot_pose_unavailable_for_scan")
            return None

        ranges = np.array(scan_msg.ranges)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
        relevant_raw = np.isfinite(ranges)
        relevant_raw &= ranges >= max(float(scan_msg.range_min), self.scan_range[0])
        relevant_raw &= ranges <= min(float(scan_msg.range_max), self.scan_range[1])
        relevant_raw &= angles > self.scan_angle_range[0]
        relevant_raw &= angles < self.scan_angle_range[1]

        points = []
        # x, y, z, yaw, pitch, roll = self.lidar_offset

        if self.flip_angle:
            angles = np.flip(angles)

        for i in range(len(ranges)):
            distance = ranges[i]
            angle = angles[i]

            if (
                distance >= self.scan_range[0]
                and distance <= self.scan_range[1]
                and angle > self.scan_angle_range[0]
                and angle < self.scan_angle_range[1]
            ):
                point = np.array([[distance * cos(angle)], [distance * sin(angle)]])
                points.append(point)

        if len(points) == 0:
            self.obstacle_points = None
            if not np.any(relevant_raw):
                self.safety_scan = (np.empty((2, 0)), scan_msg.header.stamp.to_nsec(), time.monotonic())
                self.filtered_empty_since = None
                self.mark_scan_valid(now, "CLEAR_NO_RETURNS")
                return None

            if self.filtered_empty_since is None:
                self.filtered_empty_since = now
            filtered_sec = (now - self.filtered_empty_since).to_sec()
            if filtered_sec >= self.scan_filtered_empty_timeout:
                self.mark_scan_fault("all_finite_scan_points_filtered")
            else:
                self.mark_scan_valid(now, "FILTERED_EMPTY_GRACE")
            return None

        self.filtered_empty_since = None
        point_array = np.hstack(points)

        try:
            (trans, rot) = self.listener.lookupTransform(
                self.map_frame, self.lidar_frame, scan_msg.header.stamp
            )

            yaw = self.quat_to_yaw_list(rot)
            x, y = trans[0], trans[1]

            trans_matrix, rot_matrix = get_transform(np.c_[x, y, yaw].reshape(3, 1))
            self.obstacle_points = rot_matrix @ point_array + trans_matrix

            if (
                self.scan_self_filter
                and self.scan_self_filter_length > 0.0
                and self.scan_self_filter_width > 0.0
                and self.robot_state is not None
            ):
                body_trans, body_rot = self.listener.lookupTransform(
                    self.map_frame, self.base_frame, scan_msg.header.stamp)
                robot_x, robot_y, robot_yaw = body_trans[0], body_trans[1], self.quat_to_yaw_list(body_rot)
                dx = self.obstacle_points[0, :] - robot_x
                dy = self.obstacle_points[1, :] - robot_y
                cos_yaw = cos(robot_yaw)
                sin_yaw = sin(robot_yaw)
                local_x = cos_yaw * dx + sin_yaw * dy
                local_y = -sin_yaw * dx + cos_yaw * dy
                half_length = (
                    self.scan_self_filter_length / 2.0
                    + self.scan_self_filter_margin
                )
                half_width = (
                    self.scan_self_filter_width / 2.0
                    + self.scan_self_filter_margin
                )
                outside_body = (np.abs(local_x) > half_length) | (
                    np.abs(local_y) > half_width
                )
                self.obstacle_points = self.obstacle_points[:, outside_body]

            self.safety_scan = (self.obstacle_points.copy(), scan_msg.header.stamp.to_nsec(), time.monotonic())
            self.obstacle_points = self.obstacle_points[:, ::self.scan_downsample]
            self.mark_scan_valid(now, "TRACKING")
            rospy.loginfo_once("Scan obstacle points Received")

            return self.obstacle_points

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            self.mark_scan_fault("scan_tf_unavailable")
            rospy.loginfo_throttle(
                1,
                red_terminal_text(
                    "waiting for tf for the transform from {} to {}".format(
                        self.lidar_frame, self.map_frame
                    )
                ),
            )
            return

    def update_goal_arrival(self):
        if self.arrive:
            if not self.goal_arrived_state:
                rospy.loginfo(yellow_terminal_text(
                    "Goal reached; waiting for a new goal/path"
                ))
            self.goal_arrived_state = True
            self.waiting_for_new_goal = True

    def goal_arrived_timer_callback(self, _event):
        self.goal_arrived_pub.publish(String(data="1" if self.goal_arrived_state else "0"))

    def goal_request_callback(self, goal):
        with self.route_update_lock:
            self.pending_goal_request_stamp = goal.header.stamp.to_nsec()

    def apply_pending_goal_request(self):
        with self.route_update_lock:
            stamp = self.pending_goal_request_stamp
            self.pending_goal_request_stamp = None
        if stamp is None:
            return
        # A* path and goal arrive on separate ROS connections. Do not pause a
        # route already accepted for this request if its callback arrived first.
        if stamp and self.arrival_path_key is not None and self.arrival_path_key[1] >= stamp:
            return
        self.minimum_path_stamp = stamp
        if self.start_connector:
            self.start_connector.reset()
            self.publish_connection_status()
        self.stop = False
        self.arrive = False
        self.goal_arrived_state = False
        self.waiting_for_new_goal = True
        self.vel_pub.publish(Twist())
        self.goal_arrived_timer_callback(None)
        rospy.loginfo(yellow_terminal_text(
            "New goal received; arrival cleared, waiting for planned path"
        ))

    def queue_route_update(self, kind, data, path_key=None):
        # Subscriber callbacks must not reset a planner while its solver runs.
        with self.route_update_lock:
            if kind == "path" and path_key[1] < self.minimum_path_stamp:
                return
            if kind == "path" and self.waiting_for_new_goal and path_key == self.arrival_path_key:
                return
            self.pending_route_update = (kind, data, path_key)
        rospy.loginfo("Queued new %s for NeuPAN", kind)

    def apply_pending_route(self):
        # Called only by the control loop, after obtaining the current pose.
        with self.route_update_lock:
            update = self.pending_route_update
            self.pending_route_update = None
        if update is None:
            return
        kind, data, path_key = update
        if kind == "path" and path_key[1] < self.minimum_path_stamp:
            return
        spacing = getattr(self, 'outdoor_reference_spacing', 0.)
        if kind == 'path' and spacing:
            # Before connector admission: its prefix/progress indices must
            # refer to the final sampling, not an independently resampled turn.
            try:
                data = [p.reshape(4, 1) for p in resample_forward_reference(data, spacing)]
            except (ValueError, TypeError) as error:
                self.waiting_for_new_goal = True
                self.vel_pub.publish(Twist())
                rospy.logwarn_throttle(2., 'Outdoor reference rejected; holding STOP: %s', str(error))
                return
        self.diagnostic_route_update = (kind, data, path_key)
        reseed = False
        if self.start_connector and kind == 'path':
            decision, prepared = self.start_connector.prepare(
                data, path_key, self.robot_state[:, 0], time.monotonic(), rospy.Time.now().to_sec())
            self.publish_connection_status()
            if decision == 'keep':
                self.diagnostic_route_update = None
                return
            if decision in ('wait', 'reject'):
                # Keep the old numerical planner state for diagnostics, but do
                # not run it for the rejected/new goal. Never fall back to the
                # unconnected opposite-heading reference.
                self.waiting_for_new_goal = True
                self.vel_pub.publish(Twist())
                if decision == 'wait':
                    with self.route_update_lock:
                        if self.pending_route_update is None:
                            self.pending_route_update = update
                if self.start_connector.phase == 'searching':
                    rospy.logwarn_throttle(2., yellow_terminal_text(
                        'Outdoor Hybrid A*: searching local connection; holding zero speed'))
                else:
                    admission = self.start_connector.admission or {}
                    detail = (admission.get('search') or {}).get('reason',
                        admission.get('result_rejected', self.start_connector.reason))
                    rospy.logwarn_throttle(2., yellow_terminal_text(
                        'Outdoor route held: %s; kind=%s detail=%s scan_rejected=%s; '
                        'check observation/road alignment before resending the goal'),
                        self.start_connector.reason, self.start_connector.connection_kind,
                        detail, admission.get('scan_rejected', {}))
                self.diagnostic_route_update = None
                return
            data = [p.reshape(4, 1) for p in prepared]
            report = (self.start_connector.admission or {}).get('accepted_scan_check', {})
            if self.start_connector.phase == 'connecting' and report.get('unverified_near_margin_points', 0):
                rospy.logwarn_throttle(2., yellow_terminal_text(
                    'Outdoor connection: swept body observed; %d near-field buffer samples remain '
                    'unverified (not treated as free space)'), report['unverified_near_margin_points'])
            reseed = self.start_connector.reseed
            self.start_connector.reseed = False
        progress = None
        if (not reseed and self.camera_guard is not None and kind == 'path' and
                path_key == self.arrival_path_key):
            ipath = self.neupan_planner.ipath
            if ipath.curve_index == 0 and ipath.curve_number == 1:
                old_xy = np.array([[p[0, 0], p[1, 0]] for p in ipath.cur_curve])
                end = min(ipath.point_index+1, len(old_xy))
                progress = float(np.linalg.norm(np.diff(old_xy[:end], axis=0), axis=1).sum())
        if kind == "path":
            self.neupan_planner.set_initial_path(data)
        elif kind == "waypoints":
            self.neupan_planner.update_initial_path_from_waypoints([self.robot_state] + data)
        else:
            self.goal = data
            self.neupan_planner.update_initial_path_from_goal(self.robot_state, data)
        self.neupan_planner.reset()
        if (getattr(self, 'outdoor_tracking_guard', None) is not None and
                path_key != self.arrival_path_key):
            self.outdoor_tracking_guard.reset()
        if reseed:
            xy = np.array([p[:2, 0] for p in data])
            self.neupan_planner.ipath.point_index = int(np.argmin(
                np.linalg.norm(xy-self.robot_state[:2, 0], axis=1)))
        if progress is not None:
            # Camera densification and the original route have different point
            # counts. Preserve travelled distance, not the old numeric index.
            ipath = self.neupan_planner.ipath
            xy = np.array([[p[0, 0], p[1, 0]] for p in ipath.cur_curve])
            distances = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
            ipath.point_index = min(int(np.searchsorted(distances, progress)), len(xy)-1)
        self.arrival_path_key = path_key
        self.stop = False
        self.arrive = False
        self.goal_arrived_state = False
        self.waiting_for_new_goal = False
        self.diagnostic_route_update = None
        self.goal_arrived_timer_callback(None)
        rospy.loginfo(yellow_terminal_text(
            "New {} accepted; arrival cleared, starting next route".format(kind)
        ))

    def publish_connection_status(self):
        status = self.start_connector.status()
        signature = json.dumps(status, sort_keys=True)
        now = time.monotonic()
        if signature != self.last_connection_status or now-self.last_connection_publish >= .5:
            self.connection_status_pub.publish(String(data=signature))
            self.last_connection_status, self.last_connection_publish = signature, now

    def camera_guard_status_cb(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            status = None
        self.camera_guard.status_cb(status, time.monotonic())

    def apply_camera_path_guard(self):
        if self.camera_guard is None:
            return True
        now = time.monotonic()
        path, signature, state = self.camera_guard.select(now,rospy.Time.now().to_nsec())
        self.camera_guard_selected_state = state
        self.camera_execution_contract=state.get('execution_contract')
        self.camera_safety_generation=state.get('safety_generation')
        if hasattr(self,'stop_registry'):
            if state['source']=='stop' or state.get('recovery_pending'):
                self.stop_registry.set('camera',now,'fresh verified original hazard area/path or explicit operator inspection')
            else:self.stop_registry.clear('camera')
        current = (state['source'], state['reason'])
        if current != self.camera_guard_last_state or now-self.camera_guard_last_publish >= .5:
            self.camera_guard_pub.publish(String(data=json.dumps(state)))
            self.camera_guard_last_publish = now
            self.camera_guard_last_state = current
        if state['source'] == 'stop':
            self.camera_guard_signature = None
            with self.route_update_lock:
                self.pending_route_update = None
            rospy.logwarn_throttle(5., yellow_terminal_text(
                'Camera path guard: %s; holding STOP'), state['reason'])
            return False
        if signature != self.camera_guard_signature:
            if not self.path_callback(copy.deepcopy(path)):
                return False
            self.camera_guard_signature = signature
            if state['source'] == 'osm':
                rospy.logwarn('Camera path guard: %s; using original OSM route', state['reason'])
        return True

    def path_callback(self, path):
        path_frame = path.header.frame_id if path.header.frame_id else "map"
        new_poses = []
        try:
            self.listener.waitForTransform(self.map_frame, path_frame, rospy.Time(0), rospy.Duration(0.5))
            for p in path.poses:
                p.header.frame_id = path_frame
                p.header.stamp = rospy.Time(0)
                new_poses.append(self.listener.transformPose(self.map_frame, p))
            path.poses = new_poses
        except Exception as e:
            rospy.logwarn("TF Error transforming path: {}".format(e))
            return False

        if not path.poses or len(path.poses) == 0:
            rospy.logwarn("Received empty path from initial path topic, ignoring...")
            return False

        if len(path.poses) < 2:
            rospy.logwarn(
                "Received degenerate initial path with %d pose(s); ignoring it to avoid an invalid NeuPAN reference path.",
                len(path.poses),
            )
            return False

        initial_point_list = []

        if self.verbose_runtime_log:
            rospy.loginfo("Received path with %d poses", len(path.poses))
        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once("Using the points gradient as the initial path direction")

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = initial_point_list[-1][2, 0]
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            initial_point_list.append(points)

        if self.neupan_planner.initial_path is None or self.refresh_initial_path or self.waiting_for_new_goal:
            # Refinement preserves the source stamp. Its changing points/seq
            # must not turn a completed route into another navigation task.
            path_stamp = path.header.stamp.to_nsec()
            path_key = (path_frame, path_stamp)
            if path_stamp == 0:
                path_key += (round(float(initial_point_list[-1][0, 0]), 4),
                             round(float(initial_point_list[-1][1, 0]), 4))
            self.queue_route_update("path", initial_point_list, path_key)
        return True
    
    def waypoints_callback(self, path):
        
        '''
        Utilize multiple waypoints (goals) to set the initial path
        '''

        waypoints_list = []

        for i in range(len(path.poses)):
            p = path.poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once("Using the points gradient as the initial path direction")

                if i + 1 < len(path.poses):
                    p2 = path.poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    theta = atan2(y2 - y, x2 - x)
                else:
                    theta = waypoints_list[-1][2, 0] if waypoints_list else 0.0
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            waypoints_list.append(points)

        if not waypoints_list:
            rospy.logwarn("Ignoring empty waypoint goal")
            return
        if self.neupan_planner.initial_path is None or self.refresh_initial_path or self.waiting_for_new_goal:
            self.queue_route_update("waypoints", waypoints_list)

    def goal_callback(self, goal):

        x = goal.pose.position.x
        y = goal.pose.position.y
        theta = self.quat_to_yaw(goal.pose.orientation)

        new_goal = np.array([[x], [y], [theta]])

        if self.verbose_runtime_log:
            rospy.loginfo("Set NeuPAN goal: x=%.2f y=%.2f yaw=%.1fdeg", x, y, degrees(theta))
        self.queue_route_update("goal", new_goal)


    def quat_to_yaw_list(self, quater):

        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return yaw

    # generate ros message
    def generate_path_msg(self, path_list):

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = rospy.Time.now()
        path.header.seq = 0

        for index, point in enumerate(path_list):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.seq = index

            ps.pose.position.x = point[0, 0]
            ps.pose.position.y = point[1, 0]
            ps.pose.orientation = self.yaw_to_quat(point[2, 0])

            path.poses.append(ps)

        return path

    def generate_twist_msg(self, vel):

        if vel is None:
            return Twist()

        speed = vel[0, 0]
        steer = vel[1, 0]

        if self.stop or self.arrive:
            # print('stop flag true')
            return Twist()

        else:
            action = Twist()

            action.linear.x = speed
            action.angular.z = steer

            return action

    def generate_dune_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.neupan_planner.dune_points is None:
            return
        else:
            points = self.neupan_planner.dune_points

            for index, point in enumerate(points.T):

                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.seq = 0
                marker.header.stamp = rospy.get_rostime()

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0

                marker.color.r = 160 / 255
                marker.color.g = 32 / 255
                marker.color.b = 240 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_nrmp_points_markers_msg(self):

        marker_array = MarkerArray()

        if self.neupan_planner.nrmp_points is None:
            return
        else:
            points = self.neupan_planner.nrmp_points

            for index, point in enumerate(points.T):

                marker = Marker()
                marker.header.frame_id = self.map_frame
                marker.header.seq = 0
                marker.header.stamp = rospy.get_rostime()

                marker.scale.x = self.marker_size
                marker.scale.y = self.marker_size
                marker.scale.z = self.marker_size
                marker.color.a = 1.0

                marker.color.r = 255 / 255
                marker.color.g = 128 / 255
                marker.color.b = 0 / 255

                marker.id = index
                marker.type = 1
                marker.pose.position.x = point[0]
                marker.pose.position.y = point[1]
                marker.pose.position.z = 0.3
                marker.pose.orientation = Quaternion()

                marker_array.markers.append(marker)

            return marker_array

    def generate_robot_marker_msg(self):

        marker = Marker()

        marker.header.frame_id = self.map_frame
        marker.header.seq = 0
        marker.header.stamp = rospy.get_rostime()

        marker.color.a = 1.0
        marker.color.r = 0 / 255
        marker.color.g = 255 / 255
        marker.color.b = 0 / 255

        marker.id = 0

        if self.neupan_planner.robot.shape == "rectangle":
            length = self.neupan_planner.robot.length
            width = self.neupan_planner.robot.width
            wheelbase = self.neupan_planner.robot.wheelbase

            marker.scale.x = length
            marker.scale.y = width
            marker.scale.z = self.marker_z

            marker.type = 1

            x = self.robot_state[0, 0]
            y = self.robot_state[1, 0]
            theta = self.robot_state[2, 0]

            if self.neupan_planner.robot.kinematics == "acker":
                diff_len = (length - wheelbase) / 2
                marker_x = x + diff_len * cos(theta)
                marker_y = y + diff_len * sin(theta)
            else:
                marker_x = x
                marker_y = y

            marker.pose.position.x = marker_x
            marker.pose.position.y = marker_y
            marker.pose.position.z = 0
            marker.pose.orientation = self.yaw_to_quat(self.robot_state[2, 0])

        return marker

    @staticmethod
    def yaw_to_quat(yaw):

        quater = Quaternion()

        quater.x = 0
        quater.y = 0
        quater.z = sin(yaw / 2)
        quater.w = cos(yaw / 2)

        return quater

    @staticmethod
    def quat_to_yaw(quater):

        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w

        raw = atan2(2 * (w * z + x * y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw
