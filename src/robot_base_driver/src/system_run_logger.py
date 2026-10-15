#!/usr/bin/env python3

import json
import glob
import math
import os
import time
import threading
from datetime import datetime, timedelta, timezone

import rospy
from geometry_msgs.msg import PoseStamped, QuaternionStamped, Twist
from nav_msgs.msg import Odometry, Path
from nmea_msgs.msg import Gpgga
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from robot_base_driver.msg import MotorStatus


class SystemRunLogger:
    """Record navigation diagnostics on one shared time axis and write one file."""

    def __init__(self):
        rospy.init_node("system_run_logger")
        self.log_lock = threading.RLock()
        self.previous_auto_receive = None

        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.gps_goal_topic = rospy.get_param("~gps_goal_topic", "/gps_goal_ll")
        self.map_goal_topic = rospy.get_param("~map_goal_topic", "/move_base_simple/goal")
        self.global_path_ll_topic = rospy.get_param("~global_path_ll_topic", "/osm_global_path_ll")
        self.goal_arrived_topic = rospy.get_param("~goal_arrived_topic", "/goal_arrived")
        self.gga_topic = rospy.get_param("~gga_topic", "/gnss/gpgga")
        self.heading_topic = rospy.get_param("~heading_topic", "/heading")
        self.solution_topic = rospy.get_param("~solution_topic", "/gnss/solution_status")
        self.gps_odom_topic = rospy.get_param("~gps_odom_topic", "/odometry/gps")
        self.global_odom_topic = rospy.get_param("~global_odom_topic", "/odometry/global")
        self.local_odom_topic = rospy.get_param("~local_odom_topic", "/odometry/local")
        self.command_topic = rospy.get_param("~command_topic", "/neupan_cmd_vel")
        self.auto_command_topic = rospy.get_param("~auto_command_topic", "/cmd_vel/auto")
        self.control_mode_topic = rospy.get_param("~control_mode_topic", "/vehicle/control_mode/status")
        self.neupan_runtime_topic = rospy.get_param("~neupan_runtime_topic", "/neupan/runtime")
        self.global_path_topic = rospy.get_param("~global_path_topic", "/osm_global_path")
        self.gps_guard_topic = rospy.get_param("~gps_guard_topic", "/gps_guard/status")
        self.lio_guard_topic = rospy.get_param("~lio_guard_topic", "/lio_guard/status")
        self.tracker_status_topic = rospy.get_param(
            "~tracker_status_topic", "/global_path_tracker/status"
        )
        self.scan_health_topic = rospy.get_param(
            "~scan_health_topic", "/neupan/scan_health"
        )
        self.safety_topic = rospy.get_param("~safety_topic", "/localization_safe")
        self.motor_status_topic = rospy.get_param("~motor_status_topic", "/motor/status")

        self.label = rospy.get_param("~label", "system_run_full")
        self.log_dir = rospy.get_param(
            "~log_dir", "/home/nuc/lxw/neupan_ws/tracker_logs"
        )
        self.sample_rate = max(1.0, float(rospy.get_param("~sample_rate", 10.0)))
        self.flush_interval = max(0.1, float(rospy.get_param("~flush_interval", 1.0)))
        # Zero (the default) disables the file-size cap; positive values opt in.
        self.max_file_size_bytes = max(
            0, int(float(rospy.get_param("~max_file_size_mb", 0.0)) * 1024 * 1024)
        )
        self.print_to_screen = bool(rospy.get_param("~print_to_screen", False))
        self.timezone_offset_hours = float(rospy.get_param("~timezone_offset_hours", 8.0))
        self.local_timezone = timezone(timedelta(hours=self.timezone_offset_hours))

        self.latest = {
            "gps_fix": None,
            "gps_goal": None,
            "map_goal": None,
            "global_path_ll": None,
            "goal_arrived": None,
            "gga": None,
            "heading": None,
            "gnss_solution": None,
            "gps_odom": None,
            "global_odom": None,
            "local_odom": None,
            "command": None,
            "auto_command": None,
            "control_mode": None,
            "neupan_runtime": None,
            "lio_guarded": None,
            "motor_alarm": None,
            "local_fault_event": None,
            "neupan_resources": None,
            "neupan_config": None,
            "global_path": None,
            "gps_guard": None,
            "lio_guard": None,
            "tracker_status": None,
            "scan_health": None,
            "localization_safe": None,
            "motor": None,
            "camera_right": None,
            "neupan_failure": None,
            "neupan_reference_path": None,
            "camera_path_guard": None,
            "route_connection": None,
        }
        self.received_at = {key: None for key in self.latest}
        self.previous_heading_deg = None
        self.previous_reference_path = None
        self.seen_failure_snapshots = set()
        self.last_failure_scan = float('-inf')
        self.failure_snapshot_dir = rospy.get_param('~failure_snapshot_dir', self.log_dir)
        self.sample_count = 0
        self.log_file = None
        self.log_path = None
        self.log_size_bytes = 0
        self.log_capped = False
        self.last_flush_time = time.monotonic()

        self.open_log_file()

        rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_cb, queue_size=20)
        rospy.Subscriber(self.gps_goal_topic, NavSatFix, self.gps_goal_cb, queue_size=20)
        rospy.Subscriber(self.map_goal_topic, PoseStamped, self.map_goal_cb, queue_size=20)
        rospy.Subscriber(self.global_path_ll_topic, String, self.global_path_ll_cb, queue_size=2)
        rospy.Subscriber(self.goal_arrived_topic, String, self.goal_arrived_cb, queue_size=20)
        rospy.Subscriber(self.gga_topic, Gpgga, self.gga_cb, queue_size=20)
        rospy.Subscriber(self.heading_topic, QuaternionStamped, self.heading_cb, queue_size=30)
        rospy.Subscriber(self.solution_topic, String, self.solution_cb, queue_size=10)
        rospy.Subscriber(self.gps_odom_topic, Odometry, self.gps_odom_cb, queue_size=30)
        rospy.Subscriber(self.global_odom_topic, Odometry, self.global_odom_cb, queue_size=30)
        rospy.Subscriber(self.local_odom_topic, Odometry, self.local_odom_cb, queue_size=50)
        rospy.Subscriber(self.command_topic, Twist, self.command_cb, queue_size=100)
        rospy.Subscriber(self.auto_command_topic, Twist, self.auto_command_cb, queue_size=100)
        rospy.Subscriber(self.control_mode_topic, String, self.control_mode_cb, queue_size=50)
        rospy.Subscriber(self.neupan_runtime_topic, String, self.neupan_runtime_cb, queue_size=50)
        rospy.Subscriber("/odometry/lio_guarded", Odometry,
                         lambda msg: self.update("lio_guarded", self.odom_dict(msg)), queue_size=50)
        rospy.Subscriber("/motor/alarm", Bool,
                         lambda msg: self.update("motor_alarm", bool(msg.data)), queue_size=20)
        rospy.Subscriber("/gps_guard/local_fault_event", String,
                         self.local_fault_event_cb, queue_size=100)
        rospy.Subscriber("/neupan/resources", String,
                         lambda msg: self.update("neupan_resources", self.parse_status(msg)), queue_size=10)
        rospy.Subscriber("/neupan/config_snapshot", String,
                         self.neupan_config_cb, queue_size=1)
        rospy.Subscriber("/neupan/failure", String, self.neupan_failure_cb, queue_size=10)
        rospy.Subscriber('/neupan/camera_path_guard', String,
                         lambda msg: self.update('camera_path_guard', self.parse_status(msg)), queue_size=10)
        rospy.Subscriber('/neupan/route_connection', String,
                         self.route_connection_cb, queue_size=10)
        rospy.Subscriber("/neupan_initial_path", Path, self.neupan_reference_path_cb, queue_size=1)
        rospy.Subscriber("/camera_right/status", String,
                         lambda msg: self.update("camera_right", self.parse_status(msg)), queue_size=10)
        rospy.Subscriber(self.global_path_topic, Path, self.global_path_cb, queue_size=2)
        rospy.Subscriber(self.gps_guard_topic, String, self.gps_guard_cb, queue_size=20)
        rospy.Subscriber(self.lio_guard_topic, String, self.lio_guard_cb, queue_size=20)
        rospy.Subscriber(
            self.tracker_status_topic, String, self.tracker_status_cb, queue_size=50
        )
        rospy.Subscriber(self.scan_health_topic, String, self.scan_health_cb, queue_size=20)
        rospy.Subscriber(self.safety_topic, Bool, self.safety_cb, queue_size=20)
        rospy.Subscriber(
            self.motor_status_topic, MotorStatus, self.motor_status_cb, queue_size=50
        )
        rospy.Timer(rospy.Duration(1.0 / self.sample_rate), self.timer_cb)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo(
            "system_run_logger: streaming one combined log at %.1fHz to %s",
            self.sample_rate,
            self.log_path if self.log_path is not None else self.log_dir,
        )

    def topic_map(self):
        return {
            "fix": self.fix_topic,
            "gps_goal": self.gps_goal_topic,
            "map_goal": self.map_goal_topic,
            "global_path_ll": self.global_path_ll_topic,
            "goal_arrived": self.goal_arrived_topic,
            "gga": self.gga_topic,
            "heading": self.heading_topic,
            "gnss_solution": self.solution_topic,
            "gps_odom": self.gps_odom_topic,
            "global_odom": self.global_odom_topic,
            "local_odom": self.local_odom_topic,
            "command": self.command_topic,
            "auto_command": self.auto_command_topic,
            "control_mode": self.control_mode_topic,
            "neupan_runtime": self.neupan_runtime_topic,
            "lio_guarded": "/odometry/lio_guarded",
            "motor_alarm": "/motor/alarm",
            "local_fault_event": "/gps_guard/local_fault_event",
            "neupan_resources": "/neupan/resources",
            "neupan_config": "/neupan/config_snapshot",
            "global_path": self.global_path_topic,
            "gps_guard": self.gps_guard_topic,
            "lio_guard": self.lio_guard_topic,
            "tracker_status": self.tracker_status_topic,
            "scan_health": self.scan_health_topic,
            "safety": self.safety_topic,
            "motor": self.motor_status_topic,
            "camera_right": "/camera_right/status",
            "neupan_failure": "/neupan/failure",
            "neupan_reference_path": "/neupan_initial_path",
            "camera_path_guard": "/neupan/camera_path_guard",
            "route_connection": "/neupan/route_connection",
        }

    def local_time_string(self, timestamp=None):
        if timestamp is None:
            timestamp = time.time()
        return datetime.fromtimestamp(timestamp, tz=self.local_timezone).isoformat(
            timespec="milliseconds"
        )

    def open_log_file(self):
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = datetime.now(self.local_timezone).strftime("%Y%m%d_%H%M%S")
            self.log_path = os.path.join(
                self.log_dir, "{}_{}.jsonl".format(self.label, stamp)
            )
            self.log_file = open(self.log_path, "w", buffering=1)
            self.write_record(
                {
                    "record_type": "metadata",
                    "label": self.label,
                    "created_at": rospy.Time.now().to_sec(),
                    "created_at_local": self.local_time_string(),
                    "timezone_offset_hours": self.timezone_offset_hours,
                    "sample_rate": self.sample_rate,
                    "max_file_size_mb": self.max_file_size_bytes / (1024.0 * 1024.0),
                    "topics": self.topic_map(),
                },
                force_flush=True,
                ignore_size_limit=True,
            )
        except Exception as exc:
            self.log_file = None
            rospy.logerr("system_run_logger: failed to open log file: %s", exc)

    def write_record(self, record, force_flush=False, ignore_size_limit=False):
        with self.log_lock:
            return self.write_record_locked(record, force_flush, ignore_size_limit)

    def write_record_locked(self, record, force_flush=False, ignore_size_limit=False):
        if self.log_file is None:
            return False
        line = json.dumps(record, separators=(",", ":"), ensure_ascii=True) + "\n"
        line_size = len(line.encode("utf-8"))
        if (
            not ignore_size_limit
            and self.max_file_size_bytes > 0
            and self.log_size_bytes + line_size > self.max_file_size_bytes
        ):
            if not self.log_capped:
                self.log_capped = True
                rospy.logwarn(
                    "system_run_logger: reached %.1fMB limit; preserving current log and stopping new samples",
                    self.max_file_size_bytes / (1024.0 * 1024.0),
                )
            return False
        try:
            self.log_file.write(line)
            self.log_size_bytes += line_size
            now = time.monotonic()
            if force_flush or now - self.last_flush_time >= self.flush_interval:
                self.log_file.flush()
                os.fsync(self.log_file.fileno())
                self.last_flush_time = now
            return True
        except Exception as exc:
            rospy.logerr_throttle(1.0, "system_run_logger: write failed: %s", exc)
            return False

    @staticmethod
    def normalize_deg(angle_deg):
        angle_rad = math.radians(angle_deg)
        return math.degrees(math.atan2(math.sin(angle_rad), math.cos(angle_rad)))

    @staticmethod
    def yaw_deg(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    @staticmethod
    def message_stamp(header):
        return None if header.stamp == rospy.Time() else header.stamp.to_sec()

    def update(self, key, value):
        with self.log_lock:
            self.latest[key] = value
            self.received_at[key] = rospy.Time.now().to_sec()

    def fix_dict(self, msg):
        return {
            "msg_stamp": self.message_stamp(msg.header),
            "frame_id": msg.header.frame_id,
            "status": int(msg.status.status),
            "service": int(msg.status.service),
            "latitude": float(msg.latitude),
            "longitude": float(msg.longitude),
            "altitude": float(msg.altitude),
            "cov_x": float(msg.position_covariance[0]),
            "cov_y": float(msg.position_covariance[4]),
            "cov_z": float(msg.position_covariance[8]),
            "position_covariance": [float(v) for v in msg.position_covariance],
            "covariance_type": int(msg.position_covariance_type),
        }

    def fix_cb(self, msg):
        self.update("gps_fix", self.fix_dict(msg))

    def record_navigation_event(self, key, payload, summary=None):
        """Log full event data with a coherent snapshot; keep periodic samples compact."""
        with self.log_lock:
            self.update(key, payload if summary is None else summary)
            now = self.received_at[key]
            context_keys = (
                "gps_goal", "map_goal", "goal_arrived", "gps_fix", "gga", "heading", "gnss_solution",
                "gps_odom", "global_odom", "local_odom", "gps_guard",
                "localization_safe", "control_mode", "command",
            )
            self.write_record({
                "record_type": key + "_event",
                "stamp": now,
                "local_time": self.local_time_string(),
                "topic": self.topic_map()[key],
                "data": payload,
                "context": {name: self.latest[name] for name in context_keys},
                "context_received_at": {name: self.received_at[name] for name in context_keys},
            }, force_flush=True)

    def gps_goal_cb(self, msg):
        # Record all received targets, including ones rejected by the goal bridge.
        self.record_navigation_event("gps_goal", self.fix_dict(msg))

    def map_goal_cb(self, msg):
        self.record_navigation_event("map_goal", {
            "msg_stamp": self.message_stamp(msg.header),
            "frame_id": msg.header.frame_id,
            "x": float(msg.pose.position.x),
            "y": float(msg.pose.position.y),
            "z": float(msg.pose.position.z),
            "yaw_deg": self.yaw_deg(msg.pose.orientation),
        })

    def goal_arrived_cb(self, msg):
        with self.log_lock:
            payload = {"state": msg.data, "valid": msg.data in ("0", "1"),
                       "arrived": msg.data == "1"}
            previous = self.latest["goal_arrived"]
            if previous is None or previous["state"] != msg.data:
                self.record_navigation_event("goal_arrived", payload)
            else:
                self.update("goal_arrived", payload)

    def global_path_ll_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict) or not isinstance(payload.get("points"), list):
                raise ValueError("expected a JSON object with a points array")
            points = payload["points"]
            if any(not isinstance(point, dict) or any(
                    not isinstance(point.get(name), (int, float))
                    or isinstance(point.get(name), bool)
                    or not math.isfinite(point[name])
                    for name in ("latitude", "longitude")) for point in points):
                raise ValueError("expected finite latitude/longitude in every point")
            summary = {"valid": True, "points": len(points),
                       "start": points[0] if points else None,
                       "end": points[-1] if points else None}
        except (TypeError, ValueError) as exc:
            payload = {"valid": False, "error": str(exc), "raw": msg.data}
            summary = {"valid": False, "error": str(exc)}
        self.record_navigation_event("global_path_ll", payload, summary)

    def gga_cb(self, msg):
        self.update(
            "gga",
            {
                "msg_stamp": self.message_stamp(msg.header),
                "quality": int(msg.gps_qual),
                "satellites": int(msg.num_sats),
                "hdop": float(msg.hdop),
                "altitude": float(msg.alt),
            },
        )

    def solution_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("GNSS solution status must be an object")
        except (TypeError, ValueError) as exc:
            payload = {"valid": False, "error": str(exc)}
        self.update("gnss_solution", payload)

    def heading_cb(self, msg):
        yaw = self.yaw_deg(msg.quaternion)
        jump = None
        if self.previous_heading_deg is not None:
            jump = self.normalize_deg(yaw - self.previous_heading_deg)
        self.previous_heading_deg = yaw
        self.update(
            "heading",
            {
                "msg_stamp": self.message_stamp(msg.header),
                "yaw_deg": yaw,
                "jump_deg": jump,
            },
        )

    def odom_dict(self, msg):
        return {
            "msg_stamp": self.message_stamp(msg.header),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "z": float(msg.pose.pose.position.z),
            "yaw_deg": self.yaw_deg(msg.pose.pose.orientation),
            "linear_x": float(msg.twist.twist.linear.x),
            "linear_y": float(msg.twist.twist.linear.y),
            "angular_z": float(msg.twist.twist.angular.z),
        }

    def global_odom_cb(self, msg):
        self.update("global_odom", self.odom_dict(msg))

    def gps_odom_cb(self, msg):
        self.update("gps_odom", self.odom_dict(msg))

    def local_odom_cb(self, msg):
        self.update("local_odom", self.odom_dict(msg))

    def command_cb(self, msg):
        self.update(
            "command",
            {
                "linear_x": float(msg.linear.x),
                "linear_y": float(msg.linear.y),
                "angular_z": float(msg.angular.z),
                "steering_deg": math.degrees(float(msg.angular.z)),
            },
        )

    def auto_command_cb(self, msg):
        now = time.monotonic()
        payload = {
            "linear_x": float(msg.linear.x),
            "angular_z": float(msg.angular.z),
            "steering_deg": math.degrees(float(msg.angular.z)),
            "receive_stamp": rospy.Time.now().to_sec(),
            "receive_interval_ms": None if self.previous_auto_receive is None
            else (now - self.previous_auto_receive) * 1000.0,
        }
        self.previous_auto_receive = now
        self.update("auto_command", payload)
        self.write_record({"record_type": "auto_command_event", "data": payload})

    def control_mode_cb(self, msg):
        payload = self.parse_status(msg)
        self.update("control_mode", payload)
        self.write_record({"record_type": "control_mode_event", "data": payload})

    def neupan_runtime_cb(self, msg):
        payload = self.parse_status(msg)
        self.update("neupan_runtime", payload)
        self.write_record({"record_type": "neupan_runtime_event", "data": payload})

    def route_connection_cb(self, msg):
        payload = self.parse_status(msg)
        previous = self.latest.get('route_connection')
        self.update('route_connection', payload)
        if payload != previous:
            self.write_record({'record_type': 'route_connection_event', 'data': payload}, force_flush=True)

    def neupan_failure_cb(self, msg):
        payload = self.parse_status(msg)
        self.ingest_neupan_failure(payload)

    def ingest_neupan_failure(self, payload, source='ros'):
        if not isinstance(payload, dict) or not isinstance(payload.get('traceback'), str):
            return False
        identity = payload.get('snapshot_json') or json.dumps(payload, sort_keys=True)
        with self.log_lock:
            if identity in self.seen_failure_snapshots:
                return True
            self.update("neupan_failure", {
                key: payload.get(key) for key in
                ("stamp", "stage", "exception_type", "message", "snapshot_json", "snapshot_npz")
            })
            saved = self.write_record({"record_type": "neupan_failure_event", "data": payload,
                                       "ingest_source": source},
                                      force_flush=True, ignore_size_limit=True)
            if saved:
                self.seen_failure_snapshots.add(identity)
            return saved

    def poll_failure_files(self):
        now = time.monotonic()
        if now-self.last_failure_scan < 1.:
            return
        self.last_failure_scan = now
        # A dead planner cannot retransmit a lost ROS event. Its fsynced JSON
        # is the durable source, including after the logging node restarts.
        for path in sorted(glob.glob(os.path.join(self.failure_snapshot_dir, 'neupan_failure_*.json'))):
            if os.path.abspath(path) in self.seen_failure_snapshots:
                continue
            try:
                with open(path) as stream:
                    payload = json.load(stream)
                if payload.get('snapshot_json') != os.path.abspath(path):
                    continue
                self.ingest_neupan_failure(payload, source='snapshot_file')
            except (OSError, ValueError, AttributeError):
                # The snapshot may still be being written; retry next scan.
                continue

    def neupan_reference_path_cb(self, msg):
        signature = (msg.header.frame_id, tuple(
            (p.pose.position.x, p.pose.position.y, p.pose.position.z,
             p.pose.orientation.x, p.pose.orientation.y,
             p.pose.orientation.z, p.pose.orientation.w) for p in msg.poses))
        if signature == self.previous_reference_path:
            return
        self.previous_reference_path = signature
        payload = {
            "msg_stamp": self.message_stamp(msg.header),
            "frame_id": msg.header.frame_id,
            "poses": [{"x": p.pose.position.x, "y": p.pose.position.y,
                       "z": p.pose.position.z,
                       "yaw_deg": self.yaw_deg(p.pose.orientation)} for p in msg.poses],
        }
        self.record_navigation_event("neupan_reference_path", payload, {
            "msg_stamp": payload["msg_stamp"], "frame_id": msg.header.frame_id,
            "points": len(msg.poses),
        })

    def local_fault_event_cb(self, msg):
        payload = self.parse_status(msg)
        self.update("local_fault_event", payload)
        self.write_record({"record_type": "local_fault_event", "data": payload})

    def neupan_config_cb(self, msg):
        payload = self.parse_status(msg)
        self.update("neupan_config", {
            key: payload.get(key) for key in
            ("stamp", "config_file", "dune_checkpoint", "checkpoint_sha256")
        })
        self.write_record({"record_type": "neupan_config", "data": payload})

    def global_path_cb(self, msg):
        payload = {
            "msg_stamp": self.message_stamp(msg.header),
            "frame_id": msg.header.frame_id,
            "poses": [{"x": float(p.pose.position.x), "y": float(p.pose.position.y),
                       "z": float(p.pose.position.z),
                       "yaw_deg": self.yaw_deg(p.pose.orientation)} for p in msg.poses],
        }
        if not msg.poses:
            self.record_navigation_event("global_path", payload, {
                "msg_stamp": payload["msg_stamp"],
                "frame_id": msg.header.frame_id, "points": 0})
            return
        first = msg.poses[0].pose
        last = msg.poses[-1].pose
        self.record_navigation_event(
            "global_path", payload,
            {
                "msg_stamp": self.message_stamp(msg.header),
                "frame_id": msg.header.frame_id,
                "points": len(msg.poses),
                "start_x": float(first.position.x),
                "start_y": float(first.position.y),
                "start_yaw_deg": self.yaw_deg(first.orientation),
                "end_x": float(last.position.x),
                "end_y": float(last.position.y),
                "end_yaw_deg": self.yaw_deg(last.orientation),
            },
        )

    @staticmethod
    def parse_status(msg):
        try:
            return json.loads(msg.data)
        except (TypeError, ValueError):
            return {"mode": "INVALID_STATUS", "reason": msg.data}

    def gps_guard_cb(self, msg):
        self.update("gps_guard", self.parse_status(msg))

    def lio_guard_cb(self, msg):
        self.update("lio_guard", self.parse_status(msg))

    def tracker_status_cb(self, msg):
        self.update("tracker_status", self.parse_status(msg))

    def scan_health_cb(self, msg):
        self.update("scan_health", self.parse_status(msg))

    def safety_cb(self, msg):
        self.update("localization_safe", bool(msg.data))

    def motor_status_cb(self, msg):
        self.update(
            "motor",
            {
                "msg_stamp": self.message_stamp(msg.header),
                "left_torque_nm": float(msg.left_torque),
                "right_torque_nm": float(msg.right_torque),
                "left_mos_temperature_c": float(msg.left_mos_temperature),
                "left_coil_temperature_c": float(msg.left_coil_temperature),
                "right_mos_temperature_c": float(msg.right_mos_temperature),
                "right_coil_temperature_c": float(msg.right_coil_temperature),
                "left_state_code": msg.left_state_code,
                "left_state_text": msg.left_state_text,
                "right_state_code": msg.right_state_code,
                "right_state_text": msg.right_state_text,
                "alarm": bool(msg.alarm),
                "alarm_reason": msg.alarm_reason,
            },
        )

    def timer_cb(self, _event):
        self.poll_failure_files()
        now = rospy.Time.now().to_sec()
        sample = {"stamp": now, "local_time": self.local_time_string(now)}
        with self.log_lock:
            latest = dict(self.latest)
            received_at = dict(self.received_at)
        for key, value in latest.items():
            sample[key] = value
            received = received_at[key]
            sample[key + "_age_sec"] = None if received is None else max(0.0, now - received)

        heading = latest["heading"]
        global_odom = latest["global_odom"]
        local_odom = latest["local_odom"]
        sample["global_heading_diff_deg"] = None
        if heading is not None and global_odom is not None:
            sample["global_heading_diff_deg"] = self.normalize_deg(
                global_odom["yaw_deg"] - heading["yaw_deg"]
            )

        if self.write_record({"record_type": "sample", "data": sample}):
            self.sample_count += 1

        if self.print_to_screen:
            rospy.loginfo_throttle(
                1.0,
                "system_log: samples=%d gps=%s global_odom=%s local_odom=%s cmd=%s",
                self.sample_count,
                self.latest["gps_fix"] is not None,
                global_odom is not None,
                local_odom is not None,
                self.latest["command"] is not None,
            )

    def on_shutdown(self):
        with self.log_lock:
            self.close_log_locked()

    def close_log_locked(self):
        if self.log_file is None:
            return
        try:
            self.write_record(
                {
                    "record_type": "summary",
                    "closed_at": rospy.Time.now().to_sec(),
                    "closed_at_local": self.local_time_string(),
                    "sample_count": self.sample_count,
                    "size_bytes": self.log_size_bytes,
                    "size_limit_reached": self.log_capped,
                },
                force_flush=True,
                ignore_size_limit=True,
            )
            self.log_file.close()
            self.log_file = None
            rospy.loginfo("system_run_logger: closed combined log %s", self.log_path)
        except Exception as exc:
            rospy.logwarn("system_run_logger: failed to close log: %s", exc)


if __name__ == "__main__":
    try:
        SystemRunLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
