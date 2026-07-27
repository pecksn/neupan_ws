#!/usr/bin/env python3

import json
import math
import os
import time
from datetime import datetime

import rospy
from geometry_msgs.msg import QuaternionStamped, Twist
from nav_msgs.msg import Odometry, Path
from nmea_msgs.msg import Gpgga
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String


class SystemRunLogger:
    """Record navigation diagnostics on one shared time axis and write one file."""

    def __init__(self):
        rospy.init_node("system_run_logger")

        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.gga_topic = rospy.get_param("~gga_topic", "/gnss/gpgga")
        self.heading_topic = rospy.get_param("~heading_topic", "/heading")
        self.global_odom_topic = rospy.get_param("~global_odom_topic", "/odometry/global")
        self.local_odom_topic = rospy.get_param("~local_odom_topic", "/odometry/local")
        self.command_topic = rospy.get_param("~command_topic", "/neupan_cmd_vel")
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

        self.label = rospy.get_param("~label", "system_run_full")
        self.log_dir = rospy.get_param(
            "~log_dir", "/home/nuc/lxw/neupan_ws/tracker_logs"
        )
        self.sample_rate = max(1.0, float(rospy.get_param("~sample_rate", 10.0)))
        self.flush_interval = max(0.1, float(rospy.get_param("~flush_interval", 1.0)))
        self.max_file_size_bytes = max(
            1, int(float(rospy.get_param("~max_file_size_mb", 256.0)) * 1024 * 1024)
        )
        self.print_to_screen = bool(rospy.get_param("~print_to_screen", False))

        self.latest = {
            "gps_fix": None,
            "gga": None,
            "heading": None,
            "global_odom": None,
            "local_odom": None,
            "command": None,
            "global_path": None,
            "gps_guard": None,
            "lio_guard": None,
            "tracker_status": None,
            "scan_health": None,
            "localization_safe": None,
        }
        self.received_at = {key: None for key in self.latest}
        self.previous_heading_deg = None
        self.sample_count = 0
        self.log_file = None
        self.log_path = None
        self.log_size_bytes = 0
        self.log_capped = False
        self.last_flush_time = time.monotonic()

        self.open_log_file()

        rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_cb, queue_size=20)
        rospy.Subscriber(self.gga_topic, Gpgga, self.gga_cb, queue_size=20)
        rospy.Subscriber(self.heading_topic, QuaternionStamped, self.heading_cb, queue_size=30)
        rospy.Subscriber(self.global_odom_topic, Odometry, self.global_odom_cb, queue_size=30)
        rospy.Subscriber(self.local_odom_topic, Odometry, self.local_odom_cb, queue_size=50)
        rospy.Subscriber(self.command_topic, Twist, self.command_cb, queue_size=100)
        rospy.Subscriber(self.global_path_topic, Path, self.global_path_cb, queue_size=2)
        rospy.Subscriber(self.gps_guard_topic, String, self.gps_guard_cb, queue_size=20)
        rospy.Subscriber(self.lio_guard_topic, String, self.lio_guard_cb, queue_size=20)
        rospy.Subscriber(
            self.tracker_status_topic, String, self.tracker_status_cb, queue_size=50
        )
        rospy.Subscriber(self.scan_health_topic, String, self.scan_health_cb, queue_size=20)
        rospy.Subscriber(self.safety_topic, Bool, self.safety_cb, queue_size=20)
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
            "gga": self.gga_topic,
            "heading": self.heading_topic,
            "global_odom": self.global_odom_topic,
            "local_odom": self.local_odom_topic,
            "command": self.command_topic,
            "global_path": self.global_path_topic,
            "gps_guard": self.gps_guard_topic,
            "lio_guard": self.lio_guard_topic,
            "tracker_status": self.tracker_status_topic,
            "scan_health": self.scan_health_topic,
            "safety": self.safety_topic,
        }

    @staticmethod
    def local_time_string(timestamp=None):
        if timestamp is None:
            timestamp = time.time()
        return datetime.fromtimestamp(timestamp).astimezone().isoformat(timespec="milliseconds")

    def open_log_file(self):
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
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
        if self.log_file is None:
            return False
        line = json.dumps(record, separators=(",", ":"), ensure_ascii=True) + "\n"
        line_size = len(line.encode("utf-8"))
        if (
            not ignore_size_limit
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
        self.latest[key] = value
        self.received_at[key] = rospy.Time.now().to_sec()

    def fix_cb(self, msg):
        self.update(
            "gps_fix",
            {
                "msg_stamp": self.message_stamp(msg.header),
                "status": int(msg.status.status),
                "service": int(msg.status.service),
                "latitude": float(msg.latitude),
                "longitude": float(msg.longitude),
                "altitude": float(msg.altitude),
                "cov_x": float(msg.position_covariance[0]),
                "cov_y": float(msg.position_covariance[4]),
                "cov_z": float(msg.position_covariance[8]),
                "covariance_type": int(msg.position_covariance_type),
            },
        )

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
            "yaw_deg": self.yaw_deg(msg.pose.pose.orientation),
            "linear_x": float(msg.twist.twist.linear.x),
            "linear_y": float(msg.twist.twist.linear.y),
            "angular_z": float(msg.twist.twist.angular.z),
        }

    def global_odom_cb(self, msg):
        self.update("global_odom", self.odom_dict(msg))

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

    def global_path_cb(self, msg):
        if not msg.poses:
            self.update("global_path", {"frame_id": msg.header.frame_id, "points": 0})
            return
        first = msg.poses[0].pose
        last = msg.poses[-1].pose
        self.update(
            "global_path",
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

    def timer_cb(self, _event):
        now = rospy.Time.now().to_sec()
        sample = {"stamp": now, "local_time": self.local_time_string(now)}
        for key, value in self.latest.items():
            sample[key] = value
            received = self.received_at[key]
            sample[key + "_age_sec"] = None if received is None else max(0.0, now - received)

        heading = self.latest["heading"]
        global_odom = self.latest["global_odom"]
        local_odom = self.latest["local_odom"]
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
        if self.log_file is None:
            return
        try:
            self.write_record(
                {
                    "record_type": "summary",
                    "closed_at": rospy.Time.now().to_sec(),
                    "sample_count": self.sample_count,
                    "size_bytes": self.log_size_bytes,
                    "size_limit_reached": self.log_capped,
                },
                force_flush=True,
                ignore_size_limit=True,
            )
            self.log_file.close()
            rospy.loginfo("system_run_logger: closed combined log %s", self.log_path)
        except Exception as exc:
            rospy.logwarn("system_run_logger: failed to close log: %s", exc)


if __name__ == "__main__":
    try:
        SystemRunLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
