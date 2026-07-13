#!/usr/bin/env python3
import json
import math
import os
from datetime import datetime

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from nmea_msgs.msg import Gpgga
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


class G90HealthMonitor:
    def __init__(self):
        rospy.init_node("g90_health_monitor")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.heading_topic = rospy.get_param("~heading_topic", "/heading")
        self.odom_topic = rospy.get_param("~odom_topic", "/odometry/global")
        self.guard_status_topic = rospy.get_param("~guard_status_topic", "/gps_guard/status")
        self.gga_topic = rospy.get_param("~gga_topic", "/gnss/gpgga")
        self.label = rospy.get_param("~label", "g90_health")
        self.log_to_file = bool(rospy.get_param("~log_to_file", True))
        self.log_dir = rospy.get_param("~log_dir", "/home/nuc/lxw/neupan_ws/tracker_logs")
        self.max_samples = int(rospy.get_param("~max_samples", 4000))
        self.heading_jump_warn_deg = float(rospy.get_param("~heading_jump_warn_deg", 8.0))
        self.pos_cov_warn_m2 = float(rospy.get_param("~pos_cov_warn_m2", 0.25))

        self.last_fix = None
        self.last_heading_yaw_deg = None
        self.last_heading_stamp = None
        self.prev_heading_yaw_deg = None
        self.last_odom_yaw_deg = None
        self.last_guard_status = None
        self.last_gga = None
        self.samples = []

        rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_cb, queue_size=10)
        rospy.Subscriber(self.heading_topic, QuaternionStamped, self.heading_cb, queue_size=10)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber(self.guard_status_topic, String, self.guard_status_cb, queue_size=10)
        rospy.Subscriber(self.gga_topic, Gpgga, self.gga_cb, queue_size=10)
        rospy.Timer(rospy.Duration(0.5), self.timer_cb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "g90_health_monitor: fix=%s heading=%s odom=%s label=%s",
            self.fix_topic,
            self.heading_topic,
            self.odom_topic,
            self.label,
        )

    @staticmethod
    def normalize_deg(angle_deg):
        return math.degrees(math.atan2(math.sin(math.radians(angle_deg)), math.cos(math.radians(angle_deg))))

    @staticmethod
    def yaw_from_quaternion(q):
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return math.degrees(yaw)

    def fix_cb(self, msg):
        self.last_fix = msg

    def heading_cb(self, msg):
        yaw_deg = self.yaw_from_quaternion(msg.quaternion)
        self.prev_heading_yaw_deg = self.last_heading_yaw_deg
        self.last_heading_yaw_deg = yaw_deg
        self.last_heading_stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

    def odom_cb(self, msg):
        self.last_odom_yaw_deg = self.yaw_from_quaternion(msg.pose.pose.orientation)

    def guard_status_cb(self, msg):
        try:
            self.last_guard_status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.last_guard_status = {"mode": "INVALID_STATUS", "reason": msg.data}

    def gga_cb(self, msg):
        self.last_gga = msg

    def timer_cb(self, _event):
        if self.last_fix is None or self.last_heading_yaw_deg is None:
            rospy.loginfo_throttle(
                1.0,
                "%s: waiting fix=%s heading=%s",
                self.label,
                self.last_fix is not None,
                self.last_heading_yaw_deg is not None,
            )
            return

        pos_cov_x = float(self.last_fix.position_covariance[0])
        pos_cov_y = float(self.last_fix.position_covariance[4])
        pos_cov_z = float(self.last_fix.position_covariance[8])
        heading_jump = None
        if self.prev_heading_yaw_deg is not None:
            heading_jump = self.normalize_deg(self.last_heading_yaw_deg - self.prev_heading_yaw_deg)

        odom_diff = None
        if self.last_odom_yaw_deg is not None:
            odom_diff = self.normalize_deg(self.last_odom_yaw_deg - self.last_heading_yaw_deg)

        sample = {
            "stamp": rospy.Time.now().to_sec(),
            "fix_status": int(self.last_fix.status.status),
            "fix_service": int(self.last_fix.status.service),
            "latitude": float(self.last_fix.latitude),
            "longitude": float(self.last_fix.longitude),
            "altitude": float(self.last_fix.altitude),
            "cov_x": pos_cov_x,
            "cov_y": pos_cov_y,
            "cov_z": pos_cov_z,
            "heading_yaw_deg": self.last_heading_yaw_deg,
            "heading_jump_deg": heading_jump,
            "odom_global_yaw_deg": self.last_odom_yaw_deg,
            "odom_heading_diff_deg": odom_diff,
            "gps_guard": self.last_guard_status,
            "gga_quality": None if self.last_gga is None else int(self.last_gga.gps_qual),
            "num_satellites": None if self.last_gga is None else int(self.last_gga.num_sats),
            "hdop": None if self.last_gga is None else float(self.last_gga.hdop),
        }
        self.samples.append(sample)
        if self.max_samples > 0 and len(self.samples) > self.max_samples:
            self.samples = self.samples[-self.max_samples:]

        warn_tokens = []
        if heading_jump is not None and abs(heading_jump) >= self.heading_jump_warn_deg:
            warn_tokens.append("heading_jump")
        if pos_cov_x >= self.pos_cov_warn_m2 or pos_cov_y >= self.pos_cov_warn_m2:
            warn_tokens.append("cov_large")
        if self.last_fix.status.status < 0:
            warn_tokens.append("no_fix")

        rospy.loginfo_throttle(
            0.5,
            "%s: fix=%d lat=%.8f lon=%.8f cov=[%.4f, %.4f, %.4f] heading=%.1fdeg jump=%s odom_diff=%s guard=%s/%s warn=%s",
            self.label,
            self.last_fix.status.status,
            self.last_fix.latitude,
            self.last_fix.longitude,
            pos_cov_x,
            pos_cov_y,
            pos_cov_z,
            self.last_heading_yaw_deg,
            "none" if heading_jump is None else f"{heading_jump:.1f}deg",
            "none" if odom_diff is None else f"{odom_diff:.1f}deg",
            "none" if self.last_guard_status is None else self.last_guard_status.get("mode", "unknown"),
            "none" if self.last_guard_status is None else self.last_guard_status.get("reason", "unknown"),
            ",".join(warn_tokens) if warn_tokens else "none",
        )

    def on_shutdown(self):
        if not self.log_to_file or not self.samples:
            return
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.log_dir, f"{self.label}_{stamp}.json")
            temp_path = path + ".tmp"
            with open(temp_path, "w") as f:
                json.dump(
                    {
                        "label": self.label,
                        "fix_topic": self.fix_topic,
                        "heading_topic": self.heading_topic,
                        "odom_topic": self.odom_topic,
                        "sample_count": len(self.samples),
                        "samples": self.samples,
                    },
                    f,
                    indent=2,
                    ensure_ascii=True,
                )
                f.flush()
                os.fsync(f.fileno())
            os.replace(temp_path, path)
            rospy.loginfo("%s: wrote health log to %s", self.label, path)
        except Exception as exc:
            rospy.logwarn("%s: failed to write health log: %s", self.label, exc)


if __name__ == "__main__":
    try:
        G90HealthMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
