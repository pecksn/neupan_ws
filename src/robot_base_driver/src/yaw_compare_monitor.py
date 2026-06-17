#!/usr/bin/env python3
import json
import math
import os
from datetime import datetime

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class YawCompareMonitor:
    def __init__(self):
        rospy.init_node("yaw_compare_monitor")
        self.odom_topic = rospy.get_param("~odom_topic", "/odometry/global")
        self.heading_topic = rospy.get_param("~heading_topic", "/heading")
        self.label = rospy.get_param("~label", "yaw_compare")
        self.log_to_file = bool(rospy.get_param("~log_to_file", True))
        self.log_dir = rospy.get_param("~log_dir", "/home/nuc/lxw/neupan_ws/tracker_logs")
        self.max_samples = int(rospy.get_param("~max_samples", 2000))

        self.last_odom_yaw = None
        self.last_heading_yaw = None
        self.last_odom_pos = None
        self.samples = []

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber(self.heading_topic, QuaternionStamped, self.heading_cb, queue_size=10)
        rospy.Timer(rospy.Duration(0.5), self.timer_cb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo(
            "yaw_compare_monitor: odom_topic=%s heading_topic=%s label=%s log_to_file=%s",
            self.odom_topic,
            self.heading_topic,
            self.label,
            self.log_to_file,
        )

    @staticmethod
    def normalize_deg(angle_deg):
        return math.degrees(math.atan2(math.sin(math.radians(angle_deg)), math.cos(math.radians(angle_deg))))

    @staticmethod
    def yaw_from_quaternion(q):
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return math.degrees(yaw)

    def odom_cb(self, msg):
        self.last_odom_yaw = self.yaw_from_quaternion(msg.pose.pose.orientation)
        self.last_odom_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.header.frame_id)

    def heading_cb(self, msg):
        self.last_heading_yaw = self.yaw_from_quaternion(msg.quaternion)

    def timer_cb(self, _event):
        if self.last_odom_yaw is None or self.last_heading_yaw is None:
            rospy.loginfo_throttle(
                1.0,
                "%s: waiting odom_yaw=%s heading_yaw=%s",
                self.label,
                self.last_odom_yaw is not None,
                self.last_heading_yaw is not None,
            )
            return

        diff_deg = self.normalize_deg(self.last_odom_yaw - self.last_heading_yaw)
        sample = {
            "stamp": rospy.Time.now().to_sec(),
            "odom_global_yaw_deg": self.last_odom_yaw,
            "g90_yaw_deg": self.last_heading_yaw,
            "diff_deg": diff_deg,
        }
        if self.last_odom_pos is None:
            self._append_sample(sample)
            rospy.loginfo_throttle(
                0.5,
                "%s: odom_global_yaw=%.1fdeg g90_yaw=%.1fdeg diff=%.1fdeg",
                self.label,
                self.last_odom_yaw,
                self.last_heading_yaw,
                diff_deg,
            )
            return

        x, y, frame_id = self.last_odom_pos
        sample["position"] = {"x": x, "y": y, "frame_id": frame_id}
        self._append_sample(sample)
        rospy.loginfo_throttle(
            0.5,
            "%s: pos=(%.2f, %.2f) odom_global_yaw=%.1fdeg g90_yaw=%.1fdeg diff=%.1fdeg frame=%s",
            self.label,
            x,
            y,
            self.last_odom_yaw,
            self.last_heading_yaw,
            diff_deg,
            frame_id,
        )

    def _append_sample(self, sample):
        self.samples.append(sample)
        if self.max_samples > 0 and len(self.samples) > self.max_samples:
            self.samples = self.samples[-self.max_samples:]

    def on_shutdown(self):
        if not self.log_to_file or not self.samples:
            return

        try:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self.log_dir, "{}_{}.json".format(self.label, stamp))
            payload = {
                "label": self.label,
                "odom_topic": self.odom_topic,
                "heading_topic": self.heading_topic,
                "sample_count": len(self.samples),
                "samples": self.samples,
            }
            with open(path, "w") as f:
                json.dump(payload, f, indent=2, ensure_ascii=True)
            rospy.loginfo("%s: wrote yaw log to %s", self.label, path)
        except Exception as exc:
            rospy.logwarn("%s: failed to write yaw log: %s", self.label, exc)


if __name__ == "__main__":
    try:
        YawCompareMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
