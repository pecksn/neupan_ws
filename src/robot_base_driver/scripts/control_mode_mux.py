#!/usr/bin/env python3
import json
import math
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String, UInt8


class ControlModeMux:
    AUTO = 0
    MANUAL = 1
    MODE_NAMES = {AUTO: "AUTO", MANUAL: "MANUAL"}

    def __init__(self):
        rospy.init_node("control_mode_mux")
        self.mode_topic = rospy.get_param("~mode_topic", "/vehicle/control_mode")
        self.auto_topic = rospy.get_param("~auto_topic", "/cmd_vel/auto")
        self.remote_topic = rospy.get_param("~remote_topic", "/cmd_vel/remote")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel/selected")
        self.status_topic = rospy.get_param("~status_topic", "/vehicle/control_mode/status")
        self.safety_topic = rospy.get_param("~safety_topic", "/localization_safe")
        self.enable_auto_localization_safety_stop = bool(
            rospy.get_param("~enable_auto_localization_safety_stop", False)
        )
        self.localization_safety_timeout = float(rospy.get_param("~localization_safety_timeout", 1.0))
        if not math.isfinite(self.localization_safety_timeout) or self.localization_safety_timeout <= 0:
            raise ValueError("localization_safety_timeout must be finite and positive")
        # A latched/previous True is not a permanent driving permit. Use a
        # monotonic receipt clock, independent of ROS/wall-clock adjustments.
        # Keep the value/time together so the timer sees a consistent sample.
        self.last_safety_sample = None
        self.publish_rate = max(1.0, float(rospy.get_param("~publish_rate", 20.0)))
        self.auto_timeout = max(0.05, float(rospy.get_param("~auto_timeout", 0.5)))
        self.remote_timeout = max(0.05, float(rospy.get_param("~remote_timeout", 0.5)))
        self.mode = int(rospy.get_param("~default_mode", self.AUTO))
        if self.mode not in self.MODE_NAMES:
            rospy.logwarn("Invalid default control mode %d; falling back to AUTO", self.mode)
            self.mode = self.AUTO

        self.last_auto = None
        self.last_auto_time = rospy.Time(0)
        self.last_remote = None
        self.last_remote_time = rospy.Time(0)
        self.last_reason = "starting"
        self.last_status_monotonic = None

        self.output_pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=2, latch=True)
        rospy.Subscriber(self.mode_topic, UInt8, self.mode_callback, queue_size=10)
        rospy.Subscriber(self.auto_topic, Twist, self.auto_callback, queue_size=10)
        rospy.Subscriber(self.remote_topic, Twist, self.remote_callback, queue_size=10)
        if self.enable_auto_localization_safety_stop:
            rospy.Subscriber(self.safety_topic, Bool, self.localization_safety_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_callback)
        self.publish_status("starting")

    def mode_callback(self, msg):
        requested = int(msg.data)
        if requested not in self.MODE_NAMES:
            rospy.logwarn_throttle(1.0, "Ignoring invalid control mode %d (valid: 0=AUTO, 1=MANUAL)", requested)
            return

        if requested != self.mode:
            previous = self.MODE_NAMES[self.mode]
            self.mode = requested
            # NeuPAN remains active in MANUAL mode. Switching back to AUTO
            # immediately uses its latest command without restarting the node.
            command, reason = self.selected_command(rospy.Time.now())
            self.output_pub.publish(command)
            self.publish_status(reason)
            rospy.logwarn("Control mode changed: %s -> %s", previous, self.MODE_NAMES[self.mode])

    def auto_callback(self, msg):
        self.last_auto = msg
        self.last_auto_time = rospy.Time.now()

    def remote_callback(self, msg):
        self.last_remote = msg
        self.last_remote_time = rospy.Time.now()

    def localization_safety_callback(self, msg):
        self.last_safety_sample = (bool(msg.data), time.monotonic())
        if not msg.data and self.mode == self.AUTO:
            self.output_pub.publish(Twist())
            if self.last_reason != "localization_unsafe":
                self.publish_status("localization_unsafe")

    def localization_safety_state(self):
        if not self.enable_auto_localization_safety_stop:
            return True, "disabled", None
        sample = self.last_safety_sample
        if sample is None:
            return False, "localization_safety_missing", None
        age = max(0.0, time.monotonic() - sample[1])
        if age >= self.localization_safety_timeout:
            return False, "localization_safety_timeout", age
        return sample[0], "localization_safe" if sample[0] else "localization_unsafe", age

    def selected_command(self, now):
        if self.mode == self.AUTO:
            safe, reason, _ = self.localization_safety_state()
            if not safe:
                return Twist(), reason
            if self.last_auto is None or (now - self.last_auto_time).to_sec() > self.auto_timeout:
                return Twist(), "auto_timeout"
            return self.last_auto, "auto_active"
        if self.last_remote is None or (now - self.last_remote_time).to_sec() > self.remote_timeout:
            return Twist(), "remote_timeout"
        return self.last_remote, "remote_active"

    def publish_status(self, reason):
        now = rospy.Time.now()
        safe, safety_reason, safety_age = self.localization_safety_state()
        sample = self.last_safety_sample
        payload = {
            "stamp": now.to_sec(), "mode": self.mode,
            "mode_name": self.MODE_NAMES[self.mode], "reason": reason,
            "auto_age_sec": None if self.last_auto is None else
            (now - self.last_auto_time).to_sec(),
            "remote_age_sec": None if self.last_remote is None else
            (now - self.last_remote_time).to_sec(),
            "auto_timeout_sec": self.auto_timeout,
            "remote_timeout_sec": self.remote_timeout,
            "localization_safe": safe,
            "localization_safe_reported": None if sample is None else sample[0],
            "localization_safety_enabled": self.enable_auto_localization_safety_stop,
            "localization_safety_reason": safety_reason,
            "localization_safety_age_sec": safety_age,
            "localization_safety_timeout_sec": self.localization_safety_timeout,
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        if reason != self.last_reason and reason in (
                "localization_safety_missing", "localization_safety_timeout", "localization_unsafe"):
            rospy.logwarn("AUTO localization safety stop: %s (heartbeat age=%s s, timeout=%.2f s)",
                          reason, "none" if safety_age is None else "%.3f" % safety_age,
                          self.localization_safety_timeout)
        self.last_reason = reason
        self.last_status_monotonic = time.monotonic()

    def timer_callback(self, _event):
        command, reason = self.selected_command(rospy.Time.now())
        self.output_pub.publish(command)
        if (reason != self.last_reason or self.last_status_monotonic is None
                or time.monotonic() - self.last_status_monotonic >= 0.5):
            self.publish_status(reason)


if __name__ == "__main__":
    try:
        ControlModeMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
