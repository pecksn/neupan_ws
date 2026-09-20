#!/usr/bin/env python3
import json

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8


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

        self.output_pub = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=2, latch=True)
        rospy.Subscriber(self.mode_topic, UInt8, self.mode_callback, queue_size=10)
        rospy.Subscriber(self.auto_topic, Twist, self.auto_callback, queue_size=10)
        rospy.Subscriber(self.remote_topic, Twist, self.remote_callback, queue_size=10)
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

    def selected_command(self, now):
        if self.mode == self.AUTO:
            if self.last_auto is None or (now - self.last_auto_time).to_sec() > self.auto_timeout:
                return Twist(), "auto_timeout"
            return self.last_auto, "auto_active"
        if self.last_remote is None or (now - self.last_remote_time).to_sec() > self.remote_timeout:
            return Twist(), "remote_timeout"
        return self.last_remote, "remote_active"

    def publish_status(self, reason):
        payload = {"mode": self.mode, "mode_name": self.MODE_NAMES[self.mode], "reason": reason}
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        self.last_reason = reason

    def timer_callback(self, _event):
        command, reason = self.selected_command(rospy.Time.now())
        self.output_pub.publish(command)
        if reason != self.last_reason:
            self.publish_status(reason)


if __name__ == "__main__":
    try:
        ControlModeMux()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
