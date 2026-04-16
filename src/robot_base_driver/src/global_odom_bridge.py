#!/usr/bin/env python

import math

import rospy
from nav_msgs.msg import Odometry


class GlobalOdomBridge:
    def __init__(self):
        rospy.init_node("global_odom_bridge")

        self.input_topic = rospy.get_param("~input_topic", "/odometry/gps")
        self.output_topic = rospy.get_param("~output_topic", "/odometry/global")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "base_link")

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.cb, queue_size=10)

        rospy.loginfo(
            "global_odom_bridge: %s -> %s (frame %s)",
            self.input_topic,
            self.output_topic,
            self.output_frame,
        )

    def _finite(self, v):
        return math.isfinite(v)

    def cb(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        if not (self._finite(px) and self._finite(py)):
            rospy.logwarn_throttle(1.0, "global_odom_bridge got non-finite odom, drop")
            return

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.output_frame
        out.child_frame_id = self.child_frame if self.child_frame else msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)


if __name__ == "__main__":
    try:
        GlobalOdomBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
