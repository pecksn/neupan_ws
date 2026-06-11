#!/usr/bin/env python3
import math

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


class OdomYawMonitor:
    def __init__(self):
        rospy.init_node("odom_yaw_monitor")
        self.topic = rospy.get_param("~topic", "/odometry/global")
        self.label = rospy.get_param("~label", self.topic)
        self.last_yaw = None
        self.last_pos = None
        rospy.Subscriber(self.topic, Odometry, self.cb, queue_size=10)
        rospy.loginfo("odom_yaw_monitor: topic=%s label=%s", self.topic, self.label)

    def cb(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dyaw_deg = 0.0 if self.last_yaw is None else math.degrees(
            math.atan2(math.sin(yaw - self.last_yaw), math.cos(yaw - self.last_yaw))
        )
        ds = 0.0 if self.last_pos is None else math.hypot(x - self.last_pos[0], y - self.last_pos[1])
        self.last_yaw = yaw
        self.last_pos = (x, y)
        rospy.loginfo_throttle(
            0.5,
            "%s: pos=(%.2f, %.2f) yaw=%.1fdeg dyaw=%.2fdeg ds=%.3fm frame=%s",
            self.label,
            x,
            y,
            math.degrees(yaw),
            dyaw_deg,
            ds,
            msg.header.frame_id,
        )


if __name__ == "__main__":
    try:
        OdomYawMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
