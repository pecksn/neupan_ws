#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import QuaternionStamped
from tf.transformations import euler_from_quaternion


class QuaternionYawMonitor:
    def __init__(self):
        rospy.init_node("quaternion_yaw_monitor")
        self.topic = rospy.get_param("~topic", "/heading")
        self.label = rospy.get_param("~label", self.topic)
        self.last_yaw = None
        rospy.Subscriber(self.topic, QuaternionStamped, self.cb, queue_size=10)
        rospy.loginfo("quaternion_yaw_monitor: topic=%s label=%s", self.topic, self.label)

    def cb(self, msg):
        q = msg.quaternion
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        dyaw_deg = 0.0 if self.last_yaw is None else math.degrees(
            math.atan2(math.sin(yaw - self.last_yaw), math.cos(yaw - self.last_yaw))
        )
        self.last_yaw = yaw
        rospy.loginfo_throttle(
            0.5,
            "%s: yaw=%.1fdeg dyaw=%.2fdeg frame=%s",
            self.label,
            math.degrees(yaw),
            dyaw_deg,
            msg.header.frame_id,
        )


if __name__ == "__main__":
    try:
        QuaternionYawMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
