#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry


class GlobalOdomBridge:
    def __init__(self):
        rospy.init_node("global_odom_bridge")

        self.input_topic = rospy.get_param("~input_topic", "/odometry/gps")
        self.output_topic = rospy.get_param("~output_topic", "/odometry/global")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "base_link")
        self.heading_topic = rospy.get_param("~heading_topic", "/heading")
        self.use_heading_orientation = bool(rospy.get_param("~use_heading_orientation", False))
        self.heading_timeout_sec = float(rospy.get_param("~heading_timeout_sec", 0.5))

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(self.input_topic, Odometry, self.cb, queue_size=10)
        self.heading_sub = None
        self.last_heading = None
        self.last_heading_stamp = None
        if self.use_heading_orientation:
            self.heading_sub = rospy.Subscriber(
                self.heading_topic, QuaternionStamped, self.heading_cb, queue_size=10
            )

        rospy.loginfo(
            "global_odom_bridge: %s -> %s (frame %s, use_heading_orientation=%s, heading_topic=%s)",
            self.input_topic,
            self.output_topic,
            self.output_frame,
            self.use_heading_orientation,
            self.heading_topic,
        )

    def _finite(self, v):
        return math.isfinite(v)

    def heading_cb(self, msg):
        q = msg.quaternion
        if not all(self._finite(v) for v in (q.x, q.y, q.z, q.w)):
            rospy.logwarn_throttle(1.0, "global_odom_bridge got non-finite heading quaternion, drop")
            return
        self.last_heading = q
        self.last_heading_stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

    def _heading_is_fresh(self, now):
        if self.last_heading is None or self.last_heading_stamp is None:
            return False
        if self.heading_timeout_sec <= 0.0:
            return True
        return (now - self.last_heading_stamp).to_sec() <= self.heading_timeout_sec

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

        if self.use_heading_orientation:
            now = out.header.stamp if out.header.stamp != rospy.Time() else rospy.Time.now()
            if self._heading_is_fresh(now):
                out.pose.pose.orientation = self.last_heading
            else:
                rospy.logwarn_throttle(
                    1.0,
                    "global_odom_bridge heading unavailable/stale, keep /odometry/gps orientation"
                )

        self.pub.publish(out)


if __name__ == "__main__":
    try:
        GlobalOdomBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
