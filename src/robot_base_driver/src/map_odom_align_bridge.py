#!/usr/bin/env python3

import math

import rospy
import tf
from nav_msgs.msg import Odometry


class MapOdomAlignBridge:
    def __init__(self):
        rospy.init_node("map_odom_align_bridge")

        self.global_topic = rospy.get_param("~global_topic", "/odometry/global")
        self.local_topic = rospy.get_param("~local_topic", "/odometry/local")
        self.parent_frame = rospy.get_param("~parent_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "odom")
        self.timeout_sec = float(rospy.get_param("~timeout_sec", 2.0))
        self.yaw_bias_deg = float(rospy.get_param("~yaw_bias_deg", 0.0))
        self.yaw_bias_rad = math.radians(self.yaw_bias_deg)
        self.align_yaw_with_global = bool(rospy.get_param("~align_yaw_with_global", False))
        self.print_yaw_status = bool(rospy.get_param("~print_yaw_status", False))

        self.last_global = None
        self.last_local = None
        self.last_global_recv_time = None
        self.last_local_recv_time = None

        self.br = tf.TransformBroadcaster()
        self.global_sub = rospy.Subscriber(self.global_topic, Odometry, self._global_cb, queue_size=20)
        self.local_sub = rospy.Subscriber(self.local_topic, Odometry, self._local_cb, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.02), self._timer_cb)

        rospy.loginfo(
            "map_odom_align_bridge: %s + %s -> TF %s -> %s (yaw_bias=%.1f deg, align_yaw_with_global=%s)",
            self.global_topic,
            self.local_topic,
            self.parent_frame,
            self.child_frame,
            self.yaw_bias_deg,
            self.align_yaw_with_global,
        )

    @staticmethod
    def _yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _global_cb(self, msg):
        self.last_global = msg
        self.last_global_recv_time = rospy.Time.now()

    def _local_cb(self, msg):
        self.last_local = msg
        self.last_local_recv_time = rospy.Time.now()

    @staticmethod
    def _is_finite(v):
        return math.isfinite(v)

    @staticmethod
    def _normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _timer_cb(self, _event):
        if (
            self.last_global is None
            or self.last_local is None
            or self.last_global_recv_time is None
            or self.last_local_recv_time is None
        ):
            return

        now = rospy.Time.now()
        if (now - self.last_global_recv_time).to_sec() > self.timeout_sec:
            rospy.logwarn_throttle(1.0, "map_odom_align_bridge global odom timeout")
        if (now - self.last_local_recv_time).to_sec() > self.timeout_sec:
            rospy.logwarn_throttle(1.0, "map_odom_align_bridge local odom timeout")

        # 即便消息短时超时，也继续发布最近一次可用的 map->odom，避免 TF 链断裂。

        gx = self.last_global.pose.pose.position.x
        gy = self.last_global.pose.pose.position.y
        lx = self.last_local.pose.pose.position.x
        ly = self.last_local.pose.pose.position.y
        if not (self._is_finite(gx) and self._is_finite(gy) and self._is_finite(lx) and self._is_finite(ly)):
            rospy.logwarn_throttle(1.0, "map_odom_align_bridge got non-finite odom pose")
            return

        gyaw = self._yaw_from_quat(self.last_global.pose.pose.orientation)
        lyaw = self._yaw_from_quat(self.last_local.pose.pose.orientation)

        # GPS odometry heading is often invalid/default in stable mode, so keep
        # map->odom yaw fixed unless explicitly asked to align to global yaw.
        raw_dyaw = gyaw - lyaw
        if self.align_yaw_with_global:
            dyaw = self._normalize_angle(raw_dyaw + self.yaw_bias_rad)
            yaw_mode = "global"
        else:
            dyaw = self._normalize_angle(self.yaw_bias_rad)
            yaw_mode = "local"
        if self.print_yaw_status:
            rospy.loginfo_throttle(
                1.0,
                "map_odom_align_bridge yaw raw=%.1f deg, bias=%.1f deg, used=%.1f deg mode=%s global=%.1f local=%.1f",
                math.degrees(raw_dyaw),
                self.yaw_bias_deg,
                math.degrees(dyaw),
                yaw_mode,
                math.degrees(gyaw),
                math.degrees(lyaw),
            )
        c = math.cos(dyaw)
        s = math.sin(dyaw)
        tx = gx - (c * lx - s * ly)
        ty = gy - (s * lx + c * ly)

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, dyaw)
        self.br.sendTransform((tx, ty, 0.0), q, now, self.child_frame, self.parent_frame)


if __name__ == "__main__":
    try:
        MapOdomAlignBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
