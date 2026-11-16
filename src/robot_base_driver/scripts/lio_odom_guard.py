#!/usr/bin/env python3
# Module contract: Guard local LiDAR odometry against discontinuities using local-motion
# evidence. Corrections preserve continuity; they do not prove absolute map position.


import copy
import json
import math

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler


class LioOdomGuard:
    """Reject impossible FAST-LIO motion before it can contaminate the local EKF."""

    TRACKING = "TRACKING"
    RECOVERING = "RECOVERING"

    # Configure local-motion checks and recovery limits, then initialize continuity
    # offsets and LiDAR/wheel odometry I/O.
    def __init__(self):
        rospy.init_node("lio_odom_guard")

        self.input_topic = rospy.get_param("~input_topic", "/Odometry")
        self.wheel_topic = rospy.get_param("~wheel_topic", "/odom")
        self.output_topic = rospy.get_param("~output_topic", "/odometry/lio_guarded")
        self.status_topic = rospy.get_param("~status_topic", "/lio_guard/status")
        self.healthy_topic = rospy.get_param("~healthy_topic", "/lio_guard/healthy")

        self.max_speed = max(0.1, float(rospy.get_param("~max_speed", 2.0)))
        self.max_lateral_speed = max(
            0.1, float(rospy.get_param("~max_lateral_speed", 0.8))
        )
        self.max_yaw_rate = math.radians(
            max(1.0, float(rospy.get_param("~max_yaw_rate_deg", 90.0)))
        )
        self.max_sample_gap = max(0.1, float(rospy.get_param("~max_sample_gap", 0.5)))
        self.recovery_stable_sec = max(
            0.0, float(rospy.get_param("~recovery_stable_sec", 2.0))
        )
        self.status_period = max(0.1, float(rospy.get_param("~status_period", 0.5)))
        self.mode = self.TRACKING
        self.reason = "waiting_for_lio"
        self.last_raw_pose = None
        self.last_raw_time = None
        self.last_output_pose = None
        self.last_wheel_pose = None
        self.wheel_anchor = None
        self.output_anchor = None
        self.stable_since = None

        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_yaw = 0.0
        self.offset_initialized = False

        self.speed = None
        self.lateral_speed = None
        self.yaw_rate = None
        self.sample_dt = None
        self.rejected_samples = 0
        self.published_samples = 0

        self.output_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=2, latch=True
        )
        self.healthy_pub = rospy.Publisher(
            self.healthy_topic, Bool, queue_size=2, latch=True
        )
        rospy.Subscriber(self.input_topic, Odometry, self.lio_cb, queue_size=50)
        rospy.Subscriber(self.wheel_topic, Odometry, self.wheel_cb, queue_size=50)
        rospy.Timer(rospy.Duration(self.status_period), self.status_timer)

        rospy.loginfo(
            "lio_odom_guard: %s -> %s (speed<=%.1fm/s lateral<=%.1fm/s yaw_rate<=%.0fdeg/s)",
            self.input_topic,
            self.output_topic,
            self.max_speed,
            self.max_lateral_speed,
            math.degrees(self.max_yaw_rate),
        )

    # Wrap a heading difference into the signed-pi interval before comparing
    # orientations.
    # Inputs: angle.
    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    # Extract planar yaw in radians from a ROS quaternion; callers remain responsible
    # for validating measurement provenance.
    # Inputs: q.
    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    # Extract the planar pose representation from the supplied ROS odometry message.
    # Inputs: msg.
    @classmethod
    def pose_from_odom(cls, msg):
        pose = msg.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            return None
        return (
            pose.position.x,
            pose.position.y,
            cls.yaw_from_quaternion(pose.orientation),
        )

    # Retain wheel-odometry evidence for local-motion consistency checks.
    # Inputs: msg.
    def wheel_cb(self, msg):
        pose = self.pose_from_odom(msg)
        if pose is not None:
            self.last_wheel_pose = pose

    # Evaluate a pose transition against the configured local-motion limits.
    # Inputs: pose, sample_time.
    def assess_motion(self, pose, sample_time):
        self.speed = None
        self.lateral_speed = None
        self.yaw_rate = None
        self.sample_dt = None
        if self.last_raw_pose is None or self.last_raw_time is None:
            return True, "first_sample"

        dt = (sample_time - self.last_raw_time).to_sec()
        self.sample_dt = dt
        if dt <= 0.001:
            return False, "non_monotonic_time"
        if dt > self.max_sample_gap:
            return False, "sample_gap_{:.2f}s".format(dt)

        dx = pose[0] - self.last_raw_pose[0]
        dy = pose[1] - self.last_raw_pose[1]
        previous_yaw = self.last_raw_pose[2]
        lateral_delta = -math.sin(previous_yaw) * dx + math.cos(previous_yaw) * dy
        self.speed = math.hypot(dx, dy) / dt
        self.lateral_speed = abs(lateral_delta) / dt
        self.yaw_rate = abs(self.normalize_angle(pose[2] - previous_yaw)) / dt

        reasons = []
        if self.speed > self.max_speed:
            reasons.append("speed_{:.2f}".format(self.speed))
        if self.lateral_speed > self.max_lateral_speed:
            reasons.append("lateral_{:.2f}".format(self.lateral_speed))
        if self.yaw_rate > self.max_yaw_rate:
            reasons.append("yaw_rate_{:.1f}deg_s".format(math.degrees(self.yaw_rate)))
        return not reasons, "ok" if not reasons else "+".join(reasons)

    # Apply the retained planar continuity offset to an odometry pose.
    # Inputs: raw_pose.
    def apply_offset(self, raw_pose):
        c = math.cos(self.offset_yaw)
        s = math.sin(self.offset_yaw)
        return (
            c * raw_pose[0] - s * raw_pose[1] + self.offset_x,
            s * raw_pose[0] + c * raw_pose[1] + self.offset_y,
            self.normalize_angle(raw_pose[2] + self.offset_yaw),
        )

    # Construct the wheel-supported target used by the existing continuity-recovery
    # logic.
    def wheel_extrapolated_target(self):
        if self.output_anchor is None:
            return self.last_output_pose
        if self.wheel_anchor is None or self.last_wheel_pose is None:
            return self.output_anchor

        wx0, wy0, wyaw0 = self.wheel_anchor
        wx1, wy1, wyaw1 = self.last_wheel_pose
        dx_world = wx1 - wx0
        dy_world = wy1 - wy0
        dx_local = math.cos(wyaw0) * dx_world + math.sin(wyaw0) * dy_world
        dy_local = -math.sin(wyaw0) * dx_world + math.cos(wyaw0) * dy_world

        ox, oy, oyaw = self.output_anchor
        return (
            ox + math.cos(oyaw) * dx_local - math.sin(oyaw) * dy_local,
            oy + math.sin(oyaw) * dx_local + math.cos(oyaw) * dy_local,
            self.normalize_angle(oyaw + self.normalize_angle(wyaw1 - wyaw0)),
        )

    # Update the continuity offset from the observed local pose and selected target.
    # Inputs: raw_pose, target_pose.
    def align_offset(self, raw_pose, target_pose):
        self.offset_yaw = self.normalize_angle(target_pose[2] - raw_pose[2])
        c = math.cos(self.offset_yaw)
        s = math.sin(self.offset_yaw)
        self.offset_x = target_pose[0] - (c * raw_pose[0] - s * raw_pose[1])
        self.offset_y = target_pose[1] - (s * raw_pose[0] + c * raw_pose[1])
        self.offset_initialized = True

    # Publish local odometry with the currently selected continuity transform.
    # Inputs: msg, output_pose.
    def publish_corrected(self, msg, output_pose):
        out = copy.deepcopy(msg)
        out.pose.pose.position.x = output_pose[0]
        out.pose.pose.position.y = output_pose[1]
        q = quaternion_from_euler(0.0, 0.0, output_pose[2])
        out.pose.pose.orientation.x = q[0]
        out.pose.pose.orientation.y = q[1]
        out.pose.pose.orientation.z = q[2]
        out.pose.pose.orientation.w = q[3]
        self.output_pub.publish(out)
        self.last_output_pose = output_pose
        self.output_anchor = output_pose
        self.wheel_anchor = self.last_wheel_pose
        self.published_samples += 1

    # Update guard state and its diagnostic transition reason.
    # Inputs: mode, reason, now.
    def set_mode(self, mode, reason, now):
        changed = mode != self.mode or reason != self.reason
        self.mode = mode
        self.reason = reason
        if not changed:
            return
        if mode == self.TRACKING:
            rospy.loginfo("LIO guard recovered: %s", reason)
        else:
            rospy.logwarn("LIO guard rejected FAST-LIO: %s", reason)

    # Evaluate one LiDAR odometry update, handle faults, and publish the supported
    # corrected output.
    # Inputs: msg.
    def lio_cb(self, msg):
        now = rospy.Time.now()
        sample_time = msg.header.stamp if msg.header.stamp != rospy.Time() else now
        pose = self.pose_from_odom(msg)
        if pose is None:
            self.rejected_samples += 1
            self.stable_since = None
            self.set_mode(self.RECOVERING, "non_finite_pose", now)
            return

        valid, reason = self.assess_motion(pose, sample_time)
        self.last_raw_pose = pose
        self.last_raw_time = sample_time

        if not valid:
            self.rejected_samples += 1
            self.stable_since = None
            self.set_mode(self.RECOVERING, reason, now)
            return

        if self.mode == self.RECOVERING:
            if self.stable_since is None:
                self.stable_since = now
                self.reason = "waiting_stable_lio"
            stable_duration = (now - self.stable_since).to_sec()
            if stable_duration < self.recovery_stable_sec:
                self.rejected_samples += 1
                return

            target = self.wheel_extrapolated_target()
            if target is None:
                target = pose
            self.align_offset(pose, target)
            self.set_mode(self.TRACKING, "stable_after_fault", now)

        if not self.offset_initialized:
            self.offset_initialized = True
        output_pose = self.apply_offset(pose)
        self.publish_corrected(msg, output_pose)
        if self.reason in ("waiting_for_lio", "first_sample"):
            self.set_mode(self.TRACKING, "lio_ok", now)

    # Build the local-odometry guard's status representation.
    # Inputs: now.
    def status_payload(self, now):
        stable_sec = None
        if self.stable_since is not None:
            stable_sec = max(0.0, (now - self.stable_since).to_sec())
        payload = {
            "stamp": now.to_sec(),
            "mode": self.mode,
            "reason": self.reason,
            "healthy": self.mode == self.TRACKING and self.last_output_pose is not None,
            "sample_dt": self.sample_dt,
            "speed_mps": self.speed,
            "lateral_speed_mps": self.lateral_speed,
            "yaw_rate_deg_s": (
                None if self.yaw_rate is None else math.degrees(self.yaw_rate)
            ),
            "stable_sec": stable_sec,
            "rejected_samples": self.rejected_samples,
            "published_samples": self.published_samples,
            "offset_x": self.offset_x,
            "offset_y": self.offset_y,
            "offset_yaw_deg": math.degrees(self.offset_yaw),
        }
        for prefix, pose in (
            ("raw_lio", self.last_raw_pose),
            ("guarded_lio", self.last_output_pose),
            ("wheel_odom", self.last_wheel_pose),
        ):
            payload[prefix + "_x"] = None if pose is None else pose[0]
            payload[prefix + "_y"] = None if pose is None else pose[1]
            payload[prefix + "_yaw_deg"] = (
                None if pose is None else math.degrees(pose[2])
            )
        return payload

    # Publish current local-odometry health and reason fields on the diagnostic timer.
    # Inputs: _event.
    def status_timer(self, _event):
        now = rospy.Time.now()
        payload = self.status_payload(now)
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        self.healthy_pub.publish(Bool(data=payload["healthy"]))


if __name__ == "__main__":
    try:
        LioOdomGuard()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
