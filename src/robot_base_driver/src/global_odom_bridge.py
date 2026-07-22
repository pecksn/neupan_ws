#!/usr/bin/env python3

import copy
import json
import math
from collections import deque

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Gpgga
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler


class GlobalOdomBridge:
    """Build a continuous global odometry using GPS corrections and local dead reckoning."""

    WAITING = "WAITING_FOR_GPS"
    TRACKING = "GPS_TRACKING"
    DEAD_RECKONING = "LOCAL_DEAD_RECKONING"
    RECOVERING = "GPS_RECOVERING"

    def __init__(self):
        rospy.init_node("global_odom_bridge")

        self.input_topic = rospy.get_param("~input_topic", "/odometry/gps")
        self.local_topic = rospy.get_param("~local_topic", "/odometry/local")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.gga_topic = rospy.get_param("~gga_topic", "/gnss/gpgga")
        self.heading_topic = rospy.get_param("~heading_topic", "/heading")
        self.output_topic = rospy.get_param("~output_topic", "/odometry/global")
        self.status_topic = rospy.get_param("~status_topic", "/gps_guard/status")
        self.safety_topic = rospy.get_param("~safety_topic", "/localization_safe")
        self.output_frame = rospy.get_param("~output_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "base_link")
        self.use_heading_orientation = bool(rospy.get_param("~use_heading_orientation", True))
        self.heading_filter_window_sec = max(
            0.0, float(rospy.get_param("~heading_filter_window_sec", 10.0))
        )
        self.guard_enabled = bool(rospy.get_param("~guard_enabled", True))
        self.print_status_to_screen = bool(rospy.get_param("~print_status_to_screen", False))

        self.publish_rate = max(1.0, float(rospy.get_param("~publish_rate", 30.0)))
        self.data_timeout_sec = max(0.1, float(rospy.get_param("~data_timeout_sec", 1.0)))
        self.heading_timeout_sec = max(0.1, float(rospy.get_param("~heading_timeout_sec", 0.5)))
        self.min_fix_status = int(rospy.get_param("~min_fix_status", 2))
        self.required_gga_quality = int(rospy.get_param("~required_gga_quality", 4))
        self.min_satellites = max(0, int(rospy.get_param("~min_satellites", 8)))
        self.max_hdop = max(0.0, float(rospy.get_param("~max_hdop", 1.5)))
        self.max_position_covariance = max(
            0.0, float(rospy.get_param("~max_position_covariance", 0.25))
        )
        self.heading_jump_limit_deg = max(
            0.0, float(rospy.get_param("~heading_jump_limit_deg", 8.0))
        )
        self.heading_jump_hold_sec = max(
            0.0, float(rospy.get_param("~heading_jump_hold_sec", 1.0))
        )
        self.max_tracking_position_error = max(
            0.1, float(rospy.get_param("~max_tracking_position_error", 0.5))
        )
        self.max_tracking_candidate_step = max(
            0.0, float(rospy.get_param("~max_tracking_candidate_step", 0.35))
        )
        self.max_tracking_yaw_error_rad = math.radians(
            max(1.0, float(rospy.get_param("~max_tracking_yaw_error_deg", 20.0)))
        )
        self.recovery_stable_sec = max(
            0.0, float(rospy.get_param("~recovery_stable_sec", 4.0))
        )
        self.initial_stable_sec = max(
            0.0, float(rospy.get_param("~initial_stable_sec", 2.0))
        )
        self.recovery_consistency_m = max(
            0.05, float(rospy.get_param("~recovery_consistency_m", 0.75))
        )
        self.recovery_consistency_yaw_rad = math.radians(
            max(1.0, float(rospy.get_param("~recovery_consistency_yaw_deg", 8.0)))
        )
        self.recovery_position_rate = max(
            0.01, float(rospy.get_param("~recovery_position_rate", 0.5))
        )
        self.recovery_yaw_rate = math.radians(
            max(0.1, float(rospy.get_param("~recovery_yaw_rate_deg", 8.0)))
        )
        self.tracking_position_rate = max(
            self.recovery_position_rate,
            float(rospy.get_param("~tracking_position_rate", 2.0)),
        )
        self.tracking_yaw_rate = math.radians(
            max(1.0, float(rospy.get_param("~tracking_yaw_rate_deg", 30.0)))
        )
        self.local_max_speed = max(0.1, float(rospy.get_param("~local_max_speed", 2.0)))
        self.local_max_lateral_speed = max(
            0.1, float(rospy.get_param("~local_max_lateral_speed", 0.8))
        )
        self.local_max_yaw_rate = math.radians(
            max(1.0, float(rospy.get_param("~local_max_yaw_rate_deg", 90.0)))
        )
        self.local_fault_hold_sec = max(
            0.0, float(rospy.get_param("~local_fault_hold_sec", 2.0))
        )

        self.last_gps_odom = None
        self.last_gps_recv = None
        self.last_local_odom = None
        self.last_local_recv = None
        self.last_fix = None
        self.last_fix_recv = None
        self.last_gga = None
        self.last_gga_recv = None
        self.last_heading = None
        self.last_heading_yaw = None
        self.last_heading_recv = None
        self.heading_bad_until = rospy.Time(0)
        self.local_bad_until = rospy.Time(0)
        self.local_fault_reason = "local_odom_missing"
        self.local_speed_mps = None
        self.local_lateral_speed_mps = None
        self.local_yaw_rate_deg_s = None
        self.previous_local_pose = None
        self.previous_local_recv = None
        self.last_global_pose = None

        # map->odom transform estimated from trustworthy GPS and G90 heading.
        self.transform_x = 0.0
        self.transform_y = 0.0
        self.transform_yaw = 0.0
        self.transform_initialized = False
        self.mode = self.WAITING
        self.reason = "waiting_for_local_gps_fix_heading"
        self.stable_since = None
        self.last_recovery_candidate = None
        self.recovery_candidates = deque()
        self.heading_transform_history = deque()
        self.last_processed_gps_recv = None
        self.last_correction_time = rospy.Time.now()
        self.last_status_publish = rospy.Time(0)
        self.last_position_error = None
        self.last_yaw_error_deg = None
        self.last_tracking_candidate = None
        self.last_candidate_step_m = None

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=2, latch=True)
        self.safety_pub = rospy.Publisher(self.safety_topic, Bool, queue_size=2, latch=True)
        rospy.Subscriber(self.input_topic, Odometry, self.gps_odom_cb, queue_size=10)
        rospy.Subscriber(self.local_topic, Odometry, self.local_odom_cb, queue_size=30)
        rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_cb, queue_size=10)
        rospy.Subscriber(self.gga_topic, Gpgga, self.gga_cb, queue_size=10)
        if self.use_heading_orientation:
            rospy.Subscriber(self.heading_topic, QuaternionStamped, self.heading_cb, queue_size=20)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_cb)
        rospy.loginfo(
            "global_odom_bridge GPS guard: gps=%s fix=%s gga=%s heading=%s local=%s -> %s "
            "(GGA=%d sats>=%d HDOP<=%.1f status>=%d cov<=%.3fm2 recovery=%.1fs)",
            self.input_topic,
            self.fix_topic,
            self.gga_topic,
            self.heading_topic,
            self.local_topic,
            self.output_topic,
            self.required_gga_quality,
            self.min_satellites,
            self.max_hdop,
            self.min_fix_status,
            self.max_position_covariance,
            self.recovery_stable_sec,
        )

    @staticmethod
    def finite(value):
        return math.isfinite(value)

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def gps_odom_cb(self, msg):
        self.last_gps_odom = msg
        self.last_gps_recv = rospy.Time.now()

    def local_odom_cb(self, msg):
        now = rospy.Time.now()
        pose = msg.pose.pose
        if self.previous_local_pose is not None and self.previous_local_recv is not None:
            dt = (now - self.previous_local_recv).to_sec()
            if 0.005 <= dt <= self.data_timeout_sec:
                dx = pose.position.x - self.previous_local_pose.position.x
                dy = pose.position.y - self.previous_local_pose.position.y
                distance = math.hypot(dx, dy)
                previous_yaw = self.yaw_from_quaternion(self.previous_local_pose.orientation)
                yaw = self.yaw_from_quaternion(pose.orientation)
                yaw_rate = abs(self.normalize_angle(yaw - previous_yaw)) / dt
                speed = distance / dt
                lateral_speed = abs(-dx * math.sin(previous_yaw) + dy * math.cos(previous_yaw)) / dt
                self.local_speed_mps = speed
                self.local_lateral_speed_mps = lateral_speed
                self.local_yaw_rate_deg_s = math.degrees(yaw_rate)
                reasons = []
                if speed > self.local_max_speed:
                    reasons.append("speed_%.2f" % speed)
                if lateral_speed > self.local_max_lateral_speed:
                    reasons.append("lateral_%.2f" % lateral_speed)
                if yaw_rate > self.local_max_yaw_rate:
                    reasons.append("yaw_rate_%.1f" % math.degrees(yaw_rate))
                if reasons:
                    self.handle_local_fault(now, msg, ",".join(reasons))

        self.previous_local_pose = copy.deepcopy(pose)
        self.previous_local_recv = now
        self.last_local_odom = msg
        self.last_local_recv = now

    def fix_cb(self, msg):
        self.last_fix = msg
        self.last_fix_recv = rospy.Time.now()

    def gga_cb(self, msg):
        self.last_gga = msg
        self.last_gga_recv = rospy.Time.now()

    def handle_local_fault(self, now, new_local_odom, reason):
        self.local_bad_until = now + rospy.Duration(self.local_fault_hold_sec)
        self.local_fault_reason = reason
        self.stable_since = None
        self.recovery_candidates.clear()
        self.heading_transform_history.clear()

        # FAST-LIO can reset its local origin after degeneracy. Preserve the
        # last published global pose while re-anchoring to the new local pose.
        if self.transform_initialized and self.last_global_pose is not None:
            local_pose = new_local_odom.pose.pose
            local_yaw = self.yaw_from_quaternion(local_pose.orientation)
            global_x, global_y, global_yaw = self.last_global_pose
            self.transform_yaw = self.normalize_angle(global_yaw - local_yaw)
            c = math.cos(self.transform_yaw)
            s = math.sin(self.transform_yaw)
            self.transform_x = global_x - (
                c * local_pose.position.x - s * local_pose.position.y
            )
            self.transform_y = global_y - (
                s * local_pose.position.x + c * local_pose.position.y
            )

        rospy.logerr_throttle(
            1.0,
            "Localization safety stop: local odometry jump detected (%s)",
            reason,
        )
        self.safety_pub.publish(Bool(data=False))

    def heading_cb(self, msg):
        q = msg.quaternion
        if not all(self.finite(v) for v in (q.x, q.y, q.z, q.w)):
            return
        now = rospy.Time.now()
        yaw = self.yaw_from_quaternion(q)
        if self.last_heading_yaw is not None:
            jump = abs(self.normalize_angle(yaw - self.last_heading_yaw))
            if jump > math.radians(self.heading_jump_limit_deg):
                self.heading_bad_until = now + rospy.Duration(self.heading_jump_hold_sec)
                rospy.logwarn_throttle(
                    1.0,
                    "GPS guard rejected heading jump %.1f deg",
                    math.degrees(jump),
                )
        self.last_heading = q
        self.last_heading_yaw = yaw
        self.last_heading_recv = now

    def data_quality(self, now):
        if self.last_local_odom is None or self.last_local_recv is None:
            return False, "local_odom_missing"
        if (now - self.last_local_recv).to_sec() > self.data_timeout_sec:
            return False, "local_odom_stale"
        if now < self.local_bad_until:
            return False, "local_odom_jump_{}".format(self.local_fault_reason)
        if self.last_gps_odom is None or self.last_gps_recv is None:
            return False, "gps_odom_missing"
        if (now - self.last_gps_recv).to_sec() > self.data_timeout_sec:
            return False, "gps_odom_stale"
        if self.guard_enabled:
            if self.last_fix is None or self.last_fix_recv is None:
                return False, "gps_fix_missing"
            if (now - self.last_fix_recv).to_sec() > self.data_timeout_sec:
                return False, "gps_fix_stale"
            if self.last_fix.status.status < self.min_fix_status:
                return False, "fix_status_{}".format(self.last_fix.status.status)

            cov_x = float(self.last_fix.position_covariance[0])
            cov_y = float(self.last_fix.position_covariance[4])
            if not (self.finite(cov_x) and self.finite(cov_y)):
                return False, "position_covariance_invalid"
            if max(cov_x, cov_y) > self.max_position_covariance:
                return False, "position_covariance_large"
            if self.last_gga is None or self.last_gga_recv is None:
                return False, "gga_missing"
            if (now - self.last_gga_recv).to_sec() > self.data_timeout_sec:
                return False, "gga_stale"
            if (
                self.required_gga_quality > 0
                and int(self.last_gga.gps_qual) != self.required_gga_quality
            ):
                return False, "gga_quality_{}".format(self.last_gga.gps_qual)
            if int(self.last_gga.num_sats) < self.min_satellites:
                return False, "satellites_{}".format(self.last_gga.num_sats)
            if not self.finite(float(self.last_gga.hdop)):
                return False, "hdop_invalid"
            if float(self.last_gga.hdop) > self.max_hdop:
                return False, "hdop_%.2f" % self.last_gga.hdop

        gps_position = self.last_gps_odom.pose.pose.position
        local_position = self.last_local_odom.pose.pose.position
        if not all(
            self.finite(v)
            for v in (gps_position.x, gps_position.y, local_position.x, local_position.y)
        ):
            return False, "position_non_finite"

        if self.use_heading_orientation:
            if self.last_heading_yaw is None or self.last_heading_recv is None:
                return False, "heading_missing"
            if (now - self.last_heading_recv).to_sec() > self.heading_timeout_sec:
                return False, "heading_stale"
            if self.guard_enabled and now < self.heading_bad_until:
                return False, "heading_jump"
        return True, "ok" if self.guard_enabled else "guard_disabled"

    def filtered_heading_transform_yaw(self, now, raw_yaw):
        self.heading_transform_history.append((now.to_sec(), raw_yaw))
        cutoff = now.to_sec() - self.heading_filter_window_sec
        while (
            self.heading_transform_history
            and self.heading_transform_history[0][0] < cutoff
        ):
            self.heading_transform_history.popleft()

        if self.heading_filter_window_sec <= 0.0:
            return raw_yaw

        sin_sum = sum(math.sin(entry[1]) for entry in self.heading_transform_history)
        cos_sum = sum(math.cos(entry[1]) for entry in self.heading_transform_history)
        if abs(sin_sum) < 1e-9 and abs(cos_sum) < 1e-9:
            return raw_yaw
        return math.atan2(sin_sum, cos_sum)

    def discard_latest_heading_candidate(self, now):
        if not self.use_heading_orientation or not self.heading_transform_history:
            return
        if abs(self.heading_transform_history[-1][0] - now.to_sec()) <= 1e-6:
            self.heading_transform_history.pop()

    def candidate_transform(self, now):
        gps_pos = self.last_gps_odom.pose.pose.position
        local_pose = self.last_local_odom.pose.pose
        local_yaw = self.yaw_from_quaternion(local_pose.orientation)
        if self.use_heading_orientation:
            global_yaw = self.last_heading_yaw
            raw_dyaw = self.normalize_angle(global_yaw - local_yaw)
            dyaw = self.filtered_heading_transform_yaw(now, raw_dyaw)
        else:
            global_yaw = self.yaw_from_quaternion(self.last_gps_odom.pose.pose.orientation)
            dyaw = self.normalize_angle(global_yaw - local_yaw)
        c = math.cos(dyaw)
        s = math.sin(dyaw)
        tx = gps_pos.x - (c * local_pose.position.x - s * local_pose.position.y)
        ty = gps_pos.y - (s * local_pose.position.x + c * local_pose.position.y)
        return tx, ty, dyaw

    def transform_error(self, candidate):
        tx, ty, yaw = candidate
        position_error = math.hypot(tx - self.transform_x, ty - self.transform_y)
        yaw_error = self.normalize_angle(yaw - self.transform_yaw)
        return position_error, yaw_error

    def candidates_consistent(self, first, second):
        if first is None or second is None:
            return True
        position_delta = math.hypot(first[0] - second[0], first[1] - second[1])
        yaw_delta = abs(self.normalize_angle(first[2] - second[2]))
        return (
            position_delta <= self.recovery_consistency_m
            and yaw_delta <= self.recovery_consistency_yaw_rad
        )

    def append_recovery_candidate(self, now, candidate):
        self.recovery_candidates.append((now.to_sec(), candidate))
        keep_sec = max(self.initial_stable_sec, self.recovery_stable_sec) + 1.0
        cutoff = now.to_sec() - keep_sec
        while self.recovery_candidates and self.recovery_candidates[0][0] < cutoff:
            self.recovery_candidates.popleft()

    def recovery_window_stable(self, duration_sec):
        if not self.recovery_candidates:
            return False
        first_stamp = self.recovery_candidates[0][0]
        last_stamp = self.recovery_candidates[-1][0]
        if last_stamp - first_stamp < duration_sec:
            return False

        candidates = [entry[1] for entry in self.recovery_candidates]
        mean_x = sum(candidate[0] for candidate in candidates) / len(candidates)
        mean_y = sum(candidate[1] for candidate in candidates) / len(candidates)
        yaw_reference = candidates[0][2]
        max_position_spread = max(
            math.hypot(candidate[0] - mean_x, candidate[1] - mean_y)
            for candidate in candidates
        )
        max_yaw_spread = max(
            abs(self.normalize_angle(candidate[2] - yaw_reference))
            for candidate in candidates
        )
        return (
            max_position_spread <= self.recovery_consistency_m
            and max_yaw_spread <= self.recovery_consistency_yaw_rad
        )

    def move_transform_toward(self, target, dt, position_rate, yaw_rate):
        dx = target[0] - self.transform_x
        dy = target[1] - self.transform_y
        distance = math.hypot(dx, dy)
        max_distance = position_rate * dt
        if distance > max_distance and distance > 1e-9:
            scale = max_distance / distance
            dx *= scale
            dy *= scale
        self.transform_x += dx
        self.transform_y += dy

        yaw_error = self.normalize_angle(target[2] - self.transform_yaw)
        max_yaw = yaw_rate * dt
        yaw_step = max(-max_yaw, min(max_yaw, yaw_error))
        self.transform_yaw = self.normalize_angle(self.transform_yaw + yaw_step)

    def set_mode(self, mode, reason):
        if mode != self.mode:
            if mode == self.DEAD_RECKONING:
                rospy.logwarn("GPS guard: %s -> %s (%s)", self.mode, mode, reason)
            else:
                rospy.loginfo("GPS guard: %s -> %s (%s)", self.mode, mode, reason)
        elif reason != self.reason and self.print_status_to_screen:
            rospy.loginfo("GPS guard: mode=%s reason=%s", mode, reason)
        self.mode = mode
        self.reason = reason

    def update_guard(self, now):
        good, reason = self.data_quality(now)
        if not good:
            self.stable_since = None
            self.last_recovery_candidate = None
            self.recovery_candidates.clear()
            if self.transform_initialized:
                self.set_mode(self.DEAD_RECKONING, reason)
            else:
                self.set_mode(self.WAITING, reason)
            return

        # A GPS sample is corrected once. Reusing an old GPS position with a
        # newer local pose would create a small backward sawtooth between fixes.
        if self.last_processed_gps_recv == self.last_gps_recv:
            return
        self.last_processed_gps_recv = self.last_gps_recv
        correction_dt = max(0.0, min(0.5, (now - self.last_correction_time).to_sec()))
        self.last_correction_time = now

        candidate = self.candidate_transform(now)
        self.append_recovery_candidate(now, candidate)
        if not self.transform_initialized:
            self.last_recovery_candidate = candidate
            if self.recovery_window_stable(self.initial_stable_sec):
                self.transform_x, self.transform_y, self.transform_yaw = candidate
                self.transform_initialized = True
                self.last_tracking_candidate = candidate
                self.last_candidate_step_m = 0.0
                self.stable_since = None
                self.recovery_candidates.clear()
                self.set_mode(self.TRACKING, "gps_initialized")
            else:
                self.set_mode(self.WAITING, "gps_stability_check")
            return

        position_error, yaw_error = self.transform_error(candidate)
        self.last_position_error = position_error
        self.last_yaw_error_deg = math.degrees(yaw_error)

        if self.mode == self.TRACKING:
            if self.last_tracking_candidate is not None:
                self.last_candidate_step_m = math.hypot(
                    candidate[0] - self.last_tracking_candidate[0],
                    candidate[1] - self.last_tracking_candidate[1],
                )
                if (
                    self.max_tracking_candidate_step > 0.0
                    and self.last_candidate_step_m > self.max_tracking_candidate_step
                ):
                    # Reject the whole GPS/G90 sample before it can move map->odom.
                    # The transform remains frozen until the normal recovery window passes.
                    self.discard_latest_heading_candidate(now)
                    self.stable_since = now
                    self.last_recovery_candidate = candidate
                    self.recovery_candidates.clear()
                    self.set_mode(self.DEAD_RECKONING, "gps_candidate_step")
                    return
            else:
                self.last_candidate_step_m = 0.0
            if position_error > self.max_tracking_position_error:
                self.stable_since = now
                self.last_recovery_candidate = candidate
                self.recovery_candidates.clear()
                self.set_mode(self.DEAD_RECKONING, "gps_position_jump")
                return
            if abs(yaw_error) > self.max_tracking_yaw_error_rad:
                self.stable_since = now
                self.last_recovery_candidate = candidate
                self.recovery_candidates.clear()
                self.set_mode(self.DEAD_RECKONING, "gps_heading_inconsistent")
                return
            self.move_transform_toward(
                candidate, correction_dt, self.tracking_position_rate, self.tracking_yaw_rate
            )
            self.last_tracking_candidate = candidate
            self.set_mode(self.TRACKING, "gps_good")
            return

        if self.mode == self.DEAD_RECKONING:
            self.last_recovery_candidate = candidate
            if not self.recovery_window_stable(self.recovery_stable_sec):
                self.set_mode(self.DEAD_RECKONING, "gps_recovery_wait")
                return
            self.set_mode(self.RECOVERING, "gps_stable")

        if self.mode == self.RECOVERING:
            if not self.recovery_window_stable(self.recovery_stable_sec):
                self.set_mode(self.DEAD_RECKONING, "gps_recovery_unstable")
                return
            self.move_transform_toward(
                candidate, correction_dt, self.recovery_position_rate, self.recovery_yaw_rate
            )
            position_error, yaw_error = self.transform_error(candidate)
            self.last_position_error = position_error
            self.last_yaw_error_deg = math.degrees(yaw_error)
            if position_error <= 0.10 and abs(yaw_error) <= math.radians(1.0):
                self.last_tracking_candidate = candidate
                self.last_candidate_step_m = 0.0
                self.stable_since = None
                self.recovery_candidates.clear()
                self.set_mode(self.TRACKING, "gps_recovered")

    def publish_global_odom(self, now):
        if not self.transform_initialized or self.last_local_odom is None:
            return
        local = self.last_local_odom
        local_pose = local.pose.pose
        c = math.cos(self.transform_yaw)
        s = math.sin(self.transform_yaw)

        out = Odometry()
        out.header.stamp = now
        out.header.frame_id = self.output_frame
        out.child_frame_id = self.child_frame if self.child_frame else local.child_frame_id
        out.pose = copy.deepcopy(local.pose)
        out.twist = copy.deepcopy(local.twist)
        out.pose.pose.position.x = self.transform_x + (
            c * local_pose.position.x - s * local_pose.position.y
        )
        out.pose.pose.position.y = self.transform_y + (
            s * local_pose.position.x + c * local_pose.position.y
        )
        local_yaw = self.yaw_from_quaternion(local_pose.orientation)
        global_yaw = self.normalize_angle(self.transform_yaw + local_yaw)
        q = quaternion_from_euler(0.0, 0.0, global_yaw)
        out.pose.pose.orientation.x = q[0]
        out.pose.pose.orientation.y = q[1]
        out.pose.pose.orientation.z = q[2]
        out.pose.pose.orientation.w = q[3]
        self.last_global_pose = (
            out.pose.pose.position.x,
            out.pose.pose.position.y,
            global_yaw,
        )
        self.pub.publish(out)

    def localization_is_safe(self, now):
        return bool(
            self.transform_initialized
            and self.last_local_recv is not None
            and (now - self.last_local_recv).to_sec() <= self.data_timeout_sec
            and now >= self.local_bad_until
        )

    def publish_status(self, now):
        fix_status = None if self.last_fix is None else int(self.last_fix.status.status)
        cov_x = None if self.last_fix is None else float(self.last_fix.position_covariance[0])
        cov_y = None if self.last_fix is None else float(self.last_fix.position_covariance[4])
        gga_quality = None if self.last_gga is None else int(self.last_gga.gps_qual)
        satellites = None if self.last_gga is None else int(self.last_gga.num_sats)
        hdop = None if self.last_gga is None else float(self.last_gga.hdop)
        localization_safe = self.localization_is_safe(now)
        payload = {
            "stamp": now.to_sec(),
            "mode": self.mode,
            "reason": self.reason,
            "initialized": self.transform_initialized,
            "fix_status": fix_status,
            "cov_x": cov_x,
            "cov_y": cov_y,
            "gga_quality": gga_quality,
            "satellites": satellites,
            "hdop": hdop,
            "localization_safe": localization_safe,
            "local_fault_reason": self.local_fault_reason,
            "local_speed_mps": self.local_speed_mps,
            "local_lateral_speed_mps": self.local_lateral_speed_mps,
            "local_yaw_rate_deg_s": self.local_yaw_rate_deg_s,
            "position_error_m": self.last_position_error,
            "yaw_error_deg": self.last_yaw_error_deg,
            "candidate_step_m": self.last_candidate_step_m,
            "max_tracking_candidate_step_m": self.max_tracking_candidate_step,
            "transform_x": self.transform_x if self.transform_initialized else None,
            "transform_y": self.transform_y if self.transform_initialized else None,
            "transform_yaw_deg": math.degrees(self.transform_yaw)
            if self.transform_initialized
            else None,
            "heading_filter_window_sec": self.heading_filter_window_sec,
            "heading_filter_samples": len(self.heading_transform_history),
            "yaw_frozen": bool(self.transform_initialized and self.mode == self.DEAD_RECKONING),
        }
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))
        if self.print_status_to_screen:
            rospy.loginfo_throttle(
                1.0,
                "GPS guard mode=%s reason=%s GGA=%s sats=%s HDOP=%s fix=%s cov=[%s,%s] "
                "local_safe=%s correction_error=[%s m,%s deg]",
                self.mode,
                self.reason,
                "none" if gga_quality is None else str(gga_quality),
                "none" if satellites is None else str(satellites),
                "none" if hdop is None else "%.2f" % hdop,
                "none" if fix_status is None else str(fix_status),
                "none" if cov_x is None else "%.3f" % cov_x,
                "none" if cov_y is None else "%.3f" % cov_y,
                localization_safe,
                "none" if self.last_position_error is None else "%.2f" % self.last_position_error,
                "none" if self.last_yaw_error_deg is None else "%.1f" % self.last_yaw_error_deg,
            )

    def timer_cb(self, _event):
        now = rospy.Time.now()
        self.update_guard(now)
        self.publish_global_odom(now)
        self.safety_pub.publish(Bool(data=self.localization_is_safe(now)))
        if (now - self.last_status_publish).to_sec() >= 0.5:
            self.last_status_publish = now
            self.publish_status(now)


if __name__ == "__main__":
    try:
        GlobalOdomBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
