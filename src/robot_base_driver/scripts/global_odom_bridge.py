#!/usr/bin/env python3

import copy
import json
import math
import statistics
from collections import deque

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Gpgga
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler


class LocalMotionGuard:
    """Judge pose motion using source timestamps, not callback scheduling."""

    def __init__(self, max_speed, max_lateral, max_yaw_rate,
                 window_sec=0.2, confirm_sec=0.1, max_gap_sec=0.5,
                 position_jump_m=0.35, yaw_jump_rad=math.radians(15)):
        self.max_speed = max_speed
        self.max_lateral = max_lateral
        self.max_yaw_rate = max_yaw_rate
        self.window = window_sec
        self.confirm = confirm_sec
        self.max_gap = max_gap_sec
        self.position_jump = position_jump_m
        self.yaw_jump = yaw_jump_rad
        self.history = deque()
        self.suspect_since = None

    @staticmethod
    def angle(value):
        return math.atan2(math.sin(value), math.cos(value))

    def observe(self, stamp, pose):
        result = {"accepted": False, "fault": None, "candidate_reason": None,
                  "window_speed_mps": None, "window_lateral_speed_mps": None,
                  "window_yaw_rate_deg_s": None}
        if not all(math.isfinite(v) for v in (stamp,) + pose) or stamp <= 0:
            result["candidate_reason"] = "invalid_pose_or_timestamp"
            return result
        if self.history and stamp <= self.history[-1][0]:
            result["candidate_reason"] = "non_monotonic_timestamp"
            return result
        result["accepted"] = True
        if self.history:
            old_stamp, old_pose = self.history[-1]
            dt = stamp - old_stamp
            dx, dy = pose[0] - old_pose[0], pose[1] - old_pose[1]
            distance = math.hypot(dx, dy)
            yaw_delta = abs(self.angle(pose[2] - old_pose[2]))
            result.update(message_dt_sec=dt, dx_m=dx, dy_m=dy,
                          distance_m=distance, message_speed_mps=distance / dt)
            # Preserve an immediate stop for a discontinuity too large for
            # normal motion, even if the source interval is unusually short.
            if dt > self.max_gap:
                result["fault"] = "message_gap_%.3fs" % dt
            elif distance > max(self.position_jump, self.max_speed * dt):
                result["fault"] = "position_jump_%.3fm" % distance
            elif yaw_delta > max(self.yaw_jump, self.max_yaw_rate * dt):
                result["fault"] = "yaw_jump_%.1fdeg" % math.degrees(yaw_delta)
            if result["fault"]:
                self.history.clear()
                self.suspect_since = None
                self.history.append((stamp, pose))
                return result
        self.history.append((stamp, pose))
        while len(self.history) > 2 and stamp - self.history[1][0] >= self.window:
            self.history.popleft()
        old_stamp, old_pose = self.history[0]
        dt = stamp - old_stamp
        if dt + 1e-9 < self.window:
            return result
        dx, dy = pose[0] - old_pose[0], pose[1] - old_pose[1]
        speed = math.hypot(dx, dy) / dt
        lateral = abs(-dx * math.sin(old_pose[2]) + dy * math.cos(old_pose[2])) / dt
        yaw_rate = abs(self.angle(pose[2] - old_pose[2])) / dt
        result.update(window_dt_sec=dt, window_speed_mps=speed,
                      window_lateral_speed_mps=lateral,
                      window_yaw_rate_deg_s=math.degrees(yaw_rate))
        reasons = []
        if speed > self.max_speed:
            reasons.append("speed_%.2f" % speed)
        if lateral > self.max_lateral:
            reasons.append("lateral_%.2f" % lateral)
        if yaw_rate > self.max_yaw_rate:
            reasons.append("yaw_rate_%.1f" % math.degrees(yaw_rate))
        if reasons:
            result["candidate_reason"] = ",".join(reasons)
            if self.suspect_since is None:
                self.suspect_since = stamp
            if stamp - self.suspect_since + 1e-9 >= self.confirm:
                result["fault"] = "window_" + result["candidate_reason"]
                self.history.clear()
                self.history.append((stamp, pose))
                self.suspect_since = None
        else:
            self.suspect_since = None
        return result


class DeadReckoningBudget:
    """Bound uncorrected travel/time, not an estimate of actual pose error."""

    def __init__(self, max_distance_m, max_duration_sec):
        self.max_distance_m = max(0.1, float(max_distance_m))
        self.max_duration_sec = max(1.0, float(max_duration_sec))
        self.reset()

    def reset(self):
        self.started_at = None
        self.last_xy = None
        self.distance_m = 0.0
        self.limit_reason = None

    def begin(self, now_sec, xy=None):
        if self.started_at is None:
            self.started_at = now_sec
            self.last_xy = xy

    def observe(self, xy):
        if self.started_at is None:
            return
        if self.last_xy is not None:
            self.distance_m += math.hypot(xy[0] - self.last_xy[0], xy[1] - self.last_xy[1])
        self.last_xy = xy

    def elapsed_sec(self, now_sec):
        return 0.0 if self.started_at is None else max(0.0, now_sec - self.started_at)

    def check(self, now_sec):
        if self.started_at is None:
            return None
        if self.limit_reason is None:
            if self.distance_m >= self.max_distance_m:
                self.limit_reason = "dead_reckoning_distance_limit"
            elif self.elapsed_sec(now_sec) >= self.max_duration_sec:
                self.limit_reason = "dead_reckoning_time_limit"
        return self.limit_reason


class RuntimePositionWindow:
    """Check independent GPS position anchors against local odometry motion.

    Candidates are map->odom translations computed with the existing yaw.
    A fresh, consistent window is evidence for a coarse position correction,
    not a claim of RTK accuracy or independently verified heading.
    """
    def __init__(self, stable_sec=5., consistency_m=.75):
        if not all(math.isfinite(v) and v > 0 for v in (stable_sec, consistency_m)):
            raise ValueError('invalid runtime GPS position window')
        self.stable_sec, self.consistency_m = stable_sec, consistency_m
        self.history = deque()
        self.profile = None

    def clear(self):
        self.history.clear()
        self.profile = None

    def observe(self, stamp, candidate, profile):
        if not all(math.isfinite(v) for v in (stamp,) + tuple(candidate)):
            self.clear()
            return None, 'nonfinite_candidate'
        if self.history and stamp <= self.history[-1][0]:
            return None, 'duplicate_or_old_sample'
        reason = 'stability_wait'
        if self.history:
            previous = self.history[-1]
            if profile != self.profile or stamp-previous[0] > 1.:
                self.clear()
            elif math.hypot(candidate[0]-previous[1], candidate[1]-previous[2]) > 1.:
                self.clear()
                reason = 'candidate_jump'
        self.profile = profile
        self.history.append((stamp, candidate[0], candidate[1]))
        while len(self.history) > 2 and stamp-self.history[1][0] >= self.stable_sec:
            self.history.popleft()
        if len(self.history) < 8 or stamp-self.history[0][0] < self.stable_sec-1.e-6:
            return None, reason
        target = (statistics.median(p[1] for p in self.history),
                  statistics.median(p[2] for p in self.history))
        if max(math.hypot(p[1]-target[0], p[2]-target[1]) for p in self.history) > self.consistency_m:
            return None, 'position_inconsistent_with_lio'
        return target, 'stable'


class GlobalOdomBridge:
    """Build a continuous global odometry using GPS corrections and local dead reckoning."""

    WAITING = "WAITING_FOR_GPS"
    TRACKING = "GPS_TRACKING"
    DEAD_RECKONING = "LOCAL_DEAD_RECKONING"
    RECOVERING = "GPS_RECOVERING"
    POSITION_TRACKING = "GPS_POSITION_TRACKING"

    def __init__(self):
        rospy.init_node("global_odom_bridge")

        self.input_topic = rospy.get_param("~input_topic", "/odometry/gps")
        self.local_topic = rospy.get_param("~local_topic", "/odometry/local")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.gga_topic = rospy.get_param("~gga_topic", "/gnss/gpgga")
        self.heading_topic = rospy.get_param("~heading_topic", "/heading")
        self.solution_topic = rospy.get_param("~solution_topic", "/gnss/solution_status")
        self.require_heading_fixed = bool(rospy.get_param("~require_heading_fixed", True))
        self.solution_timeout_sec = max(0.1, float(rospy.get_param("~solution_timeout_sec", 2.5)))
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
        self.low_precision_startup = bool(rospy.get_param("~low_precision_startup", False))
        self.low_precision_max_covariance = max(0.0, float(rospy.get_param("~low_precision_max_covariance", 100.0)))
        self.low_precision_max_hdop = max(0.0, float(rospy.get_param("~low_precision_max_hdop", 5.0)))
        self.low_precision_min_satellites = max(1, int(rospy.get_param("~low_precision_min_satellites", 6)))
        self.low_precision_consistency_m = max(0.05, float(rospy.get_param("~low_precision_consistency_m", 2.0)))
        # Off by default for other consumers of this node. NeuPAN's outdoor
        # launch explicitly enables the position-only runtime policy.
        self.allow_nonfixed_runtime = bool(rospy.get_param('~allow_nonfixed_runtime', False))
        self.runtime_position_max_covariance = float(rospy.get_param('~runtime_position_max_covariance', 4.))
        self.runtime_position_max_hdop = float(rospy.get_param('~runtime_position_max_hdop', 2.5))
        self.runtime_position_min_satellites = int(rospy.get_param('~runtime_position_min_satellites', 8))
        self.runtime_position_max_error_m = float(rospy.get_param('~runtime_position_max_error_m', 3.))
        self.runtime_position_correction_rate = float(rospy.get_param('~runtime_position_correction_rate', .1))
        self.runtime_position_window = RuntimePositionWindow(
            float(rospy.get_param('~runtime_position_stable_sec', 5.)),
            float(rospy.get_param('~runtime_position_consistency_m', .75)))
        if (not all(math.isfinite(v) and v > 0 for v in (
                self.runtime_position_max_covariance, self.runtime_position_max_hdop,
                self.runtime_position_max_error_m, self.runtime_position_correction_rate))
                or self.runtime_position_min_satellites < 1
                or (self.allow_nonfixed_runtime and not self.guard_enabled)):
            raise ValueError('invalid nonfixed runtime GPS configuration')
        self.runtime_position_reason = 'waiting_for_initialization'
        self.runtime_position_profile = None
        self.runtime_position_active = False
        self.runtime_position_residual_m = None
        self.runtime_position_last_stamp = None
        self.runtime_position_last_update = None
        self.runtime_position_last_credit = None
        self.runtime_position_credit_count = 0
        self.initialization_precision = None
        self.global_reference_precision = None
        self.startup_candidate_profile = None
        if self.low_precision_startup and not (
                self.guard_enabled and self.use_heading_orientation and self.require_heading_fixed):
            raise ValueError("low_precision_startup requires GPS guard and fixed heading checks")
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
        self.local_motion_guard = LocalMotionGuard(
            self.local_max_speed, self.local_max_lateral_speed, self.local_max_yaw_rate,
            window_sec=max(0.05, float(rospy.get_param("~local_motion_window_sec", 0.2))),
            confirm_sec=max(0.0, float(rospy.get_param("~local_motion_confirm_sec", 0.1))),
            max_gap_sec=max(0.1, float(rospy.get_param("~local_motion_max_gap_sec", 0.5))),
            position_jump_m=max(0.05, float(rospy.get_param("~local_position_jump_m", 0.35))),
            yaw_jump_rad=math.radians(max(1.0, float(rospy.get_param("~local_yaw_jump_deg", 15.0)))),
        )
        self.local_motion_diagnostics = None
        self.dead_reckoning_budget = DeadReckoningBudget(
            rospy.get_param("~dead_reckoning_max_distance_m", 100.0),
            rospy.get_param("~dead_reckoning_max_duration_sec", 600.0),
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
        self.last_heading_stamp = None
        self.last_solution = None
        self.last_solution_recv = None
        self.heading_bad_until = rospy.Time(0)
        self.local_bad_until = rospy.Time(0)
        self.local_fault_reason = "local_odom_missing"
        self.local_speed_mps = None
        self.local_lateral_speed_mps = None
        self.local_yaw_rate_deg_s = None
        self.previous_local_pose = None
        self.previous_local_recv = None
        self.previous_local_stamp = None
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
        self.last_processed_gps_stamp_ns = None
        self.source_stamp_watermarks = {}
        self.last_correction_time = rospy.Time.now()
        self.last_status_publish = rospy.Time(0)
        self.last_position_error = None
        self.last_yaw_error_deg = None
        self.last_tracking_candidate = None
        self.last_candidate_step_m = None

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=2, latch=True)
        self.safety_pub = rospy.Publisher(self.safety_topic, Bool, queue_size=2, latch=True)
        self.local_fault_pub = rospy.Publisher("/gps_guard/local_fault_event", String, queue_size=30)
        rospy.Subscriber(self.input_topic, Odometry, self.gps_odom_cb, queue_size=10)
        rospy.Subscriber(self.local_topic, Odometry, self.local_odom_cb, queue_size=30)
        rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_cb, queue_size=10)
        rospy.Subscriber(self.gga_topic, Gpgga, self.gga_cb, queue_size=10)
        if self.use_heading_orientation:
            rospy.Subscriber(self.heading_topic, QuaternionStamped, self.heading_cb, queue_size=20)
            rospy.Subscriber(self.solution_topic, String, self.solution_cb, queue_size=10)

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
        current = (pose.position.x, pose.position.y,
                   self.yaw_from_quaternion(pose.orientation))
        result = self.local_motion_guard.observe(msg.header.stamp.to_sec(), current)
        receive_dt = None if self.previous_local_recv is None else (
            now - self.previous_local_recv).to_sec()
        result.update(
            stamp=now.to_sec(), message_stamp=msg.header.stamp.to_sec(),
            previous_message_stamp=None if self.previous_local_stamp is None
            else self.previous_local_stamp.to_sec(),
            receive_dt_sec=receive_dt,
            receive_speed_mps=result.get("distance_m", 0.0) / receive_dt
            if receive_dt is not None and receive_dt > 0.0 else None,
            speed_limit_mps=self.local_max_speed,
            fault_hold_sec=self.local_fault_hold_sec,
            motion_window_sec=self.local_motion_guard.window,
            motion_confirm_sec=self.local_motion_guard.confirm,
            fault_triggered=result["fault"] is not None,
        )
        if result["fault"] or not result["accepted"]:
            result["reason"] = result["fault"] or result["candidate_reason"]
            self.local_fault_pub.publish(json.dumps(result, separators=(",", ":")))
        self.local_motion_diagnostics = result
        if not result["accepted"]:
            # Replayed/duplicate/invalid messages must not refresh freshness or
            # replace the pose used by the map->odom bridge.
            return
        self.local_speed_mps = result["window_speed_mps"]
        self.local_lateral_speed_mps = result["window_lateral_speed_mps"]
        self.local_yaw_rate_deg_s = result["window_yaw_rate_deg_s"]
        if result["fault"]:
            self.handle_local_fault(now, msg, result["fault"])
        if self.transform_initialized and self.mode != self.TRACKING:
            xy = (pose.position.x, pose.position.y)
            if result["fault"]:
                # A reset/jump is not measured vehicle travel. Keep accumulated
                # budget and elapsed time while rebasing the local coordinate.
                self.dead_reckoning_budget.last_xy = xy
            else:
                self.dead_reckoning_budget.observe(xy)
        self.previous_local_pose = copy.deepcopy(pose)
        self.previous_local_recv = now
        self.previous_local_stamp = msg.header.stamp
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
        self.clear_runtime_position('local_odom_fault')

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

    def solution_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise ValueError("GNSS solution status must be an object")
        except (ValueError, TypeError):
            payload = {"heading_valid": False, "heading_fixed": False}
        self.last_solution = payload
        self.last_solution_recv = rospy.Time.now()

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
        self.last_heading_stamp = msg.header.stamp

    def check_source_times(self, now, sources):
        """Validate source clocks independently of fresh ROS reception times.

        Equal stamps remain fresh cached observations, but never become new
        correction samples. Commit watermarks only after ALL clocks validate,
        so a future/broken packet cannot poison subsequent valid observations.
        """
        observed = {}
        for name, stamp_ns, timeout in sources:
            if stamp_ns is None or stamp_ns <= 0:
                return False, 'source_stamp_invalid_'+name
            age = (now.to_nsec()-stamp_ns)*1.e-9
            if age < -.05:
                return False, 'source_future_'+name
            if age > timeout:
                return False, 'source_stale_'+name
            previous = self.source_stamp_watermarks.get(name)
            if previous is not None and stamp_ns < previous:
                return False, 'source_out_of_order_'+name
            observed[name] = stamp_ns
        self.source_stamp_watermarks.update(observed)
        return True, 'ok'

    def position_source_quality(self, now):
        sources = [('gps_odom', self.last_gps_odom.header.stamp.to_nsec(), self.data_timeout_sec),
                   ('local_odom', self.last_local_odom.header.stamp.to_nsec(), self.data_timeout_sec)]
        if self.guard_enabled:
            sources.extend((('gps_fix', self.last_fix.header.stamp.to_nsec(), self.data_timeout_sec),
                            ('gga', self.last_gga.header.stamp.to_nsec(), self.data_timeout_sec)))
        good, reason = self.check_source_times(now, sources)
        if not good:
            return good, reason
        if self.guard_enabled:
            fix_ns = self.last_fix.header.stamp.to_nsec()
            if (abs(fix_ns-self.last_gps_odom.header.stamp.to_nsec()) > 50000000 or
                    abs(fix_ns-self.last_local_odom.header.stamp.to_nsec()) > 250000000 or
                    abs(fix_ns-self.last_gga.header.stamp.to_nsec()) > 250000000):
                return False, 'position_source_time_mismatch'
        return True, 'ok'

    def heading_source_quality(self, now):
        sources = [('heading', None if self.last_heading_stamp is None else
                    self.last_heading_stamp.to_nsec(), self.heading_timeout_sec)]
        if self.require_heading_fixed:
            try:
                stamp = float(self.last_solution.get('stamp'))
                if not math.isfinite(stamp) or stamp <= 0:
                    raise ValueError('invalid solution source clock')
                stamp_ns = int(stamp*1.e9)
            except (ValueError, TypeError, OverflowError):
                return False, 'source_stamp_invalid_gnss_solution'
            sources.append(('gnss_solution', stamp_ns, self.solution_timeout_sec))
        return self.check_source_times(now, sources)

    def gps_observation_stamp_ns(self):
        source = self.last_fix if self.guard_enabled else self.last_gps_odom
        return source.header.stamp.to_nsec()

    def data_quality(self, now, low_precision=False, runtime_position=False):
        # Coarse startup still requires fixed heading. Runtime position-only
        # corrections are separately qualified and retain the existing yaw.
        if low_precision and (not self.low_precision_startup or self.transform_initialized):
            return False, "low_precision_startup_not_allowed"
        if runtime_position and (not self.allow_nonfixed_runtime or not self.transform_initialized):
            return False, 'runtime_position_not_allowed'
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
            min_fix_status = 1 if runtime_position else 0 if low_precision else self.min_fix_status
            if self.last_fix.status.status < min_fix_status:
                return False, "fix_status_{}".format(self.last_fix.status.status)

            cov_x = float(self.last_fix.position_covariance[0])
            cov_y = float(self.last_fix.position_covariance[4])
            if not (self.finite(cov_x) and self.finite(cov_y)) or min(cov_x, cov_y) < 0:
                return False, "position_covariance_invalid"
            max_covariance = (self.runtime_position_max_covariance if runtime_position else
                              self.low_precision_max_covariance if low_precision else self.max_position_covariance)
            if max(cov_x, cov_y) > max_covariance:
                return False, "position_covariance_large"
            if (low_precision or runtime_position) and not (
                    self.finite(self.last_fix.latitude) and self.finite(self.last_fix.longitude)
                    and -90 <= self.last_fix.latitude <= 90 and -180 <= self.last_fix.longitude <= 180):
                return False, "gps_coordinates_invalid"
            if self.last_gga is None or self.last_gga_recv is None:
                return False, "gga_missing"
            if (now - self.last_gga_recv).to_sec() > self.data_timeout_sec:
                return False, "gga_stale"
            if low_precision and int(self.last_gga.gps_qual) not in (1, 2, 4, 5):
                return False, "gga_quality_{}".format(self.last_gga.gps_qual)
            if runtime_position and int(self.last_gga.gps_qual) not in (2, 4, 5):
                return False, "gga_quality_{}".format(self.last_gga.gps_qual)
            if (not low_precision and not runtime_position and
                self.required_gga_quality > 0
                and int(self.last_gga.gps_qual) != self.required_gga_quality
            ):
                return False, "gga_quality_{}".format(self.last_gga.gps_qual)
            min_satellites = (self.runtime_position_min_satellites if runtime_position else
                              self.low_precision_min_satellites if low_precision else self.min_satellites)
            if int(self.last_gga.num_sats) < min_satellites:
                return False, "satellites_{}".format(self.last_gga.num_sats)
            if not self.finite(float(self.last_gga.hdop)) or float(self.last_gga.hdop) < 0:
                return False, "hdop_invalid"
            max_hdop = (self.runtime_position_max_hdop if runtime_position else
                        self.low_precision_max_hdop if low_precision else self.max_hdop)
            if float(self.last_gga.hdop) > max_hdop:
                return False, "hdop_%.2f" % self.last_gga.hdop

        gps_position = self.last_gps_odom.pose.pose.position
        local_position = self.last_local_odom.pose.pose.position
        if not all(
            self.finite(v)
            for v in (gps_position.x, gps_position.y, local_position.x, local_position.y)
        ):
            return False, "position_non_finite"

        if runtime_position:
            if int(self.last_fix.position_covariance_type) == 0:
                return False, 'position_covariance_unknown'
        # Fixed, non-Fixed and startup share source freshness/pairing checks.
        source_good, source_reason = self.position_source_quality(now)
        if not source_good:
            return source_good, source_reason

        if self.use_heading_orientation and not runtime_position:
            if self.require_heading_fixed:
                if self.last_solution is None or self.last_solution_recv is None:
                    return False, "heading_solution_missing"
                age = (now - self.last_solution_recv).to_sec()
                if age < 0 or age > self.solution_timeout_sec:
                    return False, "heading_solution_stale"
                if (self.last_solution.get("heading_fixed") is not True
                        or self.last_solution.get("heading_valid") is not True):
                    return False, "heading_solution_unfixed"
            if self.last_heading_yaw is None or self.last_heading_recv is None:
                return False, "heading_missing"
            if (now - self.last_heading_recv).to_sec() > self.heading_timeout_sec:
                return False, "heading_stale"
            if self.guard_enabled and now < self.heading_bad_until:
                return False, "heading_jump"
            source_good, source_reason = self.heading_source_quality(now)
            if not source_good:
                return source_good, source_reason
        return True, "ok" if self.guard_enabled else "guard_disabled"

    def clear_runtime_position(self, reason):
        self.runtime_position_window.clear()
        self.runtime_position_active = False
        self.runtime_position_reason = reason
        self.runtime_position_last_update = None
        self.runtime_position_residual_m = None

    def update_runtime_position(self, now):
        stamp_ns = self.gps_observation_stamp_ns()
        stamp = self.last_fix.header.stamp.to_sec()
        if self.last_processed_gps_stamp_ns is not None and stamp_ns <= self.last_processed_gps_stamp_ns:
            return  # the same fix cannot count as another observation/credit
        self.last_processed_gps_stamp_ns = stamp_ns
        self.runtime_position_last_stamp = stamp
        local = self.last_local_odom.pose.pose.position
        gps = self.last_gps_odom.pose.pose.position
        c, s = math.cos(self.transform_yaw), math.sin(self.transform_yaw)
        candidate = (gps.x-(c*local.x-s*local.y), gps.y-(s*local.x+c*local.y))
        error = math.hypot(candidate[0]-self.transform_x, candidate[1]-self.transform_y)
        if error > self.runtime_position_max_error_m:
            self.clear_runtime_position('position_innovation_too_large')
            self.set_mode(self.DEAD_RECKONING, 'runtime_position_rejected')
            return
        quality = int(self.last_gga.gps_qual)
        self.runtime_position_profile = {2: 'differential', 4: 'fixed_position_only', 5: 'float'}[quality]
        target, reason = self.runtime_position_window.observe(stamp, candidate, quality)
        self.runtime_position_reason = reason
        if target is None:
            self.runtime_position_active = False
            self.runtime_position_last_update = None
            self.set_mode(self.DEAD_RECKONING, 'runtime_position_'+reason)
            return
        dt = (min(.5, max(0., (now-self.runtime_position_last_update).to_sec()))
              if self.runtime_position_last_update is not None else 0.)
        self.runtime_position_last_update = now
        # Retain map->odom yaw exactly; an invalid GNSS heading must neither
        # rotate the map nor stop otherwise qualified position corrections.
        self.move_transform_toward((target[0], target[1], self.transform_yaw),
                                   dt, self.runtime_position_correction_rate, 0.)
        residual = math.hypot(target[0]-self.transform_x, target[1]-self.transform_y)
        self.runtime_position_residual_m = self.last_position_error = residual
        self.runtime_position_active = True
        self.global_reference_precision = 'runtime_'+self.runtime_position_profile
        self.runtime_position_reason = 'position_correcting' if residual > .5 else 'position_confirmed'
        if residual > .5:
            self.set_mode(self.DEAD_RECKONING, 'runtime_position_correcting')
            return
        # Renew only after a complete fresh window AND completed bounded
        # correction; at most once per window, never for a lone Float packet.
        if (self.runtime_position_last_credit is None or
                (now-self.runtime_position_last_credit).to_sec() >= self.runtime_position_window.stable_sec):
            self.dead_reckoning_budget.reset()
            self.dead_reckoning_budget.begin(now.to_sec(), (local.x, local.y))
            self.runtime_position_last_credit = now
            self.runtime_position_credit_count += 1
        self.set_mode(self.POSITION_TRACKING, 'runtime_position_confirmed')
        rospy.logwarn_throttle(5., '\033[33mGPS position-only tracking (%s): yaw from LIO; '
                              'coarse position, not RTK Fixed accuracy; budget renewals=%d\033[0m',
                              self.runtime_position_profile, self.runtime_position_credit_count)

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

    def recovery_window_stable(self, duration_sec, position_tolerance=None):
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
            max_position_spread <= (self.recovery_consistency_m if position_tolerance is None else position_tolerance)
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
        if mode == self.TRACKING:
            # Reset only after trusted GPS correction/initialization/recovery.
            self.dead_reckoning_budget.reset()
            self.global_reference_precision = "high"
        elif self.transform_initialized:
            xy = None
            if self.last_local_odom is not None:
                position = self.last_local_odom.pose.pose.position
                xy = (position.x, position.y)
            self.dead_reckoning_budget.begin(rospy.Time.now().to_sec(), xy)
        if mode != self.mode:
            if mode == self.DEAD_RECKONING:
                rospy.logwarn("GPS guard: %s -> %s (%s)", self.mode, mode, reason)
            else:
                rospy.loginfo("GPS guard: %s -> %s (%s)", self.mode, mode, reason)
        elif reason != self.reason and self.print_status_to_screen:
            rospy.loginfo("GPS guard: mode=%s reason=%s", mode, reason)
        self.mode = mode
        self.reason = reason

    def wait_for_position_pair(self):
        # Different ROS connections can deliver a new fix before its matching
        # projection. Do not treat that callback order as a new observation or
        # erase startup/recovery histories. Source age checks still reject a
        # stale pair, and an initialized bridge must start its budget first.
        self.runtime_position_active = False
        self.runtime_position_reason = 'position_source_time_mismatch'
        if not self.transform_initialized:
            self.set_mode(self.WAITING, 'position_source_time_mismatch')

    def update_guard(self, now):
        good, reason = self.data_quality(now)
        if self.transform_initialized and not good:
            # Start the budget BEFORE any pairing/runtime early return. A
            # previous Fixed mode has no active budget and does not accumulate
            # local travel. Transition it out of TRACKING immediately; begin()
            # is idempotent and must never erase accumulated distance/time.
            if self.mode == self.TRACKING:
                self.recovery_candidates.clear()
                self.heading_transform_history.clear()
                self.set_mode(self.DEAD_RECKONING, reason)
            xy = None
            if self.last_local_odom is not None:
                position = self.last_local_odom.pose.pose.position
                xy = (position.x, position.y)
            self.dead_reckoning_budget.begin(now.to_sec(), xy)
        if not good and reason == 'position_source_time_mismatch':
            self.wait_for_position_pair()
            return
        if self.transform_initialized and self.allow_nonfixed_runtime and not good:
            position_good, position_reason = self.data_quality(now, runtime_position=True)
            if position_good:
                self.recovery_candidates.clear()
                self.heading_transform_history.clear()
                self.update_runtime_position(now)
                return
            if position_reason == 'position_source_time_mismatch':
                self.wait_for_position_pair()
                return
            self.clear_runtime_position(position_reason)
        else:
            self.clear_runtime_position('fixed_policy' if good else 'waiting_for_initialization')
        if good and self.mode == self.POSITION_TRACKING:
            # Recover full GNSS position+heading through the original window,
            # not by silently promoting a coarse anchor to high precision.
            self.set_mode(self.DEAD_RECKONING, 'fixed_heading_recovery_wait')
        low_precision = False
        if not good and not self.transform_initialized and self.low_precision_startup:
            good, reason = self.data_quality(now, low_precision=True)
            low_precision = good
        if not good and reason == 'position_source_time_mismatch':
            self.wait_for_position_pair()
            return
        if not good:
            self.stable_since = None
            self.last_recovery_candidate = None
            self.recovery_candidates.clear()
            if not self.transform_initialized:
                self.startup_candidate_profile = None
                self.heading_transform_history.clear()
            if self.transform_initialized:
                self.set_mode(self.DEAD_RECKONING, reason)
            else:
                self.set_mode(self.WAITING, reason)
            return

        if not self.transform_initialized:
            profile = "low" if low_precision else "high"
            if profile != self.startup_candidate_profile:
                self.recovery_candidates.clear()
                self.heading_transform_history.clear()
                self.startup_candidate_profile = profile

        # A GPS sample is corrected once. Reusing an old GPS position with a
        # newer local pose would create a small backward sawtooth between fixes.
        stamp_ns = self.gps_observation_stamp_ns()
        if self.last_processed_gps_stamp_ns is not None and stamp_ns <= self.last_processed_gps_stamp_ns:
            return
        self.last_processed_gps_stamp_ns = stamp_ns
        correction_dt = max(0.0, min(0.5, (now - self.last_correction_time).to_sec()))
        self.last_correction_time = now

        candidate = self.candidate_transform(now)
        self.append_recovery_candidate(now, candidate)
        if not self.transform_initialized:
            self.last_recovery_candidate = candidate
            tolerance = self.low_precision_consistency_m if low_precision else None
            stable = self.recovery_window_stable(self.initial_stable_sec, tolerance)
            # A reported RTK fix can still have too much position jitter for
            # precise initialization. Explicit coarse mode may accept it under
            # its own stability bound, still with fixed heading.
            if (not stable and not low_precision and self.low_precision_startup
                    and self.data_quality(now, low_precision=True)[0]
                    and self.recovery_window_stable(self.initial_stable_sec, self.low_precision_consistency_m)):
                low_precision = True
                stable = True
            if stable:
                if low_precision:
                    # One coarse position anchor; do not chase subsequent GPS noise.
                    samples = [entry[1] for entry in self.recovery_candidates]
                    candidate = (sum(p[0] for p in samples) / len(samples),
                                 sum(p[1] for p in samples) / len(samples), candidate[2])
                self.transform_x, self.transform_y, self.transform_yaw = candidate
                self.transform_initialized = True
                self.initialization_precision = "low" if low_precision else "high"
                self.global_reference_precision = self.initialization_precision
                self.last_tracking_candidate = candidate
                self.last_candidate_step_m = 0.0
                self.stable_since = None
                self.recovery_candidates.clear()
                if low_precision:
                    self.set_mode(self.DEAD_RECKONING, "low_precision_initialized")
                    rospy.logwarn("Low precision GPS start: fixed heading, coarse map position; uncorrected budget %.1fm / %.1fs; nonfixed runtime position=%s",
                                  self.dead_reckoning_budget.max_distance_m,
                                  self.dead_reckoning_budget.max_duration_sec,
                                  self.allow_nonfixed_runtime)
                else:
                    self.set_mode(self.TRACKING, "gps_initialized")
            else:
                self.set_mode(self.WAITING, "low_precision_stability_check" if low_precision else "gps_stability_check")
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
            and self.dead_reckoning_budget.check(now.to_sec()) is None
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
            "low_precision_startup_enabled": self.low_precision_startup,
            "runtime_position": {
                "enabled": self.allow_nonfixed_runtime,
                "active": self.runtime_position_active,
                "reason": self.runtime_position_reason,
                "profile": self.runtime_position_profile,
                "samples": len(self.runtime_position_window.history),
                "residual_m": self.runtime_position_residual_m,
                "budget_renewals": self.runtime_position_credit_count,
                "last_budget_renewal_sec": None if self.runtime_position_last_credit is None else
                self.runtime_position_last_credit.to_sec(),
                "max_covariance_m2": self.runtime_position_max_covariance,
                "max_hdop": self.runtime_position_max_hdop,
                "min_satellites": self.runtime_position_min_satellites,
                "stable_sec": self.runtime_position_window.stable_sec,
                "consistency_m": self.runtime_position_window.consistency_m,
                "max_position_error_m": self.runtime_position_max_error_m,
                "correction_rate_mps": self.runtime_position_correction_rate,
            },
            "initialization_precision": self.initialization_precision,
            "global_reference_precision": self.global_reference_precision,
            "last_processed_gps_source_ns": self.last_processed_gps_stamp_ns,
            "low_precision_limits": {
                "max_covariance_m2": self.low_precision_max_covariance,
                "max_hdop": self.low_precision_max_hdop,
                "min_satellites": self.low_precision_min_satellites,
                "position_consistency_m": self.low_precision_consistency_m,
            },
            "fix_status": fix_status,
            "cov_x": cov_x,
            "cov_y": cov_y,
            "gga_quality": gga_quality,
            "gnss_solution": self.last_solution,
            "gnss_solution_age_sec": None if self.last_solution_recv is None else
            (now - self.last_solution_recv).to_sec(),
            "satellites": satellites,
            "hdop": hdop,
            "localization_safe": localization_safe,
            "dead_reckoning_elapsed_sec": self.dead_reckoning_budget.elapsed_sec(now.to_sec()),
            "dead_reckoning_distance_m": self.dead_reckoning_budget.distance_m,
            "dead_reckoning_limit_reason": self.dead_reckoning_budget.check(now.to_sec()),
            "dead_reckoning_max_distance_m": self.dead_reckoning_budget.max_distance_m,
            "dead_reckoning_max_duration_sec": self.dead_reckoning_budget.max_duration_sec,
            "local_motion_diagnostics": self.local_motion_diagnostics,
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
            "yaw_frozen": bool(self.transform_initialized and self.mode in (self.DEAD_RECKONING, self.POSITION_TRACKING)),
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
