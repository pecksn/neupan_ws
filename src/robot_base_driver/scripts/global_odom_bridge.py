#!/usr/bin/env python3

import copy
import json
import math
import statistics
import threading
import time
from collections import deque

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Gpgga
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler
from gnss_timebase import GnssTimebase


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
        self.state_lock = threading.RLock()

        self.input_topic = rospy.get_param("~input_topic", "/odometry/gps")
        self.local_topic = rospy.get_param("~local_topic", "/odometry/local")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.gga_topic = rospy.get_param("~gga_topic", "/gnss/gpgga")
        self.heading_topic = rospy.get_param("~heading_topic", "/heading")
        self.solution_topic = rospy.get_param("~solution_topic", "/gnss/solution_status")
        self.gnss_timebase = GnssTimebase(rospy.get_param('~gps_time_mode', 'shadow'),
                                        rospy.get_param('~gps_time_clock_verified', False))
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
        # Authoritative latest source health is NOT replaced by a historical
        # observation selected for time alignment.
        self.latest_solution_health = None
        self.latest_solution_health_watermark = 0
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
        self.local_pose_history = deque(maxlen=400)
        self.observation_history = {name: deque(maxlen=128)
                                    for name in ('gps', 'fix', 'gga', 'heading', 'solution')}
        self.pair_wait_timeout_sec = float(rospy.get_param(
            '~pair_wait_timeout_sec', min(.5, self.data_timeout_sec, self.heading_timeout_sec)))
        if not math.isfinite(self.pair_wait_timeout_sec) or not 0 < self.pair_wait_timeout_sec <= self.data_timeout_sec:
            raise ValueError('pair_wait_timeout_sec must be positive and <= data_timeout_sec')
        self.pair_wait_since = None
        self.pair_wait_xy = None
        self.observation_buffer_diagnostics = {'state': 'empty'}
        self.position_pair = None
        self.heading_pair = None
        self.source_pairing = {'valid': False, 'reason': 'waiting_for_sources'}
        self.recovery_diagnostics = {'valid': False, 'reason': 'empty_window'}
        self.correction_diagnostics = {'valid': False, 'reason': 'not_corrected'}
        self.alignment_error_stamp = None
        self.alignment_error_valid = False
        self.alignment_error_reason = 'not_observed'
        # Reuse the existing innovation bound and recovery duration. Only a
        # stable, full-quality Fixed observation can latch this stop, not Float.
        self.alignment_conflict_position_m = float(rospy.get_param(
            '~alignment_conflict_position_m', self.runtime_position_max_error_m))
        self.alignment_conflict_stable_sec = float(rospy.get_param(
            '~alignment_conflict_stable_sec', max(1.0, self.recovery_stable_sec)))
        if not all(math.isfinite(v) and v > 0 for v in (
                self.alignment_conflict_position_m, self.alignment_conflict_stable_sec)):
            raise ValueError('invalid alignment conflict configuration')
        self.alignment_conflict = False
        self.alignment_conflict_since = None
        self.alignment_conflict_diagnostics = {'state': 'not_observed'}

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=2, latch=True)
        self.safety_pub = rospy.Publisher(self.safety_topic, Bool, queue_size=2, latch=True)
        self.local_fault_pub = rospy.Publisher("/gps_guard/local_fault_event", String, queue_size=30)
        rospy.Subscriber(self.input_topic, Odometry, self.gps_odom_cb, queue_size=10)
        rospy.Subscriber(self.local_topic, Odometry, self.local_odom_cb, queue_size=30)
        rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_cb, queue_size=10)
        rospy.Subscriber(self.gga_topic, Gpgga, self.gga_cb, queue_size=10)
        rospy.Subscriber('/gnss/time_diagnostics', String, self.time_metadata_cb, queue_size=200)
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
        with self.state_lock:
            self.last_gps_odom = msg
            self.last_gps_recv = rospy.Time.now()
            self.remember_observation('gps', msg, self.last_gps_recv)

    def time_metadata_cb(self, msg):
        with self.state_lock:
            try:
                event = json.loads(msg.data)
                if not isinstance(event, dict):
                    raise ValueError('time metadata must be an object')
                self.gnss_timebase.receive(event, rospy.Time.now().to_nsec(), time.monotonic_ns())
            except (ValueError, TypeError):
                if self.gnss_timebase.mode == 'measurement':
                    self.gnss_timebase.fault = 'malformed_time_metadata'

    def local_odom_cb(self, msg):
        with self.state_lock:
            self._local_odom_cb(msg)

    def _local_odom_cb(self, msg):
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
        if result['fault']:
            self.local_pose_history.clear()
        self.local_pose_history.append((msg.header.stamp.to_nsec(), current))

    def fix_cb(self, msg):
        with self.state_lock:
            self.last_fix = msg
            self.last_fix_recv = rospy.Time.now()
            self.remember_observation('fix', msg, self.last_fix_recv)

    def gga_cb(self, msg):
        with self.state_lock:
            self.last_gga = msg
            self.last_gga_recv = rospy.Time.now()
            self.remember_observation('gga', msg, self.last_gga_recv)

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
        with self.state_lock:
            if (payload.get('time_mode') == 'measurement') != (self.gnss_timebase.mode == 'measurement'):
                self.gnss_timebase.fault = 'time_mode_mismatch_restart_required'
            self.last_solution = payload
            self.last_solution_recv = rospy.Time.now()
            self.remember_observation('solution', payload, self.last_solution_recv)
            try:
                stamp_ns = self.solution_stamp_ns(payload)
            except (ValueError, TypeError, OverflowError):
                stamp_ns = None
            if stamp_ns is None or stamp_ns > self.last_solution_recv.to_nsec()+50000000:
                self.latest_solution_health = {'payload': copy.deepcopy(payload),
                    'received': self.last_solution_recv, 'source_ns': None}
            elif stamp_ns >= self.latest_solution_health_watermark:
                prior = self.latest_solution_health
                new_fixed = (payload.get('heading_fixed') is True and payload.get('heading_valid') is True
                             and payload.get('error') is None)
                prior_fixed = (prior is not None and prior['source_ns'] is not None
                               and prior['payload'].get('heading_fixed') is True
                               and prior['payload'].get('heading_valid') is True
                               and prior['payload'].get('error') is None)
                # A cached/duplicate Fixed packet cannot clear a later failure
                # (including one with an invalid clock). Recovery needs a NEW
                # trustworthy source identity, not another receive timestamp.
                if stamp_ns > self.latest_solution_health_watermark or prior is None or not new_fixed or prior_fixed:
                    self.latest_solution_health_watermark = stamp_ns
                    self.latest_solution_health = {'payload': copy.deepcopy(payload),
                        'received': self.last_solution_recv, 'source_ns': stamp_ns}

    def latest_heading_health(self, now):
        """Current source failure overrides old, individually qualified data."""
        if not self.use_heading_orientation or not self.require_heading_fixed:
            return {'valid': True, 'reason': 'not_required', 'source_ns': None}
        state = self.latest_solution_health
        # Synchronous callers/tests may assign latest data without callbacks.
        if state is None:
            try:
                stamp_ns = self.solution_stamp_ns(self.last_solution or {})
            except (ValueError, TypeError, OverflowError):
                stamp_ns = None
            state = {'payload': self.last_solution, 'received': self.last_solution_recv,
                     'source_ns': stamp_ns}
        payload, received, stamp = state['payload'], state['received'], state['source_ns']
        reason = 'ok'
        if payload is None or received is None:
            reason = 'latest_heading_solution_missing'
        elif stamp is None:
            reason = 'latest_heading_solution_invalid'
        elif not -.05 <= (now.to_nsec()-stamp)*1e-9 <= self.solution_timeout_sec or not 0 <= (now-received).to_sec() <= self.solution_timeout_sec:
            reason = 'latest_heading_solution_stale'
        elif payload.get('error') is not None:
            reason = 'latest_heading_solution_invalid'
        elif payload.get('heading_fixed') is not True or payload.get('heading_valid') is not True:
            reason = 'latest_heading_solution_unfixed'
        return {'valid': reason == 'ok', 'reason': reason, 'source_ns': stamp,
                'heading_fixed': None if payload is None else payload.get('heading_fixed'),
                'heading_valid': None if payload is None else payload.get('heading_valid'),
                'received_stamp': None if received is None else received.to_sec()}

    def heading_cb(self, msg):
        with self.state_lock:
            self._heading_cb(msg)

    def _heading_cb(self, msg):
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
        self.remember_observation('heading', msg, now)

    def remember_observation(self, name, message, received):
        # Store quality and source time with the measurement, not with a later
        # packet. Bound memory even if a source floods while local odom stalls.
        try:
            stamp = (self.solution_stamp_ns(message) if name == 'solution'
                     else message.header.stamp.to_nsec())
        except (TypeError, ValueError, OverflowError):
            stamp = 0
        self.observation_history[name].append((stamp, received, copy.deepcopy(message)))

    def observation_entries(self, name):
        entries = {entry[0]: entry for entry in self.observation_history[name]}
        # Include latest values for startup and callers that supply messages
        # directly. Actual callbacks always populate the bounded history too.
        message = {'gps': self.last_gps_odom, 'fix': self.last_fix,
                   'gga': self.last_gga, 'solution': self.last_solution}.get(name)
        received = {'gps': self.last_gps_recv, 'fix': self.last_fix_recv,
                    'gga': self.last_gga_recv, 'solution': self.last_solution_recv}.get(name)
        if name == 'heading' and self.last_heading_stamp is not None and self.last_heading is not None:
            msg = QuaternionStamped()
            msg.header.stamp = self.last_heading_stamp
            msg.quaternion = self.last_heading
            message, received = msg, self.last_heading_recv
        if message is not None and received is not None:
            try:
                stamp = (self.solution_stamp_ns(message) if name == 'solution'
                         else message.header.stamp.to_nsec())
            except (TypeError, ValueError, OverflowError):
                stamp = 0
            entries[stamp] = (stamp, received, message)
        return list(entries.values())

    @staticmethod
    def solution_stamp_ns(message):
        value = message.get('stamp_ns')
        if type(value) is not int or value <= 0:
            raise ValueError('missing/invalid exact solution identity')
        if message.get('time_mode') == 'measurement':
            if (message.get('time_valid') is not True or message.get('clock_verified') is not True
                    or message.get('measurement_stamp_ns') != value
                    or message.get('observation_id') != '{}:{}:PVTSLNA:{}'.format(
                        message.get('session_id'), message.get('clock_epoch'), value)):
                raise ValueError('invalid measurement solution identity')
            return value  # Exact integer identity; legacy float is display only.
        rounded = float(message.get('stamp'))*1e9
        if not math.isfinite(rounded) or abs(rounded-value) > 1000:
            raise ValueError('inconsistent solution source time')
        return value

    @staticmethod
    def same_solution_observation(solution, heading_ns):
        try:
            return GlobalOdomBridge.solution_stamp_ns(solution) == heading_ns
        except (TypeError, ValueError, OverflowError):
            return False

    def select_observation(self, now):
        """Choose the newest unprocessed observation that local odom brackets.

        A 120ms local delay with 10Hz GPS must consume an older *new* GPS
        observation, not wait forever for the continually replaced latest one.
        All associated quality messages retain their own receive/source times.
        """
        if not self.observation_history['gps']:
            return None
        entries = {name: self.observation_entries(name) for name in self.observation_history}
        def nearest(name, stamp, tolerance):
            candidates = [e for e in entries[name] if e[0] > 0 and abs(e[0]-stamp) <= tolerance*1e9]
            return min(candidates, key=lambda e: (abs(e[0]-stamp), e[0])) if candidates else None
        def exact(name, stamp):
            candidates = [e for e in entries[name] if
                (self.same_solution_observation(e[2], stamp) if name == 'solution' else e[0] == stamp)]
            return max(candidates, key=lambda e: e[1].to_nsec()) if candidates else None
        self.observation_buffer_diagnostics = {'state': 'waiting', 'stamp': now.to_sec(),
            'gps_samples': len(entries['gps']), 'selected_source_ns': None,
            'latest_gps_ns': None if self.last_gps_odom is None else self.last_gps_odom.header.stamp.to_nsec()}
        # Invalid/future/out-of-order latest inputs still pass through the
        # ordinary rejection path; a cache must not hide a broken source clock.
        if self.last_gps_odom is not None:
            stamp = self.last_gps_odom.header.stamp.to_nsec()
            watermark = self.source_stamp_watermarks.get('gps_odom', 0)
            if stamp <= 0 or stamp > now.to_nsec()+50000000 or stamp < watermark:
                return None
        for gps in sorted(entries['gps'], key=lambda e: e[0], reverse=True):
            stamp = gps[0]
            if not 0 <= now.to_nsec()-stamp <= self.data_timeout_sec*1e9:
                continue
            fix = exact('fix', stamp) if self.guard_enabled else None
            observation_stamp = fix[0] if fix is not None else stamp
            if self.last_processed_gps_stamp_ns is not None and observation_stamp <= self.last_processed_gps_stamp_ns:
                continue
            gga = exact('gga', observation_stamp) if self.guard_enabled else None
            if self.guard_enabled and (fix is None or gga is None):
                continue
            if self.local_pose_at(stamp)[0] is None:
                continue
            heading = nearest('heading', stamp, self.heading_timeout_sec) if self.use_heading_orientation else None
            solution = exact('solution', heading[0]) if heading is not None else None
            heading_paired = (heading is not None and solution is not None
                              and self.local_pose_at(heading[0])[0] is not None)
            runtime_allowed = self.transform_initialized and self.allow_nonfixed_runtime
            if self.use_heading_orientation and self.require_heading_fixed and not heading_paired:
                health = self.latest_heading_health(now)
                # A genuinely new heading awaiting its own quality is still a
                # bounded wait. An absent/old heading in Float is NOT a reason
                # to starve a fully paired position-only observation.
                new_unqualified_heading = (heading is not None and solution is None and
                    -.05 <= (now.to_nsec()-heading[0])*1e-9 <= self.heading_timeout_sec and
                    (health['source_ns'] is None or heading[0] > health['source_ns']))
                if not runtime_allowed or new_unqualified_heading:
                    continue
                heading = solution = None
            selected = {'last_gps_odom': gps[2], 'last_gps_recv': gps[1]}
            if self.guard_enabled:
                selected.update(last_fix=fix[2], last_fix_recv=fix[1], last_gga=gga[2], last_gga_recv=gga[1])
            if self.use_heading_orientation:
                selected.update(last_heading=None if heading is None else heading[2].quaternion,
                    last_heading_yaw=None if heading is None else self.yaw_from_quaternion(heading[2].quaternion),
                    last_heading_stamp=None if heading is None else heading[2].header.stamp,
                    last_heading_recv=None if heading is None else heading[1],
                    last_solution=None if solution is None else solution[2],
                    last_solution_recv=None if solution is None else solution[1])
            self.observation_buffer_diagnostics.update(state='selected', selected_source_ns=observation_stamp,
                source_age_sec=(now.to_nsec()-stamp)*1e-9,
                selected_fix_status=None if fix is None else int(fix[2].status.status),
                selected_gga_quality=None if gga is None else int(gga[2].gps_qual),
                selected_heading_ns=None if heading is None else heading[0],
                quality_binding='same_observation',
                selected_heading_fixed=None if solution is None else solution[2].get('heading_fixed'))
            self.observation_buffer_diagnostics['heading_paired'] = heading_paired
            return selected
        return None

    def current_anchor(self):
        if self.last_local_odom is None:
            raise ValueError('local anchor unavailable')
        p = self.last_local_odom.pose.pose.position
        return float(p.x), float(p.y)

    @staticmethod
    def project_alignment(transform, anchor):
        tx, ty, angle = transform
        x, y = anchor
        c, s = math.cos(angle), math.sin(angle)
        return tx+c*x-s*y, ty+s*x+c*y

    def local_pose_at(self, stamp_ns):
        """Interpolate accepted source poses; never extrapolate a moving pose."""
        history = list(self.local_pose_history)
        # Also support an exact latest observation before a history is formed.
        if self.last_local_odom is not None:
            msg = self.last_local_odom
            point = (msg.pose.pose.position.x, msg.pose.pose.position.y,
                     self.yaw_from_quaternion(msg.pose.pose.orientation))
            latest = (msg.header.stamp.to_nsec(), point)
            if not history or latest[0] > history[-1][0]:
                history.append(latest)
        if not history or stamp_ns < history[0][0] or stamp_ns > history[-1][0]:
            return None, {'reason': 'local_bracket_unavailable'}
        for i, (stamp, pose) in enumerate(history):
            if stamp == stamp_ns:
                return tuple(pose), {'left_ns': stamp, 'right_ns': stamp, 'fraction': 0.}
            if stamp > stamp_ns and i:
                old_stamp, old = history[i-1]
                if stamp-old_stamp > int(self.data_timeout_sec*1e9):
                    return None, {'reason': 'local_interpolation_gap'}
                f = (stamp_ns-old_stamp)/float(stamp-old_stamp)
                result = (old[0]+f*(pose[0]-old[0]), old[1]+f*(pose[1]-old[1]),
                          self.normalize_angle(old[2]+f*self.normalize_angle(pose[2]-old[2])))
                return result, {'left_ns': old_stamp, 'right_ns': stamp, 'fraction': f}
        return None, {'reason': 'local_bracket_unavailable'}

    def check_source_times(self, now, sources, commit=True):
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
        if commit:
            self.source_stamp_watermarks.update(observed)
        return True, 'ok'

    def position_source_quality(self, now):
        self.position_pair = None
        self.source_pairing = {'valid': False, 'stamp': now.to_sec(), 'reason': 'source_quality_rejected'}
        sources = [('gps_odom', self.last_gps_odom.header.stamp.to_nsec(), self.data_timeout_sec),
                   ('local_odom', self.last_local_odom.header.stamp.to_nsec(), self.data_timeout_sec)]
        if self.guard_enabled:
            sources.extend((('gps_fix', self.last_fix.header.stamp.to_nsec(), self.data_timeout_sec),
                            ('gga', self.last_gga.header.stamp.to_nsec(), self.data_timeout_sec)))
        good, reason = self.check_source_times(now, sources, commit=False)
        if not good:
            return good, reason
        if self.guard_enabled:
            fix_ns = self.last_fix.header.stamp.to_nsec()
            if (fix_ns != self.last_gps_odom.header.stamp.to_nsec() or
                    abs(fix_ns-self.last_local_odom.header.stamp.to_nsec()) > 250000000 or
                    fix_ns != self.last_gga.header.stamp.to_nsec()):
                return False, 'position_source_time_mismatch'
        stamp = self.last_gps_odom.header.stamp.to_nsec()
        pose, bracket = self.local_pose_at(stamp)
        self.source_pairing.update(
            gps_odom_ns=stamp, local_latest_ns=self.last_local_odom.header.stamp.to_nsec(),
            local_minus_gps_ms=(self.last_local_odom.header.stamp.to_nsec()-stamp)/1e6,
            gps_fix_ns=self.last_fix.header.stamp.to_nsec() if self.guard_enabled else None,
            position_bracket=bracket)
        if pose is None:
            self.source_pairing['reason'] = bracket['reason']
            return False, 'position_source_time_mismatch'
        p = self.last_gps_odom.pose.pose.position
        self.position_pair = {'local_pose': pose, 'gps_xy': (float(p.x), float(p.y)), 'stamp_ns': stamp,
                              'source_stamps': {name: value for name, value, _ in sources}}
        self.source_pairing.update(valid=True, reason='position_paired')
        return True, 'ok'

    def heading_source_quality(self, now, pair_local=True):
        self.heading_pair = None
        sources = [('heading', None if self.last_heading_stamp is None else
                    self.last_heading_stamp.to_nsec(), self.heading_timeout_sec)]
        if self.require_heading_fixed:
            try:
                stamp = float(self.last_solution.get('stamp'))
                if not math.isfinite(stamp) or stamp <= 0:
                    raise ValueError('invalid solution source clock')
                stamp_ns = self.solution_stamp_ns(self.last_solution)
            except (ValueError, TypeError, OverflowError):
                return False, 'source_stamp_invalid_gnss_solution'
            sources.append(('gnss_solution', stamp_ns, self.solution_timeout_sec))
            if not self.same_solution_observation(self.last_solution, self.last_heading_stamp.to_nsec()):
                self.source_pairing.update(valid=False, quality_binding='heading_solution_pending')
                return False, 'position_source_time_mismatch'
        good, reason = self.check_source_times(now, sources, commit=False)
        if not good:
            return good, reason
        if not pair_local:
            return True, 'ok'
        stamp = self.last_heading_stamp.to_nsec()
        pose, bracket = self.local_pose_at(stamp)
        self.source_pairing.update(heading_ns=stamp, heading_bracket=bracket,
            heading_minus_gps_ms=(stamp-self.position_pair['stamp_ns'])/1e6 if self.position_pair else None)
        if pose is None:
            self.source_pairing.update(valid=False, reason=bracket['reason'])
            return False, 'position_source_time_mismatch'
        self.heading_pair = {'local_pose': pose, 'global_yaw': float(self.last_heading_yaw),
                             'source_stamps': {name: value for name, value, _ in sources}}
        self.source_pairing.update(valid=True, reason='position_and_heading_paired')
        return True, 'ok'

    def gps_observation_stamp_ns(self):
        source = self.last_fix if self.guard_enabled else self.last_gps_odom
        return source.header.stamp.to_nsec()

    def data_quality(self, now, low_precision=False, runtime_position=False):
        self.position_pair = self.heading_pair = None
        self.source_pairing = {'valid': False, 'stamp': now.to_sec(), 'reason': 'quality_pending'}
        good, reason = self._data_quality(now, low_precision, runtime_position)
        if not good:
            self.source_pairing.update(valid=False, reason=reason)
        else:
            # An unbracketed newest packet must not advance a watermark past
            # the older pending packet that will become pairable next tick.
            for pair in (self.position_pair, self.heading_pair):
                if pair is not None:
                    self.source_stamp_watermarks.update(pair['source_stamps'])
        return good, reason

    def _data_quality(self, now, low_precision=False, runtime_position=False):
        if self.gnss_timebase.fault:
            return False, self.gnss_timebase.fault
        if self.gnss_timebase.mode == 'measurement' and not self.gnss_timebase.clock_ready:
            return False, 'waiting_for_measurement_clock'
        if self.last_gps_odom is not None:
            fresh, reason = self.gnss_timebase.check('GGA',
                self.last_gps_odom.header.stamp.to_nsec(), now.to_nsec(), time.monotonic_ns(), self.data_timeout_sec)
            if not fresh:
                return False, reason
        if not runtime_position and self.last_heading_stamp is not None:
            fresh, reason = self.gnss_timebase.check('PVTSLNA',
                self.last_heading_stamp.to_nsec(), now.to_nsec(), time.monotonic_ns(), self.heading_timeout_sec)
            if not fresh:
                return False, reason
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
        sources = [('gps_odom', self.last_gps_odom.header.stamp.to_nsec(), self.data_timeout_sec),
                   ('local_odom', self.last_local_odom.header.stamp.to_nsec(), self.data_timeout_sec)]
        for name, msg, received in (('gps_fix',self.last_fix,self.last_fix_recv),
                                     ('gga',self.last_gga,self.last_gga_recv)):
            if self.guard_enabled and msg is not None and received is not None:
                if (now-received).to_sec() > self.data_timeout_sec:
                    return False, name+'_stale'
                sources.append((name,msg.header.stamp.to_nsec(),self.data_timeout_sec))
        if not runtime_position and self.use_heading_orientation and self.last_heading_stamp is not None:
            sources.append(('heading',self.last_heading_stamp.to_nsec(),self.heading_timeout_sec))
        if not runtime_position and self.use_heading_orientation and self.require_heading_fixed:
            if self.last_solution_recv is not None and (now-self.last_solution_recv).to_sec() > self.solution_timeout_sec:
                return False,'heading_solution_stale'
            if self.last_solution is not None:
                try:
                    solution_ns=self.solution_stamp_ns(self.last_solution)
                except (TypeError,ValueError,OverflowError):
                    solution_ns=None
                if solution_ns is not None:
                    sources.append(('gnss_solution',solution_ns,self.solution_timeout_sec))
        fresh, stale_reason = self.check_source_times(now,sources,commit=False)
        if not fresh:
            return False,stale_reason
        # Incomplete quality is a bounded wait, not a Float packet that can
        # already be consumed through position-only fallback. Test binding
        # before quality thresholds, which may still describe the old epoch.
        if not runtime_position and self.use_heading_orientation and self.require_heading_fixed:
            health = self.latest_heading_health(now)
            awaiting_new_quality = (self.last_heading_stamp is not None and self.last_solution is not None
                and not self.same_solution_observation(self.last_solution,self.last_heading_stamp.to_nsec())
                and (health['source_ns'] is None or self.last_heading_stamp.to_nsec() > health['source_ns']))
            if not health['valid'] and not awaiting_new_quality:
                return False,health['reason']
        if (not runtime_position and self.use_heading_orientation and self.require_heading_fixed
                and self.last_heading_stamp is not None and self.last_solution is not None
                and not self.same_solution_observation(self.last_solution, self.last_heading_stamp.to_nsec())):
            self.source_pairing['quality_binding'] = 'heading_solution_pending'
            return False, 'position_source_time_mismatch'
        if (self.guard_enabled and self.last_fix is not None and self.last_gga is not None
                and self.last_fix.header.stamp != self.last_gga.header.stamp):
            self.source_pairing['quality_binding'] = 'fix_gga_pending'
            return False, 'position_source_time_mismatch'
        if self.guard_enabled and self.last_fix is not None and self.last_fix.header.stamp != self.last_gps_odom.header.stamp:
            self.source_pairing['quality_binding'] = 'gps_fix_pending'
            return False, 'position_source_time_mismatch'
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
            if self.last_gga.header.stamp != self.last_fix.header.stamp:
                self.source_pairing['quality_binding'] = 'fix_gga_pending'
                return False, 'position_source_time_mismatch'
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
        # Quality loss/stale heading is not a mere interpolation wait. Check
        # these before a missing position bracket can take the grace path.
        if self.use_heading_orientation and not runtime_position:
            if self.require_heading_fixed:
                if self.last_solution is None or self.last_solution_recv is None:
                    return False, "heading_solution_missing"
                age = (now - self.last_solution_recv).to_sec()
                if age < 0 or age > self.solution_timeout_sec:
                    return False, "heading_solution_stale"
                if self.last_heading_stamp is not None and not self.same_solution_observation(
                        self.last_solution, self.last_heading_stamp.to_nsec()):
                    self.source_pairing['quality_binding'] = 'heading_solution_pending'
                    return False, 'position_source_time_mismatch'
                if (self.last_solution.get("heading_fixed") is not True
                        or self.last_solution.get("heading_valid") is not True):
                    return False, "heading_solution_unfixed"
            if self.last_heading_yaw is None or self.last_heading_recv is None:
                return False, "heading_missing"
            if (now - self.last_heading_recv).to_sec() > self.heading_timeout_sec:
                return False, "heading_stale"
            if self.guard_enabled and now < self.heading_bad_until:
                return False, "heading_jump"
            source_good, source_reason = self.heading_source_quality(now, pair_local=False)
            if not source_good:
                return source_good, source_reason
        # Fixed, non-Fixed and startup share source freshness/pairing checks.
        source_good, source_reason = self.position_source_quality(now)
        if not source_good:
            return source_good, source_reason
        if self.use_heading_orientation and not runtime_position:
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
        self.pair_wait_since = self.pair_wait_xy = None
        local = self.last_local_odom.pose.pose.position
        lx, ly, _ = self.position_pair['local_pose']
        gx, gy = self.position_pair['gps_xy']
        c, s = math.cos(self.transform_yaw), math.sin(self.transform_yaw)
        candidate = (gx-(c*lx-s*ly), gy-(s*lx+c*ly))
        error = math.hypot(candidate[0]-self.transform_x, candidate[1]-self.transform_y)
        self.last_position_error = error
        self.last_yaw_error_deg = None
        self.alignment_error_stamp = now.to_sec()
        self.alignment_error_valid = True
        self.alignment_error_reason = 'position_only_innovation'
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
        if not self.move_transform_toward((target[0], target[1], self.transform_yaw),
                                         dt, self.runtime_position_correction_rate, 0.):
            self.clear_runtime_position('invalid_correction')
            self.set_mode(self.DEAD_RECKONING, 'invalid_alignment_correction')
            return
        residual = math.hypot(target[0]-self.transform_x, target[1]-self.transform_y)
        self.runtime_position_residual_m = self.last_position_error = residual
        self.alignment_error_reason = 'position_only_window_residual'
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
        local_x, local_y, local_yaw = self.position_pair['local_pose']
        gps_x, gps_y = self.position_pair['gps_xy']
        if self.use_heading_orientation:
            global_yaw = self.heading_pair['global_yaw']
            raw_dyaw = self.normalize_angle(global_yaw - self.heading_pair['local_pose'][2])
            dyaw = self.filtered_heading_transform_yaw(now, raw_dyaw)
        else:
            global_yaw = self.yaw_from_quaternion(self.last_gps_odom.pose.pose.orientation)
            dyaw = self.normalize_angle(global_yaw - local_yaw)
        c = math.cos(dyaw)
        s = math.sin(dyaw)
        tx = gps_x - (c * local_x - s * local_y)
        ty = gps_y - (s * local_x + c * local_y)
        return tx, ty, dyaw

    def transform_error(self, candidate):
        anchor = self.current_anchor()
        before = self.project_alignment((self.transform_x, self.transform_y, self.transform_yaw), anchor)
        after = self.project_alignment(candidate, anchor)
        position_error = math.hypot(after[0]-before[0], after[1]-before[1])
        yaw_error = self.normalize_angle(candidate[2] - self.transform_yaw)
        return position_error, yaw_error

    def candidates_consistent(self, first, second):
        if first is None or second is None:
            return True
        anchor = self.current_anchor()
        a, b = self.project_alignment(first, anchor), self.project_alignment(second, anchor)
        position_delta = math.hypot(a[0]-b[0], a[1]-b[1])
        yaw_delta = abs(self.normalize_angle(first[2] - second[2]))
        return (
            position_delta <= self.recovery_consistency_m
            and yaw_delta <= self.recovery_consistency_yaw_rad
        )

    def append_recovery_candidate(self, now, candidate):
        self.recovery_candidates.append((now.to_sec(), candidate))
        keep_sec = max(self.initial_stable_sec, self.recovery_stable_sec,
                       self.alignment_conflict_stable_sec) + 1.0
        cutoff = now.to_sec() - keep_sec
        while self.recovery_candidates and self.recovery_candidates[0][0] < cutoff:
            self.recovery_candidates.popleft()

    def recovery_window_stable(self, duration_sec, position_tolerance=None):
        self.recovery_diagnostics = {'valid': False, 'reason': 'empty_window',
                                     'stamp': rospy.Time.now().to_sec(),
                                     'samples': len(self.recovery_candidates)}
        if not self.recovery_candidates:
            return False
        first_stamp = self.recovery_candidates[0][0]
        last_stamp = self.recovery_candidates[-1][0]
        candidates = [entry[1] for entry in self.recovery_candidates]
        anchor = self.current_anchor()
        positions = [self.project_alignment(candidate, anchor) for candidate in candidates]
        mean_x = sum(p[0] for p in positions)/len(positions)
        mean_y = sum(p[1] for p in positions)/len(positions)
        yaw_reference = candidates[0][2]
        max_position_spread = max(
            math.hypot(p[0] - mean_x, p[1] - mean_y)
            for p in positions
        )
        max_yaw_spread = max(
            abs(self.normalize_angle(candidate[2] - yaw_reference))
            for candidate in candidates
        )
        limit = self.recovery_consistency_m if position_tolerance is None else position_tolerance
        passed = (last_stamp-first_stamp >= duration_sec and max_position_spread <= limit
                  and max_yaw_spread <= self.recovery_consistency_yaw_rad)
        self.recovery_diagnostics.update(valid=True, passed=passed,
            reason='ok' if passed else 'duration' if last_stamp-first_stamp < duration_sec else
            'position_spread' if max_position_spread > limit else 'yaw_spread',
            duration_sec=last_stamp-first_stamp, required_duration_sec=duration_sec,
            anchor_local_m=anchor, position_spread_m=max_position_spread,
            yaw_spread_deg=math.degrees(max_yaw_spread), position_limit_m=limit,
            yaw_limit_deg=math.degrees(self.recovery_consistency_yaw_rad))
        return passed

    def move_transform_toward(self, target, dt, position_rate, yaw_rate):
        """Bound displacement of the CAR, not displacement of transform t."""
        self.correction_diagnostics = {'valid': False, 'reason': 'invalid_input'}
        anchor = self.current_anchor()
        old = (self.transform_x, self.transform_y, self.transform_yaw)
        if (not all(math.isfinite(v) for v in tuple(target)+old+anchor+(dt, position_rate, yaw_rate))
                or position_rate < 0 or yaw_rate < 0):
            return False
        bounded_dt = min(.5, max(0., dt))
        before = self.project_alignment(old, anchor)
        goal = self.project_alignment(target, anchor)
        dx, dy = goal[0]-before[0], goal[1]-before[1]
        distance = math.hypot(dx, dy)
        max_distance = position_rate * bounded_dt
        if distance > max_distance and distance > 1e-9:
            scale = max_distance / distance
            dx *= scale
            dy *= scale
        yaw_error = self.normalize_angle(target[2] - self.transform_yaw)
        max_yaw = yaw_rate * bounded_dt
        yaw_step = max(-max_yaw, min(max_yaw, yaw_error))
        # Position-only updates must retain yaw bit-for-bit, not renormalize it.
        angle = self.transform_yaw if yaw_step == 0 else self.normalize_angle(self.transform_yaw+yaw_step)
        after = before[0]+dx, before[1]+dy
        c, s = math.cos(angle), math.sin(angle)
        result = (after[0]-(c*anchor[0]-s*anchor[1]),
                  after[1]-(s*anchor[0]+c*anchor[1]), angle)
        if not all(math.isfinite(v) for v in result+after):
            return False
        self.transform_x, self.transform_y, self.transform_yaw = result
        self.correction_diagnostics = {
            'valid': True, 'reason': 'ok', 'stamp': rospy.Time.now().to_sec(),
            'dt_sec': bounded_dt, 'requested_dt_sec': dt,
            'anchor_local_m': anchor, 'anchor_before_map_m': before,
            'anchor_target_map_m': goal, 'anchor_after_map_m': after,
            'actual_position_step_m': math.hypot(dx,dy), 'position_step_limit_m': max_distance,
            'actual_yaw_step_deg': math.degrees(yaw_step),
            'transform_translation_step_m': math.hypot(result[0]-old[0],result[1]-old[1]),
        }
        return True

    def invalidate_alignment_error(self, reason):
        self.last_position_error = self.last_yaw_error_deg = None
        self.last_candidate_step_m = None
        self.alignment_error_valid = False
        self.alignment_error_reason = reason

    def check_alignment_conflict(self, now, residual):
        """Latch only a sustained, stable Fixed conflict. Never auto-realign it."""
        fixed = (self.guard_enabled and self.last_fix is not None and self.last_gga is not None
                 and self.last_fix.status.status >= 2 and int(self.last_gga.gps_qual) == 4)
        exceeds = fixed and residual > self.alignment_conflict_position_m
        if not exceeds:
            self.alignment_conflict_since = None
            self.alignment_conflict_diagnostics = {'state': 'clear' if fixed else 'not_qualified',
                'stamp': now.to_sec(), 'position_residual_m': residual}
            return False
        if self.alignment_conflict_since is None:
            self.alignment_conflict_since = now.to_sec()
        duration = now.to_sec()-self.alignment_conflict_since
        stable = self.recovery_window_stable(self.alignment_conflict_stable_sec)
        self.alignment_conflict_diagnostics = {'state': 'pending', 'stamp': now.to_sec(),
            'position_residual_m': residual, 'duration_sec': duration,
            'qualified_fixed_window': stable}
        if stable and duration >= self.alignment_conflict_stable_sec:
            self.alignment_conflict = True
            self.alignment_conflict_diagnostics['state'] = 'latched'
            self.set_mode(self.DEAD_RECKONING, 'alignment_conflict_stop_reinitialize_when_stationary')
            self.safety_pub.publish(Bool(data=False))
            rospy.logerr('Localization Fixed conflict: %.2fm; AUTO held. Park, verify localization, '
                         'restart localization/system and send a new route; no moving realignment.', residual)
            return True
        return False

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

    def wait_for_position_pair(self, now):
        # Different ROS connections can deliver a new fix before its matching
        # projection. Do not treat that callback order as a new observation or
        # erase tracking/recovery histories. The grace period is bounded and
        # does not renew on every new unmatched GPS message.
        if self.pair_wait_since is None:
            self.pair_wait_since = now
            self.pair_wait_xy = self.current_anchor() if self.last_local_odom is not None else None
        self.runtime_position_reason = 'position_source_time_mismatch'
        elapsed = max(0., (now-self.pair_wait_since).to_sec())
        self.observation_buffer_diagnostics.update(wait_sec=elapsed,
            wait_timeout_sec=self.pair_wait_timeout_sec,
            state='pair_timeout' if elapsed >= self.pair_wait_timeout_sec else 'pair_wait')
        if not self.transform_initialized:
            self.set_mode(self.WAITING, 'position_source_time_mismatch')
        return elapsed < self.pair_wait_timeout_sec

    def update_guard(self, now):
        with self.state_lock:
            health = self.latest_heading_health(now)
            if not health['valid'] and self.mode in (self.TRACKING, self.RECOVERING):
                self.recovery_candidates.clear()
                self.heading_transform_history.clear()
                self.set_mode(self.DEAD_RECKONING, health['reason'])
            selected = self.select_observation(now)
            saved = {} if selected is None else {name: getattr(self, name) for name in selected}
            try:
                for name, value in (selected or {}).items():
                    setattr(self, name, value)
                self._update_guard(now)
            finally:
                # Do not overwrite callbacks' latest data with the historical
                # observation used for this update. The same lock covers both.
                for name, value in saved.items():
                    setattr(self, name, value)

    def _update_guard(self, now):
        if self.alignment_conflict:
            self.source_pairing = {'valid': False, 'reason': 'alignment_conflict_latched', 'stamp': now.to_sec()}
            self.runtime_position_active = False
            self.runtime_position_reason = 'alignment_conflict_latched'
            self.invalidate_alignment_error('alignment_conflict_latched')
            return
        good, reason = self.data_quality(now)
        if not good and reason == 'position_source_time_mismatch':
            self.invalidate_alignment_error(reason)
            if self.wait_for_position_pair(now):
                return
            reason = 'position_pair_timeout'
        if self.transform_initialized and not good:
            # Callback ordering/interpolation may briefly lack a pair between
            # fresh observations. Do not erase an accumulating conflict window
            # on that wait; stale/invalid quality still resets it below.
            if reason != 'position_source_time_mismatch':
                self.alignment_conflict_since = None
                self.alignment_conflict_diagnostics = {'state': 'not_qualified', 'reason': reason, 'stamp': now.to_sec()}
            # Real quality loss or an expired pairing grace starts the budget
            # before runtime fallback. Count an expired wait from its start;
            # begin() never erases an already accumulated distance/time budget.
            xy = None
            if self.last_local_odom is not None:
                position = self.last_local_odom.pose.pose.position
                xy = (position.x, position.y)
            if reason == 'position_pair_timeout' and self.pair_wait_since is not None:
                self.dead_reckoning_budget.begin(self.pair_wait_since.to_sec(), self.pair_wait_xy)
                if xy is not None:
                    self.dead_reckoning_budget.observe(xy)
            else:
                self.dead_reckoning_budget.begin(now.to_sec(), xy)
            if self.mode == self.TRACKING:
                self.recovery_candidates.clear()
                self.heading_transform_history.clear()
                self.set_mode(self.DEAD_RECKONING, reason)
        if self.transform_initialized and self.allow_nonfixed_runtime and not good:
            position_good, position_reason = self.data_quality(now, runtime_position=True)
            if position_good:
                self.recovery_candidates.clear()
                self.heading_transform_history.clear()
                self.update_runtime_position(now)
                return
            if position_reason == 'position_source_time_mismatch':
                self.invalidate_alignment_error(position_reason)
                if self.wait_for_position_pair(now):
                    return
                position_reason = reason = 'position_pair_timeout'
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
            self.invalidate_alignment_error(reason)
            if self.wait_for_position_pair(now):
                return
            reason = 'position_pair_timeout'
        if not good:
            self.invalidate_alignment_error(reason)
            self.recovery_diagnostics = {'valid': False, 'reason': reason, 'samples': 0}
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
        self.pair_wait_since = self.pair_wait_xy = None
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
                    anchor = self.current_anchor()
                    projected = [self.project_alignment(p, anchor) for p in samples]
                    mean_x = sum(p[0] for p in projected)/len(projected)
                    mean_y = sum(p[1] for p in projected)/len(projected)
                    c, s = math.cos(candidate[2]), math.sin(candidate[2])
                    candidate = (mean_x-c*anchor[0]+s*anchor[1],
                                 mean_y-s*anchor[0]-c*anchor[1], candidate[2])
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
        self.alignment_error_valid = True
        self.alignment_error_stamp = now.to_sec()
        self.alignment_error_reason = 'fixed_paired_observation'
        if self.check_alignment_conflict(now, position_error):
            return

        if self.mode == self.TRACKING:
            if self.last_tracking_candidate is not None:
                anchor = self.current_anchor()
                before = self.project_alignment(self.last_tracking_candidate, anchor)
                after = self.project_alignment(candidate, anchor)
                self.last_candidate_step_m = math.hypot(after[0]-before[0], after[1]-before[1])
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
            corrected = self.move_transform_toward(
                candidate, correction_dt, self.tracking_position_rate, self.tracking_yaw_rate
            )
            if not corrected:
                self.invalidate_alignment_error('invalid_correction')
                self.set_mode(self.DEAD_RECKONING, 'invalid_alignment_correction')
                return
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
            corrected = self.move_transform_toward(
                candidate, correction_dt, self.recovery_position_rate, self.recovery_yaw_rate
            )
            if not corrected:
                self.invalidate_alignment_error('invalid_correction')
                self.set_mode(self.DEAD_RECKONING, 'invalid_alignment_correction')
                return
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
            and not self.gnss_timebase.fault
            and not self.alignment_conflict
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
            "timebase_contract": {'mode': self.gnss_timebase.mode,
                'fault': self.gnss_timebase.fault, 'session': self.gnss_timebase.session,
                'clock_ready': self.gnss_timebase.clock_ready, 'wait_reason': self.gnss_timebase.wait_reason,
                'metadata_samples': len(self.gnss_timebase.metadata)},
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
            "latest_heading_health": self.latest_heading_health(now),
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
            "position_error_reference": "current_vehicle_anchor",
            "alignment_error_valid": self.alignment_error_valid,
            "alignment_error_reason": self.alignment_error_reason,
            "alignment_error_stamp": self.alignment_error_stamp,
            "alignment_error_age_sec": None if self.alignment_error_stamp is None else
                max(0., now.to_sec()-self.alignment_error_stamp),
            "source_pairing": self.source_pairing,
            "observation_buffer": self.observation_buffer_diagnostics,
            "recovery_window": self.recovery_diagnostics,
            "alignment_correction": self.correction_diagnostics,
            "alignment_correction_age_sec": None if self.correction_diagnostics.get('stamp') is None else
                max(0., now.to_sec()-self.correction_diagnostics['stamp']),
            "alignment_conflict": {**self.alignment_conflict_diagnostics,
                "latched": self.alignment_conflict,
                "position_limit_m": self.alignment_conflict_position_m,
                "stable_sec": self.alignment_conflict_stable_sec,
                "resume_policy": "park_verify_restart_system_and_send_new_route"},
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
        with self.state_lock:
            self._timer_cb(_event)

    def _timer_cb(self, _event):
        now = rospy.Time.now()
        self.gnss_timebase.tick(now.to_nsec(), time.monotonic_ns())
        if self.gnss_timebase.mode == 'measurement' and not self.gnss_timebase.clock_ready:
            rospy.logwarn_throttle(3., 'Waiting for GNSS measurement clock: %s (NUC sync + FINE PVTSLNA + GGA UTC)',
                                   self.gnss_timebase.wait_reason)
        if self.gnss_timebase.fault:
            self.stable_since = None
            self.recovery_candidates.clear()
            self.heading_transform_history.clear()
            self.local_pose_history.clear()
            self.runtime_position_window.clear()
            rospy.logwarn_throttle(2., 'GPS timebase fault: %s; park and restart localization/system',
                                   self.gnss_timebase.fault)
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
