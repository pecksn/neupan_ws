# Module contract: Measured-motion feedback used for diagnosis, not control synthesis.
# Pose timestamps establish motion intervals; receive times establish evidence
# freshness.

"""Read-only measured-motion diagnostics; never modifies NeuPAN controls."""

import math
import threading
import numpy as np


# Feedback is diagnostic evidence from measured poses, not a second controller.
# Source stamps establish the measurement interval; receipt time only determines
# whether the latest evidence is still usable. Do not refresh a held sample by
# repeatedly logging or reading it.
class MotionFeedback:
    """Signed wheel-odom pose motion; optional independent steering encoder.

    Never uses /odom.twist (the current driver fills it with zero), converts a
    turn-rate estimate to a fictitious measured angle, or refreshes old epochs.
    """

    # Validate measurement-age and motion limits, then initialize pose/encoder evidence
    # and its lock.
    # Inputs: timeout, max_speed, max_yaw_rate, sample_max_interval, instant_max_window,
    # instant_timeout.
    # Explicit rejection paths: ValueError.
    def __init__(
        self,
        timeout=0.5,
        max_speed=2.0,
        max_yaw_rate=math.pi / 2,
        sample_max_interval=1.5,
        instant_max_window=0.2,
        instant_timeout=0.2,
    ):
        self.timeout = timeout
        self.sample_max_interval = sample_max_interval
        self.instant_max_window = instant_max_window
        self.instant_timeout = instant_timeout
        if not all(
            math.isfinite(v) and v > 0
            for v in (
                timeout,
                sample_max_interval,
                instant_max_window,
                instant_timeout,
                max_speed,
                max_yaw_rate,
            )
        ):
            raise ValueError('invalid motion feedback timing/limits')
        self.max_speed, self.max_yaw_rate = max_speed, max_yaw_rate
        self.previous = None
        self.speed = self.steering = None
        self.lock = threading.RLock()

    # Update measured planar motion from wheel-odometry poses using original source
    # epochs and frame identity.
    # Inputs: pose, stamp_ns (nanoseconds), received, frame, now_ns (nanoseconds).
    def odom(self, pose, stamp_ns, received, frame, now_ns=None):
        pose = np.asarray(pose, float).reshape(3)
        with self.lock:
            old = self.previous
            if (
                not np.isfinite(pose).all()
                or stamp_ns <= 0
                or not frame
                or (
                    now_ns is not None
                    and not 0 <= (now_ns - stamp_ns) / 1.0e9 <= self.timeout
                )
            ):
                self.previous = None
                self.speed = None
                return
            if old is not None and stamp_ns <= old[1]:
                return  # duplicates/out-of-order cannot refresh measurement age
            self.previous = (pose.copy(), stamp_ns, received, frame)
            if old is None or old[3] != frame:
                self.speed = None
                return
            dt = (stamp_ns - old[1]) / 1.0e9
            delta = pose[:2] - old[0][:2]
            yaw_delta = math.atan2(
                math.sin(pose[2] - old[0][2]), math.cos(pose[2] - old[0][2])
            )
            yaw = old[0][2] + yaw_delta / 2
            forward = np.array([math.cos(yaw), math.sin(yaw)])
            speed = float(delta @ forward / dt)
            lateral = float(delta @ np.array([-forward[1], forward[0]]) / dt)
            if (
                not 0.01 <= dt <= self.sample_max_interval
                or np.linalg.norm(delta) / dt > self.max_speed
                or abs(lateral) > 0.8
                or abs(yaw_delta) / dt > self.max_yaw_rate
            ):
                self.speed = None
                return
            self.speed = (speed, stamp_ns, received, old[1], dt)

    # Retain independent steering-encoder evidence rather than deriving a fictitious
    # encoder angle from commanded motion.
    # Inputs: angle, stamp_ns (nanoseconds), received, now_ns (nanoseconds).
    def encoder(self, angle, stamp_ns, received, now_ns=None):
        with self.lock:
            if (
                not math.isfinite(angle)
                or stamp_ns <= 0
                or (
                    now_ns is not None
                    and not 0 <= (now_ns - stamp_ns) / 1.0e9 <= self.timeout
                )
            ):
                self.steering = None
                return
            if self.steering and stamp_ns <= self.steering[1]:
                return
            self.steering = (float(angle), stamp_ns, received)

    # Report measured motion and steering evidence without modifying the planner
    # command.
    # Inputs: now_ns (nanoseconds), now.
    def snapshot(self, now_ns, now):
        with self.lock:

            # Check that the supplied source/receive timing evidence is usable for
            # motion diagnostics.
            # Inputs: value.
            def fresh(value):
                return (
                    value is not None
                    and 0 <= (now_ns - value[1]) / 1.0e9 <= self.timeout
                    and 0 <= now - value[2] <= self.timeout
                )

            speed = self.speed if fresh(self.speed) else None
            steering = (
                self.steering
                if fresh(self.steering)
                and (now_ns - self.steering[1]) / 1.0e9 <= 0.5
                and now - self.steering[2] <= 0.5
                else None
            )
            instant = bool(
                speed
                and speed[4] <= self.instant_max_window
                and 0 <= (now_ns - speed[1]) / 1.0e9 <= self.instant_timeout
                and 0 <= now - speed[2] <= self.instant_timeout
            )
            # A 1 Hz difference describes the PREVIOUS one-second interval.
            # It is useful diagnostics, not a tight 0.1 s first-action bound.
            return dict(
                speed_mps=speed[0] if instant else None,
                average_speed_mps=None if speed is None else speed[0],
                average_start_ns=None if speed is None else speed[3],
                average_end_ns=None if speed is None else speed[1],
                sample_interval_sec=None if speed is None else speed[4],
                average_midpoint_age_ms=(
                    None
                    if speed is None
                    else (2 * now_ns - speed[1] - speed[3]) / 2.0e6
                ),
                instant_speed_usable=instant,
                speed_degraded_reason=(
                    None
                    if instant
                    else (
                        'sample_window_not_instantaneous'
                        if speed and speed[4] > self.instant_max_window
                        else (
                            'instant_feedback_stale'
                            if speed
                            else 'average_missing_or_stale'
                        )
                    )
                ),
                steering_rad=None if steering is None else steering[0],
                speed_source=(
                    'wheel_odom_pose_difference'
                    if instant
                    else (
                        'wheel_odom_interval_average_diagnostic_only'
                        if speed
                        else 'unavailable'
                    )
                ),
                steering_source=(
                    'independent_encoder'
                    if steering
                    else 'last_sent_command_not_encoder'
                ),
                odom_epoch_semantics='driver_header_currently_host_receipt_not_mcu_capture',
                stamp_ns=None if speed is None else speed[1],
                age_ms=None if speed is None else (now_ns - speed[1]) / 1.0e6,
                steering_stamp_ns=None if steering is None else steering[1],
                steering_age_ms=(
                    None if steering is None else (now_ns - steering[1]) / 1.0e6
                ),
            )
