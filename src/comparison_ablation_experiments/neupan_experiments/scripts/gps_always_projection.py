#!/usr/bin/env python3
# Module contract: Opt-in experiment GPS projection. Quality-independent GPS fusion is
# confined to this experiment; original measurement timestamps and invalid-number checks
# are retained.

"""Experimental GPS observations without solution-quality selection, never a fake fix."""
import json
import math
import time


# Project latitude/longitude into local map x/y metres and return the usable horizontal
# variances. Reject invalid coordinates/covariance without selecting observations by RTK
# quality.
# Inputs: latitude, longitude, covariance, origin_lat, origin_lon.
# Explicit rejection paths: ValueError.
def project_fix(latitude, longitude, covariance, origin_lat, origin_lon):
    if not all(math.isfinite(v) for v in (latitude, longitude, origin_lat, origin_lon)):
        raise ValueError('nonfinite coordinates')
    if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
        raise ValueError('coordinates outside geographic domain')
    scale = 6378137.0 * math.pi / 180.0
    # Keep the measured covariance/weight, but numerical invalidity is not a
    # usable observation. This is not a Fixed/Float/HDOP or satellite gate.
    variances = [float(covariance[i]) for i in (0, 4)]
    if not all(math.isfinite(v) and v >= 0 for v in variances):
        raise ValueError('nonfinite/negative covariance')
    variances = [max(0.0001, v) for v in variances]
    return (
        (longitude - origin_lon) * scale * math.cos(math.radians(origin_lat)),
        (latitude - origin_lat) * scale,
        variances,
    )


# Publish experiment-only GPS/heading observations for the lag-aware EKF. This node is
# not the production GNSS admission policy.
class GpsAlwaysProjection:
    # Configure the experimental map origin, heading convention, retained source
    # samples, ROS publishers/subscribers, and publication timer.
    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import Bool, String

        self.rospy = rospy
        self.Odometry = Odometry
        self.Bool = Bool
        self.String = String
        self.origin = (
            float(rospy.get_param('~origin_lat')),
            float(rospy.get_param('~origin_lon')),
        )
        self.latest_source = None
        self.count = 0
        self.last_fix = None
        self.samples = {}
        self.lio = None
        self.heading_source = None
        self.heading_count = 0
        self.rotation = float(rospy.get_param('~heading_rotation_deg', 90.0))
        self.north_cw = bool(rospy.get_param('~heading_source_is_north_cw', True))
        self.output = rospy.Publisher('/odometry/gps', Odometry, queue_size=20)
        self.heading_output = rospy.Publisher(
            '/experiments/gnss_heading', Odometry, queue_size=10
        )
        self.safety = rospy.Publisher('/localization_safe', Bool, queue_size=2)
        self.status = rospy.Publisher('/gps_guard/status', String, queue_size=2)
        rospy.Subscriber('/gps/fix', NavSatFix, self.fix, queue_size=50)
        rospy.Subscriber('/gnss/solution_status', String, self.heading, queue_size=10)
        rospy.Subscriber(
            '/odometry/local',
            Odometry,
            lambda msg: self.odom('local', msg),
            queue_size=10,
        )
        rospy.Subscriber(
            '/odometry/global',
            Odometry,
            lambda msg: self.odom('global', msg),
            queue_size=10,
        )
        rospy.Subscriber(
            '/lio_guard/healthy',
            Bool,
            lambda msg: setattr(self, 'lio', (time.monotonic(), msg.data)),
            queue_size=2,
        )
        rospy.Timer(rospy.Duration(0.1), self.tick)

    # Retain local odometry needed to relate experimental GPS observations to vehicle
    # motion.
    # Inputs: key, message.
    def odom(self, key, message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        finite = all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        self.samples[key] = (time.monotonic(), message.header.stamp.to_sec(), finite)

    # Handle a newly received GPS fix using its original source timestamp rather than
    # replaying a prior fix as fresh.
    # Inputs: message.
    def fix(self, message):
        try:
            x, y, variance = project_fix(
                message.latitude,
                message.longitude,
                message.position_covariance,
                *self.origin,
            )
            stamp = message.header.stamp.to_nsec()
            if (
                stamp <= 0
                or self.latest_source is not None
                and stamp <= self.latest_source
            ):
                return
        except (ValueError, TypeError, IndexError, OverflowError) as error:
            self.rospy.logwarn_throttle(
                8.0,
                'GPS always-fused experiment: unusable numeric observation: %s',
                error,
            )
            return
        result = self.Odometry()
        result.header.stamp = message.header.stamp
        result.header.frame_id = 'map'
        result.child_frame_id = 'base_link'
        result.pose.pose.position.x = x
        result.pose.pose.position.y = y
        result.pose.pose.orientation.w = 1.0
        result.pose.covariance[0] = variance[0]
        result.pose.covariance[7] = variance[1]
        self.output.publish(result)
        self.latest_source = stamp
        self.count += 1
        self.last_fix = (time.monotonic(), message.status.status)

    def heading(self, message):
        """Use the finite receiver heading at its OWN real PVT epoch, even Float."""
        try:
            value = json.loads(message.data)
            stamp = value['stamp_ns']
            angle = float(value['heading_deg'])
            if (
                not math.isfinite(angle)
                or not 0 <= angle <= 360
                or type(stamp) is not int
                or stamp <= 0
            ):
                return
            if self.heading_source is not None and stamp <= self.heading_source:
                return
            angle += self.rotation
            yaw = math.radians(90.0 - angle if self.north_cw else angle)
        except (ValueError, TypeError, KeyError, OverflowError):
            return
        result = self.Odometry()
        result.header.stamp = self.rospy.Time(stamp // 1000000000, stamp % 1000000000)
        result.header.frame_id = 'map'
        result.child_frame_id = 'base_link'
        result.pose.pose.orientation.z = math.sin(yaw / 2.0)
        result.pose.pose.orientation.w = math.cos(yaw / 2.0)
        # Explicit experiment assumption, equal for Fixed/Float (not a quality gate).
        result.pose.covariance[35] = math.radians(5.0) ** 2
        self.heading_output.publish(result)
        self.heading_source = stamp
        self.heading_count += 1

    # Evaluate and publish the currently eligible experimental observations on the timer
    # callback.
    # Inputs: _event.
    def tick(self, _event):
        now = time.monotonic()
        utc = self.rospy.Time.now().to_sec()
        # No GPS quality/age/distance-budget stop. Still require live, finite
        # estimator/local odometry and the existing independent LIO health.
        ready = all(
            key in self.samples
            and self.samples[key][2]
            and 0 <= now - self.samples[key][0] <= 0.5
            and -0.05 <= utc - self.samples[key][1] <= 0.5
            for key in ('local', 'global')
        )
        ready = bool(
            ready
            and self.count > 0
            and self.heading_count > 0
            and self.lio is not None
            and self.lio[1]
            and 0 <= now - self.lio[0] <= 1.5
        )
        self.safety.publish(self.Bool(data=ready))
        self.status.publish(
            self.String(
                data=json.dumps(
                    dict(
                        stamp=utc,
                        mode='EXPERIMENT_GPS_ALWAYS',
                        initialized=self.count > 0
                        and self.heading_count > 0
                        and 'global' in self.samples,
                        localization_safe=ready,
                        quality_gate_enabled=False,
                        reason=(
                            'gps_quality_not_gated'
                            if ready
                            else 'waiting_live_finite_ekf_local_lio'
                        ),
                        gps_observations=self.count,
                        heading_observations=self.heading_count,
                        gps_status=None if self.last_fix is None else self.last_fix[1],
                        gps_receive_age_sec=(
                            None if self.last_fix is None else now - self.last_fix[0]
                        ),
                    ),
                    allow_nan=False,
                )
            )
        )


if __name__ == '__main__':
    import rospy

    rospy.init_node('gps_fix_to_map_odom')
    GpsAlwaysProjection()
    rospy.spin()
