#!/usr/bin/env python3

import math
import json
import time
import threading
import copy
from collections import OrderedDict, Counter
from gnss_timebase import GnssTimebase

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler


class GpsFixToMapOdom:
    """Project raw GPS fix into the same map frame used by the OSM planner.

    This node intentionally does not use robot_localization/navsat_transform_node.
    The OSM A* planner converts OSM node lat/lon to map x/y with a local
    equirectangular projection around origin_lat/origin_lon. When GPS was
    converted by navsat_transform_node, /odometry/gps was based on the odom,
    datum, yaw, and TF chain instead of this exact OSM projection, which caused
    a stable meter-level offset between the RViz vehicle pose and the blue OSM
    centerline. Keeping GPS and OSM on the same projection is the important
    invariant for outdoor navigation here.
    """

    def __init__(self):
        rospy.init_node("gps_fix_to_map_odom")

        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.heading_topic = rospy.get_param("~heading_topic", "/heading")
        self.output_topic = rospy.get_param("~output_topic", "/odometry/gps")
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.child_frame_id = rospy.get_param("~child_frame_id", "base_link")
        self.origin_lat = float(rospy.get_param("~origin_lat", 23.0398))
        self.origin_lon = float(rospy.get_param("~origin_lon", 113.39002))
        self.zero_altitude = bool(rospy.get_param("~zero_altitude", True))
        self.use_heading = bool(rospy.get_param("~use_heading", True))
        self.heading_timeout_sec = max(0.1, float(rospy.get_param("~heading_timeout_sec", 0.5)))
        self.publish_rate = max(1.0, float(rospy.get_param("~publish_rate", 30.0)))
        self.transport_limit_sec=float(rospy.get_param('~gnss_transport_limit_sec',0.))
        if (not math.isfinite(self.transport_limit_sec) or not 0<=self.transport_limit_sec<=4.
                or self.transport_limit_sec>1. and rospy.get_param('~gnss_common_utc_verified',False) is not True):
            raise ValueError('extended GNSS transport requires independently verified common UTC')
        self.gga_period_sec=float(rospy.get_param('~gga_period_sec',.05))
        self.cadence_margin_sec=float(rospy.get_param('~cadence_margin_sec',.3))

        self.earth_radius = 6378137.0
        self.last_fix = None
        self.time_lock = threading.RLock()
        self.timebase = GnssTimebase(rospy.get_param('~gps_time_mode', 'shadow'),
                                    rospy.get_param('~gps_time_clock_verified', False))
        self.last_published_source_ns = None
        self.pending_fixes = OrderedDict()
        self.pair_wait_sec = float(rospy.get_param('~pair_wait_sec', .15))
        if not math.isfinite(self.pair_wait_sec) or not .05 <= self.pair_wait_sec <= .2:
            raise ValueError('pair_wait_sec must be between .05 and .2 seconds')
        self.pair_counts = Counter()
        self.pair_max_wait_sec = 0.
        self.pair_state = 'waiting_for_fix'
        self.last_pair_report = float('-inf')
        self.pair_pub = rospy.Publisher('/gps/projection_pairing', String, queue_size=2)
        rospy.Subscriber('/gnss/time_diagnostics', String, self.time_metadata_cb, queue_size=200)
        self.last_fix_recv = None
        self.last_heading = None
        self.last_heading_recv = None
        self.last_heading_yaw = 0.0

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=10)
        rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_cb, queue_size=20)
        if self.use_heading:
            rospy.Subscriber(self.heading_topic, QuaternionStamped, self.heading_cb, queue_size=20)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.timer_cb)
        rospy.loginfo(
            "gps_fix_to_map_odom: %s + %s -> %s (origin=%.6f, %.6f frame=%s child=%s)",
            self.fix_topic,
            self.heading_topic,
            self.output_topic,
            self.origin_lat,
            self.origin_lon,
            self.frame_id,
            self.child_frame_id,
        )

    @staticmethod
    def _finite(value):
        return math.isfinite(value)

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def latlon_to_xy(self, lat, lon):
        # Keep this formula identical to osm_map_handler/global_astar_node.cpp.
        # If one side changes, GPS pose and OSM path will drift apart in RViz
        # and in the path tracker.
        x = (lon - self.origin_lon) * (math.pi / 180.0) * self.earth_radius * math.cos(
            self.origin_lat * math.pi / 180.0
        )
        y = (lat - self.origin_lat) * (math.pi / 180.0) * self.earth_radius
        return x, y

    def time_metadata_cb(self, msg):
        with self.time_lock:
            self._time_metadata_cb(msg)

    def _time_metadata_cb(self, msg):
        try:
            event = json.loads(msg.data)
            if not isinstance(event, dict):
                raise ValueError('time metadata must be an object')
            self.timebase.receive(event, rospy.Time.now().to_nsec(), time.monotonic_ns())
        except (TypeError, ValueError):
            if self.timebase.mode == 'measurement':
                self.timebase.fault = 'malformed_time_metadata'

    def fix_cb(self, msg):
        if not self._finite(msg.latitude) or not self._finite(msg.longitude):
            return
        with self.time_lock:
            self.last_fix = msg
            self.last_fix_recv = rospy.Time.now()
            stamp = msg.header.stamp.to_nsec()
            if self.timebase.mode == 'measurement':
                if stamp <= 0 or (self.last_published_source_ns is not None and stamp <= self.last_published_source_ns):
                    self.pair_counts['duplicate_or_old_fix'] += 1
                    return
                if stamp not in self.pending_fixes:
                    self.pending_fixes[stamp] = (copy.deepcopy(msg), time.monotonic_ns(), self.timebase.session)
                while len(self.pending_fixes) > 64:
                    self.pending_fixes.popitem(last=False)
                    self.pair_counts['capacity_expired'] += 1

    def heading_cb(self, msg):
        q = msg.quaternion
        if not all(self._finite(v) for v in (q.x, q.y, q.z, q.w)):
            return
        self.last_heading = msg
        self.last_heading_recv = rospy.Time.now()
        self.last_heading_yaw = self._yaw_from_quaternion(q)

    def build_covariance(self, fix):
        cov = [0.0] * 36
        if fix is None:
            return cov
        cov[0] = float(fix.position_covariance[0])
        cov[7] = float(fix.position_covariance[4])
        cov[14] = 0.0 if self.zero_altitude else float(fix.position_covariance[8])
        return cov

    def timer_cb(self, _event):
        with self.time_lock:
            try:
                self._timer_cb(_event)
            finally:
                mono = time.monotonic()
                if mono-self.last_pair_report >= .5:
                    self.pair_pub.publish(String(data=json.dumps(dict(
                        stamp=rospy.Time.now().to_sec(), state=self.pair_state,
                        pending=len(self.pending_fixes), max_wait_sec=self.pair_max_wait_sec,
                        counts=dict(self.pair_counts), projected_source_ns=self.last_published_source_ns))))
                    self.last_pair_report = mono

    def _timer_cb(self, _event):
        if self.last_fix is None:
            return

        fix = self.last_fix
        now = rospy.Time.now()
        if self.timebase.mode == 'measurement':
            mono = time.monotonic_ns()
            fix = None
            reason = 'pair_pending' if self.pending_fixes else 'waiting_for_fix'
            for stamp in sorted(self.pending_fixes, reverse=True):
                message, arrived, session = self.pending_fixes[stamp]
                waited = max(0., (mono-arrived)*1.e-9)
                self.pair_max_wait_sec = max(self.pair_max_wait_sec, waited)
                if self.last_published_source_ns is not None and stamp <= self.last_published_source_ns:
                    self.pending_fixes.pop(stamp); continue
                if session is not None and session != self.timebase.session:
                    self.pending_fixes.pop(stamp)
                    self.pair_counts['session_rejected'] += 1
                    reason = 'time_session_changed_restart_required'; continue
                good, why = self.timebase.check('GGA', stamp, now.to_nsec(), mono,
                    self.transport_limit_sec or 1.,self.gga_period_sec if self.transport_limit_sec else 0.,self.cadence_margin_sec)
                if good:
                    fix = message; break
                pending = why in ('position_source_time_mismatch', 'waiting_for_measurement_clock') and not self.timebase.fault
                if pending and waited <= self.pair_wait_sec:
                    continue  # A newer pending fix cannot hide an older complete pair.
                reason = 'pair_metadata_timeout' if pending else why
                if waited > self.pair_wait_sec or not pending:
                    self.pending_fixes.pop(stamp)
                    self.pair_counts['expired_or_rejected'] += 1
            if fix is None:
                self.pair_state = reason
                if reason in ('pair_pending', 'waiting_for_fix'):
                    rospy.logdebug_throttle(8., 'GPS projection pair_pending: waiting for exact epoch metadata')
                else:
                    rospy.logwarn_throttle(8., 'GPS projection time gate: %s', reason)
                return
        source_ns = fix.header.stamp.to_nsec()
        good, reason = self.timebase.check('GGA', source_ns, now.to_nsec(), time.monotonic_ns(),
            self.transport_limit_sec or 1.,self.gga_period_sec if self.transport_limit_sec else 0.,self.cadence_margin_sec)
        if not good:
            rospy.logwarn_throttle(2., 'GPS projection time gate: %s', reason)
            return
        if self.timebase.mode == 'measurement' and (
                source_ns <= 0 or self.last_published_source_ns is not None and source_ns <= self.last_published_source_ns):
            return
        x, y = self.latlon_to_xy(fix.latitude, fix.longitude)

        odom = Odometry()
        odom.header.stamp = fix.header.stamp if fix.header.stamp != rospy.Time() else now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0 if self.zero_altitude else float(fix.altitude)
        odom.pose.covariance = self.build_covariance(fix)

        if self.use_heading and self.last_heading_recv is not None:
            heading_age = (now - self.last_heading_recv).to_sec()
            if heading_age <= self.heading_timeout_sec:
                q = quaternion_from_euler(0.0, 0.0, self.last_heading_yaw)
                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2]
                odom.pose.pose.orientation.w = q[3]
            else:
                odom.pose.pose.orientation.w = 1.0
        else:
            odom.pose.pose.orientation.w = 1.0

        self.pub.publish(odom)
        self.last_published_source_ns = source_ns
        self.pair_state = 'paired'
        self.pair_counts['projected'] += 1
        for stamp in list(self.pending_fixes):
            if stamp <= source_ns:
                self.pending_fixes.pop(stamp)


if __name__ == "__main__":
    try:
        GpsFixToMapOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
