#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
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

        self.earth_radius = 6378137.0
        self.last_fix = None
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

    def fix_cb(self, msg):
        if not self._finite(msg.latitude) or not self._finite(msg.longitude):
            return
        self.last_fix = msg
        self.last_fix_recv = rospy.Time.now()

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
        if self.last_fix is None:
            return

        fix = self.last_fix
        now = rospy.Time.now()
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


if __name__ == "__main__":
    try:
        GpsFixToMapOdom()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
