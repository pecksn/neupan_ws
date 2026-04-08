#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker


class GPSGoalBridge:
    def __init__(self):
        rospy.init_node('gps_goal_bridge')

        self.origin_lat = rospy.get_param('~default_lat', 23.0398)
        self.origin_lon = rospy.get_param('~default_lon', 113.39002)
        self.input_topic = rospy.get_param('~input_topic', '/gps_goal_ll')
        self.output_topic = rospy.get_param('~output_topic', '/move_base_simple/goal')
        self.goal_yaw = rospy.get_param('~goal_yaw', 0.0)
        self.reject_invalid_status = rospy.get_param('~reject_invalid_status', True)
        self.publish_marker = rospy.get_param('~publish_marker', True)
        self.marker_topic = rospy.get_param('~marker_topic', '/gps_goal_marker')

        self.earth_radius = 6378137.0

        self.goal_pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=1, latch=True)
        rospy.Subscriber(self.input_topic, NavSatFix, self.goal_ll_callback)

        rospy.loginfo(
            f"GPS goal bridge ready: {self.input_topic} (lat/lon) -> {self.output_topic} (map goal)"
        )

    def _latlon_to_xy(self, lat, lon):
        x = (lon - self.origin_lon) * (math.pi / 180.0) * self.earth_radius * math.cos(
            self.origin_lat * math.pi / 180.0
        )
        y = (lat - self.origin_lat) * (math.pi / 180.0) * self.earth_radius
        return x, y

    def _quat_from_yaw(self, yaw):
        half = 0.5 * yaw
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def _publish_marker(self, x, y):
        if not self.publish_marker:
            return

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'gps_goal'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.6
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.95
        self.marker_pub.publish(marker)

    def goal_ll_callback(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            rospy.logwarn('GPS goal bridge received NaN lat/lon, ignored')
            return

        if self.reject_invalid_status and msg.status.status < 0:
            rospy.logwarn('GPS goal bridge received invalid NavSatFix status, ignored')
            return

        x, y = self._latlon_to_xy(msg.latitude, msg.longitude)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        qx, qy, qz, qw = self._quat_from_yaw(self.goal_yaw)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        self.goal_pub.publish(goal)
        self._publish_marker(x, y)

        rospy.loginfo(
            f"Published map goal from lat/lon: ({msg.latitude:.8f}, {msg.longitude:.8f}) -> ({x:.2f}, {y:.2f})"
        )


if __name__ == '__main__':
    try:
        GPSGoalBridge()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
