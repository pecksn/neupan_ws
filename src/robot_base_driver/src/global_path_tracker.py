#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GlobalPathTracker(object):
    def __init__(self):
        rospy.init_node("global_path_tracker")

        self.path_topic = rospy.get_param("~path_topic", "/osm_global_path")
        self.odom_topic = rospy.get_param("~odom_topic", "/odometry/global")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/neupan_cmd_vel")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")

        self.control_rate = rospy.get_param("~control_rate", 10.0)
        self.lookahead_dist = rospy.get_param("~lookahead_dist", 2.5)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 1.2)
        self.goal_stop_tolerance = rospy.get_param("~goal_stop_tolerance", self.goal_tolerance)
        self.slowdown_radius = rospy.get_param("~slowdown_radius", 4.0)
        self.wheelbase = rospy.get_param("~wheelbase", 0.67)
        self.cruise_speed = rospy.get_param("~cruise_speed", 0.8)
        self.min_speed = rospy.get_param("~min_speed", 0.25)
        self.max_steering = rospy.get_param("~max_steering", 0.43)
        self.max_heading_error_for_drive = rospy.get_param("~max_heading_error_for_drive", 1.2)
        self.final_segment_lateral_tolerance = rospy.get_param("~final_segment_lateral_tolerance", 1.5)
        self.stop_on_missing_path = rospy.get_param("~stop_on_missing_path", True)
        self.debug_log = rospy.get_param("~debug_log", True)
        self.yaw_offset_deg = rospy.get_param("~yaw_offset_deg", 0.0)
        self.yaw_offset_rad = math.radians(self.yaw_offset_deg)

        self.path_points = []
        self.robot_pose = None
        self.current_goal = None
        self.goal_active = False
        self.last_path_stamp = None

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=1)

        rospy.loginfo(
            "global_path_tracker: path=%s odom=%s cmd=%s",
            self.path_topic,
            self.odom_topic,
            self.cmd_topic,
        )

    def path_callback(self, msg):
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.last_path_stamp = rospy.Time.now()
        self.goal_active = len(self.path_points) >= 2

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = wrap_to_pi(yaw + self.yaw_offset_rad)
        self.robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )

    def goal_callback(self, msg):
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_active = True

    def publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def find_target_point(self, x, y):
        if not self.path_points:
            return None, None

        closest_idx = 0
        closest_dist = float("inf")
        for i, (px, py) in enumerate(self.path_points):
            d = math.hypot(px - x, py - y)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        target_idx = len(self.path_points) - 1
        for i in range(closest_idx, len(self.path_points)):
            px, py = self.path_points[i]
            if math.hypot(px - x, py - y) >= self.lookahead_dist:
                target_idx = i
                break

        return target_idx, self.path_points[target_idx]

    def should_stop_at_goal(self, x, y):
        if self.current_goal is not None:
            goal_x, goal_y = self.current_goal
            clicked_goal_dist = math.hypot(goal_x - x, goal_y - y)
            if clicked_goal_dist <= self.goal_stop_tolerance:
                return True, "clicked_goal_distance"

        if not self.path_points:
            return False, "no_path"

        goal_x, goal_y = self.path_points[-1]
        goal_dist = math.hypot(goal_x - x, goal_y - y)
        if goal_dist <= self.goal_stop_tolerance:
            return True, "goal_distance"

        if len(self.path_points) < 2:
            return False, "single_point_path"

        p0x, p0y = self.path_points[-2]
        p1x, p1y = self.path_points[-1]
        seg_x = p1x - p0x
        seg_y = p1y - p0y
        seg_len = math.hypot(seg_x, seg_y)
        if seg_len < 1e-6:
            return False, "degenerate_final_segment"

        ux = seg_x / seg_len
        uy = seg_y / seg_len
        rel_x = x - p0x
        rel_y = y - p0y
        along = rel_x * ux + rel_y * uy
        lateral = abs(rel_x * (-uy) + rel_y * ux)

        # Stop if the robot has reached or passed the terminal path segment
        # while remaining close to the lane centerline.
        if along >= seg_len and lateral <= self.final_segment_lateral_tolerance:
            return True, "passed_final_segment"

        return False, "not_at_goal"

    def compute_command(self):
        if self.robot_pose is None:
            if self.debug_log:
                rospy.logwarn_throttle(1.0, "global_path_tracker waiting for robot pose from %s", self.odom_topic)
            return None
        if not self.goal_active:
            if self.debug_log:
                rospy.loginfo_throttle(1.0, "global_path_tracker idle: no active goal")
            return Twist()
        if not self.path_points:
            if self.debug_log:
                rospy.logwarn_throttle(1.0, "global_path_tracker waiting for path from %s", self.path_topic)
            return Twist() if self.stop_on_missing_path else None

        x, y, yaw = self.robot_pose
        goal_x, goal_y = self.path_points[-1]
        goal_dist = math.hypot(goal_x - x, goal_y - y)

        should_stop, stop_reason = self.should_stop_at_goal(x, y)
        if should_stop:
            self.goal_active = False
            if self.debug_log:
                rospy.loginfo_throttle(
                    1.0,
                    "global_path_tracker stop: reason=%s goal_dist=%.2f stop_tol=%.2f",
                    stop_reason,
                    goal_dist,
                    self.goal_stop_tolerance,
                )
            return Twist()

        _, target = self.find_target_point(x, y)
        if target is None:
            return Twist()

        tx, ty = target
        alpha = wrap_to_pi(math.atan2(ty - y, tx - x) - yaw)
        speed_reason = "tracking"

        if abs(alpha) > self.max_heading_error_for_drive:
            speed = 0.0
            speed_reason = "heading_error_too_large"
        else:
            speed_scale = clamp(goal_dist / max(self.slowdown_radius, 1e-3), 0.0, 1.0)
            speed = max(self.min_speed, self.cruise_speed * speed_scale)
            speed = min(speed, self.cruise_speed)

        ld = max(math.hypot(tx - x, ty - y), 0.3)
        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)
        steering = clamp(steering, -self.max_steering, self.max_steering)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering
        if self.debug_log:
            rospy.loginfo_throttle(
                1.0,
                "global_path_tracker: pose=(%.2f, %.2f, %.1fdeg) target=(%.2f, %.2f) goal_dist=%.2f alpha=%.1fdeg cmd_v=%.2f cmd_steer=%.1fdeg reason=%s",
                x,
                y,
                math.degrees(yaw),
                tx,
                ty,
                goal_dist,
                math.degrees(alpha),
                cmd.linear.x,
                math.degrees(cmd.angular.z),
                speed_reason,
            )
        return cmd

    def run(self):
        rate = rospy.Rate(self.control_rate)
        while not rospy.is_shutdown():
            cmd = self.compute_command()
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    try:
        GlobalPathTracker().run()
    except rospy.ROSInterruptException:
        pass
