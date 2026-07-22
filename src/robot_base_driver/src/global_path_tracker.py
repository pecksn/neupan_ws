#!/usr/bin/env python
import json
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String
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
        self.clicked_goal_stop_tolerance = rospy.get_param("~clicked_goal_stop_tolerance", 2.0)
        self.projected_goal_stop_tolerance = rospy.get_param("~projected_goal_stop_tolerance", 1.5)
        self.goal_approach_radius = rospy.get_param("~goal_approach_radius", 4.0)
        self.goal_overshoot_margin = rospy.get_param("~goal_overshoot_margin", 0.5)
        self.end_path_point_window = int(rospy.get_param("~end_path_point_window", 3))
        self.end_path_point_tolerance = rospy.get_param("~end_path_point_tolerance", 1.5)
        self.terminal_index_overshoot_margin = rospy.get_param("~terminal_index_overshoot_margin", 0.3)
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
        self.use_tf_pose = rospy.get_param("~use_tf_pose", False)
        self.tf_map_frame = rospy.get_param("~tf_map_frame", "map")
        self.tf_base_frame = rospy.get_param("~tf_base_frame", "base_link")
        self.road_half_width = rospy.get_param("~road_half_width", 4.5)
        self.lateral_deadband = rospy.get_param("~lateral_deadband", self.road_half_width)
        self.straight_segment_angle_deg = rospy.get_param("~straight_segment_angle_deg", 10.0)
        self.straight_heading_deadband_deg = rospy.get_param("~straight_heading_deadband_deg", 8.0)
        self.status_topic = rospy.get_param("~status_topic", "/global_path_tracker/status")

        self.path_points = []
        self.robot_pose = None
        self.goal_active = False
        self.current_goal = None
        self.last_path_stamp = None
        self.closest_goal_dist_seen = None
        self.goal_entered_approach_zone = False
        self.closest_clicked_goal_dist_seen = None
        self.clicked_goal_entered_approach_zone = False
        self.closest_terminal_dist_seen = None
        self.path_cumulative = []
        self.pose_source = None
        self.tf_listener = tf.TransformListener() if self.use_tf_pose else None

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=20)
        rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=1)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=1)

        rospy.loginfo(
            "global_path_tracker: path=%s odom=%s cmd=%s use_tf_pose=%s",
            self.path_topic,
            self.odom_topic,
            self.cmd_topic,
            self.use_tf_pose,
        )

    def path_callback(self, msg):
        self.path_points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.path_cumulative = self.compute_cumulative_lengths(self.path_points)
        self.last_path_stamp = rospy.Time.now()
        self.goal_active = len(self.path_points) >= 2
        self.closest_goal_dist_seen = None
        self.goal_entered_approach_zone = False
        self.closest_clicked_goal_dist_seen = None
        self.clicked_goal_entered_approach_zone = False
        self.closest_terminal_dist_seen = None

    def odom_callback(self, msg):
        if self.use_tf_pose:
            return
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = wrap_to_pi(yaw + self.yaw_offset_rad)
        self.robot_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
        )
        self.pose_source = "odom"

    def update_tf_pose(self):
        if not self.use_tf_pose:
            return

        try:
            trans, rot = self.tf_listener.lookupTransform(
                self.tf_map_frame,
                self.tf_base_frame,
                rospy.Time(0),
            )
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            if self.debug_log:
                rospy.logwarn_throttle(
                    1.0,
                    "global_path_tracker waiting for tf %s -> %s: %s",
                    self.tf_map_frame,
                    self.tf_base_frame,
                    exc,
                )
            return

        _, _, yaw = euler_from_quaternion(rot)
        yaw = wrap_to_pi(yaw + self.yaw_offset_rad)
        self.robot_pose = (trans[0], trans[1], yaw)
        self.pose_source = "tf"

    def goal_callback(self, msg):
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_active = True
        self.closest_clicked_goal_dist_seen = None
        self.clicked_goal_entered_approach_zone = False

    def publish_stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    def _record_debug_sample(self, event, **kwargs):
        sample = {
            "stamp": rospy.Time.now().to_sec(),
            "event": event,
            "goal_active": self.goal_active,
            "path_points": len(self.path_points),
        }
        sample.update(kwargs)
        self.status_pub.publish(String(data=json.dumps(sample, separators=(",", ":"))))

    @staticmethod
    def compute_cumulative_lengths(points):
        cumulative = [0.0]
        for i in range(1, len(points)):
            px0, py0 = points[i - 1]
            px1, py1 = points[i]
            cumulative.append(cumulative[-1] + math.hypot(px1 - px0, py1 - py0))
        return cumulative

    def project_onto_path(self, x, y):
        if len(self.path_points) < 2:
            return None

        best = None
        best_lateral = float("inf")

        for i in range(len(self.path_points) - 1):
            x0, y0 = self.path_points[i]
            x1, y1 = self.path_points[i + 1]
            dx = x1 - x0
            dy = y1 - y0
            seg_len = math.hypot(dx, dy)
            if seg_len < 1e-6:
                continue

            ux = dx / seg_len
            uy = dy / seg_len
            rel_x = x - x0
            rel_y = y - y0
            t = clamp((rel_x * ux + rel_y * uy) / seg_len, 0.0, 1.0)
            proj_x = x0 + t * dx
            proj_y = y0 + t * dy
            lateral = math.hypot(x - proj_x, y - proj_y)
            s = self.path_cumulative[i] + t * seg_len

            if lateral < best_lateral:
                best_lateral = lateral
                best = {
                    "segment_idx": i,
                    "proj_x": proj_x,
                    "proj_y": proj_y,
                    "lateral": lateral,
                    "t": t,
                    "s": s,
                    "remaining": self.path_cumulative[-1] - s,
                }

        return best

    def segment_heading_at_projection(self, projection):
        if projection is None:
            return None
        seg_idx = projection.get("segment_idx", None)
        if seg_idx is None or seg_idx < 0 or seg_idx >= len(self.path_points) - 1:
            return None
        x0, y0 = self.path_points[seg_idx]
        x1, y1 = self.path_points[seg_idx + 1]
        return math.atan2(y1 - y0, x1 - x0)

    def find_target_point(self, x, y, projection=None):
        if not self.path_points:
            return None, None, None

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

        return closest_idx, closest_dist, self.path_points[target_idx]

    def should_stop_at_goal(self, x, y, closest_idx=None, closest_dist=None, projection=None):
        if self.current_goal is not None:
            clicked_goal_x, clicked_goal_y = self.current_goal
            clicked_goal_dist = math.hypot(clicked_goal_x - x, clicked_goal_y - y)
            if self.closest_clicked_goal_dist_seen is None:
                self.closest_clicked_goal_dist_seen = clicked_goal_dist
            else:
                self.closest_clicked_goal_dist_seen = min(self.closest_clicked_goal_dist_seen, clicked_goal_dist)

            if clicked_goal_dist <= self.goal_approach_radius:
                self.clicked_goal_entered_approach_zone = True

            if clicked_goal_dist <= self.clicked_goal_stop_tolerance:
                return True, "clicked_goal_distance"

            if (
                self.clicked_goal_entered_approach_zone
                and self.closest_clicked_goal_dist_seen is not None
                and clicked_goal_dist > self.closest_clicked_goal_dist_seen + self.goal_overshoot_margin
            ):
                return True, "clicked_goal_distance_rebound"

        if not self.path_points:
            return False, "no_path"

        if projection is not None:
            if projection["remaining"] <= self.projected_goal_stop_tolerance:
                return True, "projected_path_remaining"

        goal_x, goal_y = self.path_points[-1]
        goal_dist = math.hypot(goal_x - x, goal_y - y)
        if self.closest_goal_dist_seen is None:
            self.closest_goal_dist_seen = goal_dist
        else:
            self.closest_goal_dist_seen = min(self.closest_goal_dist_seen, goal_dist)

        if goal_dist <= self.goal_approach_radius:
            self.goal_entered_approach_zone = True

        if goal_dist <= self.goal_stop_tolerance:
            return True, "goal_distance"

        if (
            self.goal_entered_approach_zone
            and self.closest_goal_dist_seen is not None
            and goal_dist > self.closest_goal_dist_seen + self.goal_overshoot_margin
        ):
            return True, "goal_distance_rebound"

        if len(self.path_points) < 2:
            return False, "single_point_path"

        if closest_idx is None or closest_dist is None:
            closest_idx = 0
            closest_dist = float("inf")
            for i, (px, py) in enumerate(self.path_points):
                d = math.hypot(px - x, py - y)
                if d < closest_dist:
                    closest_dist = d
                    closest_idx = i

        terminal_start_idx = max(0, len(self.path_points) - max(1, self.end_path_point_window))
        if closest_idx >= terminal_start_idx and closest_dist <= self.end_path_point_tolerance:
            return True, "near_terminal_path_points"

        terminal_idx = len(self.path_points) - 1
        if closest_idx == terminal_idx:
            if self.closest_terminal_dist_seen is None:
                self.closest_terminal_dist_seen = closest_dist
            else:
                self.closest_terminal_dist_seen = min(self.closest_terminal_dist_seen, closest_dist)

            if (
                self.closest_terminal_dist_seen is not None
                and closest_dist > self.closest_terminal_dist_seen + self.terminal_index_overshoot_margin
            ):
                return True, "terminal_index_distance_rebound"

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
        self.update_tf_pose()
        if self.robot_pose is None:
            if self.debug_log:
                source = "tf {}->{}".format(self.tf_map_frame, self.tf_base_frame) if self.use_tf_pose else self.odom_topic
                rospy.logwarn_throttle(1.0, "global_path_tracker waiting for robot pose from %s", source)
            self._record_debug_sample("waiting_robot_pose", pose_source=self.pose_source)
            return None
        if not self.goal_active:
            if self.debug_log:
                rospy.loginfo_throttle(1.0, "global_path_tracker idle: no active goal")
            self._record_debug_sample("idle_no_goal")
            return Twist()
        if not self.path_points:
            if self.debug_log:
                rospy.logwarn_throttle(1.0, "global_path_tracker waiting for path from %s", self.path_topic)
            self._record_debug_sample("waiting_path")
            return Twist() if self.stop_on_missing_path else None

        x, y, yaw = self.robot_pose
        goal_x, goal_y = self.path_points[-1]
        goal_dist = math.hypot(goal_x - x, goal_y - y)
        projection = self.project_onto_path(x, y)
        closest_idx, closest_dist, target = self.find_target_point(x, y, projection)

        should_stop, stop_reason = self.should_stop_at_goal(
            x, y, closest_idx, closest_dist, projection
        )
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
            self._record_debug_sample(
                "stop",
                x=x,
                y=y,
                yaw=yaw,
                pose_source=self.pose_source,
                goal_dist=goal_dist,
                stop_reason=stop_reason,
                closest_idx=closest_idx,
                closest_dist=closest_dist,
                projection=projection,
            )
            return Twist()

        if target is None:
            self._record_debug_sample("no_target", x=x, y=y, yaw=yaw, projection=projection)
            return Twist()

        tx, ty = target
        alpha = wrap_to_pi(math.atan2(ty - y, tx - x) - yaw)
        speed_reason = "tracking"
        control_alpha = alpha

        segment_heading = self.segment_heading_at_projection(projection)
        if projection is not None and segment_heading is not None:
            segment_heading_error = wrap_to_pi(segment_heading - yaw)
            straight_like = abs(math.degrees(alpha)) <= self.straight_segment_angle_deg
            within_lane = abs(projection["lateral"]) <= self.lateral_deadband
            heading_small = abs(math.degrees(segment_heading_error)) <= self.straight_heading_deadband_deg
            if straight_like and within_lane and heading_small:
                control_alpha = 0.0
                speed_reason = "lane_deadband_hold"

        if abs(control_alpha) > self.max_heading_error_for_drive:
            speed = 0.0
            speed_reason = "heading_error_too_large"
        else:
            speed_scale = clamp(goal_dist / max(self.slowdown_radius, 1e-3), 0.0, 1.0)
            speed = max(self.min_speed, self.cruise_speed * speed_scale)
            speed = min(speed, self.cruise_speed)

        ld = max(math.hypot(tx - x, ty - y), 0.3)
        steering = math.atan2(2.0 * self.wheelbase * math.sin(control_alpha), ld)
        steering = clamp(steering, -self.max_steering, self.max_steering)

        cmd = Twist()
        cmd.linear.x = speed
        cmd.angular.z = steering
        self._record_debug_sample(
            "tracking",
            x=x,
            y=y,
            yaw=yaw,
            pose_source=self.pose_source,
            goal_x=goal_x,
            goal_y=goal_y,
            goal_dist=goal_dist,
            closest_idx=closest_idx,
            closest_dist=closest_dist,
            target_x=tx,
            target_y=ty,
            alpha=alpha,
            control_alpha=control_alpha,
            cmd_v=cmd.linear.x,
            cmd_steer=cmd.angular.z,
            projection=projection,
            speed_reason=speed_reason,
            segment_heading=segment_heading,
        )
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
            rospy.loginfo_throttle(
                1.0,
                "global_path_tracker progress: closest_idx=%d/%d closest_dist=%.2f end_window=%d end_tol=%.2f",
                closest_idx if closest_idx is not None else -1,
                max(0, len(self.path_points) - 1),
                closest_dist if closest_dist is not None else -1.0,
                self.end_path_point_window,
                self.end_path_point_tolerance,
            )
            if projection is not None:
                rospy.loginfo_throttle(
                    1.0,
                    "global_path_tracker projection: s=%.2f remaining=%.2f lateral=%.2f seg=%d proj=(%.2f, %.2f)",
                    projection["s"],
                    projection["remaining"],
                    projection["lateral"],
                    projection["segment_idx"],
                    projection["proj_x"],
                    projection["proj_y"],
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
