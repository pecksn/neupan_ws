#!/usr/bin/env python3
import math
from copy import deepcopy

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float32


class PathRefineNode:
    """Apply a small lateral shift to the global path using road vision error.

    The input path is not modified in-place. This node publishes a separate
    refined path for debug/optional use. The sign convention follows
    road_vision_node: positive error means shift the path to the vehicle/right
    side. In map coordinates that corresponds to the right normal of each path
    segment. Set invert_error=true if the debug overlay shows the correction is
    moving in the wrong direction.
    """

    def __init__(self):
        self.input_path_topic = rospy.get_param("~input_path_topic", "/osm_global_path")
        self.output_path_topic = rospy.get_param("~output_path_topic", "/osm_global_path_refined")
        self.error_topic = rospy.get_param("~error_topic", "/road_vision/lateral_error")
        self.max_shift_m = float(rospy.get_param("~max_shift_m", 0.8))
        self.filter_alpha = float(rospy.get_param("~filter_alpha", 0.2))
        self.invert_error = bool(rospy.get_param("~invert_error", False))
        self.republish_on_error = bool(rospy.get_param("~republish_on_error", True))

        self.path = None
        self.filtered_error = 0.0

        self.path_pub = rospy.Publisher(self.output_path_topic, Path, queue_size=1, latch=True)
        self.path_sub = rospy.Subscriber(self.input_path_topic, Path, self.path_cb, queue_size=1)
        self.error_sub = rospy.Subscriber(self.error_topic, Float32, self.error_cb, queue_size=5)

        rospy.loginfo("path_refine_node: %s + %s -> %s",
                      self.input_path_topic, self.error_topic, self.output_path_topic)

    def path_cb(self, msg):
        self.path = msg
        self.publish_refined()

    def error_cb(self, msg):
        raw_error = -msg.data if self.invert_error else msg.data
        raw_error = max(-self.max_shift_m, min(self.max_shift_m, raw_error))
        alpha = max(0.0, min(1.0, self.filter_alpha))
        self.filtered_error = (1.0 - alpha) * self.filtered_error + alpha * raw_error
        if self.republish_on_error:
            self.publish_refined()

    def publish_refined(self):
        if self.path is None:
            return
        if not self.path.poses:
            self.path_pub.publish(self.path)
            return

        refined = Path()
        refined.header = self.path.header
        shift = self.filtered_error

        for index, pose_stamped in enumerate(self.path.poses):
            new_pose = deepcopy(pose_stamped)
            tangent = self.path_tangent(index)
            if tangent is not None:
                tx, ty = tangent
                right_nx = ty
                right_ny = -tx
                new_pose.pose.position.x += right_nx * shift
                new_pose.pose.position.y += right_ny * shift
            refined.poses.append(new_pose)

        self.path_pub.publish(refined)

    def path_tangent(self, index):
        poses = self.path.poses
        if len(poses) < 2:
            return None

        prev_i = max(0, index - 1)
        next_i = min(len(poses) - 1, index + 1)
        if prev_i == next_i:
            return None

        p0 = poses[prev_i].pose.position
        p1 = poses[next_i].pose.position
        dx = p1.x - p0.x
        dy = p1.y - p0.y
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            return None
        return dx / norm, dy / norm


if __name__ == "__main__":
    rospy.init_node("path_refine_node")
    PathRefineNode()
    rospy.spin()
