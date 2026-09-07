#!/usr/bin/env python3
"""Detect a continuous right-side curb from MID360 PointCloud2 data.

The input cloud is transformed to base_link first.  Unlike the reference
lidar_curb_detection project this deliberately does not use laser rings: a
MID360 has a non-repetitive scan pattern.  It detects a persistent upward
height step in a local x/y elevation grid instead.
"""
import json
import math
from collections import defaultdict, deque

import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs
from geometry_msgs.msg import PointStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, String


class LidarCurbNode:
    def __init__(self):
        self.cloud_topic = rospy.get_param("~cloud_topic", "/cloud_registered_body")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.x_min = float(rospy.get_param("~x_min", 0.7))
        self.x_max = float(rospy.get_param("~x_max", 5.0))
        self.y_min = float(rospy.get_param("~y_min", -3.0))
        self.y_max = float(rospy.get_param("~y_max", -0.25))
        self.z_min = float(rospy.get_param("~z_min", -0.25))
        self.z_max = float(rospy.get_param("~z_max", 0.45))
        self.grid_resolution = float(rospy.get_param("~grid_resolution", 0.10))
        self.min_step_height = float(rospy.get_param("~min_step_height", 0.045))
        self.max_step_height = float(rospy.get_param("~max_step_height", 0.28))
        self.min_x_bins = int(rospy.get_param("~min_x_bins", 10))
        self.desired_clearance_m = float(rospy.get_param("~desired_clearance_m", 0.55))
        self.max_correction_m = float(rospy.get_param("~max_correction_m", 0.45))
        self.confirm_frames = int(rospy.get_param("~confirm_frames", 4))
        self.max_edge_jump_m = float(rospy.get_param("~max_edge_jump_m", 0.35))
        self.filter_alpha = float(rospy.get_param("~filter_alpha", 0.25))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.08))

        self.edge_history = deque(maxlen=max(1, self.confirm_frames))
        self.filtered_edge_y = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.edge_pub = rospy.Publisher("/road_curb/right_edge", PointStamped, queue_size=1)
        self.error_pub = rospy.Publisher("/road_curb/lateral_error", Float32, queue_size=1)
        self.status_pub = rospy.Publisher("/road_curb/status", String, queue_size=1)
        self.cloud_sub = rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_cb, queue_size=1)
        rospy.loginfo("lidar_curb_node: %s -> right curb in %s", self.cloud_topic, self.base_frame)

    def publish_status(self, valid, reason, **extra):
        status = {"valid": bool(valid), "reason": reason, "source": "lidar_right_curb"}
        status.update(extra)
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

    def to_base_frame(self, cloud):
        if cloud.header.frame_id == self.base_frame:
            return cloud
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, cloud.header.frame_id, cloud.header.stamp,
                rospy.Duration(self.tf_timeout))
            return tf2_sensor_msgs.do_transform_cloud(cloud, transform)
        except Exception as exc:
            self.publish_status(False, "tf_unavailable", input_frame=cloud.header.frame_id,
                                detail=str(exc))
            return None

    def cloud_cb(self, cloud):
        cloud = self.to_base_frame(cloud)
        if cloud is None:
            return

        cells = defaultdict(list)
        res = self.grid_resolution
        for x, y, z in point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
            if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max and
                    self.z_min <= z <= self.z_max):
                continue
            cells[(int((x - self.x_min) / res), int((y - self.y_min) / res))].append(z)

        candidates = []
        x_cell_count = int(math.ceil((self.x_max - self.x_min) / res))
        y_cell_count = int(math.ceil((self.y_max - self.y_min) / res))
        for ix in range(x_cell_count):
            heights = {}
            for iy in range(y_cell_count):
                values = cells.get((ix, iy), [])
                if len(values) >= 2:
                    values.sort()
                    heights[iy] = values[len(values) // 2]
            for iy in range(y_cell_count - 1):
                if iy not in heights or iy + 1 not in heights:
                    continue
                # Scan from vehicle/right-road side (larger y) outward (smaller y).
                # An elevated sidewalk/curb outward of the road gives h_outer-h_inner > 0.
                inner_h = heights[iy + 1]
                outer_h = heights[iy]
                step = outer_h - inner_h
                if self.min_step_height <= step <= self.max_step_height:
                    candidates.append((self.x_min + (ix + 0.5) * res,
                                       self.y_min + (iy + 1.0) * res, step))

        if len(candidates) < self.min_x_bins:
            self.edge_history.clear()
            self.publish_status(False, "insufficient_curb_bins", candidate_count=len(candidates))
            return

        ys = sorted(item[1] for item in candidates)
        edge_y = ys[len(ys) // 2]
        unique_x_bins = len(set(int((item[0] - self.x_min) / res) for item in candidates))
        if unique_x_bins < self.min_x_bins:
            self.edge_history.clear()
            self.publish_status(False, "curb_not_continuous", x_bin_count=unique_x_bins)
            return

        if self.filtered_edge_y is not None and abs(edge_y - self.filtered_edge_y) > self.max_edge_jump_m:
            self.edge_history.clear()
            self.publish_status(False, "edge_jump_rejected", edge_y=edge_y,
                                previous_edge_y=self.filtered_edge_y)
            return
        self.edge_history.append(edge_y)
        if len(self.edge_history) < self.confirm_frames:
            self.publish_status(False, "confirming", edge_y=edge_y,
                                confirmed_frames=len(self.edge_history))
            return

        median_y = sorted(self.edge_history)[len(self.edge_history) // 2]
        if self.filtered_edge_y is None:
            self.filtered_edge_y = median_y
        else:
            alpha = max(0.0, min(1.0, self.filter_alpha))
            self.filtered_edge_y = (1.0 - alpha) * self.filtered_edge_y + alpha * median_y
        right_distance = -self.filtered_edge_y
        correction = max(-self.max_correction_m,
                         min(self.max_correction_m, right_distance - self.desired_clearance_m))

        edge = PointStamped()
        edge.header = cloud.header
        edge.header.frame_id = self.base_frame
        edge.point.x = (self.x_min + self.x_max) * 0.5
        edge.point.y = self.filtered_edge_y
        edge.point.z = 0.0
        self.edge_pub.publish(edge)
        self.error_pub.publish(Float32(data=correction))
        confidence = min(1.0, unique_x_bins / float(max(self.min_x_bins, 1) * 2))
        self.publish_status(True, "confirmed", edge_y=self.filtered_edge_y,
                            right_distance_m=right_distance, lateral_error_m=correction,
                            candidate_count=len(candidates), x_bin_count=unique_x_bins,
                            confidence=confidence)


if __name__ == "__main__":
    rospy.init_node("lidar_curb_node")
    LidarCurbNode()
    rospy.spin()
