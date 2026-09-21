#!/usr/bin/env python3
"""Overlay a PointCloud2 on the current camera image for extrinsic validation."""

import threading

import cv2
import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class LidarCameraOverlay:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.image = None
        self.image_header = None
        self.camera_matrix = None
        self.distortion = None

        self.min_depth = rospy.get_param("~min_depth", 0.5)
        self.max_depth = rospy.get_param("~max_depth", 15.0)
        self.point_stride = max(1, rospy.get_param("~point_stride", 3))
        self.point_radius = max(1, rospy.get_param("~point_radius", 2))
        self.max_rate = max(0.1, rospy.get_param("~max_rate", 5.0))
        self.last_publish = rospy.Time(0)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher("overlay", Image, queue_size=1)

        rospy.Subscriber("image", Image, self.image_callback,
                         queue_size=1, buff_size=1920 * 1080 * 3 * 2)
        rospy.Subscriber("camera_info", CameraInfo, self.info_callback,
                         queue_size=1)
        rospy.Subscriber("cloud", PointCloud2, self.cloud_callback,
                         queue_size=1, buff_size=16 * 1024 * 1024)

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(5.0, "Cannot decode camera image: %s", error)
            return
        with self.lock:
            self.image = image.copy()
            self.image_header = message.header

    def info_callback(self, message):
        with self.lock:
            self.camera_matrix = np.asarray(message.K, dtype=np.float64).reshape(3, 3)
            self.distortion = np.asarray(message.D, dtype=np.float64)

    @staticmethod
    def transform_matrix(transform):
        q = transform.rotation
        norm = np.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        rotation = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ], dtype=np.float64)
        translation = transform.translation
        return rotation, np.array([translation.x, translation.y, translation.z])

    def cloud_callback(self, message):
        now = rospy.Time.now()
        if (now - self.last_publish).to_sec() < 1.0 / self.max_rate:
            return
        with self.lock:
            if self.image is None or self.camera_matrix is None:
                rospy.logwarn_throttle(5.0, "Waiting for image and camera_info")
                return
            image = self.image.copy()
            header = self.image_header
            camera_matrix = self.camera_matrix.copy()
            distortion = self.distortion.copy()

        source_frame = message.header.frame_id.lstrip("/")
        target_frame = header.frame_id.lstrip("/") or "camera_optical_frame"
        try:
            stamped = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rospy.Time(0), rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(3.0, "TF %s -> %s unavailable: %s",
                                   source_frame, target_frame, error)
            return

        sampled = []
        for index, point in enumerate(pc2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True)):
            if index % self.point_stride == 0:
                sampled.append(point)
        if not sampled:
            return

        points = np.asarray(sampled, dtype=np.float64)
        rotation, translation = self.transform_matrix(stamped.transform)
        camera_points = points @ rotation.T + translation
        depth = camera_points[:, 2]
        keep = (depth > self.min_depth) & (depth < self.max_depth)
        camera_points = camera_points[keep]
        depth = depth[keep]
        if not len(camera_points):
            rospy.logwarn_throttle(3.0, "No LiDAR points are in front of the camera")
            return

        pixels, _ = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3),
                                      camera_matrix, distortion)
        pixels = np.rint(pixels.reshape(-1, 2)).astype(np.int32)
        height, width = image.shape[:2]
        visible = ((pixels[:, 0] >= 0) & (pixels[:, 0] < width) &
                   (pixels[:, 1] >= 0) & (pixels[:, 1] < height))
        pixels, depth = pixels[visible], depth[visible]

        normalized = np.clip((depth - self.min_depth) /
                             (self.max_depth - self.min_depth), 0.0, 1.0)
        colors = cv2.applyColorMap(
            np.uint8((1.0 - normalized) * 255), cv2.COLORMAP_TURBO
        ).reshape(-1, 3)
        for (u, v), color in zip(pixels, colors):
            cv2.circle(image, (int(u), int(v)), self.point_radius,
                       tuple(int(value) for value in color), -1,
                       lineType=cv2.LINE_AA)

        cv2.putText(image, "LiDAR overlay: red=near, blue=far",
                    (25, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (255, 255, 255), 2, cv2.LINE_AA)
        output = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        output.header = header
        self.publisher.publish(output)
        self.last_publish = now


if __name__ == "__main__":
    rospy.init_node("lidar_camera_overlay")
    LidarCameraOverlay()
    rospy.loginfo("Publishing LiDAR/camera overlay for extrinsic validation")
    rospy.spin()
