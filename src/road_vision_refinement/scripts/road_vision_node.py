#!/usr/bin/env python3
import json
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


class RoadVisionNode:
    """Estimate visible road-center offset from the RGB camera.

    This first version is intentionally conservative: it publishes debug images
    and a heuristic lateral correction, but it does not control the vehicle.
    Positive lateral_error means the visible road center is on the image right,
    so downstream code should shift the reference path to the vehicle/right side
    unless invert_error is enabled there.
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.roi_top_ratio = float(rospy.get_param("~roi_top_ratio", 0.45))
        self.min_area_ratio = float(rospy.get_param("~min_area_ratio", 0.015))
        self.max_correction_m = float(rospy.get_param("~max_correction_m", 0.8))
        self.process_rate = float(rospy.get_param("~process_rate", 10.0))

        self.h_min = int(rospy.get_param("~h_min", 0))
        self.h_max = int(rospy.get_param("~h_max", 179))
        self.s_min = int(rospy.get_param("~s_min", 0))
        self.s_max = int(rospy.get_param("~s_max", 90))
        self.v_min = int(rospy.get_param("~v_min", 35))
        self.v_max = int(rospy.get_param("~v_max", 245))

        kernel_size = int(rospy.get_param("~morph_kernel_size", 7))
        kernel_size = max(3, kernel_size if kernel_size % 2 == 1 else kernel_size + 1)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))

        self.last_process_time = 0.0
        self.mask_pub = rospy.Publisher("~mask", Image, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.error_pub = rospy.Publisher("~lateral_error", Float32, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1, buff_size=2 ** 24)

        rospy.loginfo("road_vision_node listening to %s", self.image_topic)

    def image_cb(self, msg):
        now = time.time()
        if self.process_rate > 0.0 and now - self.last_process_time < 1.0 / self.process_rate:
            return
        self.last_process_time = now

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to convert camera image: %s", exc)
            return

        height, width = bgr.shape[:2]
        roi_top = int(np.clip(self.roi_top_ratio, 0.0, 0.9) * height)
        roi = bgr[roi_top:height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)
        upper = np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)
        mask_roi = cv2.inRange(hsv, lower, upper)
        mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN, self.kernel)
        mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_CLOSE, self.kernel, iterations=2)

        mask = np.zeros((height, width), dtype=np.uint8)
        mask[roi_top:height, :] = mask_roi

        debug = bgr.copy()
        cv2.rectangle(debug, (0, roi_top), (width - 1, height - 1), (0, 200, 255), 2)

        result = self.estimate_center(mask_roi, width, height - roi_top)
        if result is None:
            self.error_pub.publish(Float32(0.0))
            self.publish_status(False, 0.0, 0.0, "no_valid_road_region")
            cv2.putText(debug, "road vision: no valid region", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            center_x, left_x, right_x, area_ratio, confidence = result
            normalized_error = (center_x - width * 0.5) / max(width * 0.5, 1.0)
            lateral_error_m = float(np.clip(normalized_error * self.max_correction_m,
                                            -self.max_correction_m, self.max_correction_m))

            y_vis = int(roi_top + 0.78 * (height - roi_top))
            cv2.line(debug, (int(left_x), y_vis), (int(right_x), y_vis), (0, 255, 255), 3)
            cv2.circle(debug, (int(center_x), y_vis), 8, (0, 255, 0), -1)
            cv2.line(debug, (width // 2, roi_top), (width // 2, height - 1), (255, 0, 0), 2)
            cv2.putText(debug, "err_m={:.2f} conf={:.2f}".format(lateral_error_m, confidence),
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            self.error_pub.publish(Float32(lateral_error_m))
            self.publish_status(True, lateral_error_m, confidence, "ok", area_ratio)

        try:
            self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding="mono8"))
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to publish road vision debug images: %s", exc)

    def estimate_center(self, mask_roi, width, roi_height):
        contours, _ = cv2.findContours(mask_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))
        area_ratio = area / max(float(width * roi_height), 1.0)
        if area_ratio < self.min_area_ratio:
            return None

        filled = np.zeros_like(mask_roi)
        cv2.drawContours(filled, [largest], -1, 255, thickness=cv2.FILLED)

        scan_ratios = (0.62, 0.72, 0.82, 0.90)
        centers = []
        spans = []
        for ratio in scan_ratios:
            y = int(np.clip(ratio * roi_height, 0, roi_height - 1))
            xs = np.where(filled[y, :] > 0)[0]
            if xs.size < 10:
                continue
            left_x = float(xs[0])
            right_x = float(xs[-1])
            centers.append((left_x + right_x) * 0.5)
            spans.append((left_x, right_x, right_x - left_x))

        if not centers:
            x, y, w, h = cv2.boundingRect(largest)
            if w < 10 or h < 10:
                return None
            left_x = float(x)
            right_x = float(x + w)
            center_x = (left_x + right_x) * 0.5
            span = right_x - left_x
        else:
            center_x = float(np.median(np.array(centers)))
            best_span = max(spans, key=lambda item: item[2])
            left_x, right_x, span = best_span

        width_conf = float(np.clip(span / max(width * 0.35, 1.0), 0.0, 1.0))
        area_conf = float(np.clip(area_ratio / max(self.min_area_ratio * 4.0, 1e-6), 0.0, 1.0))
        confidence = 0.5 * width_conf + 0.5 * area_conf
        return center_x, left_x, right_x, area_ratio, confidence

    def publish_status(self, valid, lateral_error_m, confidence, reason, area_ratio=0.0):
        status = {
            "valid": bool(valid),
            "lateral_error_m": round(float(lateral_error_m), 4),
            "confidence": round(float(confidence), 4),
            "area_ratio": round(float(area_ratio), 4),
            "reason": reason,
            "convention": "positive_error_shift_path_to_vehicle_right",
        }
        self.status_pub.publish(String(json.dumps(status, sort_keys=True)))


if __name__ == "__main__":
    rospy.init_node("road_vision_node")
    RoadVisionNode()
    rospy.spin()
