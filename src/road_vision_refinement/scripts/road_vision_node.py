#!/usr/bin/env python3
import json
import math
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String


class RoadVisionNode:
    """Publish a conservative visual correction for campus-road navigation.

    In auto mode a visible white/yellow center line is preferred. The target is
    placed to its image-right side. If no reliable line exists, the node falls
    back to the center of a road-color region. All outputs are diagnostic until
    camera geometry converts the image-relative correction to metric distance.
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.roi_top_ratio = float(rospy.get_param("~roi_top_ratio", 0.45))
        self.min_area_ratio = float(rospy.get_param("~min_area_ratio", 0.015))
        self.max_correction_m = float(rospy.get_param("~max_correction_m", 0.8))
        self.process_rate = float(rospy.get_param("~process_rate", 10.0))

        self.target_mode = rospy.get_param("~target_mode", "auto")
        if self.target_mode not in ("auto", "center_line", "road_edge", "road_region"):
            rospy.logwarn("Unknown target_mode=%s; using auto", self.target_mode)
            self.target_mode = "auto"
        self.center_line_right_offset_ratio = float(
            rospy.get_param("~center_line_right_offset_ratio", 0.18)
        )
        self.center_line_right_sign = float(rospy.get_param("~center_line_right_sign", 1.0))
        self.center_line_right_sign = 1.0 if self.center_line_right_sign >= 0.0 else -1.0
        self.invalid_hold_sec = float(rospy.get_param("~invalid_hold_sec", 1.0))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.35))
        self.min_line_confidence = float(rospy.get_param("~min_line_confidence", 0.40))
        self.center_line_requires_road = bool(rospy.get_param("~center_line_requires_road", True))
        self.road_edge_clearance_ratio = float(rospy.get_param("~road_edge_clearance_ratio", 0.18))
        self.road_min_scanline_count = max(1, int(rospy.get_param("~road_min_scanline_count", 3)))
        self.road_min_bottom_touch_ratio = float(
            rospy.get_param("~road_min_bottom_touch_ratio", 0.92)
        )
        self.line_search_margin_px = int(rospy.get_param("~line_search_margin_px", 10))
        self.line_min_length_ratio = float(rospy.get_param("~line_min_length_ratio", 0.12))
        self.line_max_jump_px = float(rospy.get_param("~line_max_jump_px", 70.0))
        self.line_max_angle_jump_deg = float(rospy.get_param("~line_max_angle_jump_deg", 25.0))
        self.line_smoothing_alpha = float(rospy.get_param("~line_smoothing_alpha", 0.25))
        self.line_confirm_frames = max(1, int(rospy.get_param("~line_confirm_frames", 3)))
        self.center_line_expected_x_ratio = float(
            rospy.get_param("~center_line_expected_x_ratio", 0.40)
        )
        self.center_line_max_offset_ratio = float(
            rospy.get_param("~center_line_max_offset_ratio", 0.32)
        )
        self.center_line_slope_sign = float(rospy.get_param("~center_line_slope_sign", -1.0))
        self.center_line_max_eval_x_ratio = float(
            rospy.get_param("~center_line_max_eval_x_ratio", 0.62)
        )

        self.white_s_max = int(rospy.get_param("~white_s_max", 75))
        self.white_v_min = int(rospy.get_param("~white_v_min", 175))
        self.yellow_h_min = int(rospy.get_param("~yellow_h_min", 15))
        self.yellow_h_max = int(rospy.get_param("~yellow_h_max", 42))
        self.yellow_s_min = int(rospy.get_param("~yellow_s_min", 60))
        self.yellow_v_min = int(rospy.get_param("~yellow_v_min", 70))

        self.h_min = int(rospy.get_param("~h_min", 0))
        self.h_max = int(rospy.get_param("~h_max", 179))
        self.s_min = int(rospy.get_param("~s_min", 0))
        self.s_max = int(rospy.get_param("~s_max", 90))
        self.v_min = int(rospy.get_param("~v_min", 35))
        self.v_max = int(rospy.get_param("~v_max", 245))

        kernel_size = int(rospy.get_param("~morph_kernel_size", 7))
        kernel_size = max(3, kernel_size if kernel_size % 2 == 1 else kernel_size + 1)
        self.road_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        self.line_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        self.last_process_time = 0.0
        self.last_valid_time = None
        self.last_error_m = 0.0
        self.last_confidence = 0.0
        self.last_source = "none"
        self.last_line_x = None
        self.last_line_angle_deg = None
        self.last_line_coeffs = None
        self.line_candidate_streak = 0
        self.pending_line_x = None
        self.pending_line_angle_deg = None
        self.current_line_confidence = 0.0
        self.current_line_confirmed = False
        self.current_road_confidence = 0.0
        self.current_road_bottom_touch_ratio = 0.0

        self.mask_pub = rospy.Publisher("~mask", Image, queue_size=1)
        self.line_mask_pub = rospy.Publisher("~center_line_mask", Image, queue_size=1)
        self.road_mask_pub = rospy.Publisher("~road_mask", Image, queue_size=1)
        self.debug_pub = rospy.Publisher("~debug_image", Image, queue_size=1)
        self.error_pub = rospy.Publisher("~lateral_error", Float32, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1, buff_size=2 ** 24)

        rospy.loginfo("road_vision_node listening to %s (mode=%s)", self.image_topic, self.target_mode)

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
        roi_height = height - roi_top
        y_eval = int(np.clip(0.82 * roi_height, 0, roi_height - 1))

        road_mask_roi = self.make_road_mask(roi)
        line_mask_roi = self.make_center_line_mask(roi)
        line_result = self.detect_center_line(line_mask_roi, width, roi_height, y_eval)
        self.current_line_confidence = 0.0 if line_result is None else line_result["confidence"]
        self.current_line_confirmed = False if line_result is None else line_result["confirmed"]
        if line_result is None or line_result["confidence"] < self.min_line_confidence:
            # A weak/absent candidate must not accumulate confirmation frames.
            self.line_candidate_streak = 0
            self.pending_line_x = None
            self.pending_line_angle_deg = None
        road_result = self.estimate_road_region(road_mask_roi, width, roi_height)
        self.current_road_confidence = 0.0 if road_result is None else road_result["confidence"]
        self.current_road_bottom_touch_ratio = (
            0.0 if road_result is None else road_result["bottom_touch_ratio"]
        )

        debug = bgr.copy()
        cv2.rectangle(debug, (0, roi_top), (width - 1, height - 1), (0, 200, 255), 2)
        cv2.line(debug, (width // 2, roi_top), (width // 2, height - 1), (255, 0, 0), 2)

        selected = None
        line_context_ok = road_result is not None and road_result["confidence"] >= self.min_confidence
        if self.target_mode in ("auto", "center_line") and line_result is not None and (
                not self.center_line_requires_road or line_context_ok):
            if line_result["confidence"] >= self.min_line_confidence:
                if line_result["confirmed"]:
                    selected = self.center_line_target(line_result, width)
                    self.draw_center_line(debug, line_result, roi_top, roi_height)
        elif self.center_line_requires_road:
            self.line_candidate_streak = 0
            self.pending_line_x = None
            self.pending_line_angle_deg = None

        if selected is None and self.target_mode in ("auto", "road_edge") and road_result is not None:
            if road_result["confidence"] >= self.min_confidence:
                selected = self.road_edge_target(road_result)
                if selected is not None:
                    self.draw_road_region(debug, road_result, roi_top, roi_height)
        if selected is None and self.target_mode == "road_region" and road_result is not None:
            if road_result["confidence"] >= self.min_confidence:
                selected = self.road_region_target(road_result)
                self.draw_road_region(debug, road_result, roi_top, roi_height)

        if selected is None:
            self.publish_invalid(debug, now)
        else:
            self.publish_selected(debug, selected, width, roi_top + y_eval, now)

        selected_mask_roi = line_mask_roi if selected is not None and selected["source"] == "center_line" else road_mask_roi
        self.publish_images(msg, debug, selected_mask_roi, line_mask_roi, road_mask_roi, roi_top, height, width)

    def make_road_mask(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([self.h_min, self.s_min, self.v_min], dtype=np.uint8)
        upper = np.array([self.h_max, self.s_max, self.v_max], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.road_kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.road_kernel, iterations=2)

    def make_center_line_mask(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(
            hsv,
            np.array([0, 0, self.white_v_min], dtype=np.uint8),
            np.array([179, self.white_s_max, 255], dtype=np.uint8),
        )
        yellow = cv2.inRange(
            hsv,
            np.array([self.yellow_h_min, self.yellow_s_min, self.yellow_v_min], dtype=np.uint8),
            np.array([self.yellow_h_max, 255, 255], dtype=np.uint8),
        )
        mask = cv2.bitwise_or(white, yellow)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.line_kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.line_kernel, iterations=2)

    def detect_center_line(self, mask, width, roi_height, y_eval):
        min_length = max(12, int(roi_height * self.line_min_length_ratio))
        lines = cv2.HoughLinesP(
            mask, 1, np.pi / 180.0, threshold=20,
            minLineLength=min_length, maxLineGap=max(15, roi_height // 12),
        )
        if lines is None:
            return None

        expected_x = self.last_line_x if self.last_line_x is not None else width * 0.42
        candidates = []
        for raw in lines[:, 0, :]:
            x1, y1, x2, y2 = [float(value) for value in raw]
            dx = x2 - x1
            dy = y2 - y1
            length = math.hypot(dx, dy)
            if length < min_length or abs(dy) < 0.35 * length:
                continue
            x_eval = x1 + (y_eval - y1) * dx / dy
            if x_eval < -0.15 * width or x_eval > 1.15 * width:
                continue
            vertical_score = abs(dy) / max(length, 1.0)
            proximity = 1.0 - min(abs(x_eval - expected_x) / max(width * 0.55, 1.0), 1.0)
            score = length * (0.55 + 0.25 * vertical_score + 0.20 * proximity)
            candidates.append((score, x1, y1, x2, y2, x_eval, length))

        if not candidates:
            return None

        best = max(candidates, key=lambda item: item[0])
        _, x1, y1, x2, y2, _, segment_length = best
        yy, xx = np.nonzero(mask)
        line_dx = x2 - x1
        line_dy = y2 - y1
        denom = max(math.hypot(line_dx, line_dy), 1.0)
        distance = np.abs(line_dy * (xx - x1) - line_dx * (yy - y1)) / denom
        near = distance <= max(4, self.line_search_margin_px)
        fit_x = xx[near]
        fit_y = yy[near]
        if fit_x.size < 30:
            return None

        degree = 2 if fit_x.size >= 120 and np.ptp(fit_y) >= roi_height * 0.35 else 1
        coeffs = np.polyfit(fit_y.astype(np.float64), fit_x.astype(np.float64), degree)
        raw_line_x = float(np.polyval(coeffs, y_eval))
        angle_deg = math.degrees(math.atan2(line_dx, line_dy))
        slope_sign = 1.0 if line_dx >= 0.0 else -1.0
        expected_slope_sign = 1.0 if self.center_line_slope_sign >= 0.0 else -1.0
        if abs(line_dx) > 0.15 * abs(line_dy) and slope_sign != expected_slope_sign:
            self.line_candidate_streak = 0
            self.pending_line_x = None
            self.pending_line_angle_deg = None
            return None
        if raw_line_x > np.clip(self.center_line_max_eval_x_ratio, 0.5, 0.95) * width:
            self.line_candidate_streak = 0
            self.pending_line_x = None
            self.pending_line_angle_deg = None
            return None
        expected_x = float(np.clip(self.center_line_expected_x_ratio, 0.0, 1.0) * width)
        max_offset = float(np.clip(self.center_line_max_offset_ratio, 0.05, 0.9) * width)
        if abs(raw_line_x - expected_x) > max_offset:
            self.line_candidate_streak = 0
            self.pending_line_x = None
            self.pending_line_angle_deg = None
            return None
        if self.pending_line_x is None:
            self.line_candidate_streak = 1
        else:
            pending_jump = abs(raw_line_x - self.pending_line_x)
            pending_angle_delta = abs(
                (angle_deg - self.pending_line_angle_deg + 90.0) % 180.0 - 90.0
            )
            if pending_jump > self.line_max_jump_px or pending_angle_delta > self.line_max_angle_jump_deg:
                # A different Hough segment appeared; require a fresh run of
                # consecutive frames before exposing it as a center line.
                self.line_candidate_streak = 1
            else:
                self.line_candidate_streak += 1
        self.pending_line_x = raw_line_x
        self.pending_line_angle_deg = angle_deg
        if self.last_line_x is not None and abs(raw_line_x - self.last_line_x) > self.line_max_jump_px:
            self.line_candidate_streak = 0
            return None
        if self.last_line_angle_deg is not None:
            # HoughLinesP may return the same segment with reversed endpoints;
            # compare line orientation modulo 180 degrees.
            angle_delta = abs((angle_deg - self.last_line_angle_deg + 90.0) % 180.0 - 90.0)
            if angle_delta > self.line_max_angle_jump_deg:
                self.line_candidate_streak = 0
                return None

        # Smooth the fitted curve itself, not just the final scalar error. This
        # prevents HoughLinesP from switching between nearby pavement seams.
        alpha = float(np.clip(self.line_smoothing_alpha, 0.0, 1.0))
        if self.last_line_coeffs is not None and len(self.last_line_coeffs) == len(coeffs):
            coeffs = (1.0 - alpha) * self.last_line_coeffs + alpha * coeffs
        line_x = float(np.polyval(coeffs, y_eval))
        coverage = float(np.ptp(fit_y)) / max(float(roi_height), 1.0)
        pixel_score = min(1.0, fit_x.size / max(roi_height * 0.7, 1.0))
        length_score = min(1.0, segment_length / max(roi_height * 0.55, 1.0))
        confidence = float(np.clip(0.45 * coverage + 0.30 * pixel_score + 0.25 * length_score, 0.0, 1.0))
        return {
            "coeffs": coeffs,
            "line_x": line_x,
            "angle_deg": angle_deg,
            "raw_line_x": raw_line_x,
            "confirmed": self.line_candidate_streak >= self.line_confirm_frames,
            "confidence": confidence,
            "coverage": coverage,
            "fit_y_min": int(np.min(fit_y)),
            "fit_y_max": int(np.max(fit_y)),
        }

    def center_line_target(self, line_result, width):
        offset_px = float(np.clip(self.center_line_right_offset_ratio, 0.0, 0.45) * width)
        target_x = float(np.clip(line_result["line_x"] + self.center_line_right_sign * offset_px,
                                 0.0, width - 1.0))
        return {
            "source": "center_line",
            "target_x": target_x,
            "line_x": line_result["line_x"],
            "coeffs": line_result["coeffs"],
            "angle_deg": line_result["angle_deg"],
            "confidence": line_result["confidence"],
            "area_ratio": 0.0,
        }

    def estimate_road_region(self, mask, width, roi_height):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        largest = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(largest))
        area_ratio = area / max(float(width * roi_height), 1.0)
        if area_ratio < self.min_area_ratio:
            return None

        _, contour_y, _, contour_h = cv2.boundingRect(largest)
        bottom_touch_ratio = float(contour_y + contour_h) / max(float(roi_height), 1.0)
        if bottom_touch_ratio < np.clip(self.road_min_bottom_touch_ratio, 0.5, 1.0):
            return None

        filled = np.zeros_like(mask)
        cv2.drawContours(filled, [largest], -1, 255, thickness=cv2.FILLED)
        spans = []
        for ratio in (0.62, 0.72, 0.82, 0.90):
            y = int(np.clip(ratio * roi_height, 0, roi_height - 1))
            xs = np.where(filled[y, :] > 0)[0]
            if xs.size >= 10:
                spans.append((float(xs[0]), float(xs[-1]), float(xs[-1] - xs[0])))
        if len(spans) < self.road_min_scanline_count:
            return None
        left_x, right_x, span = max(spans, key=lambda item: item[2])
        center_x = float(np.median([(left + right) * 0.5 for left, right, _ in spans]))
        width_conf = float(np.clip(span / max(width * 0.35, 1.0), 0.0, 1.0))
        area_conf = float(np.clip(area_ratio / max(self.min_area_ratio * 4.0, 1e-6), 0.0, 1.0))
        confidence = 0.5 * width_conf + 0.5 * area_conf
        return {
            "center_x": center_x,
            "left_x": left_x,
            "right_x": right_x,
            "image_width": width,
            "area_ratio": area_ratio,
            "bottom_touch_ratio": bottom_touch_ratio,
            "confidence": confidence,
        }

    def road_edge_target(self, result):
        # This is only a visual edge candidate. The ratio is not metric until
        # IPM and camera extrinsics are added, so it remains debug-only.
        if result["right_x"] >= 0.98 * result["image_width"]:
            return None
        span = max(result["right_x"] - result["left_x"], 1.0)
        target_x = result["right_x"] - np.clip(self.road_edge_clearance_ratio, 0.0, 0.45) * span
        return {
            "source": "road_edge",
            "target_x": float(target_x),
            "line_x": None,
            "confidence": result["confidence"],
            "area_ratio": result["area_ratio"],
        }

    @staticmethod
    def road_region_target(result):
        return {
            "source": "road_region_center",
            "target_x": result["center_x"],
            "line_x": None,
            "confidence": result["confidence"],
            "area_ratio": result["area_ratio"],
        }

    def publish_selected(self, debug, selected, width, y_vis, now):
        target_x = selected["target_x"]
        normalized_error = (target_x - width * 0.5) / max(width * 0.5, 1.0)
        error_m = float(np.clip(normalized_error * self.max_correction_m,
                                -self.max_correction_m, self.max_correction_m))
        confidence = selected["confidence"]
        valid = confidence >= self.min_confidence
        if valid:
            self.last_valid_time = now
            self.last_error_m = error_m
            self.last_confidence = confidence
            self.last_source = selected["source"]
            if selected["line_x"] is not None:
                self.last_line_x = selected["line_x"]
                self.last_line_angle_deg = selected.get("angle_deg", self.last_line_angle_deg)
                self.last_line_coeffs = np.asarray(selected.get("coeffs"), dtype=np.float64)

        cv2.circle(debug, (int(target_x), int(y_vis)), 9, (0, 165, 255), -1)
        cv2.putText(
            debug,
            "source={} err={:.2f} conf={:.2f}".format(selected["source"], error_m, confidence),
            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.72,
            (0, 255, 0) if valid else (0, 0, 255), 2,
        )
        self.error_pub.publish(Float32(error_m))
        self.publish_status(
            valid, error_m, confidence, "ok" if valid else "low_confidence",
            selected["source"], selected["area_ratio"], target_x, selected["line_x"],
        )

    def publish_invalid(self, debug, now):
        held = self.last_valid_time is not None and now - self.last_valid_time <= self.invalid_hold_sec
        error = self.last_error_m if held else 0.0
        confidence = self.last_confidence if held else 0.0
        source = self.last_source if held else "none"
        reason = "hold_last_valid" if held else "no_visual_reference"
        if not held:
            self.last_line_x = None
            self.last_line_angle_deg = None
            self.last_line_coeffs = None
        self.error_pub.publish(Float32(error))
        self.publish_status(held, error, confidence, reason, source)
        cv2.putText(debug, "vision: {}".format(reason), (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    @staticmethod
    def draw_center_line(debug, result, roi_top, roi_height):
        y_min = max(0, int(result.get("fit_y_min", 0)))
        y_max = min(roi_height - 1, int(result.get("fit_y_max", roi_height - 1)))
        if y_max - y_min < 5:
            return
        ys = np.linspace(y_min, y_max, 40)
        xs = np.polyval(result["coeffs"], ys)
        points = []
        for x, y in zip(xs, ys):
            if 0 <= x < debug.shape[1]:
                points.append([int(x), int(y + roi_top)])
        if len(points) >= 2:
            cv2.polylines(debug, [np.asarray(points, dtype=np.int32)], False, (255, 0, 255), 4)

    @staticmethod
    def draw_road_region(debug, result, roi_top, roi_height):
        y_vis = int(roi_top + 0.82 * roi_height)
        cv2.line(debug, (int(result["left_x"]), y_vis),
                 (int(result["right_x"]), y_vis), (0, 255, 255), 3)
        cv2.circle(debug, (int(result["center_x"]), y_vis), 7, (0, 255, 0), -1)

    def publish_images(self, source_msg, debug, selected_roi, line_roi, road_roi,
                       roi_top, height, width):
        selected = np.zeros((height, width), dtype=np.uint8)
        line_mask = np.zeros((height, width), dtype=np.uint8)
        road_mask = np.zeros((height, width), dtype=np.uint8)
        selected[roi_top:height, :] = selected_roi
        line_mask[roi_top:height, :] = line_roi
        road_mask[roi_top:height, :] = road_roi
        try:
            for publisher, image, encoding in (
                (self.mask_pub, selected, "mono8"),
                (self.line_mask_pub, line_mask, "mono8"),
                (self.road_mask_pub, road_mask, "mono8"),
                (self.debug_pub, debug, "bgr8"),
            ):
                output = self.bridge.cv2_to_imgmsg(image, encoding=encoding)
                output.header = source_msg.header
                publisher.publish(output)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "Failed to publish road vision debug images: %s", exc)

    def publish_status(self, valid, lateral_error_m, confidence, reason, source,
                       area_ratio=0.0, target_x=None, line_x=None):
        status = {
            "valid": bool(valid),
            "source": source,
            "lateral_error_m": round(float(lateral_error_m), 4),
            "confidence": round(float(confidence), 4),
            "area_ratio": round(float(area_ratio), 4),
            "target_x": None if target_x is None else round(float(target_x), 1),
            "center_line_x": None if line_x is None else round(float(line_x), 1),
            "reason": reason,
            "mode": self.target_mode,
            "center_line_right_sign": self.center_line_right_sign,
            "line_candidate_confidence": round(float(self.current_line_confidence), 4),
            "line_candidate_confirmed": bool(self.current_line_confirmed),
            "road_candidate_confidence": round(float(self.current_road_confidence), 4),
            "road_bottom_touch_ratio": round(float(self.current_road_bottom_touch_ratio), 4),
            "metric_calibrated": False,
            "convention": "positive_error_shift_path_to_vehicle_right",
        }
        self.status_pub.publish(String(json.dumps(status, sort_keys=True)))


if __name__ == "__main__":
    rospy.init_node("road_vision_node")
    RoadVisionNode()
    rospy.spin()
