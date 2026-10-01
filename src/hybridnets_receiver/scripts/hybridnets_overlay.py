#!/usr/bin/env python3
"""Draw each HybridNets result on the closest buffered camera frame."""

from collections import deque
from itertools import count
import json
import threading

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String


class HybridNetsOverlay:
    def __init__(self):
        self.lock = threading.Lock()
        self.max_age = float(rospy.get_param("~max_age", 0.8))
        self.buffer_sec = float(rospy.get_param("~image_buffer_sec", 2.0))
        self.video_delay = float(rospy.get_param("~video_delay_sec", 0.10))
        self.sync_tolerance = float(rospy.get_param("~sync_tolerance_sec", 0.25))
        self.images = deque()
        self.pending = deque()
        self.debug_sequence = count()
        self.result_publisher = rospy.Publisher(
            "/hybridnets/debug_result_json", String, queue_size=10)
        self.publisher = rospy.Publisher(
            rospy.get_param("~debug_image_topic", "/hybridnets/debug_image"),
            Image, queue_size=1)
        rospy.Subscriber(rospy.get_param("~result_json_topic", "/hybridnets/result_json"),
                         String, self.result_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("~image_topic", "/camera/remote_image_raw"),
                         Image, self.image_callback, queue_size=1,
                         buff_size=1920 * 1080 * 3 * 2)

    def result_callback(self, message):
        try:
            result = json.loads(message.data)
            now = rospy.Time.now().to_sec()
            capture = int(result["capture_timestamp_ns"]) / 1.0e9
            source_age = max(0.0, float(result.get("source_age_ms", 0.0)) / 1000.0)
        except (KeyError, TypeError, ValueError, OverflowError):
            return

        # Prefer synchronized GPU/NUC wall clocks.  During replay or without clock
        # synchronization, estimate capture time from source_age_ms instead.
        wall_age = now - capture
        capture_on_nuc = capture if -1.0 <= wall_age <= 60.0 else now - source_age
        target = capture_on_nuc + self.video_delay
        with self.lock:
            self.pending.append((target, now, result))
            pairs = self.collect_ready_locked()
        self.publish_pairs(pairs)

    @staticmethod
    def points(value):
        if not value:
            return None
        return np.rint(np.asarray(value, dtype=np.float64)).astype(np.int32).reshape(-1, 1, 2)

    @staticmethod
    def boundary_segments(result, side):
        key = side + "_boundary_segments_px"
        if key in result:
            return result[key]
        points = result.get(side + "_boundary_px", [])
        threshold = 1.5 * float(result.get("boundary_row_step_px", 20.0))
        segments = []
        for point in points:
            if not segments or abs(point[1] - segments[-1][-1][1]) > threshold:
                segments.append([])
            segments[-1].append(point)
        return segments

    def collect_ready_locked(self):
        if not self.images:
            return []
        latest = self.images[-1][0]
        pairs = []
        while self.pending:
            target, received, result = self.pending[0]
            if target > latest + self.sync_tolerance:
                break
            self.pending.popleft()
            if rospy.Time.now().to_sec() - received > self.max_age:
                continue
            stamp, image = min(self.images, key=lambda item: abs(item[0] - target))
            error = stamp - target
            if abs(error) <= self.sync_tolerance:
                pairs.append((image, result, error))
            else:
                rospy.logwarn_throttle(
                    2.0, "No camera frame near HybridNets result (error %.0f ms)",
                    error * 1000.0)
        return pairs

    def image_callback(self, message):
        stamp = message.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.Time.now().to_sec()
        with self.lock:
            self.images.append((stamp, message))
            cutoff = stamp - self.buffer_sec
            while self.images and self.images[0][0] < cutoff:
                self.images.popleft()
            pairs = self.collect_ready_locked()
        self.publish_pairs(pairs)

    def publish_pairs(self, pairs):
        for message, result, sync_error in pairs:
            self.draw_and_publish(message, result, sync_error)

    def draw_and_publish(self, message, result, sync_error):
        if message.encoding not in ("bgr8", "rgb8") or message.step != message.width * 3:
            rospy.logwarn_throttle(3.0, "Overlay expects packed bgr8/rgb8 images")
            return
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3).copy()
        if message.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        polygon = self.points(result.get("road_polygon_px"))
        if polygon is not None:
            mask = image.copy()
            cv2.fillPoly(mask, [polygon], (0, 180, 0))
            image = cv2.addWeighted(image, 0.75, mask, 0.25, 0.0)

        for side, color, dim_color in (
                ("left", (255, 0, 255), (128, 0, 128)),
                ("right", (0, 255, 255), (0, 128, 128))):
            measured = self.boundary_segments(result, side)
            valid_key = side + "_valid_segments_px"
            if valid_key in result:
                for segment in measured:
                    points = self.points(segment)
                    if points is not None and len(points) >= 2:
                        cv2.polylines(image, [points], False, dim_color, 2)
                visible = result[valid_key]
            else:
                # Legacy JSON has no separation between measured and usable
                # segments, so retain the previous bright rendering.
                visible = measured
            for segment in visible:
                points = self.points(segment)
                if points is not None and len(points) >= 2:
                    cv2.polylines(image, [points], False, color, 4)
        text = "HybridNets valid=%s scene=%s L=%d R=%d sync=%+.0fms" % (
            bool(result.get("valid", False)), result.get("scene_type", "legacy"),
            int(bool(result.get("left_boundary_valid", result.get("left_boundary_px")))),
            int(bool(result.get("right_boundary_valid", result.get("right_boundary_px")))),
            sync_error * 1000.0)
        color = (0, 255, 0) if result.get("valid", False) else (0, 0, 255)
        cv2.putText(image, text, (25, 45), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, color, 2, cv2.LINE_AA)
        header = Header(seq=next(self.debug_sequence) % (2 ** 32),
                        stamp=message.header.stamp, frame_id=message.header.frame_id)
        output = Image(header=header, height=message.height, width=message.width,
                       encoding="bgr8", is_bigendian=False,
                       step=message.width * 3, data=image.tobytes())
        self.publisher.publish(output)
        matched = dict(result)
        matched["debug_image_seq"] = header.seq
        matched["debug_image_timestamp_ns"] = header.stamp.to_nsec()
        matched["debug_sync_error_ms"] = sync_error * 1000.0
        self.result_publisher.publish(json.dumps(matched, separators=(",", ":")))


if __name__ == "__main__":
    rospy.init_node("hybridnets_overlay")
    HybridNetsOverlay()
    rospy.spin()
