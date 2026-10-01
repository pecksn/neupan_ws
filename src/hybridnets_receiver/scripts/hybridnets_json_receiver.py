#!/usr/bin/env python3
"""Validate HybridNets UDP JSON and expose health-aware ROS1 topics."""

import json
import math
import socket

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool, Float32, Header, String, UInt64


REQUIRED = (
    "capture_timestamp_ns", "image_width", "image_height", "valid",
    "road_polygon_px", "left_boundary_px", "right_boundary_px",
)
POINT_FIELDS = ("road_polygon_px", "left_boundary_px", "right_boundary_px")
CONTROL_FIELDS = (
    "version", "frame_seq", "publish_timestamp_ns", "coordinate_space",
    "confidence", "source_age_ms", "scene_type", "road_polygon_valid",
    "left_boundary_valid", "right_boundary_valid",
)
SCENE_TYPES = ("normal", "intersection", "open_area", "unknown")


class HybridNetsJsonReceiver:
    def __init__(self):
        bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
        port = int(rospy.get_param("~result_port", 6000))
        self.allowed_sender = rospy.get_param("~allowed_sender_ip", "").strip()
        self.timeout = float(rospy.get_param("~result_timeout_sec", 0.8))
        self.max_source_age_ms = float(rospy.get_param("~max_source_age_ms", 800.0))
        self.max_points = int(rospy.get_param("~max_polygon_points", 20000))
        self.width = int(rospy.get_param("~image_width", 1920))
        self.height = int(rospy.get_param("~image_height", 1080))
        self.require_control_fields = bool(rospy.get_param(
            "~require_control_fields", False))
        self.coordinate_space = rospy.get_param(
            "~coordinate_space", "rotated_image_pixels")
        self.min_right_points = int(rospy.get_param(
            "~min_right_boundary_points", 3))
        self.min_left_points = int(rospy.get_param(
            "~min_left_boundary_points", 3))
        self.min_road_polygon_points = int(rospy.get_param(
            "~min_road_polygon_points", 3))
        self.min_control_confidence = float(rospy.get_param(
            "~min_control_confidence", 0.65))
        self.confirm_valid_frames = int(rospy.get_param(
            "~confirm_valid_frames", 3))
        self.last_receive = None
        self.last_frame_seq = None
        self.valid_streak = 0
        self.last_sender = ""
        self.last_reason = "waiting_for_packet"

        self.json_pub = rospy.Publisher(
            rospy.get_param("~result_json_topic", "/hybridnets/result_json"),
            String, queue_size=1)
        self.valid_pub = rospy.Publisher(
            rospy.get_param("~valid_topic", "/hybridnets/valid"),
            Bool, queue_size=1, latch=True)
        self.hz_pub = rospy.Publisher(
            rospy.get_param("~inference_hz_topic", "/hybridnets/inference_hz"),
            Float32, queue_size=1)
        self.age_pub = rospy.Publisher(
            rospy.get_param("~network_age_topic", "/hybridnets/network_age_ms"),
            Float32, queue_size=1)
        self.confidence_pub = rospy.Publisher(
            rospy.get_param("~confidence_topic", "/hybridnets/confidence"),
            Float32, queue_size=1)
        self.frame_seq_pub = rospy.Publisher(
            rospy.get_param("~frame_seq_topic", "/hybridnets/frame_seq"),
            UInt64, queue_size=1)
        self.left_valid_pub = rospy.Publisher(
            "/hybridnets/left_boundary_valid", Bool, queue_size=1)
        self.right_valid_pub = rospy.Publisher(
            "/hybridnets/right_boundary_valid", Bool, queue_size=1)
        self.intersection_pub = rospy.Publisher(
            "/hybridnets/intersection", Bool, queue_size=1)
        self.header_pub = rospy.Publisher(
            rospy.get_param("~capture_header_topic", "/hybridnets/capture_header"),
            Header, queue_size=1)
        self.diag_pub = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=2)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_ip, port))
        self.sock.settimeout(0.1)
        rospy.Timer(rospy.Duration(0.1), self.health_timer)
        rospy.on_shutdown(self.sock.close)
        self.valid_pub.publish(False)
        rospy.loginfo("Listening for HybridNets JSON on UDP %s:%d", bind_ip, port)

    def validate_points(self, result, name):
        points = result[name]
        if not isinstance(points, list) or len(points) > self.max_points:
            raise ValueError("%s is not a valid bounded list" % name)
        for point in points:
            if (not isinstance(point, (list, tuple)) or len(point) != 2 or
                    not all(isinstance(v, (int, float)) and not isinstance(v, bool) and
                            math.isfinite(v)
                            for v in point)):
                raise ValueError("%s contains an invalid pixel" % name)
            if not (0.0 <= point[0] < self.width and
                    0.0 <= point[1] < self.height):
                raise ValueError("%s contains an out-of-range pixel" % name)

    def validate_control_fields(self, result):
        missing = [name for name in CONTROL_FIELDS if name not in result]
        if missing:
            if self.require_control_fields:
                raise ValueError("control fields are missing: %s" % ",".join(missing))
            return (None, 0.0, "normal", bool(result["road_polygon_px"]),
                    bool(result["left_boundary_px"]), bool(result["right_boundary_px"]))

        version = result["version"]
        frame_seq = result["frame_seq"]
        confidence = result["confidence"]
        publish = result["publish_timestamp_ns"]
        source_age = result["source_age_ms"]
        scene_type = result["scene_type"]
        road_valid = result["road_polygon_valid"]
        left_valid = result["left_boundary_valid"]
        right_valid = result["right_boundary_valid"]
        if not isinstance(version, int) or isinstance(version, bool) or version < 3:
            raise ValueError("version must be an integer >= 3")
        if not isinstance(frame_seq, int) or isinstance(frame_seq, bool) or frame_seq < 0:
            raise ValueError("frame_seq must be a non-negative integer")
        if (not isinstance(confidence, (int, float)) or isinstance(confidence, bool) or
                not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0):
            raise ValueError("confidence must be in [0, 1]")
        if (not isinstance(publish, int) or isinstance(publish, bool) or
                publish < int(result["capture_timestamp_ns"])):
            raise ValueError("publish_timestamp_ns precedes capture_timestamp_ns")
        if (not isinstance(source_age, (int, float)) or isinstance(source_age, bool) or
                not math.isfinite(source_age) or source_age < 0.0):
            raise ValueError("source_age_ms must be finite and non-negative")
        if result["coordinate_space"] != self.coordinate_space:
            raise ValueError("unexpected coordinate_space")
        if scene_type not in SCENE_TYPES:
            raise ValueError("unexpected scene_type")
        if not all(isinstance(value, bool) for value in
                   (road_valid, left_valid, right_valid)):
            raise ValueError("boundary validity fields must be JSON booleans")
        for name in ("inference_ms", "inference_hz"):
            value = result.get(name, 0.0)
            if (not isinstance(value, (int, float)) or isinstance(value, bool) or
                    not math.isfinite(value) or value < 0.0):
                raise ValueError("%s must be finite and non-negative" % name)
        if result["valid"]:
            if not road_valid or len(result["road_polygon_px"]) < self.min_road_polygon_points:
                raise ValueError("valid result has too few road-polygon points")
            if left_valid and len(result["left_boundary_px"]) < self.min_left_points:
                raise ValueError("valid result has too few left-boundary points")
            if right_valid and len(result["right_boundary_px"]) < self.min_right_points:
                raise ValueError("valid result has too few right-boundary points")
            # HybridNets does not classify road/intersection scenes.  Treat
            # "unknown" as missing scene metadata, not as an invalid result;
            # consumers may ignore the scene field while still using the
            # road polygon and independently validated boundaries.
        return (frame_seq, float(confidence), scene_type, road_valid,
                left_valid, right_valid)

    def validate_boundary_segments(self, result):
        row_step = result.get("boundary_row_step_px", 20.0)
        if (not isinstance(row_step, (int, float)) or isinstance(row_step, bool) or
                not math.isfinite(row_step) or row_step <= 0):
            raise ValueError("boundary_row_step_px must be finite and positive")
        for side in ("left", "right"):
            measured_name = side + "_boundary_segments_px"
            valid_name = side + "_valid_segments_px"
            quality_name = side + "_segment_quality"
            measured = self.validate_segments(result, measured_name)
            valid_segments = self.validate_segments(result, valid_name)
            qualities = result.get(quality_name)
            if qualities is not None:
                if (not isinstance(qualities, list) or
                        measured is not None and len(qualities) != len(measured)):
                    raise ValueError("%s does not match measured segments" % quality_name)
                if not all(isinstance(value, (int, float)) and
                           not isinstance(value, bool) and math.isfinite(value) and
                           0.0 <= value <= 1.0 for value in qualities):
                    raise ValueError("%s contains an invalid score" % quality_name)
            temporal_name = side + "_temporal_score"
            temporal = result.get(temporal_name)
            if (temporal is not None and
                    (not isinstance(temporal, (int, float)) or
                     isinstance(temporal, bool) or not math.isfinite(temporal) or
                     not 0.0 <= temporal <= 1.0)):
                raise ValueError("%s must be in [0, 1]" % temporal_name)
            if valid_segments is not None:
                side_valid = result.get(side + "_boundary_valid", False)
                minimum = self.min_left_points if side == "left" else self.min_right_points
                if side_valid and not any(len(segment) >= minimum
                                          for segment in valid_segments):
                    raise ValueError("%s has no usable segment" % valid_name)
                if not side_valid and valid_segments:
                    raise ValueError("%s must be empty when side is invalid" % valid_name)

    def validate_segments(self, result, name):
        if name not in result:
            return None
        segments = result[name]
        if not isinstance(segments, list) or len(segments) > self.max_points:
            raise ValueError("%s is not a valid bounded list" % name)
        count = 0
        for segment in segments:
            self.validate_points({name: segment}, name)
            count += len(segment)
        if count > self.max_points:
            raise ValueError("%s has too many points" % name)
        return segments

    def receive_once(self):
        try:
            raw, sender = self.sock.recvfrom(65535)
        except socket.timeout:
            return
        if self.allowed_sender and sender[0] != self.allowed_sender:
            rospy.logwarn_throttle(2.0, "Ignoring HybridNets packet from %s", sender[0])
            return
        try:
            result = json.loads(raw.decode("utf-8"))
            if not isinstance(result, dict) or not all(key in result for key in REQUIRED):
                raise ValueError("required fields are missing")
            if not isinstance(result["valid"], bool):
                raise ValueError("valid must be a JSON boolean")
            if (not isinstance(result["capture_timestamp_ns"], int) or
                    isinstance(result["capture_timestamp_ns"], bool) or
                    result["capture_timestamp_ns"] <= 0):
                raise ValueError("capture_timestamp_ns must be a positive integer")
            if (result["image_width"] != self.width or
                    result["image_height"] != self.height):
                raise ValueError("result resolution does not match camera calibration")
            for name in POINT_FIELDS:
                self.validate_points(result, name)
            self.validate_boundary_segments(result)
            lane_lines = result.get("lane_lines_px", [])
            if not isinstance(lane_lines, list):
                raise ValueError("lane_lines_px is not a list")
            if sum(len(line) for line in lane_lines if isinstance(line, list)) > self.max_points:
                raise ValueError("lane_lines_px is too large")
            for line in lane_lines:
                if not isinstance(line, list):
                        raise ValueError("lane_lines_px contains a non-list line")
                for point in line:
                    if (not isinstance(point, (list, tuple)) or len(point) != 2 or
                            not all(isinstance(v, (int, float)) and
                                    not isinstance(v, bool) and math.isfinite(v)
                                    for v in point) or
                            not (0.0 <= point[0] < self.width and
                                 0.0 <= point[1] < self.height)):
                        raise ValueError("lane_lines_px contains an invalid pixel")

            (frame_seq, confidence, scene_type, road_valid,
             left_valid, right_valid) = self.validate_control_fields(result)

            now = rospy.Time.now()
            if (self.last_receive is None or
                    (now - self.last_receive).to_sec() > self.timeout):
                self.last_frame_seq = None
            capture = int(result["capture_timestamp_ns"])
            age_ms = (now.to_nsec() - capture) / 1.0e6
            clocks_comparable = -1000.0 <= age_ms <= 60000.0
            source_age = float(result.get("source_age_ms", 0.0))
            effective_age = max(source_age, age_ms) if clocks_comparable else source_age
            sequence_ok = (frame_seq is None or self.last_frame_seq is None or
                           frame_seq > self.last_frame_seq)
            confidence_ok = (not self.require_control_fields or
                             confidence >= self.min_control_confidence)
            candidate_valid = (result["valid"] and sequence_ok and confidence_ok and
                               effective_age <= self.max_source_age_ms)
            self.valid_streak = self.valid_streak + 1 if candidate_valid else 0
            confirmed = (not self.require_control_fields or
                         self.valid_streak >= self.confirm_valid_frames)
            packet_valid = candidate_valid and confirmed

            self.last_receive = now
            if frame_seq is not None and sequence_ok:
                self.last_frame_seq = frame_seq
            self.last_sender = "%s:%d" % sender
            if packet_valid:
                self.last_reason = "ok"
            elif candidate_valid:
                self.last_reason = "confirming_valid_frames"
            elif not sequence_ok:
                self.last_reason = "out_of_order_frame"
            elif not confidence_ok:
                self.last_reason = "low_confidence"
            else:
                self.last_reason = "invalid_or_stale_result"
            normalized = json.dumps(result, separators=(",", ":"), ensure_ascii=False)
            self.json_pub.publish(normalized)
            self.valid_pub.publish(packet_valid)
            self.hz_pub.publish(float(result.get("inference_hz", 0.0)))
            self.age_pub.publish(float(effective_age))
            self.confidence_pub.publish(confidence)
            self.left_valid_pub.publish(left_valid)
            self.right_valid_pub.publish(right_valid)
            self.intersection_pub.publish(scene_type == "intersection")
            if frame_seq is not None:
                self.frame_seq_pub.publish(frame_seq)
            header = Header(stamp=rospy.Time.from_sec(capture / 1.0e9),
                            frame_id=str(result.get("frame_id", "camera_optical_frame")))
            self.header_pub.publish(header)
        except (UnicodeDecodeError, ValueError, TypeError, OverflowError) as error:
            self.valid_streak = 0
            self.last_reason = "bad_packet"
            self.valid_pub.publish(False)
            rospy.logwarn_throttle(2.0, "Rejected HybridNets packet: %s", error)

    def health_timer(self, _event):
        now = rospy.Time.now()
        timed_out = self.last_receive is None or (now - self.last_receive).to_sec() > self.timeout
        if timed_out:
            self.valid_streak = 0
            self.valid_pub.publish(False)
            self.last_reason = "network_timeout" if self.last_receive else "waiting_for_packet"
        diagnostic = DiagnosticStatus()
        diagnostic.name = "hybridnets/network_receiver"
        diagnostic.hardware_id = self.last_sender or "udp_remote_gpu"
        diagnostic.level = DiagnosticStatus.ERROR if timed_out else DiagnosticStatus.OK
        diagnostic.message = self.last_reason
        diagnostic.values = [KeyValue("timeout_sec", str(self.timeout)),
                             KeyValue("sender", self.last_sender)]
        self.diag_pub.publish(DiagnosticArray(header=Header(stamp=now),
                                              status=[diagnostic]))

    def run(self):
        while not rospy.is_shutdown():
            self.receive_once()


if __name__ == "__main__":
    rospy.init_node("hybridnets_json_receiver")
    HybridNetsJsonReceiver().run()
