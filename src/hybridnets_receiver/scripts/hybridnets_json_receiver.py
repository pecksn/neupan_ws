#!/usr/bin/env python3
"""Validate HybridNets UDP JSON and expose health-aware ROS1 topics."""

import json
import math
import socket

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Bool, Float32, Header, String


REQUIRED = (
    "capture_timestamp_ns", "image_width", "image_height", "valid",
    "road_polygon_px", "left_boundary_px", "right_boundary_px",
)
POINT_FIELDS = ("road_polygon_px", "left_boundary_px", "right_boundary_px")


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
        self.last_receive = None
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
                    not all(isinstance(v, (int, float)) and math.isfinite(v)
                            for v in point)):
                raise ValueError("%s contains an invalid pixel" % name)

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
            if int(result["image_width"]) != self.width or int(result["image_height"]) != self.height:
                raise ValueError("result resolution does not match camera calibration")
            for name in POINT_FIELDS:
                self.validate_points(result, name)
            lane_lines = result.get("lane_lines_px", [])
            if not isinstance(lane_lines, list):
                raise ValueError("lane_lines_px is not a list")
            if sum(len(line) for line in lane_lines if isinstance(line, list)) > self.max_points:
                raise ValueError("lane_lines_px is too large")

            now = rospy.Time.now()
            capture = int(result["capture_timestamp_ns"])
            age_ms = (now.to_nsec() - capture) / 1.0e6
            clocks_comparable = -1000.0 <= age_ms <= 60000.0
            source_age = float(result.get("source_age_ms", 0.0))
            effective_age = max(source_age, age_ms) if clocks_comparable else source_age
            packet_valid = bool(result["valid"]) and effective_age <= self.max_source_age_ms

            self.last_receive = now
            self.last_sender = "%s:%d" % sender
            self.last_reason = "ok" if packet_valid else "invalid_or_stale_result"
            normalized = json.dumps(result, separators=(",", ":"), ensure_ascii=False)
            self.json_pub.publish(normalized)
            self.valid_pub.publish(packet_valid)
            self.hz_pub.publish(float(result.get("inference_hz", 0.0)))
            self.age_pub.publish(float(effective_age))
            header = Header(stamp=rospy.Time.from_sec(capture / 1.0e9),
                            frame_id=str(result.get("frame_id", "camera_optical_frame")))
            self.header_pub.publish(header)
        except (UnicodeDecodeError, ValueError, TypeError, OverflowError) as error:
            self.last_reason = "bad_packet"
            self.valid_pub.publish(False)
            rospy.logwarn_throttle(2.0, "Rejected HybridNets packet: %s", error)

    def health_timer(self, _event):
        now = rospy.Time.now()
        timed_out = self.last_receive is None or (now - self.last_receive).to_sec() > self.timeout
        if timed_out:
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
