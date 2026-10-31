#!/usr/bin/env python3
"""Validate HybridNets UDP JSON and expose health-aware ROS1 topics."""

import json
import math
import os
import socket
import sys
import threading
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from hybridnets_binding import ReceiverBinding

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
        self.binding_client = ReceiverBinding()
        if not self.allowed_sender:
            raise ValueError('allowed_sender_ip is required for active GPU identity confirmation')
        self.binding_address = (self.allowed_sender, int(rospy.get_param('~management_port', 6001)))
        self.binding_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.binding_socket.setblocking(False)
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
            "~min_control_confidence", 0.60))
        self.confirm_valid_frames = int(rospy.get_param(
            "~confirm_valid_frames", 3))
        self.last_receive = None
        self.last_frame_seq = None
        self.valid_streak = 0
        self.stream_streak = 0
        self.last_sender = ""
        self.last_reason = "waiting_for_packet"
        self.receive_times = deque(maxlen=120)
        self.last_result_seq = None
        self.last_timeout_diagnostic = None
        self.state_lock = threading.RLock()
        self.active_session = None
        self.retired_sessions = set()
        self.identity_seen = False
        self.producer_identity_seen = False
        self.last_active_mono = None
        self.last_capture_ns = None
        self.received_datagrams = 0
        self.rejected_sender_datagrams = 0
        self.last_datagram_mono = None
        self.last_datagram_sender = None
        self.diag_json_pub = rospy.Publisher(
            "/hybridnets/timing_diagnostics", String, queue_size=100)

        self.observations_pub = rospy.Publisher(
            rospy.get_param("~observations_json_topic", "/hybridnets/observations_json"),
            String, queue_size=10)
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
        self.sock.bind((bind_ip, port))
        self.sock.settimeout(0.1)
        rospy.Timer(rospy.Duration(0.1), self.health_timer)
        rospy.on_shutdown(self.sock.close)
        rospy.on_shutdown(self.binding_socket.close)
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
            if self.require_control_fields or getattr(self, 'binding_client', None):
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
            if road_valid and len(result["road_polygon_px"]) < self.min_road_polygon_points:
                raise ValueError("valid result has too few road-polygon points")
            if not road_valid and not right_valid:
                raise ValueError("valid result has neither road polygon nor right boundary")
            # Independently valid right-edge evidence can feed advisory map
            # refinement even if an occlusion broke the visible road polygon.
            # Do not change a producer's valid=False or invent missing edges.
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

    @staticmethod
    def session_identity(result, sender):
        epoch = result.get('camera_time_epoch')
        producer = result.get('producer_session_id')
        if producer is None:
            producer = result.get('session_id')
        for name, value in (('camera_time_epoch', epoch), ('producer_session_id', producer)):
            if value is not None and (not isinstance(value, str) or not value.strip() or len(value) > 128):
                raise ValueError('%s must be a nonempty string of at most 128 characters' % name)
        if (result.get('producer_session_id') is not None and result.get('session_id') is not None
                and result['producer_session_id'] != result['session_id']):
            raise ValueError('conflicting producer session identities')
        # Legacy GPU packets without a producer ID use the UDP source port.
        # Explicit producer IDs are preferred: a reused source port+epoch cannot
        # unambiguously distinguish a restart from a delayed packet.
        basis = 'producer_session' if producer is not None else 'udp_endpoint'
        return (sender[0], 'producer:'+producer if producer is not None else 'udp:%d' % sender[1], epoch), basis

    def select_session(self, identity, fresh):
        # In production only the management handshake selects identity. Data
        # packets, including previously unseen UUIDs, cannot select a session.
        binding = getattr(self, 'binding_client', None)
        if binding:
            packet = dict(producer_session_id=identity[1][9:] if identity[1].startswith('producer:') else None,
                          camera_time_epoch=identity[2], binding_ticket=identity[3] if len(identity)>3 else None,
                          receiver_instance_id=identity[4] if len(identity)>4 else None)
            allowed, reason = binding.accepts(packet, time.monotonic())
            if not allowed:
                return False, False, reason
        if not binding and identity in self.retired_sessions:
            return False, False, 'retired_session'
        if not binding and self.identity_seen and identity[2] is None:
            return False, False, 'session_identity_missing'
        if self.producer_identity_seen and identity[1].startswith('udp:'):
            return False, False, 'producer_session_identity_missing'
        if identity == self.active_session:
            return True, False, None
        if not fresh:
            return False, False, 'session_time_invalid'
        if len(self.retired_sessions) >= 1024:
            if not binding:
                return False, False, 'session_history_full_restart_receiver'
            # Challenged leases, not a UUID blacklist, decide ownership.
            # Bound diagnostic history without permanently blocking a healthy lease.
            if self.active_session is not None:
                self.retired_sessions.pop()
        if self.active_session is not None:
            self.retired_sessions.add(self.active_session)
        self.active_session = identity
        self.identity_seen = self.identity_seen or identity[2] is not None
        self.producer_identity_seen = self.producer_identity_seen or not identity[1].startswith('udp:')
        self.last_frame_seq = self.last_result_seq = self.last_capture_ns = None
        self.valid_streak = 0
        self.stream_streak = 0
        self.last_active_mono = None
        self.last_receive = None
        self.receive_times.clear()
        self.valid_pub.publish(False)
        return True, True, None

    def receive_once(self):
        try:
            raw, sender = self.sock.recvfrom(65535)
            receive_ns = rospy.Time.now().to_nsec()
            receive_mono = time.monotonic()
        except socket.timeout:
            return
        with self.state_lock:
            self.process_datagram(raw, sender, receive_ns, receive_mono)

    def process_datagram(self, raw, sender, receive_ns, receive_mono):
        # Caller holds state_lock; health timer uses the same lock so a session
        # switch cannot race the confirmation/timeout state.
        self.received_datagrams = getattr(self, 'received_datagrams', 0) + 1
        self.last_datagram_mono = receive_mono
        self.last_datagram_sender = '%s:%d' % sender
        if self.allowed_sender and sender[0] != self.allowed_sender:
            self.rejected_sender_datagrams = getattr(self, 'rejected_sender_datagrams', 0) + 1
            # Do not confuse a source-IP rejection with an unplugged cable.
            # Throttle diagnostics, not validation: no rejected packet enters control.
            last = getattr(self, 'last_sender_reject_diagnostic', None)
            if last is None or receive_mono-last >= 1.:
                self.last_sender_reject_diagnostic = receive_mono
                self.diag_json_pub.publish(json.dumps({
                    'schema_version': 1, 'event': 'sender_rejected',
                    'receive_timestamp_ns': receive_ns, 'receiver_valid': False,
                    'receiver_reason': 'sender_ip_not_allowed',
                    'sender': self.last_datagram_sender, 'allowed_sender_ip': self.allowed_sender,
                    'datagram_bytes': len(raw),
                }, separators=(',', ':')))
            rospy.logwarn_throttle(2.0, "Ignoring HybridNets packet from %s", sender[0])
            return
        try:
            result = json.loads(raw.decode("utf-8"))
            binding = getattr(self, 'binding_client', None)
            if binding and isinstance(result, dict):
                eligible, binding_reason = binding.accepts(result, receive_mono)
                if not eligible:
                    # Even an UNSEEN old producer cannot retire the active one.
                    # Reject before geometry parsing; data cannot nominate identity.
                    if (binding.pending is None and
                            receive_mono-getattr(self, 'last_binding_query_trigger', 0.) >= 1.):
                        self.last_binding_query_trigger = receive_mono
                        binding.next_probe = min(binding.next_probe, receive_mono)
                    self.diag_json_pub.publish(json.dumps(dict(event='binding_rejected',
                        receive_timestamp_ns=receive_ns, receiver_valid=False,
                        receiver_reason=binding_reason, forwarded=False,
                        producer_session_id=result.get('producer_session_id'),
                        receiver_instance_id=result.get('receiver_instance_id'),
                        binding=binding.status(receive_mono))))
                    return
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
            capture = int(result["capture_timestamp_ns"])
            age_ms = (now.to_nsec() - capture) / 1.0e6
            clocks_comparable = -1000.0 <= age_ms <= 60000.0
            source_age = float(result.get("source_age_ms", 0.0))
            # Clock disagreement is not permission to accept arbitrarily old images.
            # Keep the raw skew diagnostic, but fail closed until clocks are fixed.
            effective_age = max(source_age, age_ms)
            fresh = (effective_age <= self.max_source_age_ms and age_ms >= -50.)
            identity, session_basis = self.session_identity(result, sender)
            if getattr(self, 'binding_client', None):
                identity = identity+(result.get('binding_ticket'), result.get('receiver_instance_id'))
                session_basis = 'challenge_lease'
            session_ok, session_changed, session_reason = self.select_session(identity, fresh)
            result_seq = result.get('result_seq')
            if not isinstance(result_seq, int) or isinstance(result_seq, bool) or result_seq < 1:
                result_seq = None
            sequence_ok = (session_ok and
                           (frame_seq is None or self.last_frame_seq is None or frame_seq > self.last_frame_seq) and
                           (result_seq is None or self.last_result_seq is None or result_seq > self.last_result_seq))
            capture_ok = self.last_capture_ns is None or capture > self.last_capture_ns
            confidence_ok = ((not self.require_control_fields and not getattr(self, 'binding_client', None)) or
                             confidence >= self.min_control_confidence)
            forward = session_ok and sequence_ok and capture_ok and fresh
            candidate_valid = result['valid'] and forward and confidence_ok
            # A late retired session must not disturb the active session's
            # confirmation or heartbeat. Invalid current-session frames do.
            active_timed_out = (session_ok and (self.last_active_mono is None or
                                receive_mono-self.last_active_mono > self.timeout))
            if session_ok:
                if active_timed_out:
                    self.valid_streak = 0
                    self.stream_streak = 0
                # Duplicated/out-of-order datagrams are not new observations.
                # Ignore them without extending the heartbeat or breaking a
                # still-fresh confirmation streak. New invalid frames still reset it.
                if sequence_ok and capture_ok:
                    self.valid_streak = self.valid_streak + 1 if candidate_valid else 0
                    self.stream_streak = getattr(self, 'stream_streak', 0)+1 if forward else 0
            confirmed = ((not self.require_control_fields and not getattr(self, 'binding_client', None)) or
                         self.valid_streak >= self.confirm_valid_frames)
            packet_valid = candidate_valid and confirmed

            if not session_ok and session_reason != 'session_time_invalid':
                reason = session_reason
            elif not fresh:
                reason = 'capture_timestamp_in_future' if age_ms < -50. else 'receiver_source_age_exceeded'
            elif not capture_ok:
                reason = 'capture_timestamp_not_increasing'
            elif not sequence_ok:
                reason = 'out_of_order_frame'
            elif packet_valid:
                reason = 'ok'
            elif candidate_valid:
                reason = 'confirming_valid_frames'
            elif not confidence_ok:
                reason = 'low_confidence'
            else:
                reason = 'sender_result_invalid'
            gap = (max(0, result_seq-self.last_result_seq-1)
                   if forward and result_seq is not None and self.last_result_seq is not None else None)
            if forward:
                self.last_receive = now
                self.last_active_mono = receive_mono
                self.last_capture_ns = capture
                if frame_seq is not None:
                    self.last_frame_seq = frame_seq
                if result_seq is not None:
                    self.last_result_seq = result_seq
                self.last_sender = '%s:%d' % sender
            if session_ok and sequence_ok and capture_ok:
                self.last_reason = reason
            if packet_valid:
                self.last_reason = "ok"
            interval_ms = ((receive_mono-self.receive_times[-1])*1000
                           if self.receive_times else None)
            if forward:
                self.receive_times.append(receive_mono)
            span = self.receive_times[-1]-self.receive_times[0] if self.receive_times else 0.
            receive_hz = ((len(self.receive_times)-1)/span if span > 0 else None)
            send_ns = result.get("send_timestamp_ns")
            raw_send_age = ((receive_ns-send_ns)/1.e6
                            if isinstance(send_ns, int) and not isinstance(send_ns, bool) and send_ns > 0 else None)
            gpu_timing = result.get("timing_ms", {})
            gpu_timing = ({str(k): v for k, v in list(gpu_timing.items())[:32]
                           if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)}
                          if isinstance(gpu_timing, dict) else {})
            diagnostic = {
                "schema_version": 1, "event": "packet", "frame_seq": frame_seq,
                "sender": self.last_datagram_sender,
                "allowed_sender_ip": self.allowed_sender,
                "result_seq": result_seq, "result_seq_gap": gap,
                "receive_timestamp_ns": receive_ns,
                "capture_timestamp_ns": capture, "send_timestamp_ns": send_ns if raw_send_age is not None else None,
                "publish_timestamp_ns": result.get("publish_timestamp_ns"),
                "receiver_valid": bool(packet_valid), "receiver_reason": reason,
                "session_changed": session_changed, "session_basis": session_basis,
                "active_session": self.active_session, "packet_session": identity,
                "retired_session_count": len(self.retired_sessions),
                "valid_streak": self.valid_streak, "forwarded": bool(forward),
                "stream_streak": getattr(self, 'stream_streak', 0),
                "submitted_to_path": bool(forward and (not getattr(self, 'binding_client', None) or packet_valid)),
                "confidence": confidence, "source_age_ms": source_age,
                "capture_age_at_receive_ms_clock_dependent": (receive_ns-capture)/1.e6,
                "send_to_receive_ms_clock_dependent": raw_send_age,
                "clocks_plausible_not_verified": clocks_comparable,
                "effective_age_ms": effective_age, "receiver_max_source_age_ms": self.max_source_age_ms,
                "receiver_min_confidence": self.min_control_confidence,
                "receive_interval_ms": interval_ms, "receive_hz_window": receive_hz,
                "gpu_send_hz": result.get("gpu_send_hz"),
                "inference_hz": result.get("inference_hz"),
                "inference_ms": result.get("inference_ms"),
                "inference_backend": result.get("inference_backend"),
                "camera_time_epoch": result.get("camera_time_epoch"),
                "producer_session_id": result.get("producer_session_id", result.get("session_id")),
                "receiver_session_id": json.dumps(identity, separators=(',', ':')),
                "binding": self.binding_client.status(receive_mono) if getattr(self, 'binding_client', None) else None,
                "timing_ms": gpu_timing, "datagram_bytes": len(raw),
                "receiver_validation_wall_ms": (time.monotonic()-receive_mono)*1000,
            }
            result["receiver_diagnostics"] = diagnostic
            self.diag_json_pub.publish(json.dumps(diagnostic, separators=(",", ":")))
            if not forward:
                if (session_ok and ((sequence_ok and capture_ok) or active_timed_out)
                        or self.active_session is None):
                    self.valid_pub.publish(False)
                return
            result['receiver_session_id'] = json.dumps(identity, separators=(',', ':'))
            normalized = json.dumps(result, separators=(",", ":"), ensure_ascii=False)
            # Observations retain failed geometry/quality and confirmation frames.
            # Only the control topic is gated by visual confirmation.
            self.observations_pub.publish(normalized)
            if diagnostic['submitted_to_path']:
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
            self.diag_json_pub.publish(json.dumps({
                "schema_version": 1, "event": "bad_packet",
                "receive_timestamp_ns": receive_ns,
                "receiver_valid": False, "receiver_reason": "bad_packet",
                "error": str(error), "datagram_bytes": len(raw),
            }, separators=(",", ":")))
            rospy.logwarn_throttle(2.0, "Rejected HybridNets packet: %s", error)

    def health_timer(self, _event):
        with self.state_lock:
            self.binding_tick()
            self.health_timer_locked(_event)

    def binding_tick(self):
        client = getattr(self, 'binding_client', None)
        if not client:
            return
        try:
            for _ in range(16):
                try:
                    raw, sender = self.binding_socket.recvfrom(4096)
                except BlockingIOError:
                    break
                if sender != self.binding_address:
                    continue
                try:
                    response = json.loads(raw.decode('utf8'))
                except (ValueError, UnicodeError):
                    continue
                before = (client.bound['identity'], client.bound['ticket']) if client.bound else None
                message = client.reply(response, time.monotonic())
                if message:
                    self.binding_socket.sendto(json.dumps(message).encode('utf8'), self.binding_address)
                after = (client.bound['identity'], client.bound['ticket']) if client.bound else None
                if after != before:
                    self.valid_streak = 0
                    self.stream_streak = 0
                    self.last_active_mono = self.last_receive = None
                    self.valid_pub.publish(False)
                    self.diag_json_pub.publish(json.dumps(dict(event='binding_confirmed' if after else 'binding_invalidated',
                        receive_timestamp_ns=rospy.Time.now().to_nsec(), receiver_valid=False,
                        receiver_reason='binding_changed_reconfirm_three_frames' if after else client.reason,
                        binding=client.status(time.monotonic()))))
            message = client.request(time.monotonic())
            if message:
                self.binding_socket.sendto(json.dumps(message).encode('utf8'), self.binding_address)
        except OSError as error:
            rospy.logwarn_throttle(2., 'GPU binding management unavailable: %s', error)

    def health_timer_locked(self, _event):
        now = rospy.Time.now()
        binding = getattr(self, 'binding_client', None)
        binding_invalid = binding is not None and not binding.alive(time.monotonic())
        timed_out = binding_invalid or self.last_active_mono is None or time.monotonic()-self.last_active_mono > self.timeout
        if timed_out:
            self.valid_streak = 0
            self.stream_streak = 0
            self.valid_pub.publish(False)
            self.last_reason = "network_timeout" if self.last_receive else "waiting_for_packet"
            if binding_invalid:
                self.last_reason = binding.status(time.monotonic())['reason']
            mono = time.monotonic()
            if self.last_timeout_diagnostic is None or mono-self.last_timeout_diagnostic >= 1.0:
                self.last_timeout_diagnostic = mono
                self.diag_json_pub.publish(json.dumps({
                    "schema_version": 1, "event": "health",
                    "receive_timestamp_ns": now.to_nsec(),
                    "receiver_valid": False, "receiver_reason": self.last_reason,
                    "last_receive_age_ms": (mono-self.last_active_mono)*1000 if self.last_active_mono is not None else None,
                    "receiver_timeout_sec": self.timeout,
                    "active_session": self.active_session,
                    "binding": binding.status(mono) if binding else None,
                    "retired_session_count": len(self.retired_sessions),
                    "last_active_age_ms": None if self.last_active_mono is None
                    else (mono-self.last_active_mono)*1000.,
                    "received_datagrams": getattr(self, 'received_datagrams', 0),
                    "rejected_sender_datagrams": getattr(self, 'rejected_sender_datagrams', 0),
                    "allowed_sender_ip": self.allowed_sender,
                    "last_datagram_sender": getattr(self, 'last_datagram_sender', None),
                    "last_datagram_age_ms": None if getattr(self, 'last_datagram_mono', None) is None
                    else (mono-self.last_datagram_mono)*1000.,
                }, separators=(",", ":")))
                if self.require_control_fields:
                    rospy.logwarn_throttle(5.,
                        '\033[33mHybridNets control unavailable: %s; UDP received=%d rejected_sender=%d; '
                        'GPU must send inference JSON to NUC 192.168.10.21:6000 (video alone is insufficient)\033[0m',
                        self.last_reason, getattr(self, 'received_datagrams', 0),
                        getattr(self, 'rejected_sender_datagrams', 0))
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
