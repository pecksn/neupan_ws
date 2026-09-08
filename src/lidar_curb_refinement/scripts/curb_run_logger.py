#!/usr/bin/env python3
"""Continuously log MID360 curb observations until the ROS launch stops."""

import json
import os
import threading
import time
from datetime import datetime

import rospy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String


class CurbRunLogger:
    def __init__(self):
        rospy.init_node("curb_run_logger")
        self.edge_topic = rospy.get_param("~edge_topic", "/road_curb/right_edge")
        self.error_topic = rospy.get_param("~error_topic", "/road_curb/lateral_error")
        self.status_topic = rospy.get_param("~status_topic", "/road_curb/status")
        self.input_path_topic = rospy.get_param("~input_path_topic", "/osm_global_path")
        self.output_path_topic = rospy.get_param(
            "~output_path_topic", "/osm_global_path_curb_refined"
        )
        self.log_dir = rospy.get_param(
            "~log_dir", "/home/nuc/lxw/neupan_ws/tracker_logs"
        )
        self.sample_rate = max(1.0, float(rospy.get_param("~sample_rate", 10.0)))
        self.flush_interval = max(0.1, float(rospy.get_param("~flush_interval", 1.0)))
        self.max_bytes = max(
            1024, int(float(rospy.get_param("~max_file_size_mb", 256.0)) * 1024 * 1024)
        )

        self.lock = threading.Lock()
        self.latest = {
            "status": None,
            "lateral_error_m": None,
            "right_edge": None,
            "input_path": None,
            "refined_path": None,
        }
        self.received_at = {key: None for key in self.latest}
        self.samples = 0
        self.valid_samples = 0
        self.invalid_samples = 0
        self.bytes_written = 0
        self.capped = False
        self.last_flush = time.monotonic()
        self.file = None

        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(self.log_dir, "lidar_curb_{}.jsonl".format(stamp))
        self.file = open(self.log_path, "w", buffering=1)
        self.write(
            {
                "record_type": "metadata",
                "created_at_local": self.local_time(),
                "sample_rate_hz": self.sample_rate,
                "max_file_size_mb": self.max_bytes / (1024.0 * 1024.0),
                "topics": {
                    "status": self.status_topic,
                    "lateral_error": self.error_topic,
                    "right_edge": self.edge_topic,
                    "input_path": self.input_path_topic,
                    "refined_path": self.output_path_topic,
                },
            },
            force_flush=True,
        )

        rospy.Subscriber(self.status_topic, String, self.status_cb, queue_size=20)
        rospy.Subscriber(self.error_topic, Float32, self.error_cb, queue_size=20)
        rospy.Subscriber(self.edge_topic, PointStamped, self.edge_cb, queue_size=10)
        rospy.Subscriber(self.input_path_topic, Path, self.input_path_cb, queue_size=2)
        rospy.Subscriber(self.output_path_topic, Path, self.refined_path_cb, queue_size=2)
        rospy.Timer(rospy.Duration(1.0 / self.sample_rate), self.timer_cb)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("curb_run_logger: writing %.1f Hz log to %s", self.sample_rate, self.log_path)

    @staticmethod
    def local_time():
        return datetime.now().astimezone().isoformat(timespec="milliseconds")

    @staticmethod
    def msg_stamp(header):
        return None if header.stamp == rospy.Time() else header.stamp.to_sec()

    @staticmethod
    def path_summary(msg):
        result = {"frame_id": msg.header.frame_id, "points": len(msg.poses)}
        if msg.poses:
            first = msg.poses[0].pose.position
            last = msg.poses[-1].pose.position
            result.update(
                {"start_x": first.x, "start_y": first.y, "end_x": last.x, "end_y": last.y}
            )
        return result

    def update(self, key, value):
        with self.lock:
            self.latest[key] = value
            self.received_at[key] = rospy.Time.now().to_sec()

    def status_cb(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            status = {"valid": False, "reason": "invalid_status_json", "raw": msg.data}
        self.update("status", status)

    def error_cb(self, msg):
        self.update("lateral_error_m", float(msg.data))

    def edge_cb(self, msg):
        self.update(
            "right_edge",
            {
                "msg_stamp": self.msg_stamp(msg.header),
                "frame_id": msg.header.frame_id,
                "x": float(msg.point.x),
                "y": float(msg.point.y),
                "z": float(msg.point.z),
            },
        )

    def input_path_cb(self, msg):
        self.update("input_path", self.path_summary(msg))

    def refined_path_cb(self, msg):
        self.update("refined_path", self.path_summary(msg))

    def timer_cb(self, _event):
        with self.lock:
            latest = dict(self.latest)
            received_at = dict(self.received_at)
        valid = bool((latest.get("status") or {}).get("valid", False))
        self.samples += 1
        if valid:
            self.valid_samples += 1
        else:
            self.invalid_samples += 1
        self.write(
            {
                "record_type": "sample",
                "ros_time": rospy.Time.now().to_sec(),
                "local_time": self.local_time(),
                "sample_index": self.samples,
                "valid": valid,
                "received_at": received_at,
                **latest,
            }
        )

    def write(self, record, force_flush=False, ignore_size_limit=False):
        if self.file is None or (self.capped and not ignore_size_limit):
            return
        line = json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n"
        size = len(line.encode("utf-8"))
        if not ignore_size_limit and self.bytes_written + size > self.max_bytes:
            self.capped = True
            rospy.logwarn("curb_run_logger reached file limit; current log is preserved")
            return
        self.file.write(line)
        self.bytes_written += size
        now = time.monotonic()
        if force_flush or now - self.last_flush >= self.flush_interval:
            self.file.flush()
            os.fsync(self.file.fileno())
            self.last_flush = now

    def shutdown(self):
        if self.file is None:
            return
        self.write(
            {
                "record_type": "summary",
                "closed_at_local": self.local_time(),
                "samples": self.samples,
                "valid_samples": self.valid_samples,
                "invalid_samples": self.invalid_samples,
                "file_capped": self.capped,
            },
            force_flush=True,
            ignore_size_limit=True,
        )
        self.file.flush()
        os.fsync(self.file.fileno())
        self.file.close()
        self.file = None
        rospy.loginfo("curb_run_logger closed: %s", self.log_path)


if __name__ == "__main__":
    CurbRunLogger()
    rospy.spin()
