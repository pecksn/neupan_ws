#!/usr/bin/python3
"""Record matched HybridNets debug images and JSON to MP4/JSONL pairs."""

import datetime
import json
from collections import OrderedDict
import os
import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DebugMp4Recorder:
    def __init__(self):
        self.topic = rospy.get_param("~image_topic", "/hybridnets/debug_image")
        self.output_dir = os.path.abspath(os.path.expanduser(rospy.get_param(
            "~output_directory", "/home/nuc/lxw/neupan_ws/videos/hybridnets_debug")))
        self.fps = float(rospy.get_param("~fps", 30.0))
        self.idle_timeout = float(rospy.get_param("~idle_timeout_sec", 3.0))
        if self.fps <= 0 or self.idle_timeout <= 0:
            raise ValueError("fps and idle_timeout_sec must be positive")

        self.lock = threading.Lock()
        self.writer = None
        self.size = None
        self.path = None
        self.frame_count = 0
        self.last_frame_at = None
        self.first_stamp = None
        self.last_frame = None
        self.json_file = None
        self.recorded_images = OrderedDict()
        self.pending_results = OrderedDict()

        rospy.Subscriber(self.topic, Image, self.on_image, queue_size=1,
                         buff_size=1920 * 1080 * 3 * 2)
        rospy.Subscriber("/hybridnets/debug_result_json", String,
                         self.on_result, queue_size=100)
        rospy.Timer(rospy.Duration(0.5), self.check_idle)
        rospy.on_shutdown(self.finish)
        rospy.loginfo("Waiting for %s; MP4 output directory: %s", self.topic,
                      self.output_dir)

    def start(self, width, height):
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.datetime.now(
            datetime.timezone(datetime.timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.output_dir, "%s.mp4" % timestamp)
        suffix = 1
        while os.path.exists(path) or os.path.exists(os.path.splitext(path)[0] + ".jsonl"):
            path = os.path.join(self.output_dir, "%s_%d.mp4" % (timestamp, suffix))
            suffix += 1
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"),
                                 self.fps, (width, height))
        if not writer.isOpened():
            raise RuntimeError("Could not open MP4 writer: %s" % path)
        try:
            self.json_file = open(os.path.splitext(path)[0] + ".jsonl", "x", buffering=1)
        except OSError:
            writer.release()
            raise
        self.writer = writer
        self.size = (width, height)
        self.path = path
        self.frame_count = 0
        self.first_stamp = None
        self.last_frame = None
        rospy.loginfo("Recording HybridNets debug MP4: %s", path)

    def write_result_locked(self, key):
        if self.json_file is None or key not in self.recorded_images or key not in self.pending_results:
            return
        frame_index = self.recorded_images.pop(key)
        result = self.pending_results.pop(key)
        result["mp4_frame_index"] = frame_index
        result["mp4_time_sec"] = frame_index / self.fps
        self.json_file.write(json.dumps(result, separators=(",", ":")) + "\n")

    def on_result(self, message):
        try:
            result = json.loads(message.data)
            key = (result["debug_image_seq"], result["debug_image_timestamp_ns"])
        except (ValueError, KeyError, TypeError):
            return
        with self.lock:
            self.pending_results[key] = result
            self.write_result_locked(key)
            while len(self.pending_results) > 512:
                self.pending_results.popitem(last=False)

    def on_image(self, message):
        width, height = message.width, message.height
        if (message.encoding not in ("bgr8", "rgb8") or
                message.step != width * 3 or
                len(message.data) != width * height * 3):
            rospy.logwarn_throttle(3.0, "MP4 recorder needs packed bgr8/rgb8 images")
            return

        frame = np.frombuffer(message.data, dtype=np.uint8).reshape(height, width, 3)
        if message.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        with self.lock:
            if self.writer is None:
                try:
                    self.start(width, height)
                except (OSError, RuntimeError) as error:
                    rospy.logerr_throttle(3.0, "MP4 recording failed: %s", error)
                    return
            elif self.size != (width, height):
                rospy.logwarn("Debug image size changed; closing current MP4")
                self.finish_locked()
                try:
                    self.start(width, height)
                except (OSError, RuntimeError) as error:
                    rospy.logerr_throttle(3.0, "MP4 recording failed: %s", error)
                    return
            stamp = message.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = time.time()
            if self.first_stamp is None:
                self.first_stamp = stamp
                self.writer.write(frame)
                self.frame_count = 1
                written_index = 0
            else:
                written_index = None
                # Preserve message time in a constant-frame-rate MP4. Sparse
                # inference frames are held until the next matched result.
                target_index = max(0, int(round(
                    (stamp - self.first_stamp) * self.fps)))
                while self.frame_count < target_index:
                    self.writer.write(self.last_frame)
                    self.frame_count += 1
                if self.frame_count == target_index:
                    self.writer.write(frame)
                    written_index = self.frame_count
                    self.frame_count += 1
            if written_index is not None:
                key = (message.header.seq, message.header.stamp.to_nsec())
                self.recorded_images[key] = written_index
                self.write_result_locked(key)
                while len(self.recorded_images) > 512:
                    self.recorded_images.popitem(last=False)
            self.last_frame = frame.copy()
            self.last_frame_at = time.monotonic()

    def check_idle(self, _event):
        with self.lock:
            if (self.writer is not None and self.last_frame_at is not None and
                    time.monotonic() - self.last_frame_at >= self.idle_timeout):
                self.finish_locked()

    def finish_locked(self):
        if self.writer is None:
            return
        self.writer.release()
        if self.json_file is not None:
            self.json_file.close()
            self.json_file = None
        self.recorded_images.clear()
        self.pending_results.clear()
        rospy.loginfo("Saved HybridNets debug MP4: %s (%d frames)",
                      self.path, self.frame_count)
        self.writer = None
        self.size = None
        self.path = None
        self.frame_count = 0
        self.last_frame_at = None
        self.first_stamp = None
        self.last_frame = None

    def finish(self):
        with self.lock:
            self.finish_locked()


if __name__ == "__main__":
    rospy.init_node("hybridnets_debug_mp4_recorder")
    DebugMp4Recorder()
    rospy.spin()
