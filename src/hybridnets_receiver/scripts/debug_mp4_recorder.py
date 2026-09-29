#!/usr/bin/python3
"""Record the received HybridNets debug image to one MP4 per video burst."""

import datetime
import os
import threading
import time

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image


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

        rospy.Subscriber(self.topic, Image, self.on_image, queue_size=1,
                         buff_size=1920 * 1080 * 3 * 2)
        rospy.Timer(rospy.Duration(0.5), self.check_idle)
        rospy.on_shutdown(self.finish)
        rospy.loginfo("Waiting for %s; MP4 output directory: %s", self.topic,
                      self.output_dir)

    def start(self, width, height):
        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = os.path.join(self.output_dir, "hybridnets_debug_%s.mp4" % timestamp)
        writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"),
                                 self.fps, (width, height))
        if not writer.isOpened():
            raise RuntimeError("Could not open MP4 writer: %s" % path)
        self.writer = writer
        self.size = (width, height)
        self.path = path
        self.frame_count = 0
        self.first_stamp = None
        self.last_frame = None
        rospy.loginfo("Recording HybridNets debug MP4: %s", path)

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
            else:
                # Preserve message time in a constant-frame-rate MP4. Sparse
                # inference frames are held until the next matched result.
                target_index = max(0, int(round(
                    (stamp - self.first_stamp) * self.fps)))
                while self.frame_count < target_index:
                    self.writer.write(self.last_frame)
                    self.frame_count += 1
                if self.frame_count == target_index:
                    self.writer.write(frame)
                    self.frame_count += 1
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
