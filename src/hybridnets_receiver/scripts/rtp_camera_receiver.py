#!/usr/bin/python3
# Module contract: Decode the remote H.264/RTP stream into ROS images and camera
# calibration metadata. Optional virtual-video forwarding is bounded so it cannot stall
# ROS image reception.

"""Receive H.264/RTP with GStreamer and publish ROS1 Image + CameraInfo."""

import os
import math
import queue
import shutil
import subprocess
import threading
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

import rospy
import yaml
from sensor_msgs.msg import CameraInfo, Image


# Load the calibration YAML into CameraInfo for the requested optical frame; retain the
# calibration's image resolution and matrices.
# Inputs: path, frame_id.
def load_camera_info(path, frame_id):
    with open(os.path.expanduser(path), "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    message = CameraInfo()
    message.width = int(data["image_width"])
    message.height = int(data["image_height"])
    message.distortion_model = data.get("distortion_model", "plumb_bob")
    message.D = list(data["distortion_coefficients"]["data"])
    message.K = list(data["camera_matrix"]["data"])
    message.R = list(data["rectification_matrix"]["data"])
    message.P = list(data["projection_matrix"]["data"])
    message.binning_x = int(data.get("binning_x", 0))
    message.binning_y = int(data.get("binning_y", 0))
    roi = data.get("roi", {})
    message.roi.x_offset = int(roi.get("x_offset", 0))
    message.roi.y_offset = int(roi.get("y_offset", 0))
    message.roi.height = int(roi.get("height", 0))
    message.roi.width = int(roi.get("width", 0))
    message.roi.do_rectify = bool(roi.get("do_rectify", False))
    message.header.frame_id = frame_id
    return message


class VideoReceptionHealth:
    """Wall-clock health from accepted image frames, not sockets or node PIDs."""

    # Initialize the receive-health epoch and timeout in the supplied elapsed-time
    # convention.
    # Inputs: now, timeout.
    # Explicit rejection paths: ValueError.
    def __init__(self, now, timeout=3.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('stream_timeout_sec must be finite and positive')
        self.timeout = timeout
        self.last_frame = None
        self.frames = self.previous_frames = 0
        self.previous_sample = now
        self.lock = threading.Lock()

    # Record a newly accepted video frame in the video-health timing state.
    # Inputs: now.
    def accept(self, now):
        with self.lock:
            self.frames += 1
            self.last_frame = now

    # Return video health and timing evidence without pretending an old frame is newly
    # received.
    # Inputs: now.
    def report(self, now):
        with self.lock:
            elapsed = now - self.previous_sample
            fps = (self.frames - self.previous_frames) / elapsed if elapsed > 0 else 0.0
            age = None if self.last_frame is None else max(0.0, now - self.last_frame)
            state = (
                'waiting'
                if age is None
                else 'disconnected' if age >= self.timeout else 'receiving'
            )
            self.previous_sample, self.previous_frames = now, self.frames
            return dict(
                state=state,
                fps=fps if state == 'receiving' else 0.0,
                total_frames=self.frames,
                last_frame_age_sec=age,
            )


# Own the GStreamer receive pipeline, ROS image publication, health reporting, and
# optional virtual-video forwarding.
class RtpCameraReceiver:
    # Configure image/calibration publication, the GStreamer receive pipeline,
    # receive-health tracking, and optional virtual-video forwarding.
    # Explicit rejection paths: RuntimeError.
    def __init__(self):
        Gst.init(None)
        port = int(rospy.get_param("~video_port", 5000))
        latency = int(rospy.get_param("~rtp_latency_ms", 100))
        self.frame_id = rospy.get_param("~camera_frame_id", "camera_optical_frame")
        image_topic = rospy.get_param("~image_topic", "/camera/remote_image_raw")
        info_topic = rospy.get_param("~camera_info_topic", "/camera/remote_camera_info")
        info_file = rospy.get_param("~camera_info_file")
        self.expected_width = int(rospy.get_param("~image_width", 1920))
        self.expected_height = int(rospy.get_param("~image_height", 1080))
        self.virtual_device = rospy.get_param("~virtual_video_device", "").strip()
        self.virtual_fps = int(rospy.get_param("~virtual_video_fps", 30))
        self.virtual_process = None
        self.virtual_queue = queue.Queue(maxsize=1)
        self.virtual_stop = threading.Event()
        self.virtual_thread = None
        self.reception_health = VideoReceptionHealth(
            time.monotonic(), float(rospy.get_param('~stream_timeout_sec', 3.0))
        )
        self.health_stop = threading.Event()

        self.image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
        self.info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=1)
        self.camera_info = load_camera_info(info_file, self.frame_id)
        if (
            self.camera_info.width != self.expected_width
            or self.camera_info.height != self.expected_height
        ):
            raise RuntimeError(
                "camera_info resolution does not match receiver resolution"
            )

        description = (
            "udpsrc port={port} caps=\"application/x-rtp,media=video,"
            "encoding-name=H264,payload=96\" ! "
            "rtpjitterbuffer latency={latency} drop-on-latency=true ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=true drop=true max-buffers=1 sync=false "
        ).format(port=port, latency=latency)
        if self.virtual_device:
            self.start_virtual_video()
        self.pipeline = Gst.parse_launch(description)
        self.sink = self.pipeline.get_by_name("sink")
        self.sink.connect("new-sample", self.on_sample)
        self.bus = self.pipeline.get_bus()
        self.pipeline.set_state(Gst.State.PLAYING)
        rospy.Timer(rospy.Duration(0.2), self.check_bus)
        rospy.on_shutdown(self.shutdown)
        self.health_thread = threading.Thread(target=self.health_loop, daemon=True)
        self.health_thread.start()
        rospy.loginfo(
            "Receiving H.264/RTP camera on UDP %d -> %s%s",
            port,
            image_topic,
            " and " + self.virtual_device if self.virtual_device else "",
        )

    # Publish video reception health independently of image arrival.
    def health_loop(self):
        previous_state, emitted = None, -math.inf
        while not self.health_stop.is_set():
            now = time.monotonic()
            status = self.reception_health.report(now)
            state = status['state']
            interval = 1.0 if state == 'receiving' else 5.0
            if state != previous_state or now - emitted >= interval:
                if state == 'receiving':
                    rospy.loginfo(
                        '视频接收成功 | 接收帧率=%.1f FPS | 累计=%d帧 | 距上一帧=%.2fs',
                        status['fps'],
                        status['total_frames'],
                        status['last_frame_age_sec'],
                    )
                elif state == 'waiting':
                    rospy.loginfo(
                        '等待推流 | 尚未收到有效视频帧 | 接收帧率=0.0 FPS | 正在监听UDP视频端口'
                    )
                else:
                    rospy.logwarn(
                        '视频断流 | 已%.1fs没有有效视频帧 | 接收帧率=0.0 FPS | 继续监听，恢复来帧后自动更新',
                        status['last_frame_age_sec'],
                    )
                previous_state, emitted = state, now
            self.health_stop.wait(1.0)

    # Start the optional subprocess that forwards decoded frames to the configured
    # virtual video device.
    # Explicit rejection paths: RuntimeError.
    def start_virtual_video(self):
        if not os.path.exists(self.virtual_device):
            raise RuntimeError(
                "virtual video device does not exist: %s" % self.virtual_device
            )
        ffmpeg = shutil.which("ffmpeg")
        if ffmpeg is None:
            raise RuntimeError("ffmpeg is required for virtual_video_device")
        command = [
            ffmpeg,
            "-hide_banner",
            "-loglevel",
            "warning",
            "-f",
            "rawvideo",
            "-pixel_format",
            "bgr24",
            "-video_size",
            "%dx%d" % (self.expected_width, self.expected_height),
            "-framerate",
            str(self.virtual_fps),
            "-i",
            "pipe:0",
            "-an",
            "-pix_fmt",
            "yuyv422",
            "-f",
            "v4l2",
            self.virtual_device,
        ]
        self.virtual_process = subprocess.Popen(command, stdin=subprocess.PIPE)
        self.virtual_thread = threading.Thread(target=self.virtual_video_loop)
        self.virtual_thread.daemon = True
        self.virtual_thread.start()

    # Drain queued frames into the owned virtual-video process without blocking the
    # GStreamer streaming callback.
    def virtual_video_loop(self):
        while not self.virtual_stop.is_set():
            try:
                frame = self.virtual_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self.virtual_process.stdin.write(frame)
            except (BrokenPipeError, OSError, ValueError) as error:
                if not self.virtual_stop.is_set():
                    rospy.logerr("Writing %s failed: %s", self.virtual_device, error)
                    rospy.signal_shutdown("virtual camera writer failed")
                return

    # Queue a decoded frame for the bounded virtual-video forwarding path.
    # Inputs: frame.
    def queue_virtual_frame(self, frame):
        if not self.virtual_device or self.virtual_stop.is_set():
            return
        try:
            self.virtual_queue.put_nowait(frame)
        except queue.Full:
            try:
                self.virtual_queue.get_nowait()
            except queue.Empty:
                pass
            try:
                self.virtual_queue.put_nowait(frame)
            except queue.Full:
                pass

    # Validate a decoded GStreamer sample and publish its ROS image/calibration
    # metadata.
    # Inputs: sink.
    def on_sample(self, sink):
        # GStreamer invokes this on its streaming thread. Validate the decoded
        # shape before publishing or forwarding pixels; configured calibration
        # cannot be assumed valid for a different incoming image resolution.
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        caps = sample.get_caps().get_structure(0)
        width, height = caps.get_value("width"), caps.get_value("height")
        if width != self.expected_width or height != self.expected_height:
            rospy.logerr_throttle(
                2.0,
                "Remote video is %dx%d, expected %dx%d",
                width,
                height,
                self.expected_width,
                self.expected_height,
            )
            return Gst.FlowReturn.OK
        buffer = sample.get_buffer()
        ok, mapped = buffer.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        try:
            stamp = rospy.Time.now()
            image = Image()
            image.header.stamp = stamp
            image.header.frame_id = self.frame_id
            image.height = height
            image.width = width
            image.encoding = "bgr8"
            image.is_bigendian = False
            image.step = width * 3
            frame = bytes(mapped.data)
            expected_size = width * height * 3
            if len(frame) != expected_size:
                rospy.logerr_throttle(
                    2.0,
                    "Unexpected BGR frame size: %d, expected %d",
                    len(frame),
                    expected_size,
                )
                return Gst.FlowReturn.OK
            image.data = frame
            info = self.camera_info
            info.header.stamp = stamp
            self.image_pub.publish(image)
            self.info_pub.publish(info)
            self.reception_health.accept(time.monotonic())
            self.queue_virtual_frame(frame)
        finally:
            buffer.unmap(mapped)
        return Gst.FlowReturn.OK

    # Poll GStreamer messages and handle pipeline errors or end-of-stream notifications.
    # Inputs: _event.
    def check_bus(self, _event):
        message = self.bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.WARNING)
        if message is None:
            return
        error, debug = (
            message.parse_error()
            if message.type == Gst.MessageType.ERROR
            else message.parse_warning()
        )
        log = rospy.logerr if message.type == Gst.MessageType.ERROR else rospy.logwarn
        log("GStreamer: %s (%s)", error, debug)
        if message.type == Gst.MessageType.ERROR:
            rospy.signal_shutdown("GStreamer receiver failed")

    # Stop this component and release its resources without leaving background work
    # active.
    def shutdown(self):
        self.health_stop.set()
        self.pipeline.set_state(Gst.State.NULL)
        self.virtual_stop.set()
        if self.virtual_process is not None:
            if self.virtual_process.stdin is not None:
                try:
                    self.virtual_process.stdin.close()
                except OSError:
                    pass
            try:
                self.virtual_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.virtual_process.terminate()
                try:
                    self.virtual_process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    self.virtual_process.kill()


if __name__ == "__main__":
    rospy.init_node("rtp_camera_receiver")
    RtpCameraReceiver()
    rospy.spin()
