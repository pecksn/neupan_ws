#!/usr/bin/env python3
"""Optional CLRNet inference bridge for the road_vision_refinement package.

The bridge publishes a binary mask in the source camera resolution.  The
existing road vision node still performs temporal filtering and metric IPM,
so this node can be enabled or disabled independently.
"""

import importlib
import os
import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CLRNetLaneNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_rect_color")
        self.mask_topic = rospy.get_param("~mask_topic", "/road_vision/clrnet_mask")
        self.clrnet_root = os.path.abspath(rospy.get_param("~clrnet_root", ""))
        self.config_file = os.path.abspath(rospy.get_param("~config_file", ""))
        self.checkpoint = os.path.abspath(rospy.get_param("~checkpoint", ""))
        self.device_name = rospy.get_param("~device", "cpu")
        self.conf_threshold = float(rospy.get_param("~confidence_threshold", 0.40))
        self.line_width = max(1, int(rospy.get_param("~line_width", 7)))
        self.cut_height_ratio = float(rospy.get_param("~cut_height_ratio", 0.45))
        self.process_rate = float(rospy.get_param("~process_rate", 10.0))
        self.last_process_time = 0.0
        self.model = None
        self.cfg = None
        self.torch = None
        self.device = None
        self.model_ready = False

        self.mask_pub = rospy.Publisher(self.mask_topic, Image, queue_size=1)
        self.load_model()
        self.sub = rospy.Subscriber(self.image_topic, Image, self.image_cb,
                                     queue_size=1, buff_size=2 ** 24)

    def load_model(self):
        if not self.clrnet_root or not os.path.isdir(self.clrnet_root):
            rospy.logerr("CLRNet disabled: clrnet_root does not exist: %s", self.clrnet_root)
            return
        if not self.config_file or not os.path.isfile(self.config_file):
            rospy.logerr("CLRNet disabled: config_file does not exist: %s", self.config_file)
            return
        if not self.checkpoint or not os.path.isfile(self.checkpoint):
            rospy.logerr("CLRNet disabled: checkpoint does not exist: %s", self.checkpoint)
            return

        try:
            sys.path.insert(0, self.clrnet_root)
            self.torch = importlib.import_module("torch")
            config_module = importlib.import_module("clrnet.utils.config")
            registry_module = importlib.import_module("clrnet.models.registry")
            self.cfg = config_module.Config.fromfile(self.config_file)
            # The camera may not have the dataset's native resolution.  The
            # head operates on img_w/img_h, while decoded points use ori_*.
            self.cfg.ori_img_w = int(rospy.get_param("~source_width", 640))
            self.cfg.ori_img_h = int(rospy.get_param("~source_height", 480))
            self.cfg.cut_height = int(self.cfg.ori_img_h * self.cut_height_ratio)
            # The CLRNet checkpoint already contains backbone weights.  Do
            # not let the reference config download a second ImageNet model
            # while constructing the network.
            if hasattr(self.cfg, "backbone") and hasattr(self.cfg.backbone, "pretrained"):
                self.cfg.backbone.pretrained = False
            requested = self.device_name
            if requested.startswith("cuda") and not self.torch.cuda.is_available():
                rospy.logwarn("CUDA requested but unavailable; using CPU")
                requested = "cpu"
            self.device = self.torch.device(requested)
            self.model = registry_module.build_net(self.cfg)
            checkpoint = self.torch.load(self.checkpoint, map_location="cpu")
            state = checkpoint.get("net", checkpoint)
            state = {key.replace("module.", "", 1) if key.startswith("module.") else key: value
                     for key, value in state.items()}
            missing, unexpected = self.model.load_state_dict(state, strict=False)
            if missing:
                rospy.logwarn("CLRNet checkpoint missing %d model keys", len(missing))
            if unexpected:
                rospy.logwarn("CLRNet checkpoint has %d unexpected keys", len(unexpected))
            self.model.to(self.device)
            self.model.eval()
            self.model_ready = True
            rospy.loginfo("CLRNet ready: config=%s checkpoint=%s device=%s",
                          self.config_file, self.checkpoint, self.device)
        except Exception as exc:  # Keep the HSV node alive on optional dependency failures.
            self.model = None
            self.model_ready = False
            rospy.logerr("CLRNet disabled during model load: %s", exc)

    def image_cb(self, msg):
        now = rospy.get_time()
        if self.process_rate > 0.0 and now - self.last_process_time < 1.0 / self.process_rate:
            return
        self.last_process_time = now
        height = msg.height
        width = msg.width
        mask = np.zeros((height, width), dtype=np.uint8)
        if self.model_ready:
            try:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                mask = self.infer_mask(bgr)
            except (CvBridgeError, RuntimeError, ValueError) as exc:
                rospy.logwarn_throttle(5.0, "CLRNet inference failed; publishing empty mask: %s", exc)
        try:
            output = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            output.header = msg.header
            self.mask_pub.publish(output)
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Failed to publish CLRNet mask: %s", exc)

    def infer_mask(self, bgr):
        height, width = bgr.shape[:2]
        # Decode normalized lane points back into the actual camera frame.
        # This also handles a camera stream whose resolution differs from the
        # launch defaults.
        self.cfg.ori_img_w = width
        self.cfg.ori_img_h = height
        cut_height = int(np.clip(self.cut_height_ratio, 0.0, 0.9) * height)
        self.cfg.cut_height = cut_height
        img_h = int(self.cfg.img_h)
        img_w = int(self.cfg.img_w)
        cropped = bgr[cut_height:, :] if cut_height < height else bgr
        resized = cv2.resize(cropped, (img_w, img_h), interpolation=cv2.INTER_LINEAR)
        tensor = self.torch.from_numpy(resized.astype(np.float32) / 255.0)
        tensor = tensor.permute(2, 0, 1).unsqueeze(0).to(self.device)
        with self.torch.no_grad():
            predictions = self.model({"img": tensor})
            lanes = self.model.heads.get_lanes(predictions)[0]

        mask = np.zeros((height, width), dtype=np.uint8)
        for lane in lanes:
            confidence = float(lane.metadata.get("conf", 0.0))
            if confidence < self.conf_threshold:
                continue
            points = np.asarray(lane.points, dtype=np.float32)
            if points.ndim != 2 or points.shape[0] < 2:
                continue
            # Lane.points stores normalized x/y coordinates after decoding.
            xy = np.column_stack((points[:, 0] * width, points[:, 1] * height))
            xy = np.round(xy).astype(np.int32)
            xy[:, 0] = np.clip(xy[:, 0], 0, width - 1)
            xy[:, 1] = np.clip(xy[:, 1], 0, height - 1)
            cv2.polylines(mask, [xy.reshape(-1, 1, 2)], False, 255, self.line_width)
        return mask


if __name__ == "__main__":
    rospy.init_node("clrnet_lane_node")
    CLRNetLaneNode()
    rospy.spin()
