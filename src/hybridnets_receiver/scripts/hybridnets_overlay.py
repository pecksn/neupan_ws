#!/usr/bin/env python3
"""Draw the latest HybridNets pixel result on the received camera image."""

import json
import threading

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class HybridNetsOverlay:
    def __init__(self):
        self.lock = threading.Lock()
        self.result = None
        self.received_at = rospy.Time(0)
        self.max_age = float(rospy.get_param("~max_age", 0.8))
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
        except ValueError:
            return
        with self.lock:
            self.result = result
            self.received_at = rospy.Time.now()

    @staticmethod
    def points(value):
        if not value:
            return None
        return np.rint(np.asarray(value, dtype=np.float64)).astype(np.int32).reshape(-1, 1, 2)

    def image_callback(self, message):
        if message.encoding not in ("bgr8", "rgb8") or message.step != message.width * 3:
            rospy.logwarn_throttle(3.0, "Overlay expects packed bgr8/rgb8 images")
            return
        image = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.width, 3).copy()
        if message.encoding == "rgb8":
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        with self.lock:
            result = self.result
            age = (rospy.Time.now() - self.received_at).to_sec()
        if result is not None and age <= self.max_age:
            polygon = self.points(result.get("road_polygon_px"))
            if polygon is not None:
                mask = image.copy()
                cv2.fillPoly(mask, [polygon], (0, 180, 0))
                image = cv2.addWeighted(image, 0.75, mask, 0.25, 0.0)

            for key, color in (("left_boundary_px", (255, 0, 255)),
                               ("right_boundary_px", (0, 255, 255))):
                points = self.points(result.get(key))
                if points is not None:
                    cv2.polylines(image, [points], False, color, 4)
            text = "HybridNets valid=%s age=%.0fms" % (
                bool(result.get("valid", False)), age * 1000.0)
            color = (0, 255, 0) if result.get("valid", False) else (0, 0, 255)
        else:
            text, color = "HybridNets result timeout", (0, 0, 255)
        cv2.putText(image, text, (25, 45), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, color, 2, cv2.LINE_AA)
        output = Image(header=message.header, height=message.height, width=message.width,
                       encoding="bgr8", is_bigendian=False,
                       step=message.width * 3, data=image.tobytes())
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("hybridnets_overlay")
    HybridNetsOverlay()
    rospy.spin()
