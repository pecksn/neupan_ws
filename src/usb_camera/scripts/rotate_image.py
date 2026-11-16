#!/usr/bin/env python3
# Module contract: Rotate camera images by 180 degrees before downstream consumers.
# Calibration and inference pixel coordinates must use the same image orientation.

"""Rotate a ROS sensor_msgs/Image by 180 degrees."""

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


# Rotation changes the image coordinate convention used during calibration.
# Intrinsics and any pixel-space inference coordinates must describe that same
# image orientation; display rotation alone does not recalibrate the camera.
# Publish the 180-degree-rotated camera stream through the configured ROS image
# interface.
class RotateImage180:
    # Configure image conversion and input/output topics for the rotated camera stream.
    def __init__(self):
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher("image_out", Image, queue_size=1)
        self.subscriber = rospy.Subscriber(
            "image_in",
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=1920 * 1080 * 3 * 2,
        )

    # Rotate the incoming image pixels and publish the corresponding output image.
    # Inputs: message.
    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            rotated = cv2.rotate(image, cv2.ROTATE_180)
            output = self.bridge.cv2_to_imgmsg(rotated, encoding=message.encoding)
            output.header = message.header
            self.publisher.publish(output)
        except CvBridgeError as error:
            rospy.logerr_throttle(5.0, "Failed to rotate camera image: %s", error)


if __name__ == "__main__":
    rospy.init_node("camera_rotate_180")
    RotateImage180()
    rospy.loginfo("Publishing camera images rotated by 180 degrees")
    rospy.spin()
