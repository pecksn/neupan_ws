#!/usr/bin/env python3
"""Save the next ROS image message to one file, then exit."""

import argparse
import os

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


def parse_args():
    parser = argparse.ArgumentParser(
        description="Save exactly one image from a sensor_msgs/Image topic."
    )
    parser.add_argument("topic", help="ROS image topic")
    parser.add_argument("output", help="output image path, such as image.png")
    parser.add_argument(
        "--timeout",
        type=float,
        default=10.0,
        help="seconds to wait for an image (default: 10)",
    )
    return parser.parse_args(rospy.myargv()[1:])


def main():
    args = parse_args()
    rospy.init_node("save_one_image", anonymous=True, disable_signals=True)

    output = os.path.abspath(os.path.expanduser(args.output))
    output_dir = os.path.dirname(output)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    try:
        message = rospy.wait_for_message(args.topic, Image, timeout=args.timeout)
        image = CvBridge().imgmsg_to_cv2(message, desired_encoding="bgr8")
    except rospy.ROSException as exc:
        raise SystemExit("Timed out waiting for {}: {}".format(args.topic, exc))
    except CvBridgeError as exc:
        raise SystemExit("Failed to convert image: {}".format(exc))

    if not cv2.imwrite(output, image):
        raise SystemExit("Failed to write image: {}".format(output))

    print("Saved one image: {}".format(output))


if __name__ == "__main__":
    main()
