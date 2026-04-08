#!/usr/bin/env python3
"""Outdoor GPS goal test tool.

Usage example:
python3 outdoor_gps_goal_test.py --lat 23.0399 --lon 113.3901
"""

import argparse
import time

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus


class OutdoorGPSGoalTester:
    def __init__(self, input_topic: str, output_topic: str, frame_id: str):
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.frame_id = frame_id
        self.last_goal = None

        self.goal_sub = rospy.Subscriber(self.output_topic, PoseStamped, self._goal_cb, queue_size=1)
        self.goal_pub = rospy.Publisher(self.input_topic, NavSatFix, queue_size=1)

    def _goal_cb(self, msg: PoseStamped):
        self.last_goal = msg

    def wait_for_input_subscriber(self, timeout_sec: float) -> bool:
        deadline = time.time() + timeout_sec
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.goal_pub.get_num_connections() > 0:
                return True
            rospy.sleep(0.1)
        return False

    def publish_goal_ll(self, lat: float, lon: float, status: int, repeat: int, rate_hz: float):
        msg = NavSatFix()
        msg.header.frame_id = self.frame_id
        msg.status.status = status
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        msg.position_covariance = [
            2.0,
            0.0,
            0.0,
            0.0,
            2.0,
            0.0,
            0.0,
            0.0,
            5.0,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        rate = rospy.Rate(rate_hz)
        for _ in range(repeat):
            msg.header.stamp = rospy.Time.now()
            self.goal_pub.publish(msg)
            rate.sleep()

    def wait_for_converted_goal(self, timeout_sec: float):
        deadline = time.time() + timeout_sec
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.last_goal is not None:
                return self.last_goal
            rospy.sleep(0.05)
        return None


def parse_args():
    parser = argparse.ArgumentParser(description="Send outdoor GPS goal and verify bridge output")
    parser.add_argument("--lat", type=float, required=True, help="Target latitude")
    parser.add_argument("--lon", type=float, required=True, help="Target longitude")
    parser.add_argument("--input-topic", default="/gps_goal_ll", help="Input NavSatFix topic")
    parser.add_argument("--output-topic", default="/move_base_simple/goal", help="Output PoseStamped topic")
    parser.add_argument("--frame-id", default="gps_link", help="frame_id for NavSatFix header")
    parser.add_argument("--status", type=int, default=0, help="NavSatFix status, 0 is valid")
    parser.add_argument("--repeat", type=int, default=8, help="How many messages to publish")
    parser.add_argument("--rate", type=float, default=4.0, help="Publish rate in Hz")
    parser.add_argument("--wait-input", type=float, default=5.0, help="Wait time for input subscriber")
    parser.add_argument("--wait-output", type=float, default=5.0, help="Wait time for converted output")
    parser.add_argument(
        "--no-output-check",
        action="store_true",
        help="Only publish input GPS goal and skip output verification",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    rospy.init_node("outdoor_gps_goal_test", anonymous=True)
    tester = OutdoorGPSGoalTester(args.input_topic, args.output_topic, args.frame_id)

    rospy.loginfo("[GPS-TEST] waiting for gps goal bridge subscriber...")
    if tester.wait_for_input_subscriber(args.wait_input):
        rospy.loginfo("[GPS-TEST] subscriber detected on %s", args.input_topic)
    else:
        rospy.logwarn(
            "[GPS-TEST] no subscriber detected on %s within %.1fs, still publishing",
            args.input_topic,
            args.wait_input,
        )

    rospy.loginfo("[GPS-TEST] publishing lat/lon: %.8f, %.8f", args.lat, args.lon)
    tester.publish_goal_ll(args.lat, args.lon, args.status, args.repeat, args.rate)

    if args.no_output_check:
        rospy.loginfo("[GPS-TEST] publish done (output check skipped)")
        return 0

    goal = tester.wait_for_converted_goal(args.wait_output)
    if goal is None:
        rospy.logerr(
            "[GPS-TEST] did not receive converted goal on %s within %.1fs",
            args.output_topic,
            args.wait_output,
        )
        return 2

    rospy.loginfo(
        "[GPS-TEST] success: map goal frame=%s, x=%.3f, y=%.3f",
        goal.header.frame_id,
        goal.pose.position.x,
        goal.pose.position.y,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
