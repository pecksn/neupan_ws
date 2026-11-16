#!/usr/bin/env python3
# Module contract: Guard experimental TEB output before it reaches the shared command
# mux. Scan freshness and command freshness remain independent requirements.

"""Preserve scan/command watchdogs between experimental TEB and the existing mux."""
import math
import time


# Check both monotonic arrival age and ROS source age; a recently received scan may
# still contain stale measurements.
# Inputs: received, stamp, mono, utc.
def fresh(received, stamp, mono, utc):
    return (
        received is not None
        and 0 <= mono - received <= 0.5
        and -0.05 <= utc - stamp <= 0.5
    )


# Forward bounded, finite Ackermann TEB commands only while scan and command evidence
# remain fresh.
class TebCommandGate:
    # Create the experimental command/scan watchdog state and ROS I/O. Commands remain
    # unforwarded until their evidence is valid.
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import LaserScan

        self.rospy = rospy
        self.Twist = Twist
        self.command = None
        self.scan = None
        self.scan_valid = False
        self.output = rospy.Publisher('/cmd_vel/auto', Twist, queue_size=2)
        rospy.Subscriber(
            '/experiments/teb_cmd_raw', Twist, self.command_cb, queue_size=1
        )
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)
        rospy.on_shutdown(lambda: self.output.publish(Twist()))

    # Validate the TEB command's finite speed and steering-angle bounds before retaining
    # it with a monotonic receive time.
    # Inputs: message.
    def command_cb(self, message):
        # cmd_angle_instead_rotvel=true: angular.z is front-wheel radians,
        # matching base_driver_node, not differential-drive yaw rate.
        if not all(math.isfinite(v) for v in (message.linear.x, message.angular.z)):
            self.command = None
            return
        if (
            not -0.45 <= message.linear.x <= 0.800001
            or abs(message.angular.z) > 0.350001
        ):
            self.command = None
            return
        self.command = (time.monotonic(), message)

    # Record scan timing and whether enough finite in-range rays exist for the
    # experimental watchdog.
    # Inputs: message.
    def scan_cb(self, message):
        self.scan_valid = (
            sum(
                math.isfinite(v) and message.range_min <= v <= message.range_max
                for v in message.ranges
            )
            >= 3
        )
        self.scan = (time.monotonic(), message.header.stamp.to_sec())

    # Publish the retained TEB command only when its command and scan evidence pass the
    # watchdog; otherwise publish zero.
    # Inputs: _event.
    def tick(self, _event):
        now = time.monotonic()
        utc = self.rospy.Time.now().to_sec()
        valid = (
            self.command is not None
            and 0 <= now - self.command[0] <= 0.5
            and self.scan_valid
            and self.scan is not None
            and fresh(*self.scan, now, utc)
        )
        self.output.publish(self.command[1] if valid else self.Twist())


if __name__ == '__main__':
    import rospy

    rospy.init_node('teb_experiment_command_gate')
    TebCommandGate()
    rospy.spin()
