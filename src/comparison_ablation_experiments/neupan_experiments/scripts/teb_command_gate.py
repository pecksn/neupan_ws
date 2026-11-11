#!/usr/bin/env python3
"""Preserve scan/command watchdogs between experimental TEB and the existing mux."""
import math
import time


def fresh(received,stamp,mono,utc):
    return received is not None and 0<=mono-received<=.5 and -.05<=utc-stamp<=.5


class TebCommandGate:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import LaserScan
        self.rospy=rospy;self.Twist=Twist;self.command=None;self.scan=None;self.scan_valid=False
        self.output=rospy.Publisher('/cmd_vel/auto',Twist,queue_size=2)
        rospy.Subscriber('/experiments/teb_cmd_raw',Twist,self.command_cb,queue_size=1)
        rospy.Subscriber('/scan',LaserScan,self.scan_cb,queue_size=1)
        rospy.Timer(rospy.Duration(.05),self.tick)
        rospy.on_shutdown(lambda:self.output.publish(Twist()))

    def command_cb(self,message):
        # cmd_angle_instead_rotvel=true: angular.z is front-wheel radians,
        # matching base_driver_node, not differential-drive yaw rate.
        if not all(math.isfinite(v) for v in (message.linear.x,message.angular.z)):
            self.command=None;return
        if not -.45<=message.linear.x<=.800001 or abs(message.angular.z)>.350001:
            self.command=None;return
        self.command=(time.monotonic(),message)

    def scan_cb(self,message):
        self.scan_valid=sum(math.isfinite(v) and message.range_min<=v<=message.range_max for v in message.ranges)>=3
        self.scan=(time.monotonic(),message.header.stamp.to_sec())

    def tick(self,_event):
        now=time.monotonic();utc=self.rospy.Time.now().to_sec()
        valid=(self.command is not None and 0<=now-self.command[0]<=.5
               and self.scan_valid and self.scan is not None and fresh(*self.scan,now,utc))
        self.output.publish(self.command[1] if valid else self.Twist())


if __name__=='__main__':
    import rospy
    rospy.init_node('teb_experiment_command_gate');TebCommandGate();rospy.spin()
