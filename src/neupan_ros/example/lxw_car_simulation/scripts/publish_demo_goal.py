#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("lxw_car_simulation_demo_goal")
    frame_id = rospy.get_param("~frame_id", "odom")
    x = float(rospy.get_param("~x", 6.0))
    y = float(rospy.get_param("~y", 0.0))
    yaw = float(rospy.get_param("~yaw", 0.0))
    delay = float(rospy.get_param("~delay", 2.0))

    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
    rospy.sleep(delay)

    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = frame_id
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = 0.0
    q = quaternion_from_euler(0.0, 0.0, yaw)
    goal.pose.orientation.x = q[0]
    goal.pose.orientation.y = q[1]
    goal.pose.orientation.z = q[2]
    goal.pose.orientation.w = q[3]

    pub.publish(goal)
    rospy.loginfo("Published demo goal: x=%.2f y=%.2f yaw=%.1fdeg", x, y, math.degrees(yaw))
    rospy.spin()


if __name__ == "__main__":
    main()
