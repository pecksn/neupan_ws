#!/usr/bin/env python3
import math

import rospy
import tf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelProperties, SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class AckermannSimDriver:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "lxwcar")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/neupan_cmd_vel")
        self.wheelbase = float(rospy.get_param("~wheelbase", 0.67))
        self.wheel_radius = float(rospy.get_param("~wheel_radius", 0.1275))
        self.max_speed = float(rospy.get_param("~max_speed", 1.0))
        self.max_steer = float(rospy.get_param("~max_steer", 0.35))
        self.update_rate = float(rospy.get_param("~update_rate", 50.0))
        self.model_z = float(rospy.get_param("~model_z", 0.05))

        self.x = float(rospy.get_param("~x", 0.0))
        self.y = float(rospy.get_param("~y", 0.0))
        self.yaw = float(rospy.get_param("~yaw", 0.0))
        self.v = 0.0
        self.steer = 0.0
        self.wheel_pos = 0.0
        self.last_time = rospy.Time.now()

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_model_properties = rospy.ServiceProxy(
            "/gazebo/get_model_properties", GetModelProperties
        )
        self.gazebo_ready = False
        self.model_ready = False

        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback, queue_size=1)

    def cmd_callback(self, msg):
        self.v = clamp(msg.linear.x, -self.max_speed, self.max_speed)
        self.steer = clamp(msg.angular.z, -self.max_steer, self.max_steer)

    def step(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0.0:
            return
        self.last_time = now

        yaw_rate = 0.0
        if abs(self.wheelbase) > 1e-6:
            yaw_rate = self.v * math.tan(self.steer) / self.wheelbase

        self.yaw += yaw_rate * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.wheel_pos += (self.v / max(self.wheel_radius, 1e-6)) * dt

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        self.publish_odom(now, quat, yaw_rate)
        self.publish_joints(now)
        self.publish_model_state(quat)

    def publish_odom(self, stamp, quat, yaw_rate):
        self.tf_br.sendTransform(
            (self.x, self.y, 0.0),
            quat,
            stamp,
            self.base_frame,
            self.odom_frame,
        )

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = yaw_rate
        self.odom_pub.publish(odom)

    def publish_joints(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = [
            "front_left_steer_joint",
            "front_right_steer_joint",
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "back_left_joint",
            "back_right_joint",
        ]
        msg.position = [
            self.steer,
            self.steer,
            self.wheel_pos,
            self.wheel_pos,
            self.wheel_pos,
            self.wheel_pos,
        ]
        self.joint_pub.publish(msg)

    def publish_model_state(self, quat):
        try:
            if not self.gazebo_ready:
                rospy.wait_for_service("/gazebo/set_model_state", timeout=0.1)
                self.gazebo_ready = True
        except rospy.ROSException:
            return

        if not self.model_ready:
            try:
                rospy.wait_for_service("/gazebo/get_model_properties", timeout=0.1)
                props = self.get_model_properties(self.model_name)
                if props.success:
                    self.model_ready = True
            except (rospy.ROSException, rospy.ServiceException):
                return
            if not self.model_ready:
                return

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.model_z
        state.pose.orientation.x = quat[0]
        state.pose.orientation.y = quat[1]
        state.pose.orientation.z = quat[2]
        state.pose.orientation.w = quat[3]
        state.twist.linear.x = self.v
        try:
            self.set_model_state(state)
        except rospy.ServiceException:
            self.gazebo_ready = False
            self.model_ready = False

    def run(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ackermann_sim_driver")
    AckermannSimDriver().run()
