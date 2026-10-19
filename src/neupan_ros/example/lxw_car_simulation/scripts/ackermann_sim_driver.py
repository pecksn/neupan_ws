#!/usr/bin/env python3
import math
import threading

import rospy
import tf
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def integrate_ackermann(x, y, yaw, speed, steer, wheelbase, dt):
    """Exact constant-input bicycle arc; x/y are the rear axle."""
    rate = speed * math.tan(steer) / wheelbase
    next_yaw = yaw + rate * dt
    if abs(rate) < 1e-9:
        return x + speed * math.cos(yaw) * dt, y + speed * math.sin(yaw) * dt, next_yaw
    return (x + speed / rate * (math.sin(next_yaw) - math.sin(yaw)),
            y - speed / rate * (math.cos(next_yaw) - math.cos(yaw)), next_yaw)


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
        self.cmd_timeout = max(0.0, float(rospy.get_param("~cmd_timeout", 0.5)))
        self.model_z = float(rospy.get_param("~model_z", 0.0))
        if self.wheelbase <= 0 or self.update_rate <= 0 or self.wheel_radius <= 0:
            raise ValueError("wheelbase, wheel radius and update rate must be positive")
        self.command_lock = threading.Lock()

        self.x = float(rospy.get_param("~x", 0.0))
        self.y = float(rospy.get_param("~y", 0.0))
        self.yaw = float(rospy.get_param("~yaw", 0.0))
        self.initial_pose = (self.x, self.y, self.yaw)
        self.v = 0.0
        self.steer = 0.0
        self.wheel_pos = 0.0
        self.last_time = rospy.Time.now()
        self.last_cmd_time = None

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.tf_br = tf.TransformBroadcaster()
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.gazebo_ready = False
        self.model_ready = False

        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback, queue_size=1)

    def model_states_callback(self, msg):
        self.model_ready = self.model_name in msg.name

    def cmd_callback(self, msg):
        valid = math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)
        with self.command_lock:
            self.v = clamp(msg.linear.x, -self.max_speed, self.max_speed) if valid else 0.0
            self.steer = clamp(msg.angular.z, -self.max_steer, self.max_steer) if valid else 0.0
            self.last_cmd_time = rospy.Time.now()
        if not valid:
            rospy.logerr_throttle(1.0, "Rejected non-finite simulated drive command")

    def step(self):
        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        if dt <= 0.0:
            if dt < 0.0:
                # Gazebo reset rewinds /clock; discard commands from the old run.
                self.last_time = now
                self.last_cmd_time = None
                self.v = 0.0
                self.steer = 0.0
                self.x, self.y, self.yaw = self.initial_pose
                self.wheel_pos = 0.0
            return
        self.last_time = now
        # Do not integrate across startup/service stalls or before model spawn.
        dt = min(dt, 2.0 / self.update_rate)
        with self.command_lock:
            if not self.model_ready or self.last_cmd_time is None or (
                self.cmd_timeout > 0.0 and (now - self.last_cmd_time).to_sec() > self.cmd_timeout
            ):
                self.v = 0.0
                self.steer = 0.0
            speed, steer = self.v, self.steer

        yaw_rate = 0.0
        if abs(self.wheelbase) > 1e-6:
            yaw_rate = speed * math.tan(steer) / self.wheelbase

        self.x, self.y, self.yaw = integrate_ackermann(
            self.x, self.y, self.yaw, speed, steer, self.wheelbase, dt)
        self.wheel_pos += speed / self.wheel_radius * dt

        quat = tf.transformations.quaternion_from_euler(0.0, 0.0, self.yaw)
        self.publish_odom(now, quat, yaw_rate, speed)
        self.publish_joints(now, steer)
        self.publish_model_state(quat)

    def publish_odom(self, stamp, quat, yaw_rate, speed):
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
        odom.twist.twist.linear.x = speed
        odom.twist.twist.angular.z = yaw_rate
        self.odom_pub.publish(odom)

    def publish_joints(self, stamp, steer):
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
            steer,
            steer,
            self.wheel_pos,
            self.wheel_pos,
            self.wheel_pos,
            self.wheel_pos,
        ]
        self.joint_pub.publish(msg)

    def publish_model_state(self, quat):
        if not self.model_ready:
            return
        try:
            if not self.gazebo_ready:
                rospy.wait_for_service("/gazebo/set_model_state", timeout=0.1)
                self.gazebo_ready = True
        except rospy.ROSException:
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
        # The pose is integrated here; a Gazebo velocity would move it a second time.
        try:
            self.set_model_state(state)
        except rospy.ServiceException:
            self.gazebo_ready = False
            self.model_ready = False

    def run(self):
        rate = rospy.Rate(self.update_rate)
        while not rospy.is_shutdown():
            self.step()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                return


if __name__ == "__main__":
    rospy.init_node("ackermann_sim_driver")
    AckermannSimDriver().run()
