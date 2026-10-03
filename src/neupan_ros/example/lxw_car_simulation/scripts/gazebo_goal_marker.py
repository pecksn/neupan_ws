#!/usr/bin/env python3
"""Show the navigation goal in Gazebo using a visual-only model."""

import threading

import rospy
import tf
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetWorldProperties, SetModelState, SpawnModel
from geometry_msgs.msg import PoseStamped


GOAL_SDF = """<sdf version="1.6"><model name="neupan_goal_marker">
  <static>true</static><link name="marker"><gravity>false</gravity>
    <visual name="target"><pose>0 0 0.025 0 0 0</pose>
      <geometry><cylinder><radius>0.35</radius><length>0.035</length></cylinder></geometry>
      <material><ambient>0 0.6 0.9 1</ambient><diffuse>0 0.8 1 1</diffuse></material>
      <cast_shadows>false</cast_shadows>
    </visual>
    <visual name="pole"><pose>0 0 0.35 0 0 0</pose>
      <geometry><cylinder><radius>0.025</radius><length>0.65</length></cylinder></geometry>
      <material><ambient>1 0.3 0 1</ambient><diffuse>1 0.3 0 1</diffuse></material>
    </visual>
    <visual name="flag"><pose>0 0 0.7 0 0 0</pose>
      <geometry><sphere><radius>0.14</radius></sphere></geometry>
      <material><ambient>1 0.1 0 1</ambient><diffuse>1 0.1 0 1</diffuse></material>
    </visual>
  </link>
</model></sdf>"""


class GoalMarker:
    def __init__(self):
        self.model_name = "neupan_goal_marker"
        # The simulation driver publishes odometry in Gazebo world coordinates.
        self.frame = rospy.get_param("~goal_frame", "odom")
        self.tf = tf.TransformListener()
        self.lock = threading.Lock()
        self.pending = None
        self.spawned = False
        for service in ("/gazebo/spawn_sdf_model", "/gazebo/set_model_state",
                        "/gazebo/get_world_properties"):
            rospy.wait_for_service(service)
        self.spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        self.move = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.get_world = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        self.subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped,
                                           self.on_goal, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.25), self.update)

    def on_goal(self, goal):
        with self.lock:
            self.pending = goal

    def update(self, _event):
        with self.lock:
            goal = self.pending
        if goal is None:
            return
        pending_goal = goal
        try:
            if goal.header.frame_id.lstrip("/") not in (self.frame, "world"):
                goal = self.tf.transformPose(self.frame, goal)
            state = ModelState(model_name=self.model_name, reference_frame="world")
            state.pose = goal.pose
            state.pose.position.z = 0.0
            if not self.spawned:
                self.spawned = self.model_name in self.get_world().model_names
            if self.spawned:
                response = self.move(state)
                self.spawned = response.success
            else:
                response = self.spawn(self.model_name, GOAL_SDF, "", state.pose, "world")
                self.spawned = response.success
            if not response.success:
                rospy.logwarn_throttle(5, "Goal marker: %s", response.status_message)
                return
            with self.lock:
                if self.pending is pending_goal:
                    self.pending = None
            rospy.loginfo("Gazebo goal marker: x=%.2f y=%.2f",
                          state.pose.position.x, state.pose.position.y)
        except (rospy.ServiceException, tf.Exception) as error:
            rospy.logwarn_throttle(5, "Cannot show Gazebo goal: %s", error)


if __name__ == "__main__":
    rospy.init_node("gazebo_goal_marker")
    marker = GoalMarker()
    rospy.spin()
