cmake_minimum_required(VERSION 3.0.2)
project(robot_base_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/base_driver_node.py
  src/global_odom_bridge.py
  src/map_odom_align_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
