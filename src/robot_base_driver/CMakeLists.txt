cmake_minimum_required(VERSION 3.0.2)
project(robot_base_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  nmea_msgs
  sensor_msgs
  std_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/base_driver_node.py
  src/costmap_monitor.py
  src/global_odom_bridge.py
  src/global_path_tracker.py
  src/g90_health_monitor.py
  src/map_odom_align_bridge.py
  src/odom_yaw_monitor.py
  src/quaternion_yaw_monitor.py
  src/yaw_compare_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
