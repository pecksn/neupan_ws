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
  src/global_odom_bridge.py
  src/global_path_tracker.py
  src/lio_odom_guard.py
  src/map_odom_align_bridge.py
  src/system_run_logger.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
