cmake_minimum_required(VERSION 3.0.2)
project(robot_base_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  nav_msgs
  nmea_msgs
  sensor_msgs
  std_msgs
  diagnostic_msgs
  message_generation
  tf
)

add_message_files(FILES
  MotorStatus.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS diagnostic_msgs message_runtime std_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/base_driver_node.py
  src/control_mode_mux.py
  src/gps_fix_to_map_odom.py
  src/global_odom_bridge.py
  src/global_path_tracker.py
  src/lio_odom_guard.py
  src/map_odom_align_bridge.py
  src/system_run_logger.py
  src/system_resource_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
