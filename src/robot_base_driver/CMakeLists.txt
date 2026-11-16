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

catkin_python_setup()

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
  scripts/base_driver_node.py
  scripts/control_mode_mux.py
  scripts/gps_fix_to_map_odom.py
  scripts/global_odom_bridge.py
  scripts/global_path_tracker.py
  scripts/lio_odom_guard.py
  scripts/map_odom_align_bridge.py
  scripts/system_run_logger.py
  scripts/system_resource_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Imported helper: use a normal module, not a catkin executable relay (whose
# exec context does not export classes when imported by another relay).
configure_file(src/gnss_timebase.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/gnss_timebase.py COPYONLY)
