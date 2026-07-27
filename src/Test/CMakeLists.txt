cmake_minimum_required(VERSION 3.0.2)
project(test_motion_tools)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/open_loop_motion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
