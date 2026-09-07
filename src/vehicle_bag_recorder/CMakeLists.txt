cmake_minimum_required(VERSION 3.0.2)
project(vehicle_bag_recorder)

find_package(catkin REQUIRED COMPONENTS rospy rosbag)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/bag_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
