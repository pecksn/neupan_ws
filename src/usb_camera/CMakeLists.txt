cmake_minimum_required(VERSION 3.0.2)
project(usb_camera)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/rotate_image.py
  scripts/calibrate_lidar_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
