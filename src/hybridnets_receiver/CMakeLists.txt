cmake_minimum_required(VERSION 3.0.2)
project(hybridnets_receiver)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_python_setup()
catkin_package()

# Importable module, not an executable catkin relay.
configure_file(src/hybridnets_binding.py
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/hybridnets_binding.py COPYONLY)

catkin_install_python(PROGRAMS
  scripts/hybridnets_json_receiver.py
  scripts/hybridnets_overlay.py
  scripts/debug_mp4_recorder.py
  scripts/rtp_camera_receiver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
