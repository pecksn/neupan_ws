cmake_minimum_required(VERSION 3.0.2)
project(road_vision_refinement)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/save_one_image.py
  scripts/pick_ipm_points.py
  scripts/verify_ipm_points.py
  scripts/path_refine_node.py
  scripts/road_vision_node.py
  scripts/clrnet_lane_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
