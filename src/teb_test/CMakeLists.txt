cmake_minimum_required(VERSION 3.0.2)
project(teb_test)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES osm_path_planner
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(osm_path_planner
  src/osm_path_planner.cpp
)

target_link_libraries(osm_path_planner
  ${catkin_LIBRARIES}
)
