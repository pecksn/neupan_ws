cmake_minimum_required(VERSION 3.0.2)
project(osm_map_handler)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  nav_msgs 
  sensor_msgs 
  geometry_msgs 
  visualization_msgs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(global_astar_node src/global_astar_node.cpp)
target_link_libraries(global_astar_node ${catkin_LIBRARIES})
