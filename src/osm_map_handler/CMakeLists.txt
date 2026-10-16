cmake_minimum_required(VERSION 3.0.2)
project(osm_map_handler)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  nav_msgs 
  sensor_msgs 
  std_msgs
  geometry_msgs 
  visualization_msgs
  tf
  hybrid_astar
)

find_package(ompl REQUIRED)
add_compile_options(-std=c++14)

catkin_python_setup()

# Expose flat, importable modules in devel as well as the setuptools install.
# Noetic's executable relay runs in a private dictionary, so imported symbols
# must be exported using regular modules. configure_file tracks source edits.
foreach(osm_module osm_camera_path_guard osm_camera_path_refinement osm_outdoor_start_connection osm_hybrid_search)
  configure_file(src/${osm_module}.py
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${osm_module}.py COPYONLY)
endforeach()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${ompl_INCLUDE_DIRS}
)

add_executable(global_astar_node src/global_astar_node.cpp)
target_link_libraries(global_astar_node ${catkin_LIBRARIES})

add_executable(outdoor_hybrid_worker src/outdoor_hybrid_worker.cpp)
target_link_libraries(outdoor_hybrid_worker ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${ompl_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/camera_right_path.py
  scripts/gps_goal_bridge.py
  scripts/osm_alignment_analyzer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS global_astar_node outdoor_hybrid_worker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
