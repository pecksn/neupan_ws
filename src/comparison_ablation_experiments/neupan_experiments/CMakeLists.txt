cmake_minimum_required(VERSION 3.0.2)
project(neupan_experiments)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS scripts/run_experiment.py scripts/gps_always_projection.py scripts/teb_command_gate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
