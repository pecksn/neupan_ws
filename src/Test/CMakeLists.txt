cmake_minimum_required(VERSION 3.0.2)
project(test_motion_tools)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rospy
  std_msgs
)

add_message_files(
  FILES
  MotionPacket.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
  scripts/open_loop_data_logger.py
  scripts/open_loop_motion.py
  scripts/daily_rosbag_record.py
  scripts/mock_neupan_commands.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
