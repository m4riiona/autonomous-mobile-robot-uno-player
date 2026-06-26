cmake_minimum_required(VERSION 3.0.2)
project(uno_orchestrator_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs
  actionlib
  actionlib_msgs
  turtlebot3_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/uno_orchestrator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
