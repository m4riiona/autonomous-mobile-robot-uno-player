cmake_minimum_required(VERSION 3.0.2)
project(uno_wall_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  std_srvs
  move_base_msgs
  actionlib
  actionlib_msgs
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/wall_follower_node.py
  scripts/mission_controller.py
  scripts/waypoint_setup_helper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch maps config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
