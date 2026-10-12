cmake_minimum_required(VERSION 3.0.2)
project(xzb_roarm_control)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
  xzb_interfaces
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS actionlib control_msgs geometry_msgs rospy sensor_msgs std_msgs std_srvs trajectory_msgs xzb_interfaces)

catkin_install_python(PROGRAMS
  scripts/roarm_control_node.py
  scripts/arm_test_cli.py
  scripts/roarm_trajectory_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
