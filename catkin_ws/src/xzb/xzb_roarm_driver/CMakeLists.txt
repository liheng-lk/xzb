cmake_minimum_required(VERSION 3.0.2)
project(xzb_roarm_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  xzb_interfaces
)

catkin_python_setup()
catkin_package(CATKIN_DEPENDS geometry_msgs rospy sensor_msgs std_msgs std_srvs xzb_interfaces)

catkin_install_python(PROGRAMS scripts/roarm_driver_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
