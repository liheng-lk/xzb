cmake_minimum_required(VERSION 2.8.3)
project(linear_actuator_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

catkin_install_python(PROGRAMS
  scripts/linear_actuator_node.py
  scripts/actuator_keyboard.py
  scripts/actuator_servo_console.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Reference firmware is copied as documentation, never built by catkin.
install(DIRECTORY firmware_reference/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/firmware_reference
)

install(FILES README_CN.md CHANGELOG_REPLACEMENT.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
