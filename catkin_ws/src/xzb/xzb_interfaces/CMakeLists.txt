cmake_minimum_required(VERSION 3.0.2)
project(xzb_interfaces)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  ArmAction.msg
  ArmStatus.msg
)

add_service_files(
  FILES
  DriverJointCommand.srv
  DriverSingleJointCommand.srv
  MoveArmJoints.srv
  MoveArmCartesian.srv
  SetArmGripper.srv
  ComputeArmFK.srv
  SolveArmIK.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime nav_msgs sensor_msgs std_msgs
)
