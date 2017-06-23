cmake_minimum_required(VERSION 2.8.3)
project(decide_softbus_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs actionlib_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  NavigationPoint.msg
  Path.msg
  SwarmConfig.msg)

add_service_files(
  DIRECTORY srv
  FILES
  GetPlan.srv
  MakeNavPlan.srv
  SetSwarmConfig.srv
  SetControlling.srv)

add_action_files(
  DIRECTORY
  action
  FILES
  MoveBase.action
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs actionlib_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs actionlib_msgs)
