cmake_minimum_required(VERSION 2.8.3)
project(flight_barrier CXX)

############
## Catkin ##
############

set(DEPS
  message_filters
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  gps_common
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
  regex
  serialization
)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  ${DEPS}
)

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  FILES
#    GPSStatus.msg
#    GPSFix.msg
#)

#generate_messages(DEPENDENCIES
#  nav_msgs
#  sensor_msgs
#  std_msgs
#)

###################################################
## Declare things to be passed to other projects ##
###################################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
    message_runtime
    ${DEPS}
)

###########
## Build ##
###########

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_executable(${PROJECT_NAME}/flight_barrier_node src/flight_barrier_node.cpp)
set_target_properties(${PROJECT_NAME}/flight_barrier_node PROPERTIES OUTPUT_NAME "flight_barrier_node")
target_link_libraries(${PROJECT_NAME}/flight_barrier_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}/flight_barrier_node
  #${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}/flight_barrier_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ 
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
