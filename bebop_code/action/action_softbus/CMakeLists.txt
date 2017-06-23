cmake_minimum_required(VERSION 2.8.3)
project(action_softbus)

find_package(catkin REQUIRED
    COMPONENTS
        cmake_modules
        roscpp
        pluginlib
        actionlib
        dynamic_reconfigure
        message_generation
        std_msgs
        decide_softbus_msgs
        nav_core
        tf
        tf_bridge
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
        cfg/ActionSoftbus.cfg
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# action_softbus
add_library(action_softbus
  src/action_softbus.cpp
)
target_link_libraries(action_softbus
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(action_softbus action_softbus_gencfg)
add_dependencies(action_softbus ${decide_softbus_msgs_EXPORTED_TARGETS})

add_executable(action_softbus_node
  src/action_softbus_node.cpp
)
target_link_libraries(action_softbus_node action_softbus)
#set_target_properties(action_softbus_node PROPERTIES OUTPUT_NAME action_softbus_node)

## Declare a C++ library
#add_library(demo_action
#    src/demo_action.cpp
#)
#add_dependencies(demo_action action_softbus_gencfg)
#add_dependencies(demo_action ${decide_softbus_msgs_EXPORTED_TARGETS})

add_library(bebop_action
    src/bebop_action.cpp
)
add_dependencies(bebop_action action_softbus_gencfg)
add_dependencies(bebop_action ${decide_softbus_msgs_EXPORTED_TARGETS})

install(
    TARGETS
        action_softbus
        #action_softbus_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
