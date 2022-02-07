cmake_minimum_required(VERSION 2.8.3)
project(neo_platformctrl_diff)

find_package(catkin REQUIRED COMPONENTS
# nav_msgs
 roscpp
 rostest
 tf 
 std_msgs
 sensor_msgs
 trajectory_msgs
 )

#######################################
## Declare ROS messages and services ##
#######################################

#add_message_files(
#  DIRECTORY msg
#  FILES EmergencyStopState.msg
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES std_msgs
#)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS roscpp rostest std_msgs tf sensor_msgs #nav_msgs
    INCLUDE_DIRS common/include
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} common/src/DiffDrive2WKinematics.cpp )

add_executable(neo_platformctrl_diff_node ros/src/neo_platformctrl_node.cpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
add_dependencies(neo_platformctrl_diff_node ${PROJECT_NAME}_gencpp)

target_link_libraries(neo_platformctrl_diff_node ${PROJECT_NAME})
target_link_libraries(neo_platformctrl_diff_node ${catkin_LIBRARIES})


#target_link_libraries(neo_platformctrl_diff_node thread)


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} neo_platformctrl_diff_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
