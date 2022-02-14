cmake_minimum_required(VERSION 2.8.3)
project(neo_relayboard)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
  neo_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES neo_relayboard
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(SerialIO common/src/SerialIO.cpp)
add_library(${PROJECT_NAME} common/src/SerRelayBoard.cpp common/src/StrUtil.cpp common/src/neo_relayboard_lib.cpp)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
add_dependencies(SerialIO ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME} SerialIO)

add_executable(neo_relayboard_node src/neo_relayboard_node.cpp)
target_link_libraries(neo_relayboard_node ${PROJECT_NAME})
target_link_libraries(neo_relayboard_node ${catkin_LIBRARIES})
add_dependencies(neo_relayboard_node ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############



#############
## Testing ##
#############


