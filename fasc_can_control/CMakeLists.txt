cmake_minimum_required(VERSION 2.8.3)
project(fsac_can_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  # yhs_can_msgs
  sensor_msgs
  tf2
  tf2_ros
  autoware_msgs
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_node
  CATKIN_DEPENDS roscpp rospy  std_msgs message_runtime 
#  DEPENDS system_lib
)

###########
## Build ##
###########

link_directories(
  include/yhs_can_control
  ${catkin_INCLUDE_DIRS} yhs_can_control
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include include/yhs_can_control ${catkin_INCLUDE_DIRS} yhs_can_control)

add_executable(${PROJECT_NAME}_node src/yhs_can_control.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
