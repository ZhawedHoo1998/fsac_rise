cmake_minimum_required(VERSION 3.0.2)
project(rtk)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  yhs_can_msgs
  image_transport
  message_filters
  nav_msgs
  tf2_ros
  sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rtk
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  include
  include/rtk


)

add_executable(${PROJECT_NAME}_node src/can_odometry_node.cpp src/can_odometry_core.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
