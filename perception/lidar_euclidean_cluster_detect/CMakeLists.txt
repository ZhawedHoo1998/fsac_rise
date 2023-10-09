cmake_minimum_required(VERSION 2.8.3)
project(lidar_euclidean_cluster_detect)

find_package(autoware_build_flags REQUIRED)
find_package(autoware_config_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  autoware_config_msgs
  geometry_msgs
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  jsk_rviz_plugins
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
)

find_package(OpenMP)
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    grid_map_ros
    grid_map_cv
    grid_map_msgs
    jsk_rviz_plugins
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${OpenCV_LIBRARY_DIRS})

#Euclidean Cluster
add_executable(lidar_euclidean_cluster_detect
  nodes/lidar_euclidean_cluster_detect/lidar_euclidean_cluster_detect.cpp
  nodes/lidar_euclidean_cluster_detect/cluster.cpp
)

find_package(Eigen3 QUIET)

AW_CHECK_CUDA()


target_link_libraries(lidar_euclidean_cluster_detect
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)


add_dependencies(lidar_euclidean_cluster_detect
  ${catkin_EXPORTED_TARGETS}
)

if(OPENMP_FOUND)
  set_target_properties(lidar_euclidean_cluster_detect PROPERTIES
    COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
    LINK_FLAGS ${OpenMP_CXX_FLAGS}
  )
endif()

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS
  lidar_euclidean_cluster_detect
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
