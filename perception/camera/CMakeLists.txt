cmake_minimum_required(VERSION 3.0)


cmake_minimum_required(VERSION 2.8.3)
project(fsac_can_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# 设置工程名和语言
project(camera CXX)

# 查找海康威视SDK的路径
set(HIKVISION_SDK_ROOT /opt/MVS)

# 设置编译选项
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  rospy
  std_msgs
)


# 添加海康威视SDK的头文件路径
include_directories(
    include
    ${HIKVISION_SDK_ROOT}/include
    ${catkin_INCLUDE_DIRS}
)

# 添加海康威视SDK的库文件路径
link_directories(
    include
    ${HIKVISION_SDK_ROOT}/lib/64
)

# 添加需要编译的源文件
add_executable(HIKROBOT_camera
    src/HIKROBOT_camera/src/hikrobot_camera.cpp
)

# 链接海康威视SDK的库文件
target_link_libraries(HIKROBOT_camera
    ${OpenCV_LIBS}
)

