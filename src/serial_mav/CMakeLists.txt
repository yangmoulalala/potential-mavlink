cmake_minimum_required(VERSION 3.8)
project(serial_mav)

# # C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)


# 包含头文件
include_directories(
  src
  include
)

# 编译源文件
add_executable(serial_mav_node
  src/main.cpp
  src/mavlink.cpp
)

ament_target_dependencies(serial_mav_node
  rclcpp
  tf2_ros
  std_msgs
  serial
  sensor_msgs
  geometry_msgs
  rm_interfaces
  nav2_msgs
)

# 安装可执行程序
install(TARGETS serial_mav_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 include 目录（mavlink 头文件）
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()