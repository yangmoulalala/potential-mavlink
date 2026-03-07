cmake_minimum_required(VERSION 3.8)
project(rm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 告诉 ROS 2 根据你的 .msg 文件生成 C++ 和 Python 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  "msg/Cboard.msg"
  "msg/Referee.msg"
  DEPENDENCIES std_msgs
)

ament_package()
