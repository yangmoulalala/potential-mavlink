cmake_minimum_required(VERSION 3.8)
project(serial_mav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include  # 添加项目的include目录
  ${PROJECT_SOURCE_DIR}/include  # 另一种路径写法，适用于嵌套目录
)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
# find dependencies
find_package(ament_cmake_auto REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(rm_interfaces REQUIRED)
ament_auto_find_build_dependencies()
#ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY src)
ament_auto_add_executable(serival_rm src/serival_rm.cpp)
#ament_target_dependencies(serival_rm rclcpp std_msgs serial rm_interfaces)

target_link_libraries(serival_rm ${serial_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#install
ament_auto_package()
# install(TARGETS
#   mav2_test
#   DESTINATION lib/${PROJECT_NAME})
