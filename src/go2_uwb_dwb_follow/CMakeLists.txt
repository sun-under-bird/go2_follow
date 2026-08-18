cmake_minimum_required(VERSION 3.8)
project(go2_uwb_dwb_follow)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uwb_aoa_pkg REQUIRED)

set(dependencies
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  uwb_aoa_pkg
)

add_executable(uwb_point_follow_node src/uwb_point_follow_node.cpp)
ament_target_dependencies(uwb_point_follow_node ${dependencies})

# Humble 下 rosidl 包改为基于 target 导出，include 目录不再经 ament_target_dependencies 传递。
# On Humble rosidl packages export via targets, so their include dirs need explicit linking.
target_link_libraries(uwb_point_follow_node ${nav2_msgs_TARGETS})

install(
  TARGETS
    uwb_point_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
