cmake_minimum_required(VERSION 3.8)
project(go2_uwb_mppi_follow)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uwb_aoa_pkg REQUIRED)

set(dependencies
  builtin_interfaces
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  uwb_aoa_pkg
)

include_directories(include)

add_executable(uwb_path_tracker_node src/uwb_path_tracker_node.cpp)
ament_target_dependencies(uwb_path_tracker_node ${dependencies})

add_executable(uwb_follow_path_node src/uwb_follow_path_node.cpp)
ament_target_dependencies(uwb_follow_path_node ${dependencies})

add_executable(target_obstacle_filter_node src/target_obstacle_filter_node.cpp)
ament_target_dependencies(target_obstacle_filter_node ${dependencies})

install(
  TARGETS
    uwb_follow_path_node
    uwb_path_tracker_node
    target_obstacle_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
