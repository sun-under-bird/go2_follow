cmake_minimum_required(VERSION 3.8)
project(jie_deamon)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uwb_aoa_pkg REQUIRED)

include_directories(include)

add_executable(robot_nexus
  src/robot_nexus.cpp
)

ament_target_dependencies(robot_nexus
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  uwb_aoa_pkg
)

install(TARGETS
  robot_nexus
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
