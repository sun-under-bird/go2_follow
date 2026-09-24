cmake_minimum_required(VERSION 3.8)
project(go2_uwb_behavior)

# 单配置构建默认保留调试符号并启用优化；显式 Debug 等选择仍然有效。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(go2_uwb_local_follow REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowUwb.action"
  "action/RandomRoam.action"
  "srv/SetBehavior.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

add_library(behavior_core
  src/behavior_core.cpp
)
target_include_directories(behavior_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(behavior_core PUBLIC cxx_std_17)

add_executable(uwb_behavior_controller_node
  src/uwb_behavior_controller_node.cpp
)
target_include_directories(uwb_behavior_controller_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(uwb_behavior_controller_node PUBLIC cxx_std_17)
target_link_libraries(uwb_behavior_controller_node behavior_core)
ament_target_dependencies(uwb_behavior_controller_node
  diagnostic_msgs
  geometry_msgs
  go2_uwb_local_follow
  nav_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(uwb_behavior_controller_node "${cpp_typesupport_target}")

install(
  TARGETS behavior_core uwb_behavior_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config docs launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_behavior_core
    test/test_behavior_core.cpp
  )
  target_link_libraries(test_behavior_core behavior_core)
  target_include_directories(test_behavior_core PRIVATE include)

  add_launch_test(test/test_behavior_action_launch.py TIMEOUT 60)
endif()

ament_export_include_directories(include)
ament_export_libraries(behavior_core)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
