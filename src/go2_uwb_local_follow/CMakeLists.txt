cmake_minimum_required(VERSION 3.8)
project(go2_uwb_local_follow)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uwb_aoa_pkg REQUIRED)

add_library(stereo_projection_core
  src/stereo_projection_core.cpp
)
target_include_directories(stereo_projection_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(stereo_projection_core PUBLIC cxx_std_17)

add_library(follow_control_core
  src/follow_control_core.cpp
)
target_include_directories(follow_control_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(follow_control_core PUBLIC cxx_std_17)

add_library(local_planner_core
  src/local_planner_core.cpp
)
target_include_directories(local_planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(local_planner_core PUBLIC cxx_std_17)

add_executable(stereo_obstacle_projector_node
  src/stereo_obstacle_projector_node.cpp
)
target_include_directories(stereo_obstacle_projector_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(stereo_obstacle_projector_node PUBLIC cxx_std_17)
target_link_libraries(stereo_obstacle_projector_node stereo_projection_core)
ament_target_dependencies(stereo_obstacle_projector_node
  diagnostic_msgs
  rclcpp
  sensor_msgs
  stereo_msgs
  tf2
  tf2_ros
)

add_executable(uwb_target_adapter_node
  src/uwb_target_adapter_node.cpp
)
target_include_directories(uwb_target_adapter_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(uwb_target_adapter_node PUBLIC cxx_std_17)
ament_target_dependencies(uwb_target_adapter_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  uwb_aoa_pkg
)

add_executable(uwb_follow_controller_node
  src/uwb_follow_controller_node.cpp
)
target_include_directories(uwb_follow_controller_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(uwb_follow_controller_node PUBLIC cxx_std_17)
target_link_libraries(uwb_follow_controller_node follow_control_core)
ament_target_dependencies(uwb_follow_controller_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
)

add_executable(local_collision_monitor_node
  src/local_collision_monitor_node.cpp
)
target_include_directories(local_collision_monitor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(local_collision_monitor_node PUBLIC cxx_std_17)
target_link_libraries(local_collision_monitor_node local_planner_core)
ament_target_dependencies(local_collision_monitor_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(local_velocity_planner_node
  src/local_velocity_planner_node.cpp
)
target_include_directories(local_velocity_planner_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(local_velocity_planner_node PUBLIC cxx_std_17)
target_link_libraries(local_velocity_planner_node local_planner_core)
ament_target_dependencies(local_velocity_planner_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

install(
  TARGETS
    stereo_projection_core
    follow_control_core
    local_planner_core
    stereo_obstacle_projector_node
    uwb_target_adapter_node
    uwb_follow_controller_node
    local_collision_monitor_node
    local_velocity_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_stereo_projection_core
    test/test_stereo_projection_core.cpp
  )
  target_link_libraries(test_stereo_projection_core stereo_projection_core)
  target_include_directories(test_stereo_projection_core PRIVATE include)

  ament_add_gtest(test_follow_control_core
    test/test_follow_control_core.cpp
  )
  target_link_libraries(test_follow_control_core follow_control_core)
  target_include_directories(test_follow_control_core PRIVATE include)

  ament_add_gtest(test_local_planner_core
    test/test_local_planner_core.cpp
  )
  target_link_libraries(test_local_planner_core local_planner_core)
  target_include_directories(test_local_planner_core PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(stereo_projection_core follow_control_core local_planner_core)
ament_package()
