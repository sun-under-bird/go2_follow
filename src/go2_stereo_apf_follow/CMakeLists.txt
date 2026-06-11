cmake_minimum_required(VERSION 3.8)
project(go2_stereo_apf_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(uwb_aoa_pkg REQUIRED)
find_package(visualization_msgs REQUIRED)

set(APF_NODE_DEPS
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(uwb_target_seed_node src/uwb_target_seed_node.cpp)
target_include_directories(uwb_target_seed_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(uwb_target_seed_node PUBLIC cxx_std_17)
ament_target_dependencies(uwb_target_seed_node ${APF_NODE_DEPS} uwb_aoa_pkg)

add_executable(stereo_apf_controller_node src/stereo_apf_controller_node.cpp)
target_include_directories(stereo_apf_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(stereo_apf_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(stereo_apf_controller_node ${APF_NODE_DEPS})

add_executable(stereo_vfh_controller_node src/stereo_vfh_controller_node.cpp)
target_include_directories(stereo_vfh_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(stereo_vfh_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(stereo_vfh_controller_node ${APF_NODE_DEPS})

install(TARGETS
  uwb_target_seed_node
  stereo_apf_controller_node
  stereo_vfh_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_apf_core test/test_apf_core.cpp)
  if(TARGET test_apf_core)
    target_include_directories(test_apf_core PRIVATE include)
    target_compile_features(test_apf_core PUBLIC cxx_std_17)
  endif()
  ament_add_gtest(test_vfh_core test/test_vfh_core.cpp)
  if(TARGET test_vfh_core)
    target_include_directories(test_vfh_core PRIVATE include)
    target_compile_features(test_vfh_core PUBLIC cxx_std_17)
  endif()
endif()

ament_export_include_directories(include)
ament_package()
