cmake_minimum_required(VERSION 3.8)
project(uwb_aoa_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LibAoaRobotMsg.msg"
  DEPENDENCIES std_msgs
)

# 目标适配节点不依赖厂商静态库，各架构均可构建。
# The target adapter does not link the vendor archive and builds on every arch.
add_executable(one1000_target_point_node src/one1000_target_point_node.cpp)
ament_target_dependencies(one1000_target_point_node
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_compile_features(one1000_target_point_node PUBLIC cxx_std_17)

rosidl_get_typesupport_target(adapter_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(one1000_target_point_node "${adapter_typesupport_target}")

install(TARGETS one1000_target_point_node
  DESTINATION lib/${PROJECT_NAME})

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(UWB_VENDOR_DRIVER_DEFAULT ON)
else()
  set(UWB_VENDOR_DRIVER_DEFAULT OFF)
endif()

option(
  BUILD_UWB_VENDOR_DRIVER
  "Build the ARM64-only UWB vendor driver executable"
  ${UWB_VENDOR_DRIVER_DEFAULT}
)

if(BUILD_UWB_VENDOR_DRIVER)
  set(MY_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/uwb_robot_algo.a)
  set(MY_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/uwb_aoa_pkg/)

  add_library(libAoa_robot STATIC IMPORTED GLOBAL)
  add_library(uart_stack STATIC src/uart_stack.c)
  target_include_directories(uart_stack PUBLIC include/uwb_aoa_pkg/)
  target_compile_features(uart_stack PUBLIC c_std_99)

  set_target_properties(libAoa_robot PROPERTIES
    IMPORTED_LOCATION ${MY_LIB_PATH}
    INTERFACE_INCLUDE_DIRECTORIES ${MY_INCLUDE_DIR}
  )

  add_executable(libAoa_robot_example src/libAoa_robot_example.cpp)
  ament_target_dependencies(libAoa_robot_example rclcpp std_msgs)

  target_link_libraries(libAoa_robot_example
    libAoa_robot
    uart_stack
  )

  target_include_directories(libAoa_robot_example PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(libAoa_robot_example PUBLIC cxx_std_17)

  rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(libAoa_robot_example "${cpp_typesupport_target}")

  install(TARGETS libAoa_robot_example
    DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "Skipping libAoa_robot_example: bundled uwb_robot_algo.a is ARM64-only. "
    "ROS interfaces remain available for upper-layer builds and rosbag tests.")
endif()

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()
