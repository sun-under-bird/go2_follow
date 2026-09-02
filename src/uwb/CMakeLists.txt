cmake_minimum_required(VERSION 3.8)
project(uwb_aoa_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LibAoaRobotMsg.msg"
  DEPENDENCIES std_msgs
)

# 厂商静态库只提供 ARM64 版本；其他架构仍可生成消息并构建上层算法。
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
  add_library(libAoa_robot STATIC IMPORTED GLOBAL)
  set_target_properties(libAoa_robot PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/uwb_robot_algo.a
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include/uwb_aoa_pkg
  )

  add_library(uart_stack STATIC src/uart_stack.c)
  target_include_directories(uart_stack PUBLIC include/uwb_aoa_pkg)
  target_compile_features(uart_stack PUBLIC c_std_99)

  add_executable(libAoa_robot_example src/libAoa_robot_example.cpp)
  target_include_directories(libAoa_robot_example PRIVATE include/uwb_aoa_pkg)
  target_compile_features(libAoa_robot_example PUBLIC cxx_std_17)
  ament_target_dependencies(libAoa_robot_example rclcpp std_msgs)
  target_link_libraries(libAoa_robot_example libAoa_robot uart_stack)

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
