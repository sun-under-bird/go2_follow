cmake_minimum_required(VERSION 3.8)
project(uwb_aoa_pkg)

# 单配置构建默认保留调试符号并启用优化；显式 Debug 等选择仍然有效。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

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

# 串口协议不依赖厂家 ARM64 算法，所有架构均编译并测试恢复逻辑。
add_library(uart_stack STATIC src/uart_stack.c)
target_include_directories(uart_stack PUBLIC include/uwb_aoa_pkg)
target_compile_features(uart_stack PUBLIC c_std_99)

if(BUILD_UWB_VENDOR_DRIVER)
  add_library(libAoa_robot STATIC IMPORTED GLOBAL)
  set_target_properties(libAoa_robot PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/uwb_robot_algo.a
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include/uwb_aoa_pkg
  )


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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_uart_stack test/test_uart_stack.cpp)
  target_link_libraries(test_uart_stack uart_stack)
  # 厂家融合算法用测试替身隔离，串口线程与生产驱动使用同一份源代码。
  add_executable(serial_test_driver src/libAoa_robot_example.cpp test/vendor_algo_stub.cpp)
  target_include_directories(serial_test_driver PRIVATE include/uwb_aoa_pkg)
  target_compile_features(serial_test_driver PUBLIC cxx_std_17)
  ament_target_dependencies(serial_test_driver rclcpp std_msgs)
  rosidl_get_typesupport_target(test_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(serial_test_driver uart_stack "${test_typesupport_target}")
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_serial_reconnect
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_serial_reconnect.py
      $<TARGET_FILE:serial_test_driver>)
  set_tests_properties(test_serial_reconnect PROPERTIES TIMEOUT 20)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
