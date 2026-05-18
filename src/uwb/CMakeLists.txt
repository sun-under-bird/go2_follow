cmake_minimum_required(VERSION 3.8)
project(uwb_aoa_pkg)

set(MY_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/uwb_robot_algo.a)
set(MY_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/uwb_aoa_pkg/)

add_library(libAoa_robot STATIC IMPORTED GLOBAL)
add_library(uart_stack STATIC src/uart_stack.c)

target_include_directories(uart_stack PUBLIC include/uwb_aoa_pkg/)

set_target_properties(libAoa_robot PROPERTIES 
  IMPORTED_LOCATION ${MY_LIB_PATH}
  INTERFACE_INCLUDE_DIRECTORIES ${MY_INCLUDE_DIR}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LibAoaRobotMsg.msg"
)


add_executable(libAoa_robot_example src/libAoa_robot_example.cpp)
ament_target_dependencies(libAoa_robot_example rclcpp)

target_link_libraries(libAoa_robot_example
  libAoa_robot
  uart_stack
)

target_include_directories(libAoa_robot_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(libAoa_robot_example PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

rosidl_target_interfaces(libAoa_robot_example ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS libAoa_robot_example
  DESTINATION lib/${PROJECT_NAME})


rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(libAoa_robot_example "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)

ament_package()
