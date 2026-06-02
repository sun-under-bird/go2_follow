cmake_minimum_required(VERSION 3.5)
project(behavior_ext_plugins)

# 使用 C++14，兼容 ROS2 Humble apt 发布的 Navigation2。
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_lib)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  visualization_msgs
  nav2_util
  nav2_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
  nav2_behaviors
)


add_library(${library_name} SHARED
  plugins/back_up_twz_free_action.cpp
)

target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(${library_name}
  ${dependencies}
)


pluginlib_export_plugin_description_file(nav2_core behavior_ext_plugins.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_ext_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 没有统一补齐版权头时，可以按需打开下面两行跳过对应检查。
  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()
