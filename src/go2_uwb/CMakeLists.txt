cmake_minimum_required(VERSION 3.8)
project(go2_uwb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(uwb_aoa_pkg REQUIRED)

set(COMMON_DEPENDENCIES
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  uwb_aoa_pkg
)

set(OBSTACLE_DETECTOR_DEPENDENCIES
  ${COMMON_DEPENDENCIES}
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
)

add_executable(uwb_filter_node src/uwb_filter_node.cpp)
ament_target_dependencies(
  uwb_filter_node
  ${COMMON_DEPENDENCIES}
  tf2_geometry_msgs
  tf2_ros
)

add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
target_include_directories(obstacle_detector_node PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(obstacle_detector_node ${OBSTACLE_DETECTOR_DEPENDENCIES})
target_link_libraries(obstacle_detector_node
  pcl_common
  pcl_filters
  pcl_kdtree
  pcl_search
)

add_executable(follow_controller_node src/follow_controller_node.cpp)
ament_target_dependencies(follow_controller_node ${COMMON_DEPENDENCIES})

install(TARGETS
  uwb_filter_node
  obstacle_detector_node
  follow_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
