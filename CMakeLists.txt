cmake_minimum_required(VERSION 3.5)
project(manipulation_tools)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(manipulation_planner_interfaces REQUIRED)
find_package(manipulation_controller_interfaces REQUIRED)

include_directories(
  include
)

add_library(manipulation_tools SHARED
  src/Manipulation.cpp
)

add_executable(manipulation_cloud_filter_node
  src/manipulation_cloud_filter_node.cpp
)

ament_target_dependencies(manipulation_tools
  rclcpp
  geometry_msgs
  manipulation_planner_interfaces
  manipulation_controller_interfaces
)

ament_target_dependencies(manipulation_cloud_filter_node
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_srvs
  visualization_msgs
  tf2_ros
  tf2_eigen
)
target_link_libraries(manipulation_cloud_filter_node Eigen3::Eigen)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS manipulation_tools
  DESTINATION lib
)

install(TARGETS manipulation_cloud_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  manipulation_planner_interfaces
  manipulation_controller_interfaces
)

ament_export_include_directories(include)
ament_export_libraries(manipulation_tools)

ament_package()
