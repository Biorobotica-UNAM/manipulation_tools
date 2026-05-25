cmake_minimum_required(VERSION 3.5)
project(manipulation_tools)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(manipulation_planner_interfaces REQUIRED)
find_package(manipulation_controller_interfaces REQUIRED)

include_directories(
  include
)

add_library(manipulation_tools SHARED
  src/Manipulation.cpp
)

ament_target_dependencies(manipulation_tools
  rclcpp
  geometry_msgs
  manipulation_planner_interfaces
  manipulation_controller_interfaces
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS manipulation_tools
  DESTINATION lib
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

