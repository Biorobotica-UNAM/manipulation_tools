cmake_minimum_required(VERSION 3.8)
project(manipulation_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(manipulation_planner REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

add_library(manipulation_tools SHARED
  src/Manipulation.cpp
)

ament_target_dependencies(manipulation_tools
  rclcpp
  std_msgs
  manipulation_planner
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS manipulation_tools
  DESTINATION lib
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(manipulation_tools)

ament_package()
