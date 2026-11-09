cmake_minimum_required(VERSION 3.8)
project(sura_path_planning)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sura_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(
  nav2_velocity_adapter
  src/nav2_velocity_adapter.cpp
)

ament_target_dependencies(
  nav2_velocity_adapter
  rclcpp
  geometry_msgs
  sura_msgs
)

add_executable(simple_navigator src/simple_navigator.cpp)
ament_target_dependencies(simple_navigator
  rclcpp rclcpp_action geometry_msgs interactive_markers nav2_msgs nav_msgs visualization_msgs)

install(
  TARGETS nav2_velocity_adapter simple_navigator
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
