cmake_minimum_required(VERSION 3.8)
project(robot_visualization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(detection_overlay src/detection_overlay.cpp)
ament_target_dependencies(
  detection_overlay
  cv_bridge
  rclcpp
  sensor_msgs
  vision_msgs
)
target_link_libraries(detection_overlay ${OpenCV_LIBRARIES})

add_executable(planning_goal_bridge src/planning_goal_bridge.cpp)
ament_target_dependencies(
  planning_goal_bridge
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
)

install(
  TARGETS
    detection_overlay
    planning_goal_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
