cmake_minimum_required(VERSION 3.8)
project(robot_stm32_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BridgeStatus.msg"
  "msg/DemoStatus.msg"
  "msg/WheelState.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_library(protocol_v1 STATIC src/protocol_v1.cpp)
target_include_directories(protocol_v1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(robot_stm32_bridge_node src/bridge_node.cpp)
ament_target_dependencies(robot_stm32_bridge_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)
target_link_libraries(robot_stm32_bridge_node protocol_v1)

add_executable(straight_obstacle_stop_demo src/straight_obstacle_stop_demo.cpp)
ament_target_dependencies(straight_obstacle_stop_demo
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(robot_stm32_bridge_node "${cpp_typesupport_target}")
target_link_libraries(straight_obstacle_stop_demo "${cpp_typesupport_target}")

install(TARGETS
  protocol_v1
  robot_stm32_bridge_node
  straight_obstacle_stop_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch systemd DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/configure_can3.sh
  scripts/start_production_bridge.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(protocol_v1_test test/protocol_v1_test.cpp)
  if(TARGET protocol_v1_test)
    target_link_libraries(protocol_v1_test protocol_v1)
  endif()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
