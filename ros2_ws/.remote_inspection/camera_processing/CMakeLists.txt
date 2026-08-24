cmake_minimum_required(VERSION 3.8)

project(camera_processing)

# ------------------------------------------------------------
# Compiler settings
# ------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()


# ------------------------------------------------------------
# ROS 2 dependencies
# ------------------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)


# ============================================================
# AscendCL
# ============================================================

set(
  ASCEND_HOME
  "/usr/local/Ascend/ascend-toolkit/latest"
  CACHE PATH
  "Ascend Toolkit root"
)

find_path(
  ASCENDCL_INCLUDE_DIR
  NAMES
    acl/acl.h
  HINTS
    "${ASCEND_HOME}/include"
)

find_library(
  ASCENDCL_LIBRARY
  NAMES
    ascendcl
  HINTS
    "${ASCEND_HOME}/lib64"
    "${ASCEND_HOME}/runtime/lib64"
    "${ASCEND_HOME}/aarch64-linux/lib64"
)

if(NOT ASCENDCL_INCLUDE_DIR)
  message(
    FATAL_ERROR
    "AscendCL header acl/acl.h not found under ${ASCEND_HOME}"
  )
endif()

if(NOT ASCENDCL_LIBRARY)
  message(
    FATAL_ERROR
    "libascendcl.so not found under ${ASCEND_HOME}"
  )
endif()

message(
  STATUS
  "AscendCL include: ${ASCENDCL_INCLUDE_DIR}"
)

message(
  STATUS
  "AscendCL library: ${ASCENDCL_LIBRARY}"
)


# ============================================================
# Depth Processor
# ============================================================

add_executable(
  depth_processor
  src/depth_processor.cpp
)

target_compile_features(
  depth_processor
  PRIVATE
  cxx_std_17
)

ament_target_dependencies(
  depth_processor
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)


# ============================================================
# Ascend YOLOv8 RGB Detection
# ============================================================

add_executable(
  rgbd_perception_node
  src/rgbd_perception_node.cpp
)

target_compile_features(
  rgbd_perception_node
  PRIVATE
  cxx_std_17
)

target_include_directories(
  rgbd_perception_node
  PRIVATE
  "${ASCENDCL_INCLUDE_DIR}"
)

ament_target_dependencies(
  rgbd_perception_node
  rclcpp
  sensor_msgs
  vision_msgs
)

target_link_libraries(
  rgbd_perception_node
  "${ASCENDCL_LIBRARY}"
)


# ============================================================
# Install executables
# ============================================================

install(
  TARGETS
    depth_processor
    rgbd_perception_node
  DESTINATION
    lib/${PROJECT_NAME}
)


# ============================================================
# Install launch files
# ============================================================

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)


# ============================================================
# Package
# ============================================================

ament_package()