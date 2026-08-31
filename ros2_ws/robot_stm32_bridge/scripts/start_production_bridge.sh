#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /data/ros2_ws/install/setup.bash
set -u

exec ros2 launch robot_stm32_bridge bridge.launch.py can_interface:=can3
