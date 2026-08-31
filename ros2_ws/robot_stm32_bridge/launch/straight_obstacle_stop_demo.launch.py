from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("robot_stm32_bridge"))
    default_config = str(package_share / "config" / "m5_commissioning.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="robot_stm32_bridge",
                executable="straight_obstacle_stop_demo",
                name="straight_obstacle_stop_demo",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
