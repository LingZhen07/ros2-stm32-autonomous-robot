from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    configuration_directory = PathJoinSubstitution(
        [FindPackageShare("robot_slam"), "config"]
    )

    return LaunchDescription(
        [
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                arguments=[
                    "-configuration_directory",
                    configuration_directory,
                    "-configuration_basename",
                    "rplidar_a1_2d.lua",
                ],
                remappings=[("scan", "/scan")],
                output="screen",
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                parameters=[
                    {"use_sim_time": False},
                    {"resolution": 0.05},
                ],
                output="screen",
            ),
        ]
    )
