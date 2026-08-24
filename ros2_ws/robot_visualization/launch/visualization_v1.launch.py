from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


VISUALIZATION_TOPICS = [
    r"^/camera_processing/detection_image/compressed$",
    r"^/scan$",
    r"^/map$",
    r"^/tf$",
    r"^/tf_static$",
    r"^/global_costmap/costmap$",
    r"^/global_costmap/costmap_updates$",
    r"^/costmap/costmap$",
    r"^/costmap/costmap_updates$",
    r"^/global_costmap/published_footprint$",
    r"^/costmap/published_footprint$",
    r"^/visualization/planned_path$",
]


def generate_launch_description():
    address = LaunchConfiguration("address")
    port = LaunchConfiguration("port")

    detection_overlay = Node(
        package="robot_visualization",
        executable="detection_overlay",
        name="detection_overlay",
        output="screen",
        parameters=[
            {
                "max_cache_size": 30,
                "max_wait_ms": 500,
                "compressed_jpeg_quality": 75,
                "compressed_max_rate_hz": 12.0,
            }
        ],
    )

    planning_goal_bridge = Node(
        package="robot_visualization",
        executable="planning_goal_bridge",
        name="planning_goal_bridge",
        output="screen",
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "address": ParameterValue(address, value_type=str),
                "port": ParameterValue(port, value_type=int),
                "topic_whitelist": VISUALIZATION_TOPICS,
                "client_topic_whitelist": [r"^/visualization/goal$"],
                "service_whitelist": [r"(?!)"],
                "param_whitelist": [r"(?!)"],
                "capabilities": ["clientPublish"],
                "include_hidden": False,
                "publish_client_count": False,
                "sysinfo": False,
                "remote_access": False,
                "tls": False,
                "use_sim_time": False,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("address", default_value="172.20.10.3"),
            DeclareLaunchArgument("port", default_value="8765"),
            detection_overlay,
            planning_goal_bridge,
            foxglove_bridge,
        ]
    )
