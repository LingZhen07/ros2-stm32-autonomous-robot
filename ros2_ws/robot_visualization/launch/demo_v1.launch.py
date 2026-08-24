import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    perception = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("camera_processing"),
                "launch",
                "perception_pipeline.launch.xml",
            )
        )
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_navigation"),
                "launch",
                "navigation_v1.launch.py",
            )
        )
    )

    visualization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_visualization"),
                "launch",
                "visualization_v1.launch.py",
            )
        )
    )

    return LaunchDescription([perception, navigation, visualization])
