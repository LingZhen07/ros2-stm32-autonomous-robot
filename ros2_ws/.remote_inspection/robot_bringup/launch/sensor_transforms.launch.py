from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera_link",
                arguments=[
                    "--x",
                    "0.130",
                    "--y",
                    "0.000",
                    "--z",
                    "0.110",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "camera_link",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_frame",
                arguments=[
                    "--x",
                    "0.043",
                    "--y",
                    "0.000",
                    "--z",
                    "0.165",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "3.141592653589793",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "laser_frame",
                ],
                output="screen",
            ),
        ]
    )
