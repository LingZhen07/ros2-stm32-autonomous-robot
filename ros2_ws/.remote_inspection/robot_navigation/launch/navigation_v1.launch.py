import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def package_launch(package_name, launch_file, launch_arguments=None):
    source = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory(package_name),
            "launch",
            launch_file,
        )
    )
    return IncludeLaunchDescription(
        source,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    navigation_params = os.path.join(
        get_package_share_directory("robot_navigation"),
        "config",
        "navigation_v1.yaml",
    )

    sensor_transforms = package_launch(
        "robot_bringup",
        "sensor_transforms.launch.py",
    )
    rplidar = package_launch(
        "sllidar_ros2",
        "sllidar_a1_launch.py",
        {
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": "115200",
            # Mandatory: the upstream A1 launch defaults to "laser".
            "frame_id": "laser_frame",
        },
    )
    cartographer = package_launch(
        "robot_slam",
        "cartographer_mapping.launch.py",
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[navigation_params],
        output="screen",
    )

    # The installed Humble 1.1.20 standalone executable fixes its FQN as
    # /costmap/costmap even if launch-level node-name remapping is requested.
    local_costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        parameters=[navigation_params],
        output="screen",
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation_v1",
        parameters=[
            {
                "use_sim_time": False,
                "autostart": True,
                # The standalone Humble Costmap2D node does not create a
                # lifecycle bond; transitions are still managed and checked.
                "bond_timeout": 0.0,
                # PlannerServer transitions its embedded global costmap.
                "node_names": ["planner_server", "costmap/costmap"],
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            sensor_transforms,
            rplidar,
            cartographer,
            planner_server,
            local_costmap,
            lifecycle_manager,
        ]
    )
