from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("x5a_moveit_official_adapter"), "config", "adapter.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        Node(
            package="x5a_moveit_official_adapter",
            executable="official_trajectory_adapter",
            name="x5a_official_trajectory_adapter",
            output="screen",
            parameters=[LaunchConfiguration("config")],
        ),
    ])
