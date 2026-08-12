from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("x5a_control_bridge"), "config", "bridge.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=cfg),
            Node(
                package="x5a_control_bridge",
                executable="control_bridge",
                name="x5a_control_bridge",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
