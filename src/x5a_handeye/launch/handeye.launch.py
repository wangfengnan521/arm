from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("x5a_handeye")
    board = os.path.join(share, "config", "board.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("board_config", default_value=board),
            DeclareLaunchArgument(
                "image_topic", default_value="/camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera/color/camera_info"
            ),
            Node(
                package="x5a_handeye",
                executable="board_detector",
                name="x5a_board_detector",
                output="screen",
                parameters=[
                    {
                        "board_config": LaunchConfiguration("board_config"),
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    }
                ],
            ),
        ]
    )
