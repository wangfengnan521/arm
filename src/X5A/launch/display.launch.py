from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("X5A"))
    urdf_path = share / "urdf" / "X5A.urdf"
    robot_description = urdf_path.read_text(encoding="utf-8")
    description_param = {"robot_description": robot_description}

    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_gui",
                default_value="true",
                description="Use joint_state_publisher_gui (pure model mode).",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(share / "rviz" / "x5a.rviz"),
                description="RViz2 config path.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        **description_param,
                        "publish_frequency": 50.0,
                    }
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(use_gui),
                parameters=[description_param],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=UnlessCondition(use_gui),
                parameters=[description_param],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
