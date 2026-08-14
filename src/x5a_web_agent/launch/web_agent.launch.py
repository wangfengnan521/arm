from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="0.0.0.0"),
            DeclareLaunchArgument("port", default_value="8000"),
            DeclareLaunchArgument("https", default_value="false"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            Node(
                package="x5a_web_agent",
                executable="web_agent",
                name="x5a_web_agent",
                output="screen",
                arguments=[
                    "--host",
                    LaunchConfiguration("host"),
                    "--port",
                    LaunchConfiguration("port"),
                ],
                additional_env={
                    "X5A_WEB_HTTPS": LaunchConfiguration("https"),
                    "X5A_WEB_DRY_RUN": LaunchConfiguration("dry_run"),
                },
            ),
        ]
    )
