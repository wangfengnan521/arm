from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('x5a_pick_place'), 'config', 'pick_place.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=cfg),
        DeclareLaunchArgument('plan_only', default_value='false'),
        DeclareLaunchArgument('vision_enabled', default_value='false'),
        DeclareLaunchArgument('target_color', default_value='red'),
        Node(
            package='x5a_pick_place',
            executable='pick_place_node',
            name='x5a_pick_place',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'plan_only': ParameterValue(LaunchConfiguration('plan_only'), value_type=bool),
                    'vision.enabled': ParameterValue(LaunchConfiguration('vision_enabled'), value_type=bool),
                    'target_color': LaunchConfiguration('target_color'),
                },
            ],
        ),
    ])
