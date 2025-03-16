import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'config_file', default_value='$(find vff_avoidance)/config/AvoidanceNodeConfig.yaml',
            description='Path to the configuration file'
        ),


        Node(
            package='vff_avoidance',
            executable='avoidance_vff_main',
            name='vff_avoidance_node',
            output='screen',
            parameters=[launch.substitutions.LaunchConfiguration('config_file')],
            remappings=[('/scan', '/laser/scan')]
        ),
    ])
