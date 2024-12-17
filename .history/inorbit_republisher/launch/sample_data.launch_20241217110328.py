import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declaración de argumentos
        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([launch.substitutions.LaunchConfiguration('package_share_directory'), 'test', 'sample_data', 'config.yaml']),
            description='Path to the config file'
        ),
        DeclareLaunchArgument(
            'rosbag_path',
            default_value=PathJoinSubstitution([launch.substitutions.LaunchConfiguration('package_share_directory'), 'test', 'sample_data', 'harcodedRosbag', 'rosbag2_2024_12_13-19_33_34_0.db3']),
            description='Path to the rosbag file'
        ),
        
        # Primer nodo: inorbit_republisher
        Node(
            name='inorbit_republisher',
            package='inorbit_republisher',
            executable='republisher',
            output='screen',
            parameters=[{'config': LaunchConfiguration('config_path')}],
        ),

        # Segundo nodo: rosbag2 player
        Node(
            package='rosbag2',
            executable='play',
            name='player',
            output='screen',
            arguments=['-l', LaunchConfiguration('rosbag_path')],
        ),
    ])
