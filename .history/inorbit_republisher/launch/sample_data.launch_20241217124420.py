import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Primer nodo: inorbit_republisher
        DeclareLaunchArgument('config_file', default_value='/root/ros2_ws/src/inorbit_republisher/test/sample_data/config/example.yaml', description='Archivo de configuración'),

        # Ejecutar el nodo inorbit_republisher
        ExecuteProcess(
            cmd=['ros2', 'run', 'inorbit_republisher', 'republisher', '--ros-args', '--param', 'config:=' + LaunchConfiguration('config_file')],
            name='inorbit_republisher',
            output='screen'
        ),
        
        # Ejecutar rosbag2 player
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/root/ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag/rosbag2_2024_12_13-19_33_34_0.db3'],
            name='rosbag_player',
            output='screen'
        )
    ])
