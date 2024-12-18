# sample_data.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Primer nodo: inorbit_republisher
        Node(
            name='inorbit_republisher',
            package='inorbit_republisher',
            executable='republisher',
            output='screen',
            parameters=[{
                'config': '/root/ros2_ws/src/inorbit_republisher/test/sample_data/config/example.yaml'
            }]
        ),

        # Segundo proceso: Ejecutar ros2 bag play
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/root/ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag/rosbag2_2024_12_13-19_33_34_0.db3'],
            name='rosbag_player',
            output='screen'
        )
    ])
