import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Nodo inorbit_republisher
        Node(
            package='inorbit_republisher',
            executable='republisher',
            name='inorbit_republisher',
            parameters=[{'config': '/root/ros2_ws/src/inorbit_republisher/test/sample_data/config.yaml'}],
        ),

        # Comando para reproducir rosbag2
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/root/ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag/rosbag2_2024_12_13-19_33_34_0.db3'],
            output='screen'
        ),
    ])
