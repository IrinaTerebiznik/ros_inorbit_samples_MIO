import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Nodo inorbit_republisher
        Node(
            package='inorbit_republisher',
            executable='republisher.py',  # Cambia 'republisher' por 'republisher.py' si es un script Python
            name='inorbit_republisher',
            parameters=[{'config': PathJoinSubstitution([LaunchConfiguration('config_dir'), 'config.yaml'])}],
        ),

        # Nodo rosbag
        Node(
            package='rosbag2',
            executable='play',
            name='player',
            output='screen',
            arguments=['-l', PathJoinSubstitution([LaunchConfiguration('config_dir'), 'fake_sensors_sample.bag'])],
        ),
    ])
