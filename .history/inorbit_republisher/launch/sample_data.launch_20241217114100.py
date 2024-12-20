import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Rutas absolutas de tus archivos
    config_file = "ros2_ws/src/inorbit_republisher/test/sample_data/config.yaml"
    rosbag_file = "ros2_ws/src/inorbit_republisher/test/sample_data/hardcodedRosbag/rosbag2_2024_12_13-19_33_34_0.db3"

    return LaunchDescription([
        # Nodo inorbit_republisher
        Node(
            package='inorbit_republisher',
            executable='republisher',
            name='inorbit_republisher',
            parameters=[{'config': config_file}],
        ),

        # Comando para reproducir rosbag2
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', rosbag_file],
            output='screen'
        ),
    ])
