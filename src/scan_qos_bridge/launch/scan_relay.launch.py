from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scan_qos_bridge',
            executable='scan_relay',
            name='scan_qos_relay',
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    ])
