from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_control',
            executable='esp32_serial_bridge',
            name='esp32_serial_bridge',
            output='screen',
            parameters=[],
            remappings=[
                ('cmd_vel', '/cmd_vel')  # 👈 remap đúng topic toàn cục
            ]
        )
    ])
