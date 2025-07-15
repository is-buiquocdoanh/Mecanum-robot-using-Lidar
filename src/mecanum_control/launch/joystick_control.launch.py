from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Node đọc joystick từ laptop
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Node chuyển /joy -> /cmd_vel
        Node(
            package='mecanum_control',
            executable='joy_to_cmd_vel',
            name='joy_to_cmd_vel',
            output='screen'
        )
    ])
