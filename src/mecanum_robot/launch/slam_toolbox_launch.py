from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('mecanum_robot'),
                    'config',
                    'slam_config.yaml'
                ])
            ],
            remappings=[
                ('scan', '/scan_relay'),
            ]
        )
    ])
