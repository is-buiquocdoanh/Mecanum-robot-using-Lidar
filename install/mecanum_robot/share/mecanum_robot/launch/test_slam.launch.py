from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_path = os.path.join(os.getenv('HOME'), 'robot_ws', 'src', 'mecanum_robot')

    return LaunchDescription([
        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[os.path.join(pkg_path, 'description', 'robot.urdf')]
        ),

        # TF broadcaster giả lập odom -> base_link
        Node(
            package='mecanum_robot',
            executable='tf_broadcaster',
            name='tf_broadcaster',
            output='screen'
        ),

        # scan QoS relay
        Node(
            package='scan_qos_bridge',
            executable='scan_qos_relay',
            name='scan_qos_relay',
            output='screen'
        ),

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(pkg_path, 'config', 'test_slam.yaml')],
            remappings=[('/scan', '/scan_relay')]
        ),

        # RViz2
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', os.path.join(pkg_path, 'config', 'slam.rviz')],
        #     output='screen'
        # )
    ])
