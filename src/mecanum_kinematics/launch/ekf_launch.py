from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecanum_kinematics',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen'
        ),
        Node(
            package='mecanum_kinematics',
            executable='encoder_gpio_publisher',
            name='encoder_gpio_publisher',
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                '/home/pi/robot_ws/src/mecanum_kinematics/config/ekf.yaml',
                {'use_sim_time': False},
            ],
            remappings=[
                ('odometry/filtered', 'odom')
            ]
        )
    ])
