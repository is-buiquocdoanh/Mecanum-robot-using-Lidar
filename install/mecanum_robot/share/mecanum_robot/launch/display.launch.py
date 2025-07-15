import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Đọc tham số use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Lấy đường dẫn file .xacro
    pkg_path = get_package_share_directory('mecanum_robot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf')

    if not os.path.exists(xacro_file):
        raise FileNotFoundError(f"Xacro file not found: {xacro_file}")

    # Xử lý xacro -> URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': use_sim_time},
            ]
        )
    ])
