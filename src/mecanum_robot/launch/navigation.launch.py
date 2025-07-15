from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Đường dẫn đến các launch package
    mecanum_robot_launch = os.path.join(get_package_share_directory('mecanum_robot'), 'launch')
    scan_qos_launch = os.path.join(get_package_share_directory('scan_qos_bridge'), 'launch')
    ydlidar_launch = os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch')
    mecanum_kinematics_launch = os.path.join(get_package_share_directory('mecanum_kinematics'), 'launch')
    nav2_bringup_launch = os.path.join(get_package_share_directory('mecanum_robot'), 'launch')

    # Đường dẫn file bản đồ
    map_file = os.path.join(get_package_share_directory('mecanum_robot'), 'maps', 'my_map.yaml')
    print(">>> MAP PATH:", map_file)

    return LaunchDescription([
        # Load URDF & publish TF tree
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mecanum_robot_launch, 'display.launch.py'))
        ),

        # Khởi động Lidar driver (YDLidar)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ydlidar_launch, 'ydlidar_launch.py'))
        ),

        # Scan QoS relay (chuyển QoS để Nav2 nhận /scan)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(scan_qos_launch, 'scan_relay.launch.py'))
        ),

        # EKF Filter (kết hợp encoder + IMU để ra odom chuẩn)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mecanum_kinematics_launch, 'ekf_launch.py'))
        ),

        # Navigation2 stack (AMCL, Planner, Controller, Recoveries)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'false'
            }.items()
        )
    ])
