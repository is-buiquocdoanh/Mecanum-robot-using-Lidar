from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Đường dẫn đến các package
    mecanum_robot_launch = os.path.join(get_package_share_directory('mecanum_robot'), 'launch')
    scan_qos_launch = os.path.join(get_package_share_directory('scan_qos_bridge'), 'launch')
    ydlidar_launch = os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch')
    mecanum_kinematics_launch = os.path.join(get_package_share_directory('mecanum_kinematics'), 'launch')

    return LaunchDescription([
        # URDF + robot_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mecanum_robot_launch, 'display.launch.py'))
        ),

        # Lidar driver (YDLIDAR)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ydlidar_launch, 'ydlidar_launch.py'))
        ),

        # Scan QoS relay
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(scan_qos_launch, 'scan_relay.launch.py'))
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mecanum_robot_launch, 'slam_toolbox_launch.py'))
        ),

        # EKF (odom từ encoder + IMU)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mecanum_kinematics_launch, 'ekf_launch.py'))
        )
    ])

#Lưu map
#ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/src/maps/my_map
