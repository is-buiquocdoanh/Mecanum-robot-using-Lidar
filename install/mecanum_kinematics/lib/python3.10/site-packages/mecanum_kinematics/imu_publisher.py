import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from mpu6050 import mpu6050
import math
from geometry_msgs.msg import Quaternion
import tf_transformations

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.sensor = mpu6050(0x68)
        self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
        self.timer = self.create_timer(0.02, self.publish_imu)  # 50Hz

    def publish_imu(self):
        accel = self.sensor.get_accel_data()
        gyro = self.sensor.get_gyro_data()

        msg = Imu()

        # Không có bộ lọc nên orientation = 0
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        msg.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        msg.orientation_covariance[0] = -1.0  # dùng -1 nếu không có dữ liệu orientation thực

        msg.angular_velocity.x = math.radians(gyro['x'])
        msg.angular_velocity.y = math.radians(gyro['y'])
        msg.angular_velocity.z = math.radians(gyro['z'])

        msg.linear_acceleration.x = accel['x'] * 9.81
        msg.linear_acceleration.y = accel['y'] * 9.81
        msg.linear_acceleration.z = accel['z'] * 9.81

        self.publisher_.publish(msg)
        self.get_logger().info('Published IMU data')

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
