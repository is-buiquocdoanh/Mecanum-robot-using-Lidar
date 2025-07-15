import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import time
import math

class EncoderOdomPublisher(Node):
    def __init__(self):
        super().__init__('encoder_odom_publisher')

        # Serial kết nối ESP32
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        time.sleep(2)  # đợi ESP32 khởi động

        # Thông số robot
        self.R = 0.03  # bán kính bánh xe (m)
        self.L = 0.07
        self.W = 0.09
        self.ticks_per_rev = 20  # 1 kênh, 20 xung/vòng
        self.last_counts = [0, 0, 0, 0]  # FL, FR, RL, RR

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.update_odom)  # 10Hz

    def update_odom(self):
        self.ser.write(b'GET_ENCODER\n')
        line = self.ser.readline().decode().strip()

        if not line.startswith('Encoder:FL:'):
            return

        try:
            counts = [int(part.split(':')[1]) for part in line[8:].split(',')]
        except:
            self.get_logger().warn('Lỗi đọc encoder')
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        dcounts = [counts[i] - self.last_counts[i] for i in range(4)]
        self.last_counts = counts

        # Tính vận tốc mỗi bánh (rad/s)
        w = [2 * math.pi * dc / self.ticks_per_rev / dt for dc in dcounts]

        # Tính vận tốc tổng hợp (vx, vy, omega)
        R = self.R
        L, W = self.L, self.W
        vx = R/4 * (w[0] + w[1] + w[2] + w[3])
        vy = R/4 * (-w[0] + w[1] + w[2] - w[3])
        omega = R/(4*(L + W)) * (-w[0] + w[1] - w[2] + w[3])

        # Tích phân vị trí
        dx = vx * dt
        dy = vy * dt
        dth = omega * dt

        self.x += dx * math.cos(self.th) - dy * math.sin(self.th)
        self.y += dx * math.sin(self.th) + dy * math.cos(self.th)
        self.th += dth

        # Gửi tf
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = self.quaternion_from_yaw(self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Gửi odom
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = t.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

    def quaternion_from_yaw(self, yaw):
        return [0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0)]

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
