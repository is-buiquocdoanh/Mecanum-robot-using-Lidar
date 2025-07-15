import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import RPi.GPIO as GPIO
import math
import time

class EncoderGpioPublisher(Node):
    def __init__(self):
        super().__init__('encoder_gpio_publisher')

        # Thông số robot
        self.R = 0.034  # bán kính bánh xe (m)
        self.L = 0.085
        self.W = 0.11
        self.encoder_resolution = 11  # xung/vòng trục động cơ
        self.gear_ratio = 103
        self.ticks_per_rev = self.encoder_resolution * self.gear_ratio  # xung/vòng bánh

        # GPIO chân encoder (A và B)
        self.encoder_pins = {
            'FL': {'A': 18, 'B': 17},
            'FR': {'A': 27, 'B': 24},
            'RL': {'A': 15, 'B': 22},
            'RR': {'A': 23, 'B': 25}
        }
        self.encoder_counts = {'FL': 0, 'FR': 0, 'RL': 0, 'RR': 0}
        self.last_counts = self.encoder_counts.copy()

        GPIO.setmode(GPIO.BCM)
        for wheel, pins in self.encoder_pins.items():
            GPIO.setup(pins['A'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(pins['B'], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(pins['A'], GPIO.BOTH, callback=self.make_callback(wheel), bouncetime=1)

        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = time.time()

        self.odom_pub = self.create_publisher(Odometry, '/raw_odom', 10)
        self.timer = self.create_timer(0.02, self.update_odom)  # 20 Hz

    def make_callback(self, wheel):
        def cb(channel):
            pins = self.encoder_pins[wheel]
            a_state = GPIO.input(pins['A'])
            b_state = GPIO.input(pins['B'])

            if a_state == b_state:
                self.encoder_counts[wheel] += 1
            else:
                self.encoder_counts[wheel] -= 1
        return cb

    def update_odom(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        dt = max(dt, 1e-6)

        dcounts = {
            w: self.encoder_counts[w] - self.last_counts[w]
            for w in self.encoder_counts
        }
        self.last_counts = self.encoder_counts.copy()

        # Tính tốc độ từng bánh (rad/s)
        w = {
            wheel: 2 * math.pi * dcounts[wheel] / self.ticks_per_rev / dt
            for wheel in dcounts
        }

        # Tính toán động học Mecanum
        R = self.R
        L, W = self.L, self.W
        vx = R/4 * (w['FL'] + w['FR'] + w['RL'] + w['RR'])
        vy = R/4 * (-w['FL'] + w['FR'] + w['RL'] - w['RR'])
        omega = R/(4*(L + W)) * (-w['FL'] + w['FR'] - w['RL'] + w['RR'])

        dx = vx * dt
        dy = vy * dt
        dth = omega * dt

        self.x += dx * math.cos(self.th) - dy * math.sin(self.th)
        self.y += dx * math.sin(self.th) + dy * math.cos(self.th)
        self.th += dth

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = self.yaw_to_quaternion(self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

    def yaw_to_quaternion(self, yaw):
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def main(args=None):
    rclpy.init(args=args)
    node = EncoderGpioPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        GPIO.cleanup()
        node.get_logger().info('Encoder GPIO stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
