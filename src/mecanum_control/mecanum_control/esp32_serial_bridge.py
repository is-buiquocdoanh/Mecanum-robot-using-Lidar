import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_serial_bridge')
        self.ser = serial.Serial('/dev/esp32', 115200, timeout=0.1)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.L = 0.085
        self.W = 0.11

    def cmd_vel_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z

        fl = vx - vy - (self.L + self.W) * omega
        fr = vx + vy + (self.L + self.W) * omega
        rl = vx + vy - (self.L + self.W) * omega
        rr = vx - vy + (self.L + self.W) * omega

        def to_pwm(v):
            pwm = int(v * 255)
            if abs(v) > 0.1: #chỉ boost khi tốc độ >0.1ms/s
                if pwm > 0:
                    pwm = max(255, pwm)  # Ngưỡng tối thiểu tiến
                elif pwm < 0:
                    pwm = min(-255, pwm)  # Ngưỡng tối thiểu lùi
            return max(-255, min(255, pwm))

        cmd = f"FL:{to_pwm(fl)},FR:{to_pwm(fr)},RL:{to_pwm(rl)},RR:{to_pwm(rr)}\n"
        self.get_logger().info(f"SEND: {cmd.strip()}")
        self.ser.write(cmd.encode())

def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
