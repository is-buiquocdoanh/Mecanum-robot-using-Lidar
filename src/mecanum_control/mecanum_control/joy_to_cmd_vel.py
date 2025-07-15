import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyToCmdVel(Node):
    def __init__(self):
        super().__init__('joy_to_cmd_vel')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Cấu hình trục & nút
        self.axis_linear = 1        # Y: tiến/lùi
        self.axis_angular = 0       # X: quay
        self.enable_button = 5      # nút R1
        self.scale_linear = 0.5
        self.scale_angular = 1.5

    def joy_callback(self, msg: Joy):
        if len(msg.axes) <= max(self.axis_linear, self.axis_angular):
            return

        if len(msg.buttons) <= self.enable_button or msg.buttons[self.enable_button] != 1:
            return

        twist = Twist()
        twist.linear.x = self.scale_linear * msg.axes[self.axis_linear]
        twist.angular.z = self.scale_angular * msg.axes[self.axis_angular]
        self.publisher.publish(twist)

        self.get_logger().info(f'cmd_vel: x={twist.linear.x:.2f}, z={twist.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = JoyToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
