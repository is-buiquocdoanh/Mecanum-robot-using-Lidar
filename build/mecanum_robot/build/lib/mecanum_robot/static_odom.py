import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class StaticOdomPublisher(Node):
    def __init__(self):
        super().__init__('static_odom_publisher')
        # Create publisher for /odom topic
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        # Timer to publish odom and TF at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_odom)

    def publish_odom(self):
        # Current time
        current_time = self.get_clock().now().to_msg()

        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Static pose (position and orientation)
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0

        # Static velocity
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = 0.0

        # Publish odom
        self.odom_pub.publish(odom)

        # Create TF transform
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation

        # Publish TF
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = StaticOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()