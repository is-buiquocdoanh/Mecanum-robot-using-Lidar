# static_tf_broadcaster.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.broadcaster = StaticTransformBroadcaster(self)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin(node)
    rclpy.shutdown()
