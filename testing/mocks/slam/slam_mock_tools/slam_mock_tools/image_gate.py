"""Test-only loss injection: forward real images unchanged, or stop forwarding."""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageGate(Node):
    def __init__(self):
        super().__init__('slam_image_gate')
        self.declare_parameter('enabled', True)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/testing/slam/camera/image_raw')
        self.publisher = self.create_publisher(Image,
            self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('input_topic').value,
                                 self.observe, qos_profile_sensor_data)

    def observe(self, message):
        if self.get_parameter('enabled').value:
            self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ImageGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
