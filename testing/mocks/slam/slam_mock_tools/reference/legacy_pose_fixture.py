#!/usr/bin/env python3
"""
mock_aruco_publisher.py
Standalone test script that publishes synthetic 6-DOF ArUco landmark poses on /perception/aruco_pose.
Allows decoupled testing of RTAB-Map landmark graph-optimization channel without camera hardware.

Developer Track: Person 2 / Track B (Task 3B.1 & Task 3B.2)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class MockArucoPublisherNode(Node):
    def __init__(self):
        super().__init__('mock_aruco_publisher')

        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('target_frame', 'camera_link')
        self.declare_parameter('pos_x', 1.5)
        self.declare_parameter('pos_y', 0.2)
        self.declare_parameter('pos_z', 0.0)

        self.rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.pos_x = self.get_parameter('pos_x').get_parameter_value().double_value
        self.pos_y = self.get_parameter('pos_y').get_parameter_value().double_value
        self.pos_z = self.get_parameter('pos_z').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(PoseStamped, '/perception/aruco_pose', 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(
            f"Mock ArUco Publisher Node Started (Rate: {self.rate} Hz, Frame: '{self.target_frame}', Pose: [{self.pos_x}, {self.pos_y}, {self.pos_z}])."
        )

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame

        msg.pose.position.x = self.pos_x
        msg.pose.position.y = self.pos_y
        msg.pose.position.z = self.pos_z

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().debug(f"Published mock ArUco pose on /perception/aruco_pose: x={self.pos_x}, y={self.pos_y}")


def main(args=None):
    rclpy.init(args=args)
    node = MockArucoPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
