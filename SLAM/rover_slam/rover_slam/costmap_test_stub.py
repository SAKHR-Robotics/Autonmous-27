#!/usr/bin/env python3
"""
costmap_test_stub.py
Publishes simulated rock obstacle point clouds to /perception/obstacles_only
to verify nav2_costmap_2d inflation output independently without needing Perception module running.

Developer Track: Person 2 / Track B (Task 5B.1)
"""

import struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class CostmapTestStubNode(Node):
    def __init__(self):
        super().__init__('costmap_test_stub')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)
        self.declare_parameter('publish_rate', 5.0)       # Hz
        self.declare_parameter('frame_id', 'base_link')    # Target coordinate frame
        self.declare_parameter('num_rocks', 3)

        self.rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.num_rocks = self.get_parameter('num_rocks').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(PointCloud2, '/perception/obstacles_only', 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(
            f"Costmap Test Stub Node Initialized (Publishing synthetic rock obstacles to /perception/obstacles_only at {self.rate} Hz)."
        )

    def create_synthetic_rock_cloud(self):
        """Generates a binary PointCloud2 message containing 3D clusters representing rocks."""
        points = []

        # Synthetic Rock 1: Centered at (x=2.0m, y=0.5m, z=0.1m)
        for dx in np.linspace(-0.2, 0.2, 5):
            for dy in np.linspace(-0.2, 0.2, 5):
                for dz in np.linspace(0.0, 0.3, 3):
                    points.append((2.0 + dx, 0.5 + dy, 0.1 + dz))

        # Synthetic Rock 2: Centered at (x=3.0m, y=-0.8m, z=0.1m)
        for dx in np.linspace(-0.15, 0.15, 4):
            for dy in np.linspace(-0.15, 0.15, 4):
                for dz in np.linspace(0.0, 0.2, 3):
                    points.append((3.0 + dx, -0.8 + dy, 0.1 + dz))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        buffer = bytearray()
        for p in points:
            buffer.extend(struct.pack('fff', p[0], p[1], p[2]))

        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * len(points)
        cloud_msg.is_dense = True
        cloud_msg.data = bytes(buffer)

        return cloud_msg

    def timer_callback(self):
        cloud_msg = self.create_synthetic_rock_cloud()
        self.publisher_.publish(cloud_msg)
        self.get_logger().debug("Published synthetic obstacle point cloud to /perception/obstacles_only.")


def main(args=None):
    rclpy.init(args=args)
    node = CostmapTestStubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
