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
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray


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
        self.marker_publisher_ = self.create_publisher(MarkerArray, '/terrain/obstacle_markers', 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(
            f"Costmap Test Stub Node Initialized (Publishing synthetic rock obstacles to /perception/obstacles_only and markers to /terrain/obstacle_markers at {self.rate} Hz)."
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

    def create_synthetic_markers(self):
        """Generates bright wireframe bounding box and text markers corresponding to the synthetic rocks."""
        marker_array = MarkerArray()
        # Using time zero (Time(0)) tells RViz to always use the latest available TF,
        # completely eliminating extrapolation delay and marker flickering!
        stamp = rclpy.time.Time().to_msg()

        rocks = [
            {'id': 1, 'name': 'Rock A (0.5m)', 'center': (2.0, 0.5, 0.15), 'size': (0.5, 0.5, 0.35)},
            {'id': 2, 'name': 'Rock B (0.4m)', 'center': (3.0, -0.8, 0.1), 'size': (0.4, 0.4, 0.25)}
        ]

        for r in rocks:
            xc, yc, zc = r['center']
            dx, dy, dz = r['size']
            hx, hy, hz = dx / 2.0, dy / 2.0, dz / 2.0

            # 1. 3D Wireframe Box (12 Edges)
            box_marker = Marker()
            box_marker.header.stamp = stamp
            box_marker.header.frame_id = self.frame_id
            box_marker.lifetime.sec = 0
            box_marker.lifetime.nanosec = int(0.5 * 1e9)  # 500ms lifetime
            box_marker.ns = 'obstacle_wireframes'
            box_marker.id = r['id'] * 10
            box_marker.type = Marker.LINE_LIST
            box_marker.action = Marker.ADD
            box_marker.pose.orientation.w = 1.0
            box_marker.scale.x = 0.03  # Line thickness: 3 cm
            box_marker.color.r = 0.0
            box_marker.color.g = 1.0
            box_marker.color.b = 0.2
            box_marker.color.a = 1.0  # Bright neon green

            # 8 corners
            c = [
                (xc - hx, yc - hy, zc - hz),  # 0
                (xc + hx, yc - hy, zc - hz),  # 1
                (xc + hx, yc + hy, zc - hz),  # 2
                (xc - hx, yc + hy, zc - hz),  # 3
                (xc - hx, yc - hy, zc + hz),  # 4
                (xc + hx, yc - hy, zc + hz),  # 5
                (xc + hx, yc + hy, zc + hz),  # 6
                (xc - hx, yc + hy, zc + hz),  # 7
            ]

            # 12 edge pairs
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),  # bottom square
                (4, 5), (5, 6), (6, 7), (7, 4),  # top square
                (0, 4), (1, 5), (2, 6), (3, 7)   # vertical pillars
            ]

            for p1_idx, p2_idx in edges:
                pt1 = Point(x=c[p1_idx][0], y=c[p1_idx][1], z=c[p1_idx][2])
                pt2 = Point(x=c[p2_idx][0], y=c[p2_idx][1], z=c[p2_idx][2])
                box_marker.points.extend([pt1, pt2])

            marker_array.markers.append(box_marker)

            # 2. Text Label above each obstacle
            text_marker = Marker()
            text_marker.header.stamp = stamp
            text_marker.header.frame_id = self.frame_id
            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = int(0.5 * 1e9)  # 500ms lifetime
            text_marker.ns = 'obstacle_labels'
            text_marker.id = r['id'] * 10 + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = xc
            text_marker.pose.position.y = yc
            text_marker.pose.position.z = zc + hz + 0.2
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.22  # Text size
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0  # Yellow text
            text_marker.color.a = 1.0
            text_marker.text = r['name']
            marker_array.markers.append(text_marker)

        return marker_array

    def timer_callback(self):
        cloud_msg = self.create_synthetic_rock_cloud()
        self.publisher_.publish(cloud_msg)
        marker_msg = self.create_synthetic_markers()
        self.marker_publisher_.publish(marker_msg)


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
