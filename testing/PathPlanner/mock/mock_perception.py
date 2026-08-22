#!/usr/bin/env python3
"""
mock_perception.py

Mock Perception Node for Terrain Obstacle Generation.
Publishes terrain_geometry_msgs/msg/ObstacleFeatureArray to /terrain/obstacle_features
to test costmap_bridge_node, local costmap obstacle layers, and dynamic replanning.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from terrain_geometry_msgs.msg import ObstacleFeature, ObstacleFeatureArray


class MockPerception(Node):
    def __init__(self):
        super().__init__('mock_perception')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('enable_dynamic_spawning', True)

        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.enable_dynamic_spawning = self.get_parameter('enable_dynamic_spawning').value

        # Initial Obstacle List: [{id, x, y, z, radius, height, class, conf}]
        self.obstacles = [
            {'id': 1, 'x': 1.0, 'y': 1.0, 'z': 0.15, 'radius': 0.35, 'height': 0.4, 'class': 'rock', 'conf': 0.95},
            {'id': 2, 'x': -1.5, 'y': 0.5, 'z': 0.10, 'radius': 0.25, 'height': 0.3, 'class': 'crater', 'conf': 0.90},
            {'id': 3, 'x': 2.5, 'y': 2.0, 'z': 0.20, 'radius': 0.45, 'height': 0.5, 'class': 'boulder', 'conf': 0.98},
        ]

        # Publishers
        self.obstacle_pub = self.create_publisher(
            ObstacleFeatureArray,
            '/terrain/obstacle_features',
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/perception/obstacle_markers',
            10
        )

        # Subscriber to spawn new obstacle dynamically on demand
        self.spawn_sub = self.create_subscription(
            Point,
            '/mock_perception/spawn_obstacle',
            self.spawn_obstacle_callback,
            10
        )

        # Publish Timer
        dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(dt, self.publish_perception_data)

        self.get_logger().info(f"Mock Perception Node initialized. Publishing at {self.publish_rate} Hz on /terrain/obstacle_features")

    def spawn_obstacle_callback(self, msg: Point):
        new_id = len(self.obstacles) + 1
        obs = {
            'id': new_id,
            'x': msg.x,
            'y': msg.y,
            'z': msg.z if msg.z != 0.0 else 0.15,
            'radius': 0.35,
            'height': 0.4,
            'class': 'dynamic_rock',
            'conf': 0.99
        }
        self.obstacles.append(obs)
        self.get_logger().info(f"Dynamically spawned obstacle {new_id} at ({msg.x:.2f}, {msg.y:.2f})")

    def publish_perception_data(self):
        current_time = self.get_clock().now()

        # 1. Build ObstacleFeatureArray message
        array_msg = ObstacleFeatureArray()
        array_msg.header.stamp = current_time.to_msg()
        array_msg.header.frame_id = self.frame_id

        markers = MarkerArray()

        for obs in self.obstacles:
            feature = ObstacleFeature()
            feature.id = int(obs['id'])
            feature.centroid.x = float(obs['x'])
            feature.centroid.y = float(obs['y'])
            feature.centroid.z = float(obs['z'])
            feature.radius = float(obs['radius'])
            feature.height = float(obs['height'])
            feature.classification = str(obs['class'])
            feature.confidence = float(obs['conf'])
            array_msg.obstacles.append(feature)

            # RViz Marker for visual inspection
            marker = Marker()
            marker.header.stamp = current_time.to_msg()
            marker.header.frame_id = self.frame_id
            marker.ns = "mock_obstacles"
            marker.id = int(obs['id'])
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(obs['x'])
            marker.pose.position.y = float(obs['y'])
            marker.pose.position.z = float(obs['height']) / 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = float(obs['radius']) * 2.0
            marker.scale.y = float(obs['radius']) * 2.0
            marker.scale.z = float(obs['height'])
            marker.color.r = 0.9
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.75
            markers.markers.append(marker)

        self.obstacle_pub.publish(array_msg)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = MockPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
