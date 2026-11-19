#!/usr/bin/env python3
"""
mock_perception.py

Universal Mock Perception Node for Terrain Obstacle Generation.
Simulates sensor perceptions (LiDAR, Depth Camera, and 3D Obstacle Detectors)
for autonomous path planners and controllers.

Publishes:
- sensor_msgs/msg/PointCloud2 on /bridge/pointcloud and /mock_perception/pointcloud
- sensor_msgs/msg/LaserScan on /scan
- terrain_geometry_msgs/msg/ObstacleFeatureArray on /terrain/obstacle_features
- visualization_msgs/msg/MarkerArray on /perception/obstacle_markers

Supports:
- Timed dynamic obstacle injection (mid-run pop-up rocks to test replanning)
- Interactive obstacle spawning via /mock_perception/spawn_obstacle
- Scenario loading from scenarios.yaml
"""

import os
import math
import yaml
import struct
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from terrain_geometry_msgs.msg import ObstacleFeature, ObstacleFeatureArray


class MockPerception(Node):
    def __init__(self):
        super().__init__('mock_perception')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('scenario_id', '')
        self.declare_parameter('scenarios_file', 'config/scenarios.yaml')
        self.declare_parameter('enable_dynamic_spawning', True)
        self.declare_parameter('scan_frame_id', 'base_link')
        self.declare_parameter('publish_bridge_cloud', False)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.scenario_id = self.get_parameter('scenario_id').value
        self.scenarios_file = self.get_parameter('scenarios_file').value
        self.enable_dynamic_spawning = self.get_parameter('enable_dynamic_spawning').value
        self.scan_frame_id = self.get_parameter('scan_frame_id').value
        self.publish_bridge_cloud = bool(self.get_parameter('publish_bridge_cloud').value)

        # Initial default obstacles
        self.obstacles = [
            {'id': 1, 'x': 1.0, 'y': 1.0, 'z': 0.15, 'radius': 0.35, 'height': 0.4, 'class': 'rock', 'conf': 0.95},
            {'id': 2, 'x': -1.5, 'y': 0.5, 'z': 0.10, 'radius': 0.25, 'height': 0.3, 'class': 'crater', 'conf': 0.90},
            {'id': 3, 'x': 2.5, 'y': 2.0, 'z': 0.20, 'radius': 0.45, 'height': 0.5, 'class': 'boulder', 'conf': 0.98},
        ]

        # Scheduled dynamic obstacles from scenario
        self.pending_dynamic_obstacles = []
        self.load_scenario_obstacles()

        # Publishers
        self.feature_pub = self.create_publisher(
            ObstacleFeatureArray,
            '/terrain/obstacle_features',
            10
        )
        if self.publish_bridge_cloud:
            self.cloud_bridge_pub = self.create_publisher(
                PointCloud2,
                '/bridge/pointcloud',
                10
            )
        else:
            self.cloud_bridge_pub = None

        self.cloud_mock_pub = self.create_publisher(
            PointCloud2,
            '/mock_perception/pointcloud',
            10
        )
        self.scan_pub = self.create_publisher(
            LaserScan,
            '/scan',
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/perception/obstacle_markers',
            10
        )

        # On-demand obstacle spawn subscriber
        self.spawn_sub = self.create_subscription(
            Point,
            '/mock_perception/spawn_obstacle',
            self.spawn_obstacle_callback,
            10
        )
        # RViz 'Publish Point' interactive obstacle spawner
        self.clicked_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        # Timer
        self.start_time = self.get_clock().now()
        dt = 1.0 / self.publish_rate
        self.timer = self.create_timer(dt, self.publish_perception_data)

        self.get_logger().info(
            f"Universal Mock Perception Node initialized ({self.publish_rate} Hz). "
            f"Frame: {self.frame_id}, Scan Frame: {self.scan_frame_id}"
        )

    def load_scenario_obstacles(self):
        """Loads dynamic obstacles from scenarios.yaml if matching scenario_id."""
        if not self.scenario_id or not os.path.exists(self.scenarios_file):
            return

        try:
            with open(self.scenarios_file, 'r') as f:
                data = yaml.safe_load(f)
                scenarios = data.get('scenarios', [])
                for s in scenarios:
                    if s.get('id') == self.scenario_id:
                        for dyn in s.get('dynamic_obstacles', []):
                            self.pending_dynamic_obstacles.append({
                                'trigger_time': float(dyn.get('trigger_time', dyn.get('trigger_time_sec', 1.0))),
                                'x': float(dyn.get('x', 0.0)),
                                'y': float(dyn.get('y', 0.0)),
                                'z': float(dyn.get('z', 0.2)),
                                'radius': float(dyn.get('radius', 0.4)),
                                'height': float(dyn.get('height', 0.5)),
                                'class': 'dynamic_hazard',
                                'conf': 0.99,
                                'spawned': False
                            })
                        self.get_logger().info(
                            f"Loaded {len(self.pending_dynamic_obstacles)} dynamic obstacles for scenario '{self.scenario_id}'"
                        )
                        break
        except Exception as e:
            self.get_logger().warn(f"Could not load dynamic obstacles: {e}")

    def spawn_obstacle_callback(self, msg: Point):
        new_id = len(self.obstacles) + 1
        obs = {
            'id': new_id,
            'x': float(msg.x),
            'y': float(msg.y),
            'z': float(msg.z) if msg.z != 0.0 else 0.20,
            'radius': 0.35,
            'height': 0.45,
            'class': 'dynamic_rock',
            'conf': 0.99
        }
        self.obstacles.append(obs)
        self.get_logger().info(f"🪨 Dynamically spawned obstacle #{new_id} at ({msg.x:.2f}, {msg.y:.2f})")

    def clicked_point_callback(self, msg: PointStamped):
        pt = Point(x=float(msg.point.x), y=float(msg.point.y), z=float(msg.point.z) if msg.point.z != 0.0 else 0.20)
        self.get_logger().info(f"🖱️ Clicked point received from RViz at ({pt.x:.2f}, {pt.y:.2f})")
        self.spawn_obstacle_callback(pt)

    def check_dynamic_triggers(self, elapsed_sec: float):
        """Triggers scheduled obstacles based on elapsed scenario time."""
        if not self.enable_dynamic_spawning:
            return

        for dyn in self.pending_dynamic_obstacles:
            if not dyn['spawned'] and elapsed_sec >= dyn['trigger_time']:
                dyn['spawned'] = True
                new_id = len(self.obstacles) + 1
                obs = {
                    'id': new_id,
                    'x': dyn['x'],
                    'y': dyn['y'],
                    'z': dyn['z'],
                    'radius': dyn['radius'],
                    'height': dyn['height'],
                    'class': dyn['class'],
                    'conf': dyn['conf']
                }
                self.obstacles.append(obs)
                self.get_logger().info(
                    f"⏰ Triggered dynamic obstacle {new_id} at t={elapsed_sec:.2f}s: ({dyn['x']:.2f}, {dyn['y']:.2f})"
                )

    def publish_perception_data(self):
        current_time = self.get_clock().now()
        elapsed_sec = (current_time - self.start_time).nanoseconds * 1e-9
        self.check_dynamic_triggers(elapsed_sec)

        # 1. Build & Publish ObstacleFeatureArray
        array_msg = ObstacleFeatureArray()
        array_msg.header.stamp = current_time.to_msg()
        array_msg.header.frame_id = self.frame_id

        markers = MarkerArray()
        point_bytes = bytearray()
        num_points = 0

        # Sample points on cylinder boundaries for 3D point cloud
        samples_per_circle = 16

        for obs in self.obstacles:
            cx, cy, cz = float(obs['x']), float(obs['y']), float(obs['z'])
            r, h = float(obs['radius']), float(obs['height'])

            feature = ObstacleFeature()
            feature.id = int(obs['id'])
            feature.num_points = 50
            feature.centroid.x = cx
            feature.centroid.y = cy
            feature.centroid.z = cz
            feature.min_point.x = cx - r
            feature.min_point.y = cy - r
            feature.min_point.z = cz - h / 2.0
            feature.max_point.x = cx + r
            feature.max_point.y = cy + r
            feature.max_point.z = cz + h / 2.0
            feature.width = float(2.0 * r)
            feature.height = float(h)
            feature.depth = float(2.0 * r)
            feature.distance = float(math.sqrt(cx**2 + cy**2))
            array_msg.obstacles.append(feature)

            # Generate boundary points for PointCloud2
            cx, cy, cz = float(obs['x']), float(obs['y']), float(obs['z'])
            r, h = float(obs['radius']), float(obs['height'])
            for theta in np.linspace(0, 2 * math.pi, samples_per_circle, endpoint=False):
                px = cx + r * math.cos(theta)
                py = cy + r * math.sin(theta)
                # Sample bottom, mid, and top
                for z_layer in [cz - h/2.0, cz, cz + h/2.0]:
                    point_bytes.extend(struct.pack('fff', px, py, z_layer))
                    num_points += 1

            # Centroid point
            point_bytes.extend(struct.pack('fff', cx, cy, cz))
            num_points += 1

            # RViz Marker
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

        self.feature_pub.publish(array_msg)
        self.marker_pub.publish(markers)

        # 2. Build & Publish PointCloud2
        if num_points > 0:
            cloud_msg = PointCloud2()
            cloud_msg.header.stamp = current_time.to_msg()
            cloud_msg.header.frame_id = self.frame_id
            cloud_msg.height = 1
            cloud_msg.width = num_points
            cloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            cloud_msg.is_bigendian = False
            cloud_msg.point_step = 12
            cloud_msg.row_step = 12 * num_points
            cloud_msg.is_dense = True
            cloud_msg.data = bytes(point_bytes)

            if self.cloud_bridge_pub is not None:
                self.cloud_bridge_pub.publish(cloud_msg)
            self.cloud_mock_pub.publish(cloud_msg)

        # 3. Build & Publish 2D LaserScan
        scan_msg = LaserScan()
        scan_msg.header.stamp = current_time.to_msg()
        scan_msg.header.frame_id = self.scan_frame_id
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = math.radians(1.0)  # 1 degree resolution (360 beams)
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0 / self.publish_rate
        scan_msg.range_min = 0.1
        scan_msg.range_max = 20.0

        num_beams = int(round((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment))
        ranges = [scan_msg.range_max] * num_beams

        for obs in self.obstacles:
            ox, oy = float(obs['x']), float(obs['y'])
            r = float(obs['radius'])
            dist_center = math.sqrt(ox**2 + oy**2)
            if dist_center <= r:
                continue

            angle_center = math.atan2(oy, ox)
            angular_span = math.asin(min(1.0, r / dist_center))

            beam_start = int(round((angle_center - angular_span - scan_msg.angle_min) / scan_msg.angle_increment))
            beam_end = int(round((angle_center + angular_span - scan_msg.angle_min) / scan_msg.angle_increment))

            dist_surface = max(scan_msg.range_min, dist_center - r)
            for b in range(max(0, beam_start), min(num_beams, beam_end + 1)):
                if dist_surface < ranges[b]:
                    ranges[b] = dist_surface

        scan_msg.ranges = ranges
        self.scan_pub.publish(scan_msg)


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
