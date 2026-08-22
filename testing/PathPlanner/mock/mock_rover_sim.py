#!/usr/bin/env python3
"""
mock_rover_sim.py

Real-Time Rover Kinematics Simulator & TF/Odometry Broadcaster.
Simulates a differential/skid-steer rover in response to Nav2 MPPI /cmd_vel commands.
Broadcasts continuous TF transforms (map -> odom -> base_link) and publishes
/odometry/filtered at 50 Hz to enable standalone closed-loop path tracking in RViz.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, TransformStamped, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class MockRoverSim(Node):
    def __init__(self):
        super().__init__('mock_rover_sim')

        # Parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('rover_length', 0.8)
        self.declare_parameter('rover_width', 0.6)
        self.declare_parameter('rover_height', 0.3)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.update_rate = self.get_parameter('update_rate').value

        self.rover_length = self.get_parameter('rover_length').value
        self.rover_width = self.get_parameter('rover_width').value
        self.rover_height = self.get_parameter('rover_height').value

        # Robot State (in odom / map coordinates)
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)

        # Velocities
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Command timeout handling
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/rover_marker', 10)

        # Subscribers
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )

        self.set_pose_sub = self.create_subscription(
            PoseStamped,
            '/start_pose',
            self.set_pose_callback,
            10
        )

        # Simulation Timer (e.g. 50 Hz)
        dt = 1.0 / self.update_rate
        self.timer = self.create_timer(dt, self.update_physics)

        self.last_time = self.get_clock().now()
        self.get_logger().info(
            f"Mock Rover Sim initialized at ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f} rad). Rate: {self.update_rate}Hz"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.get_logger().info(f"Rover reset via /initialpose to ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f})")

    def set_pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.get_logger().info(f"Rover reset via /start_pose to ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f})")

    def update_physics(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.update_rate

        # Stop robot if no commands received recently
        cmd_age = (current_time - self.last_cmd_time).nanoseconds * 1e-9
        if cmd_age > self.cmd_timeout:
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0

        # Kinematic integration (Unicycle model with lateral velocity support)
        self.yaw += self.wz * dt
        # Normalize yaw to [-pi, pi]
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        dx = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        dy = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt

        self.x += dx
        self.y += dy

        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, self.yaw)

        # 1. Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)

        # 2. Broadcast TF transforms
        if self.publish_tf:
            # map -> odom (static identity frame alignment)
            t_map_odom = TransformStamped()
            t_map_odom.header.stamp = current_time.to_msg()
            t_map_odom.header.frame_id = self.map_frame
            t_map_odom.child_frame_id = self.odom_frame
            t_map_odom.transform.translation.x = 0.0
            t_map_odom.transform.translation.y = 0.0
            t_map_odom.transform.translation.z = 0.0
            t_map_odom.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t_map_odom)

            # odom -> base_link (dynamic pose transform)
            t_odom_base = TransformStamped()
            t_odom_base.header.stamp = current_time.to_msg()
            t_odom_base.header.frame_id = self.odom_frame
            t_odom_base.child_frame_id = self.base_frame
            t_odom_base.transform.translation.x = self.x
            t_odom_base.transform.translation.y = self.y
            t_odom_base.transform.translation.z = 0.15
            t_odom_base.transform.rotation.x = qx
            t_odom_base.transform.rotation.y = qy
            t_odom_base.transform.rotation.z = qz
            t_odom_base.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t_odom_base)

        # 3. Publish visual rover marker for RViz
        self.publish_rover_markers(current_time)

    def publish_rover_markers(self, stamp: Time):
        markers = MarkerArray()

        # Chassis Box
        chassis = Marker()
        chassis.header.stamp = stamp.to_msg()
        chassis.header.frame_id = self.base_frame
        chassis.ns = "rover_chassis"
        chassis.id = 0
        chassis.type = Marker.CUBE
        chassis.action = Marker.ADD
        chassis.pose.position.x = 0.0
        chassis.pose.position.y = 0.0
        chassis.pose.position.z = self.rover_height / 2.0
        chassis.pose.orientation.w = 1.0
        chassis.scale.x = self.rover_length
        chassis.scale.y = self.rover_width
        chassis.scale.z = self.rover_height
        chassis.color.r = 0.85
        chassis.color.g = 0.35
        chassis.color.b = 0.15
        chassis.color.a = 0.85
        markers.markers.append(chassis)

        # Heading Arrow
        arrow = Marker()
        arrow.header.stamp = stamp.to_msg()
        arrow.header.frame_id = self.base_frame
        arrow.ns = "rover_heading"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = 0.0
        arrow.pose.position.y = 0.0
        arrow.pose.position.z = self.rover_height + 0.05
        arrow.pose.orientation.w = 1.0
        arrow.scale.x = self.rover_length * 0.7
        arrow.scale.y = 0.06
        arrow.scale.z = 0.06
        arrow.color.r = 0.1
        arrow.color.g = 0.9
        arrow.color.b = 0.2
        arrow.color.a = 0.9
        markers.markers.append(arrow)

        # Wheels (4 corner cylinders)
        wheel_offsets = [
            (self.rover_length * 0.35, self.rover_width * 0.55),
            (self.rover_length * 0.35, -self.rover_width * 0.55),
            (-self.rover_length * 0.35, self.rover_width * 0.55),
            (-self.rover_length * 0.35, -self.rover_width * 0.55)
        ]

        for i, (wx, wy) in enumerate(wheel_offsets):
            wheel = Marker()
            wheel.header.stamp = stamp.to_msg()
            wheel.header.frame_id = self.base_frame
            wheel.ns = "rover_wheels"
            wheel.id = 10 + i
            wheel.type = Marker.CYLINDER
            wheel.action = Marker.ADD
            wheel.pose.position.x = wx
            wheel.pose.position.y = wy
            wheel.pose.position.z = 0.05
            # Rotate cylinder to lie horizontally along Y
            qx, qy, qz, qw = euler_to_quaternion(math.pi / 2.0, 0.0, 0.0)
            wheel.pose.orientation.x = qx
            wheel.pose.orientation.y = qy
            wheel.pose.orientation.z = qz
            wheel.pose.orientation.w = qw
            wheel.scale.x = 0.2
            wheel.scale.y = 0.2
            wheel.scale.z = 0.08
            wheel.color.r = 0.15
            wheel.color.g = 0.15
            wheel.color.b = 0.15
            wheel.color.a = 0.95
            markers.markers.append(wheel)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = MockRoverSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
