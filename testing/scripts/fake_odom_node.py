#!/usr/bin/env python3
"""
Fake odometry node for closed-loop Nav2 testing (no real hardware).

Models a tank-style / skid-steer 4-wheel rover using a unicycle kinematic
model: it consumes /cmd_vel (linear.x, angular.z only -- lateral velocity
is always forced to zero, since skid-steer cannot strafe), integrates pose
over time, and publishes:
  - nav_msgs/Odometry on /odometry/filtered and /odom
  - a dynamic TF transform odom -> base_link

This is NOT a physics simulation. It has zero wheel slip, zero latency,
zero sensor noise. It only proves the software chain (planner -> controller
-> smoother -> collision_monitor -> "motors") is wired correctly and that
MPPI's commands drive the rover toward the goal in a sane way.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom_node')

        # Pose state (in the "odom" frame)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Last received command
        self.vx = 0.0
        self.wz = 0.0

        self.last_time = self.get_clock().now()

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 50.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate = self.get_parameter('publish_rate_hz').value

        self.sub = self.create_subscription(
            TwistStamped, cmd_vel_topic, self.cmd_vel_callback, 10)

        # Publish to both names since different nav2 nodes default to
        # different odom topic names depending on version/config.
        self.odom_pub_filtered = self.create_publisher(
            Odometry, '/odometry/filtered', 10)
        self.odom_pub_plain = self.create_publisher(
            Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / rate, self.update)

        self.get_logger().info(
            f'Fake odom node started. Subscribing to {cmd_vel_topic}, '
            f'publishing TF {self.odom_frame} -> {self.base_frame}'
        )

    def cmd_vel_callback(self, msg: TwistStamped):
        # Tank/skid-steer rover: only forward velocity + yaw rate exist.
        # Explicitly ignore/zero any lateral component even if present.
        self.vx = msg.twist.linear.x
        self.wz = msg.twist.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0:
            return

        # Unicycle integration (vy forced to 0.0 -- no strafing)
        delta_theta = self.wz * dt
        self.theta += delta_theta
        # normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.x += self.vx * math.cos(self.theta) * dt
        self.y += self.vx * math.sin(self.theta) * dt

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # --- TF: odom -> base_link ---
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # --- Odometry message ---
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.wz

        self.odom_pub_filtered.publish(odom)
        self.odom_pub_plain.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
