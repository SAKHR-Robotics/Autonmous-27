#!/usr/bin/env python3
"""Standalone simulation guard; not the future autonomous command selector."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TeleopGuard(Node):
    def __init__(self):
        super().__init__('gui_teleop_guard')
        self.declare_parameter('timeout_sec', 0.3)
        self.declare_parameter('max_linear_mps', 0.5)
        self.declare_parameter('max_yaw_radps', 1.0)
        self.timeout = float(self.get_parameter('timeout_sec').value)
        self.max_linear = float(self.get_parameter('max_linear_mps').value)
        self.max_yaw = float(self.get_parameter('max_yaw_radps').value)
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.timeout, self.max_linear, self.max_yaw)):
            raise ValueError('Timeout and velocity limits must be positive and finite')
        self.command = Twist()
        self.received_at = float('-inf')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(Twist, '/cmd_vel_manual', self.receive, 1)
        self.timer = self.create_timer(0.05, self.publish_command)
        self.get_logger().info('Manual-only simulation: /cmd_vel_manual -> /cmd_vel; Nav2 must be stopped')

    def receive(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        self.command = Twist()
        if all(math.isfinite(v) for v in values) and all(
                v == 0 for v in (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)):
            self.command.linear.x = max(-self.max_linear, min(self.max_linear, msg.linear.x))
            self.command.angular.z = max(-self.max_yaw, min(self.max_yaw, msg.angular.z))
            self.received_at = time.monotonic()
        else:
            self.received_at = float('-inf')

    def publish_command(self):
        fresh = time.monotonic() - self.received_at <= self.timeout
        self.publisher.publish(self.command if fresh else Twist())


def main():
    rclpy.init()
    node = TeleopGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
