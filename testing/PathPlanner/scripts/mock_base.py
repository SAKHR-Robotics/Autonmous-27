#!/usr/bin/env python3
"""
Mock Base Node - TF & Odometry Simulator for Path Planning
Simulates robot kinematics, publishes transforms (map -> odom -> base_link),
and broadcasts odometry (/odom, /odometry/filtered) based on Nav2 velocity commands.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class MockBase(Node):
    def __init__(self):
        super().__init__('mock_base')
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.wz = 0.0

        # Subscriptions for cmd_vel (direct MPPI output and standard cmd_vel)
        self.sub = self.create_subscription(
            TwistStamped, '/cmd_vel_nav', self.cb, 10
        )
        self.sub_fallback = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cb, 10
        )

        # Odometry Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.odom_filt_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)

        # TF Broadcasters
        self.br = TransformBroadcaster(self)
        self.sbr = StaticTransformBroadcaster(self)

        self.last_t = time.time()
        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info('🚀 Mock Base (TF & Odometry Simulator) is running and ready for Nav2 goals!')

    def cb(self, msg: TwistStamped):
        self.vx = msg.twist.linear.x
        self.wz = msg.twist.angular.z

    def update(self):
        now = time.time()
        dt = min(now - self.last_t, 0.1)
        self.last_t = now

        # Dead Reckoning Kinematics
        self.yaw += self.wz * dt
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        now_msg = self.get_clock().now().to_msg()

        # 1. Map -> Odom Static Transform
        st = TransformStamped()
        st.header.stamp = now_msg
        st.header.frame_id = 'map'
        st.child_frame_id = 'odom'
        self.sbr.sendTransform(st)

        # 2. Odom -> Base_link Dynamic Transform
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.br.sendTransform(t)

        # 3. Odometry Messages
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)
        self.odom_filt_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MockBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
