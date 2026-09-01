#!/usr/bin/env python3
"""
Mock Base Node - TF & Odometry Simulator for Path Planning (Stable)
Simulates robot kinematics, publishes transforms (map -> odom -> base_link),
and broadcasts odometry (/odom, /odometry/filtered) based on Nav2 velocity commands.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, TransformStamped, PoseWithCovarianceStamped
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
        self.last_cmd_time = time.time()

        # 1. Subscriptions for cmd_vel (direct MPPI output and standard fallback)
        self.sub = self.create_subscription(
            TwistStamped, '/cmd_vel_nav', self.cb, 10
        )
        self.sub_fallback = self.create_subscription(
            TwistStamped, '/cmd_vel', self.cb, 10
        )

        # 2. Support for RViz "2D Pose Estimate"
        self.init_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_cb, 10
        )

        # 3. Odometry Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.odom_filt_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)

        # 4. TF Broadcasters
        self.br = TransformBroadcaster(self)
        self.sbr = StaticTransformBroadcaster(self)

        # Publish Map -> Odom Static Transform ONCE
        self.publish_static_map_odom()

        self.last_t = time.time()
        self.timer = self.create_timer(0.05, self.update)
        self.get_logger().info('🚀 Mock Base (Stable TF & Odometry) is running and ready for Nav2 goals!')

    def publish_static_map_odom(self):
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'map'
        st.child_frame_id = 'odom'
        st.transform.rotation.w = 1.0
        self.sbr.sendTransform(st)

    def initial_pose_cb(self, msg: PoseWithCovarianceStamped):
        """Allows resetting robot position from RViz 2D Pose Estimate"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        self.yaw = 2.0 * math.atan2(q_z, q_w)
        self.vx = 0.0
        self.wz = 0.0
        self.get_logger().info(f'📍 Reset position to ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f} rad)')

    def cb(self, msg: TwistStamped):
        self.vx = msg.twist.linear.x
        self.wz = msg.twist.angular.z
        self.last_cmd_time = time.time()

    def update(self):
        now = time.time()
        dt = min(now - self.last_t, 0.1)
        self.last_t = now

        # Safety Watchdog: Stop if no command received for 0.5s
        if now - self.last_cmd_time > 0.5:
            self.vx = 0.0
            self.wz = 0.0

        # Kinematics Update
        self.yaw += self.wz * dt
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        now_msg = self.get_clock().now().to_msg()

        # Odom -> Base_link Dynamic Transform
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        self.br.sendTransform(t)

        # Odometry Message
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
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
