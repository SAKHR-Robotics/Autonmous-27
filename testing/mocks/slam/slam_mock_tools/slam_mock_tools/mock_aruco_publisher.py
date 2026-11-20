#!/usr/bin/env python3
"""Smoke landmark fixture with configurable current timestamps and valid PSD covariance."""
import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import LandmarkDetection, LandmarkDetections


class MockArucoPublisherNode(Node):
    def __init__(self):
        super().__init__('mock_aruco_publisher')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('target_frame', 'camera_link')
        self.declare_parameter('landmark_id', 1)
        self.declare_parameter('pos_x', 1.5)
        self.declare_parameter('pos_y', 0.2)
        self.declare_parameter('pos_z', 0.0)
        self.declare_parameter('position_variance', 0.01)
        self.declare_parameter('orientation_variance', 0.02)
        self.declare_parameter('use_current_stamp', True)
        self.declare_parameter('publish_array', False)

        self.rate = float(self.get_parameter('publish_rate').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.landmark_id = int(self.get_parameter('landmark_id').value)
        self.pos_x = float(self.get_parameter('pos_x').value)
        self.pos_y = float(self.get_parameter('pos_y').value)
        self.pos_z = float(self.get_parameter('pos_z').value)
        self.pos_var = float(self.get_parameter('position_variance').value)
        self.ori_var = float(self.get_parameter('orientation_variance').value)
        self.use_current_stamp = bool(self.get_parameter('use_current_stamp').value)
        self.publish_array = bool(self.get_parameter('publish_array').value)

        self.publisher_ = self.create_publisher(LandmarkDetection, '/perception/landmark_detection', 10)
        self.array_pub = (
            self.create_publisher(LandmarkDetections, '/perception/landmark_detections', 10)
            if self.publish_array
            else None
        )
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info(
            f"Mock ArUco Publisher Started (Rate: {self.rate} Hz, Frame: '{self.target_frame}', "
            f"ID: {self.landmark_id}, Pose: [{self.pos_x}, {self.pos_y}, {self.pos_z}])."
        )

    def build_message(self):
        msg = LandmarkDetection()
        msg.id = self.landmark_id
        msg.header.stamp = (
            self.get_clock().now().to_msg() if self.use_current_stamp else rclpy.time.Time().to_msg()
        )
        msg.header.frame_id = self.target_frame

        msg.pose.pose.position.x = self.pos_x
        msg.pose.pose.position.y = self.pos_y
        msg.pose.pose.position.z = self.pos_z
        msg.pose.pose.orientation.w = 1.0

        cov = [0.0] * 36
        cov[0] = cov[7] = cov[14] = max(1e-6, self.pos_var)
        cov[21] = cov[28] = cov[35] = max(1e-6, self.ori_var)
        msg.pose.covariance = cov
        return msg

    def timer_callback(self):
        msg = self.build_message()
        self.publisher_.publish(msg)
        if self.array_pub is not None:
            arr = LandmarkDetections()
            arr.header = msg.header
            arr.landmarks = [msg]
            self.array_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = MockArucoPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
