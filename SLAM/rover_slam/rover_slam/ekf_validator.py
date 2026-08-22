#!/usr/bin/env python3
"""
ekf_validator.py
================
ROS 2 Node that validates the quality and continuity of the local EKF odometry output.

This node subscribes to the filtered odometry topic produced by robot_localization,
calculates message cadence and speed continuity, and publishes a validation flag so that
operators can quickly confirm the local estimate remains smooth and stable.

Developer Track: SLAM & State Estimation Validation
"""

import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64


class EKFValidatorNode(Node):
    """
    ROS 2 validation helper for robot_localization EKF odometry output.

    Checks:
      - Output publication frequency remains near the configured target.
      - Message timestamp gaps are not excessive.
      - Filtered velocity remains within a realistic operating envelope.
      - Node publishes a boolean health indicator for downstream monitoring.
    """

    def __init__(self) -> None:
        """Initialize the EKFValidatorNode, declare parameters, subscribers, and publishers."""
        super().__init__("ekf_validator")

        # ----------------------------------------------------------------------
        # ROS 2 Parameters Declaration
        # ----------------------------------------------------------------------
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("expected_frequency_hz", 100.0)
        self.declare_parameter("max_gap_seconds", 0.25)
        self.declare_parameter("max_linear_speed_mps", 3.0)

        self.odom_topic: str = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.expected_frequency_hz: float = self.get_parameter("expected_frequency_hz").get_parameter_value().double_value
        self.max_gap_seconds: float = self.get_parameter("max_gap_seconds").get_parameter_value().double_value
        self.max_linear_speed_mps: float = self.get_parameter("max_linear_speed_mps").get_parameter_value().double_value

        # ----------------------------------------------------------------------
        # State Tracking
        # ----------------------------------------------------------------------
        self.last_odom_time: Optional[rclpy.time.Time] = None
        self.message_count: int = 0
        self.last_speed_mps: float = 0.0
        self.last_valid_status: bool = True

        # ----------------------------------------------------------------------
        # ROS 2 Subscribers & Publishers
        # ----------------------------------------------------------------------
        self.sub_odom = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            10,
        )

        self.pub_status = self.create_publisher(Bool, "/ekf/validation_status", 10)
        self.pub_frequency = self.create_publisher(Float64, "/ekf/validation_frequency_hz", 10)

        self.get_logger().info(
            "EKF validator initialized. Monitoring filtered odometry on "
            f"{self.odom_topic} with expected frequency {self.expected_frequency_hz:.1f} Hz."
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """
        Callback to verify that the EKF output is continuous and within expected limits.

        Args:
            msg (Odometry): Incoming filtered odometry from robot_localization.
        """
        self.message_count += 1

        now = self.get_clock().now()
        current_time = Time.from_msg(msg.header.stamp) if (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0) else now

        speed_mps = math.sqrt(
            msg.twist.twist.linear.x ** 2 +
            msg.twist.twist.linear.y ** 2 +
            msg.twist.twist.linear.z ** 2
        )

        if self.last_odom_time is not None:
            dt_seconds = (current_time - self.last_odom_time).nanoseconds / 1e9
            if dt_seconds > 0.0:
                measured_frequency = 1.0 / dt_seconds
                freq_msg = Float64()
                freq_msg.data = measured_frequency
                self.pub_frequency.publish(freq_msg)

                if dt_seconds > self.max_gap_seconds:
                    self.last_valid_status = False
                    self.get_logger().warn(
                        "EKF odometry gap too large: %.3f s > %.3f s",
                        dt_seconds,
                        self.max_gap_seconds,
                    )
                elif abs(measured_frequency - self.expected_frequency_hz) > (self.expected_frequency_hz * 0.35):
                    self.last_valid_status = False
                    self.get_logger().warn(
                        "EKF frequency deviates from expected: %.2f Hz expected %.2f Hz",
                        measured_frequency,
                        self.expected_frequency_hz,
                    )
                elif speed_mps > self.max_linear_speed_mps:
                    self.last_valid_status = False
                    self.get_logger().warn(
                        "EKF filtered speed is unrealistic: %.3f m/s > %.3f m/s",
                        speed_mps,
                        self.max_linear_speed_mps,
                    )
                else:
                    self.last_valid_status = True

        self.last_odom_time = current_time
        self.last_speed_mps = speed_mps

        status_msg = Bool()
        status_msg.data = self.last_valid_status
        self.pub_status.publish(status_msg)

        if self.message_count % 50 == 0:
            self.get_logger().info(
                "EKF validation update: message #%d, speed=%.3f m/s, status=%s",
                self.message_count,
                speed_mps,
                "OK" if self.last_valid_status else "WARN",
            )


def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for starting the EKF validation node.

    Args:
        args (Optional[List[str]]): Arguments passed to rclpy initialization.
    """
    rclpy.init(args=args)
    node = EKFValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
