#!/usr/bin/env python3
"""
heuristic_slip_checker.py
=========================
ROS 2 Node that monitors wheel velocity against IMU motion telemetry to detect wheel slip
(e.g., in loose sand or steep terrain).

When wheel slippage is detected (|V_wheels - V_imu| > threshold or |W_wheels - W_imu| > threshold),
it dynamically inflates the covariance matrix of wheel odometry before passing it to EKF,
forcing the local state estimator (robot_localization) to ignore slipping wheel data and rely
on IMU and Visual SLAM (RTAB-Map).

Developer Track: SLAM & Pre-Processing Subsystem
"""

import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


class HeuristicSlipCheckerNode(Node):
    """
    ROS 2 Pre-Filter Node to detect wheel slip by comparing wheel-derived odometry
    against IMU rate readings, dynamically scaling wheel covariance during slippage.

    Attributes:
        slip_vel_threshold (float): Velocity difference threshold (m/s) triggering slip flag.
        slip_rot_threshold (float): Angular rate difference threshold (rad/s) triggering slip.
        covariance_scale (float): Covariance inflation multiplier applied during wheel slip.
    """

    def __init__(self) -> None:
        """Initialize HeuristicSlipCheckerNode, declare parameters, subscribers, and publishers."""
        super().__init__("heuristic_slip_checker")

        # ----------------------------------------------------------------------
        # ROS 2 Parameters Declaration
        # ----------------------------------------------------------------------
        # CUSTOMIZE WITH HARDWARE/TESTING TEAM: Max linear velocity difference (m/s) before flagging sand slip
        self.declare_parameter("slip_velocity_threshold", 0.15)

        # CUSTOMIZE WITH HARDWARE/TESTING TEAM: Max angular rate difference (rad/s) before flagging slip
        self.declare_parameter("slip_angular_threshold", 0.20)

        # CUSTOMIZE WITH SLAM TEAM: Covariance inflation multiplier during slippage (e.g., 100.0)
        self.declare_parameter("covariance_inflation_factor", 100.0)

        # CUSTOMIZE WITH HARDWARE TEAM: Nominal linear velocity covariance variance (m/s)^2
        self.declare_parameter("nominal_linear_covariance", 0.02)

        # CUSTOMIZE WITH HARDWARE TEAM: Nominal angular velocity covariance variance (rad/s)^2
        self.declare_parameter("nominal_angular_covariance", 0.05)

        # Retrieve parameter values
        self.slip_vel_threshold: float = self.get_parameter("slip_velocity_threshold").get_parameter_value().double_value
        self.slip_rot_threshold: float = self.get_parameter("slip_angular_threshold").get_parameter_value().double_value
        self.covariance_scale: float = self.get_parameter("covariance_inflation_factor").get_parameter_value().double_value
        self.nominal_linear_cov: float = self.get_parameter("nominal_linear_covariance").get_parameter_value().double_value
        self.nominal_angular_cov: float = self.get_parameter("nominal_angular_covariance").get_parameter_value().double_value

        # ----------------------------------------------------------------------
        # State Data Tracking
        # ----------------------------------------------------------------------
        self.latest_imu: Optional[Imu] = None
        self.latest_imu_time: Optional[rclpy.time.Time] = None
        self.imu_integrated_vx: float = 0.0
        self.last_imu_calc_time: Optional[rclpy.time.Time] = None
        self.is_slipping: bool = False

        # ----------------------------------------------------------------------
        # ROS 2 Subscribers & Publishers
        # ----------------------------------------------------------------------
        # Subscriptions
        self.sub_wheel_odom = self.create_subscription(
            Odometry,
            "/wheel/odom_raw",
            self._wheel_odom_callback,
            10,
        )

        self.sub_imu = self.create_subscription(
            Imu,
            "/imu/data",
            self._imu_callback,
            10,
        )

        # Publishers
        self.pub_filtered_odom = self.create_publisher(Odometry, "/wheel/odom_filtered", 10)
        self.pub_slip_flag = self.create_publisher(Bool, "/wheel/slip_detected", 10)

        self.get_logger().info("Heuristic Slip Checker Pre-Filter Node Initialized successfully.")

    # --------------------------------------------------------------------------
    # Callbacks & Logic Execution
    # --------------------------------------------------------------------------
    def _imu_callback(self, msg: Imu) -> None:
        """
        Callback to process IMU angular velocity and linear acceleration telemetry.

        Args:
            msg (Imu): Incoming sensor_msgs/Imu message.
        """
        now = self.get_clock().now()
        if self.last_imu_calc_time is not None:
            dt: float = (now - self.last_imu_calc_time).nanoseconds / 1e9
            if dt > 0.0:
                # Integrate forward acceleration along X-axis to estimate IMU linear velocity
                ax: float = msg.linear_acceleration.x
                # Filter out tiny accelerometer bias noise
                if abs(ax) > 0.05:
                    self.imu_integrated_vx += ax * dt

        self.latest_imu = msg
        self.last_imu_calc_time = now

    def _wheel_odom_callback(self, msg: Odometry) -> None:
        """
        Callback to receive raw wheel odometry, evaluate slippage rules against IMU,
        dynamically scale covariance, and publish filtered odometry.

        Args:
            msg (Odometry): Incoming raw wheel odometry message.
        """
        v_wheel: float = msg.twist.twist.linear.x
        w_wheel: float = msg.twist.twist.angular.z

        slip_detected: bool = False

        if self.latest_imu is not None:
            w_imu: float = self.latest_imu.angular_velocity.z

            # Slip Rule 1: Angular velocity disagreement between wheels and IMU gyro
            rot_diff: float = abs(w_wheel - w_imu)

            # Slip Rule 2: Linear velocity disagreement (wheel speed vs integrated IMU speed estimate)
            vel_diff: float = abs(v_wheel - self.imu_integrated_vx)

            if rot_diff > self.slip_rot_threshold or vel_diff > self.slip_vel_threshold:
                slip_detected = True

        self.is_slipping = slip_detected

        # Construct output odometry message with updated covariance
        filtered_odom = Odometry()
        filtered_odom.header = msg.header
        filtered_odom.child_frame_id = msg.child_frame_id
        filtered_odom.pose = msg.pose
        filtered_odom.twist = msg.twist

        # Apply dynamic covariance scaling during slippage
        multiplier: float = self.covariance_scale if slip_detected else 1.0

        linear_cov: float = self.nominal_linear_cov * multiplier
        angular_cov: float = self.nominal_angular_cov * multiplier

        # Pose Covariance
        filtered_odom.pose.covariance[0] = 0.01 * multiplier
        filtered_odom.pose.covariance[7] = 0.01 * multiplier
        filtered_odom.pose.covariance[35] = 0.05 * multiplier

        # Twist Covariance
        filtered_odom.twist.covariance[0] = linear_cov
        filtered_odom.twist.covariance[7] = linear_cov
        filtered_odom.twist.covariance[35] = angular_cov

        # Publish filtered odometry and slip telemetry status flag
        self.pub_filtered_odom.publish(filtered_odom)

        slip_msg = Bool()
        slip_msg.data = slip_detected
        self.pub_slip_flag.publish(slip_msg)

        if slip_detected:
            self.get_logger().warn(
                f"Wheel Slip Detected! Inflating wheel covariance by factor of {self.covariance_scale:.1f}x"
            )


def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for starting the Heuristic Slip Checker Node.

    Args:
        args (Optional[List[str]]): Arguments passed to rclpy initialization.
    """
    rclpy.init(args=args)
    node = HeuristicSlipCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

