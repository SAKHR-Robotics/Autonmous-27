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
from typing import Optional, List, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool
except ImportError:
    rclpy = None
    Node = object  # type: ignore
    Odometry = None  # type: ignore
    Imu = None  # type: ignore
    Bool = None  # type: ignore


class SlipCheckerCore:
    """
    Deterministic core logic for slip detection and covariance scaling.
    Decoupled from ROS node to enable unit testing.
    """

    def __init__(
        self,
        slip_rot_threshold: float = 0.35,
        slip_accel_threshold: float = 0.18,
        slip_vel_threshold: float = 0.20,
        covariance_scale: float = 100.0,
        nominal_linear_cov: float = 0.02,
        nominal_angular_cov: float = 0.05,
        slip_hold_time: float = 0.4,
    ) -> None:
        self.slip_rot_threshold = slip_rot_threshold
        self.slip_accel_threshold = slip_accel_threshold
        self.slip_vel_threshold = slip_vel_threshold
        self.covariance_scale = covariance_scale
        self.nominal_linear_cov = nominal_linear_cov
        self.nominal_angular_cov = nominal_angular_cov
        self.slip_hold_time = slip_hold_time

        self.last_slip_time: float = -100.0
        self.is_slipping: bool = False
        self.last_slip_reason: str = "None"

        # Motion & obstacle tracking
        self.last_v_wheel: Optional[float] = None
        self.obstacle_blocked: bool = False
        self.blocked_direction: int = 0
        self.takeoff_start_time: Optional[float] = None
        self.takeoff_accel_seen: bool = True

    def evaluate(
        self,
        v_wheel: float,
        w_wheel: float,
        w_imu: float,
        a_imu_x: float,
        current_time_sec: float,
        orientation_q: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
        w_imu_y: float = 0.0,
        single_wheel_slip: bool = False,
    ) -> Tuple[bool, float, float]:
        """
        Evaluate wheel motion against IMU telemetry without open-loop drift.
        Compensates for gravity leakage on sloped terrain using orientation quaternion.

        Args:
            v_wheel (float): Wheel linear velocity (m/s).
            w_wheel (float): Wheel angular velocity (rad/s).
            w_imu (float): IMU calibrated angular velocity around z-axis (rad/s).
            a_imu_x (float): IMU forward linear acceleration along x-axis (m/s^2).
            current_time_sec (float): Timestamp in seconds.
            orientation_q (Tuple[float, float, float, float]): IMU orientation quaternion (x, y, z, w).
            w_imu_y (float): IMU angular velocity around pitch y-axis (rad/s).
            single_wheel_slip (bool): Intra-side single-wheel slip flag from encoder node.

        Returns:
            Tuple[bool, float, float]: (slip_detected, linear_covariance, angular_covariance)
        """
        raw_slip = False
        reason = "None"

        # Rule 0: Intra-side wheel slip reported by encoder odometry
        if single_wheel_slip:
            raw_slip = True
            reason = "SingleWheelSlip"

        # Rule 1: Angular velocity (yaw) slip detection
        # Normal turning on open ground: wheels and chassis rotate in the same direction with genuine rotation (|w_imu| > 0.10)
        # Blocked turning against obstacle: wheels commanded to turn (|w_wheel| > 0.25), but chassis cannot rotate (|w_imu| < 0.10 or opposite)
        if not raw_slip:
            if abs(w_wheel) > 0.25 and (abs(w_imu) < 0.10 or (w_wheel * w_imu < -0.05)):
                raw_slip = True
                reason = f"YawBlocked (w_wheel={w_wheel:.2f}, w_imu={w_imu:.2f})"
            elif abs(w_wheel) < 0.10 and abs(w_imu) > 0.40:
                # Chassis spinning out with stationary wheels
                raw_slip = True
                reason = f"UncontrolledYawSpin (w_imu={w_imu:.2f})"

        # Dynamic gravity compensation on forward accelerometer (R_31)
        qx, qy, qz, qw = orientation_q
        q_norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
        if q_norm_sq > 0.5:
            inv_norm = 1.0 / math.sqrt(q_norm_sq)
            nx, ny, nz, nw = qx * inv_norm, qy * inv_norm, qz * inv_norm, qw * inv_norm
            gx = 2.0 * (nx * nz - nw * ny) * 9.81
        else:
            gx = 0.0

        a_dynamic_x = a_imu_x - gx

        # Manage Obstacle Block State & Takeoff State
        if abs(v_wheel) < 0.05:
            self.obstacle_blocked = False
            self.blocked_direction = 0
            self.takeoff_start_time = None
            self.takeoff_accel_seen = False
        else:
            # Transition from stop to motion
            if self.last_v_wheel is not None and abs(self.last_v_wheel) < 0.05:
                self.takeoff_start_time = current_time_sec
                self.takeoff_accel_seen = False

            # Clear obstacle block if driver commands motion away from obstacle
            if self.obstacle_blocked:
                if (self.blocked_direction > 0 and v_wheel < -0.05) or (self.blocked_direction < 0 and v_wheel > 0.05):
                    self.obstacle_blocked = False
                    self.blocked_direction = 0

            # Register dynamic body acceleration indicating genuine vehicle traction
            if not self.takeoff_accel_seen:
                if (v_wheel > 0.08 and a_dynamic_x > 0.08) or (v_wheel < -0.08 and a_dynamic_x < -0.08):
                    self.takeoff_accel_seen = True

        # Rule 2: Collision Impact Detection
        # Forward collision: driving forward while body suffers deceleration impact or pitch climb
        if not raw_slip:
            if v_wheel > 0.10 and (a_dynamic_x < -self.slip_accel_threshold or (a_dynamic_x < -0.10 and abs(w_imu_y) > 0.08)):
                raw_slip = True
                self.obstacle_blocked = True
                self.blocked_direction = 1
                reason = f"ForwardCollision (v={v_wheel:.2f}, a_dyn={a_dynamic_x:.2f})"
            # Reverse collision: driving backward while body suffers severe forward deceleration impact
            elif v_wheel < -0.10 and a_dynamic_x > self.slip_accel_threshold:
                raw_slip = True
                self.obstacle_blocked = True
                self.blocked_direction = -1
                reason = f"ReverseCollision (v={v_wheel:.2f}, a_dyn={a_dynamic_x:.2f})"

        # Rule 3: Obstacle Stall (sustained pushing against rock or obstacle)
        if not raw_slip and self.obstacle_blocked:
            if (self.blocked_direction > 0 and v_wheel > 0.10) or (self.blocked_direction < 0 and v_wheel < -0.10):
                raw_slip = True
                reason = f"ObstacleStall (v={v_wheel:.2f}, dir={self.blocked_direction})"

        # Rule 4: Stalled on Takeoff (spinning wheels from rest on frictionless ice)
        if not raw_slip and not self.obstacle_blocked and self.takeoff_start_time is not None:
            if abs(v_wheel) > 0.15 and not self.takeoff_accel_seen and (current_time_sec - self.takeoff_start_time) >= 0.28:
                raw_slip = True
                reason = f"StalledOnTakeoff (v={v_wheel:.2f}, a_dyn={a_dynamic_x:.2f})"

        if raw_slip:
            self.last_slip_time = current_time_sec
            self.last_slip_reason = reason

        slip_detected = (current_time_sec - self.last_slip_time) < self.slip_hold_time
        self.is_slipping = slip_detected
        self.last_v_wheel = v_wheel

        multiplier = self.covariance_scale if slip_detected else 1.0
        linear_cov = self.nominal_linear_cov * multiplier
        angular_cov = self.nominal_angular_cov * multiplier

        return slip_detected, linear_cov, angular_cov


class HeuristicSlipCheckerNode(Node):
    """
    ROS 2 Pre-Filter Node to detect wheel slip by comparing wheel-derived odometry
    against IMU rate readings, dynamically scaling wheel covariance during slippage.
    """

    def __init__(self) -> None:
        """Initialize HeuristicSlipCheckerNode, declare parameters, subscribers, and publishers."""
        super().__init__("heuristic_slip_checker")

        # ----------------------------------------------------------------------
        # ROS 2 Parameters Declaration
        # ----------------------------------------------------------------------
        self.declare_parameter("slip_velocity_threshold", 0.20)
        self.declare_parameter("slip_angular_threshold", 0.35)
        self.declare_parameter("slip_accel_threshold", 0.18)
        self.declare_parameter("covariance_inflation_factor", 100.0)
        self.declare_parameter("nominal_linear_covariance", 0.02)
        self.declare_parameter("nominal_angular_covariance", 0.05)
        self.declare_parameter("slip_hold_time", 0.4)
        self.declare_parameter("raw_odom_topic", "/wheel/odom_raw")

        # Retrieve parameter values
        self.slip_vel_threshold: float = (
            self.get_parameter("slip_velocity_threshold").get_parameter_value().double_value
        )
        self.slip_rot_threshold: float = (
            self.get_parameter("slip_angular_threshold").get_parameter_value().double_value
        )
        self.slip_accel_threshold: float = (
            self.get_parameter("slip_accel_threshold").get_parameter_value().double_value
        )
        self.covariance_scale: float = (
            self.get_parameter("covariance_inflation_factor").get_parameter_value().double_value
        )
        self.nominal_linear_cov: float = (
            self.get_parameter("nominal_linear_covariance").get_parameter_value().double_value
        )
        self.nominal_angular_cov: float = (
            self.get_parameter("nominal_angular_covariance").get_parameter_value().double_value
        )
        self.slip_hold_time: float = (
            self.get_parameter("slip_hold_time").get_parameter_value().double_value
        )
        self.raw_odom_topic: str = (
            self.get_parameter("raw_odom_topic").get_parameter_value().string_value
        )

        # Core logic helper
        self.core = SlipCheckerCore(
            slip_rot_threshold=self.slip_rot_threshold,
            slip_accel_threshold=self.slip_accel_threshold,
            slip_vel_threshold=self.slip_vel_threshold,
            covariance_scale=self.covariance_scale,
            nominal_linear_cov=self.nominal_linear_cov,
            nominal_angular_cov=self.nominal_angular_cov,
            slip_hold_time=self.slip_hold_time,
        )

        # ----------------------------------------------------------------------
        # State Data Tracking
        # ----------------------------------------------------------------------
        self.latest_imu: Optional[Imu] = None
        self.last_raw_odom_time: float = -100.0
        self._last_processed_stamp: Optional[Tuple[int, int]] = None
        self.is_slipping: bool = False

        # ----------------------------------------------------------------------
        # ROS 2 Subscribers & Publishers
        # ----------------------------------------------------------------------
        # Primary raw wheel odometry subscription (hardware / encoder_ticks_to_odom)
        self.sub_wheel_odom = self.create_subscription(
            Odometry,
            self.raw_odom_topic,
            self._wheel_odom_callback,
            10,
        )

        # Gazebo simulation fallback subscription (diff-drive plugin outputs /odom)
        self.sub_sim_odom = None
        if self.raw_odom_topic != "/odom":
            self.sub_sim_odom = self.create_subscription(
                Odometry,
                "/odom",
                self._sim_odom_callback,
                10,
            )

        self.sub_imu = self.create_subscription(
            Imu,
            "/imu/data",
            self._imu_callback,
            10,
        )

        self.single_wheel_slip: bool = False
        self.sub_single_wheel_slip = self.create_subscription(
            Bool,
            "/wheel/single_wheel_slip",
            self._single_wheel_slip_callback,
            10,
        )

        self.pub_filtered_odom = self.create_publisher(Odometry, "/wheel/odom_filtered", 10)
        self.pub_slip_flag = self.create_publisher(Bool, "/wheel/slip_detected", 10)

        self.get_logger().info("Heuristic Slip Checker Pre-Filter Node Initialized successfully.")

    # --------------------------------------------------------------------------
    # Callbacks & Logic Execution
    # --------------------------------------------------------------------------
    def _single_wheel_slip_callback(self, msg: Bool) -> None:
        """Callback for single-wheel slip telemetry flag from encoder_ticks_to_odom."""
        self.single_wheel_slip = msg.data

    def _imu_callback(self, msg: Imu) -> None:
        """
        Callback to process IMU angular velocity and linear acceleration telemetry.
        No open-loop integration is performed, eliminating unbounded sensor drift.
        """
        self.latest_imu = msg

    def _wheel_odom_callback(self, msg: Odometry) -> None:
        """Callback for primary /wheel/odom_raw odometry telemetry."""
        now_sec: float = self.get_clock().now().nanoseconds / 1e9
        self.last_raw_odom_time = now_sec
        self._process_odometry(msg)

    def _sim_odom_callback(self, msg: Odometry) -> None:
        """Fallback callback for Gazebo /odom odometry when /wheel/odom_raw is not active."""
        now_sec: float = self.get_clock().now().nanoseconds / 1e9
        # If /wheel/odom_raw has not published within the last 0.5s, process /odom
        if (now_sec - self.last_raw_odom_time) > 0.5:
            self._process_odometry(msg)

    def _process_odometry(self, msg: Odometry) -> None:
        """
        Evaluate wheel motion against IMU telemetry, dynamically scale covariance,
        and publish filtered odometry and slip flag.
        """
        # Deduplicate if remappings result in multiple triggers for the exact same message
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != (0, 0) and stamp == self._last_processed_stamp:
            return
        self._last_processed_stamp = stamp

        v_wheel: float = msg.twist.twist.linear.x
        w_wheel: float = msg.twist.twist.angular.z

        slip_detected: bool = False
        linear_cov: float = self.nominal_linear_cov
        angular_cov: float = self.nominal_angular_cov

        if self.latest_imu is not None:
            w_imu: float = self.latest_imu.angular_velocity.z
            w_imu_y: float = self.latest_imu.angular_velocity.y
            a_imu_x: float = self.latest_imu.linear_acceleration.x
            orient = self.latest_imu.orientation
            orientation_q = (orient.x, orient.y, orient.z, orient.w)
            now_sec: float = self.get_clock().now().nanoseconds / 1e9

            slip_detected, linear_cov, angular_cov = self.core.evaluate(
                v_wheel=v_wheel,
                w_wheel=w_wheel,
                w_imu=w_imu,
                a_imu_x=a_imu_x,
                current_time_sec=now_sec,
                orientation_q=orientation_q,
                w_imu_y=w_imu_y,
                single_wheel_slip=self.single_wheel_slip,
            )
        else:
            self.get_logger().warn(
                "Waiting for IMU telemetry on /imu/data... Slip detection is paused.",
                throttle_duration_sec=3.0,
            )

        self.is_slipping = slip_detected

        # Construct output odometry message with updated covariance
        filtered_odom = Odometry()
        filtered_odom.header = msg.header
        filtered_odom.child_frame_id = msg.child_frame_id
        filtered_odom.pose = msg.pose
        filtered_odom.twist = msg.twist

        multiplier: float = self.covariance_scale if slip_detected else 1.0

        # Dynamic velocity suppression during ANY confirmed slip:
        # If wheels are slipping (spinning in sand, obstacle block, single wheel slip, etc.),
        # wheel encoders do not represent true chassis ground velocity.
        # Clamping twist to 0.0 prevents EKF from integrating forward and penetrating the obstacle.
        if slip_detected:
            filtered_odom.twist.twist.linear.x = 0.0
            filtered_odom.twist.twist.linear.y = 0.0
            if self.latest_imu is not None:
                # Direct calibrated IMU gyro heading rate to EKF during wheel slippage
                filtered_odom.twist.twist.angular.z = self.latest_imu.angular_velocity.z

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
                f"Wheel Slip Detected! Reason: {self.core.last_slip_reason} (inflating covariance {self.covariance_scale:.1f}x)",
                throttle_duration_sec=1.0,
            )


def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for starting the Heuristic Slip Checker Node.
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

