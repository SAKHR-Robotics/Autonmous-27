#!/usr/bin/env python3
"""
heuristic_slip_checker.py
Compares wheel velocity against IMU acceleration/yaw rate to detect sand slip.
Dynamically scales covariance on /wheel/odom_raw during slippage.
Developer Track: Person 1 (Task 3A.1 & 3A.2)
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool


class SlipCheckerCore:
    """Core algorithmic logic for detecting slip and scaling covariance.
    Separated from ROS node for deterministic unit testing.
    """
    def __init__(self,
                 linear_threshold: float = 0.15,
                 angular_threshold: float = 0.20,
                 base_linear_cov: float = 0.01,
                 base_angular_cov: float = 0.01,
                 slip_cov_multiplier: float = 1000.0):
        self.linear_threshold = linear_threshold
        self.angular_threshold = angular_threshold
        self.base_linear_cov = base_linear_cov
        self.base_angular_cov = base_angular_cov
        self.slip_cov_multiplier = slip_cov_multiplier

        # IMU state tracking
        self.v_imu = 0.0
        self.last_imu_time = None
        self.latest_imu_yaw_rate = 0.0

    def update_imu(self, ax: float, yaw_rate: float, current_time_sec: float):
        """Update internal IMU kinematic state."""
        self.latest_imu_yaw_rate = yaw_rate
        if self.last_imu_time is not None:
            dt = current_time_sec - self.last_imu_time
            if 0.0 < dt < 1.0:
                # Integrate linear acceleration along X axis with leaky damping to avoid drift
                self.v_imu = (self.v_imu + ax * dt) * 0.98
        self.last_imu_time = current_time_sec

    def check_slip(self, v_wheel: float, omega_wheel: float) -> tuple:
        """Check for slip based on wheel vs IMU velocity/yaw rate discrepancies.
        Returns:
            (is_slipping, linear_cov, angular_cov)
        """
        linear_diff = abs(v_wheel - self.v_imu)
        angular_diff = abs(omega_wheel - self.latest_imu_yaw_rate)

        is_linear_slip = linear_diff > self.linear_threshold
        is_angular_slip = angular_diff > self.angular_threshold
        is_slipping = is_linear_slip or is_angular_slip

        if is_slipping:
            linear_cov = self.base_linear_cov * self.slip_cov_multiplier
            angular_cov = self.base_angular_cov * self.slip_cov_multiplier
        else:
            linear_cov = self.base_linear_cov
            angular_cov = self.base_angular_cov

        return is_slipping, linear_cov, angular_cov


class HeuristicSlipCheckerNode(Node):
    def __init__(self):
        super().__init__('heuristic_slip_checker')

        # Declare ROS 2 Parameters
        self.declare_parameter('linear_threshold', 0.15)
        self.declare_parameter('angular_threshold', 0.20)
        self.declare_parameter('base_linear_cov', 0.01)
        self.declare_parameter('base_angular_cov', 0.01)
        self.declare_parameter('slip_cov_multiplier', 1000.0)
        self.declare_parameter('odom_input_topic', '/wheel/odom_unfiltered')
        self.declare_parameter('imu_input_topic', '/imu/data')
        self.declare_parameter('odom_output_topic', '/wheel/odom_raw')

        linear_thresh = self.get_parameter('linear_threshold').value
        angular_thresh = self.get_parameter('angular_threshold').value
        base_lin_cov = self.get_parameter('base_linear_cov').value
        base_ang_cov = self.get_parameter('base_angular_cov').value
        slip_mult = self.get_parameter('slip_cov_multiplier').value
        odom_in = self.get_parameter('odom_input_topic').value
        imu_in = self.get_parameter('imu_input_topic').value
        odom_out = self.get_parameter('odom_output_topic').value

        self.checker = SlipCheckerCore(
            linear_threshold=linear_thresh,
            angular_threshold=angular_thresh,
            base_linear_cov=base_lin_cov,
            base_angular_cov=base_ang_cov,
            slip_cov_multiplier=slip_mult
        )

        # Publishers & Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_in,
            self.odom_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            imu_in,
            self.imu_callback,
            10
        )
        self.odom_pub = self.create_publisher(
            Odometry,
            odom_out,
            10
        )
        self.slip_flag_pub = self.create_publisher(
            Bool,
            '/rover/slip_detected',
            10
        )

        self.get_logger().info(
            f'Heuristic Slip Checker Pre-Filter Node initialized. '
            f'Subscribing: [{odom_in}, {imu_in}] -> Publishing: [{odom_out}] '
            f'(Linear Thresh: {linear_thresh} m/s, Slip Multiplier: {slip_mult}x)'
        )

    def imu_callback(self, msg: Imu):
        ax = msg.linear_acceleration.x
        yaw_rate = msg.angular_velocity.z
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.checker.update_imu(ax, yaw_rate, t)

    def odom_callback(self, msg: Odometry):
        v_wheel = msg.twist.twist.linear.x
        omega_wheel = msg.twist.twist.angular.z

        is_slipping, lin_cov, ang_cov = self.checker.check_slip(v_wheel, omega_wheel)

        # Create copy and apply dynamic covariance
        out_msg = Odometry()
        out_msg.header = msg.header
        out_msg.child_frame_id = msg.child_frame_id
        out_msg.pose = msg.pose
        out_msg.twist = msg.twist

        # Update covariance matrix: index 0 (linear.x), index 7 (linear.y), index 35 (angular.z)
        cov = list(msg.twist.covariance)
        if len(cov) == 36:
            cov[0] = lin_cov      # var(vx)
            cov[7] = lin_cov      # var(vy)
            cov[35] = ang_cov     # var(wz)
            out_msg.twist.covariance = cov

        self.odom_pub.publish(out_msg)

        # Publish diagnostic boolean
        slip_msg = Bool()
        slip_msg.data = is_slipping
        self.slip_flag_pub.publish(slip_msg)

        if is_slipping:
            self.get_logger().warn(
                f'Wheel slip detected! V_wheel={v_wheel:.2f} m/s, V_imu={self.checker.v_imu:.2f} m/s. '
                f'Inflating covariance by {self.checker.slip_cov_multiplier}x',
                throttle_duration_sec=1.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = HeuristicSlipCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
