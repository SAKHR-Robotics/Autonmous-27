#!/usr/bin/env python3
"""
encoder_ticks_to_odom.py
========================
ROS 2 Node that processes raw encoder ticks from all 4 rover wheels
(front_left, front_right, rear_left, rear_right), computes per-wheel displacement
and velocity telemetry, and calculates overall 4-wheel differential drive odometry
for EKF sensor fusion.

Features:
  - Subscribes to per-wheel encoder tick data for all 4 wheels.
  - Maintains individual tick counters, delta ticks, and computed wheel speeds for each wheel.
  - Publishes raw wheel odometry (`nav_msgs/msg/Odometry`) to `/wheel/odom_raw`.
  - Publishes individual 4-wheel velocity breakdown (`std_msgs/msg/Float64MultiArray`) to `/wheel/per_wheel_speeds`.

Developer Track: SLAM & Sensor Processing Subsystem
"""

import math
from typing import Dict, List, Optional, Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Int64MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    Convert Euler angles (roll, pitch, yaw) in radians to a ROS 2 Quaternion message.

    Args:
        roll (float): Rotation around X axis in radians.
        pitch (float): Rotation around Y axis in radians.
        yaw (float): Rotation around Z axis in radians.

    Returns:
        Quaternion: Populated geometry_msgs.msg.Quaternion object.
    """
    qx: float = math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) - math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)
    qy: float = math.cos(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0)
    qz: float = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.sin(yaw / 2.0) - math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.cos(yaw / 2.0)
    qw: float = math.cos(roll / 2.0) * math.cos(pitch / 2.0) * math.cos(yaw / 2.0) + math.sin(roll / 2.0) * math.sin(pitch / 2.0) * math.sin(yaw / 2.0)

    q = Quaternion()
    q.x = qx
    q.y = qy
    q.z = qz
    q.w = qw
    return q


class EncoderTicksToOdomNode(Node):
    """
    ROS 2 Node to read encoder ticks from all individual wheels, compute per-wheel telemetry,
    and estimate rover odometry for EKF sensor fusion.

    Attributes:
        wheel_names (List[str]): Active wheel names (supports 4-wheel and 6-wheel rover layouts).
        ticks_per_rev (int): Number of encoder counts/ticks per full wheel revolution.
        wheel_radius (float): Radius of each wheel in meters.
        track_width (float): Distance between left and right wheel baselines in meters.
        publish_tf (bool): Flag indicating whether to broadcast odom -> base_link transform.
    """

    def __init__(self) -> None:
        """Initialize the EncoderTicksToOdomNode, declare parameters, subscribers, and publishers."""
        super().__init__("encoder_ticks_to_odom")

        # ----------------------------------------------------------------------
        # ROS 2 Parameters Declaration
        # ----------------------------------------------------------------------
        # CUSTOMIZE WITH HARDWARE TEAM: Encoder resolution (CPR * gear ratio of motor driver)
        self.declare_parameter("ticks_per_revolution", 1024)

        # CUSTOMIZE WITH MECHANICAL TEAM: Measured outer radius of wheels in meters
        self.declare_parameter("wheel_radius", 0.06)

        # CUSTOMIZE WITH MECHANICAL TEAM: Left-to-right wheel baseline distance in meters
        self.declare_parameter("track_width", 0.42)

        # CUSTOMIZE WITH HARDWARE/SLAM TEAM: Set True if node should publish odom -> base_link TF
        self.declare_parameter("publish_tf", False)

        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        # CUSTOMIZE WITH HARDWARE/MECHANICAL TEAM: Naming of the 4 wheels (matches STM32/URDF)
        self.declare_parameter(
            "wheel_names",
            [
                "front_left",
                "front_right",
                "rear_left",
                "rear_right",
            ],
        )

        # Retrieve parameter values
        self.ticks_per_rev: int = self.get_parameter("ticks_per_revolution").get_parameter_value().integer_value
        self.wheel_radius: float = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.track_width: float = self.get_parameter("track_width").get_parameter_value().double_value
        self.publish_tf: bool = self.get_parameter("publish_tf").get_parameter_value().bool_value
        self.odom_frame_id: str = self.get_parameter("odom_frame_id").get_parameter_value().string_value
        self.base_frame_id: str = self.get_parameter("base_frame_id").get_parameter_value().string_value
        self.wheel_names: List[str] = self.get_parameter("wheel_names").get_parameter_value().string_array_value

        # Kinematics constants calculation
        self.wheel_circumference: float = 2.0 * math.pi * self.wheel_radius
        self.meters_per_tick: float = self.wheel_circumference / max(1, self.ticks_per_rev)

        # ----------------------------------------------------------------------
        # Per-Wheel Data Tracking Data Structures (Every Wheel Data)
        # ----------------------------------------------------------------------
        self.current_ticks: Dict[str, int] = {name: 0 for name in self.wheel_names}
        self.prev_ticks: Dict[str, Optional[int]] = {name: None for name in self.wheel_names}
        self.wheel_velocities: Dict[str, float] = {name: 0.0 for name in self.wheel_names}
        self.wheel_delta_ticks: Dict[str, int] = {name: 0 for name in self.wheel_names}

        # Robot 2D Pose State Estimation
        self.x: float = 0.0
        self.y: float = 0.0
        self.yaw: float = 0.0
        self.last_time: Optional[rclpy.time.Time] = None

        # ----------------------------------------------------------------------
        # ROS 2 Subscribers & Publishers
        # ----------------------------------------------------------------------
        # Option 1: Multi-array subscriber for combined wheel ticks on `/wheel/ticks`
        self.sub_ticks_array = self.create_subscription(
            Int64MultiArray,
            "/wheel/ticks",
            self._ticks_array_callback,
            10,
        )

        # Option 2: JointState subscriber for simulation / hardware drivers on `/joint_states`
        self.sub_joint_states = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_states_callback,
            10,
        )

        # Option 3: Subscriptions for individual per-wheel topics (`/wheel/ticks/<wheel_name>`)
        self.sub_individual_wheels: Dict[str, Any] = {}
        for wheel in self.wheel_names:
            topic_name: str = f"/wheel/ticks/{wheel}"
            self.sub_individual_wheels[wheel] = self.create_subscription(
                Int64,
                topic_name,
                self._make_individual_wheel_callback(wheel),
                10,
            )

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, "/wheel/odom_raw", 10)
        self.pub_wheel_speeds = self.create_publisher(Float64MultiArray, "/wheel/per_wheel_speeds", 10)

        # Optional Transform Broadcaster
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # CUSTOMIZE WITH COMPUTE/HARDWARE TEAM: Desired odometry publishing period (0.02s = 50Hz update rate)
        self.timer = self.create_timer(0.02, self._update_odometry)

        self.get_logger().info(
            f"Encoder Ticks Node initialized for {len(self.wheel_names)} wheels: {self.wheel_names}"
        )

    # --------------------------------------------------------------------------
    # Helper Callback Generator & Processing
    # --------------------------------------------------------------------------
    def _make_individual_wheel_callback(self, wheel_name: str):
        """
        Creates a dedicated subscriber callback function bound to a specific wheel name.

        Args:
            wheel_name (str): Identifier name of the wheel.

        Returns:
            Callable[[Int64], None]: Callback function accepting Int64 ROS message.
        """
        def callback(msg: Int64) -> None:
            self._update_single_wheel_tick(wheel_name, msg.data)
        return callback

    def _ticks_array_callback(self, msg: Int64MultiArray) -> None:
        """
        Callback for receiving an array of raw encoder ticks from all wheels.

        Args:
            msg (Int64MultiArray): Message containing raw tick counts for each wheel in order.
        """
        data: List[int] = list(msg.data)
        for idx, name in enumerate(self.wheel_names):
            if idx < len(data):
                self._update_single_wheel_tick(name, data[idx])

    def _joint_states_callback(self, msg: JointState) -> None:
        """
        Callback for receiving joint states from simulation or hardware drivers.

        Args:
            msg (JointState): Standard ROS 2 JointState message.
        """
        for idx, name in enumerate(msg.name):
            for wheel_name in self.wheel_names:
                if wheel_name in name and idx < len(msg.position):
                    rad_pos: float = msg.position[idx]
                    ticks: int = int((rad_pos / (2.0 * math.pi)) * self.ticks_per_rev)
                    self._update_single_wheel_tick(wheel_name, ticks)

    def _update_single_wheel_tick(self, wheel_name: str, new_tick_count: int) -> None:
        """
        Updates internal tick count state and delta ticks for a specific wheel.

        Args:
            wheel_name (str): Name of the wheel to update.
            new_tick_count (int): Latest raw encoder tick count reading.
        """
        if self.prev_ticks[wheel_name] is None:
            self.prev_ticks[wheel_name] = new_tick_count
            self.current_ticks[wheel_name] = new_tick_count
            self.wheel_delta_ticks[wheel_name] = 0
            return

        delta: int = new_tick_count - self.current_ticks[wheel_name]
        self.prev_ticks[wheel_name] = self.current_ticks[wheel_name]
        self.current_ticks[wheel_name] = new_tick_count
        self.wheel_delta_ticks[wheel_name] = delta

    # --------------------------------------------------------------------------
    # Odometry Update & Kinematics Computation Loop
    # --------------------------------------------------------------------------
    def _update_odometry(self) -> None:
        """
        Timer callback to calculate per-wheel linear velocities, differential drive pose kinematics,
        and publish wheel odometry and per-wheel speed data.
        """
        current_time = self.get_clock().now()

        if self.last_time is None:
            self.last_time = current_time
            return

        dt: float = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            return

        left_velocities: List[float] = []
        right_velocities: List[float] = []

        # Calculate linear velocity for every individual wheel
        for wheel_name in self.wheel_names:
            delta_ticks: int = self.wheel_delta_ticks[wheel_name]
            dist: float = delta_ticks * self.meters_per_tick
            velocity: float = dist / dt
            self.wheel_velocities[wheel_name] = velocity

            if "left" in wheel_name.lower():
                left_velocities.append(velocity)
            elif "right" in wheel_name.lower():
                right_velocities.append(velocity)

        # Average left and right side wheel speeds
        v_left: float = (sum(left_velocities) / len(left_velocities)) if left_velocities else 0.0
        v_right: float = (sum(right_velocities) / len(right_velocities)) if right_velocities else 0.0

        # Kinematics calculations
        v_x: float = (v_right + v_left) / 2.0
        v_y: float = 0.0
        omega_z: float = (v_right - v_left) / self.track_width

        # Integrate pose
        delta_x: float = (v_x * math.cos(self.yaw)) * dt
        delta_y: float = (v_x * math.sin(self.yaw)) * dt
        delta_yaw: float = omega_z * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # Construct nav_msgs/Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, self.yaw)

        # CUSTOMIZE WITH HARDWARE/TESTING TEAM: Diagonal noise covariance matrices based on wheel slip & terrain testing
        odom_msg.pose.covariance[0] = 0.01   # Position X variance (m^2)
        odom_msg.pose.covariance[7] = 0.01   # Position Y variance (m^2)
        odom_msg.pose.covariance[35] = 0.05  # Orientation Yaw variance (rad^2)

        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.linear.y = v_y
        odom_msg.twist.twist.angular.z = omega_z

        odom_msg.twist.covariance[0] = 0.02   # Linear Velocity Vx variance (m/s)^2
        odom_msg.twist.covariance[7] = 0.02   # Linear Velocity Vy variance (m/s)^2
        odom_msg.twist.covariance[35] = 0.05  # Angular Velocity Wz variance (rad/s)^2

        self.pub_odom.publish(odom_msg)

        # Publish per-wheel speeds
        wheel_speeds_msg = Float64MultiArray()
        wheel_speeds_msg.data = [self.wheel_velocities[w] for w in self.wheel_names]
        self.pub_wheel_speeds.publish(wheel_speeds_msg)

        # Broadcast odom -> base_link transform if enabled
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom_msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)


def main(args: Optional[List[str]] = None) -> None:
    """
    Main entry point for starting the Encoder Ticks Node.

    Args:
        args (Optional[List[str]]): Arguments passed to rclpy initialization.
    """
    rclpy.init(args=args)
    node = EncoderTicksToOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

