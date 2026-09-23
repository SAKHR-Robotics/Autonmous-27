#!/usr/bin/env python3
"""
encoder_ticks_to_odom.py
========================
ROS 2 Node that processes raw encoder ticks from all 4 rover wheels
(left_front, right_front, left_rear, right_rear), computes per-wheel displacement
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
from typing import Dict, List, Optional, Any, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int64, Int64MultiArray, Float64MultiArray, Bool
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Quaternion, TransformStamped
    import tf2_ros
except ImportError:
    rclpy = None
    Node = object  # type: ignore
    Int64 = None  # type: ignore
    Int64MultiArray = None  # type: ignore
    Float64MultiArray = None  # type: ignore
    Bool = None  # type: ignore
    JointState = None  # type: ignore
    Odometry = None  # type: ignore

    class Quaternion:  # type: ignore
        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
            self.x, self.y, self.z, self.w = x, y, z, w

    TransformStamped = None  # type: ignore
    tf2_ros = None  # type: ignore


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


def compute_robust_side_velocity(
    velocities: List[float],
    slip_diff_threshold: float = 0.15,
) -> Tuple[float, bool]:
    """
    Computes robust side velocity for a rover side (e.g. left or right).
    Detects if one wheel on the side is slipping (spinning faster than traction wheel)
    and selects the traction wheel (smaller magnitude) to avoid corrupting odometry.

    Args:
        velocities (List[float]): Linear velocities of wheels on one side.
        slip_diff_threshold (float): Velocity difference threshold in m/s.

    Returns:
        Tuple[float, bool]: (robust_velocity, single_wheel_slip_detected)
    """
    # Sanitize and filter out non-finite (NaN, Inf) readings
    valid_v = [v for v in velocities if math.isfinite(v)]
    if not valid_v:
        return 0.0, False
    if len(valid_v) == 1:
        slip = len(valid_v) < len(velocities)
        return valid_v[0], slip
    if len(valid_v) == 2:
        v1, v2 = valid_v[0], valid_v[1]
        diff = abs(v1 - v2)
        if diff > slip_diff_threshold:
            # Slipping wheel spins faster; ground speed is bounded by the wheel with lower absolute speed
            v_traction = v1 if abs(v1) < abs(v2) else v2
            return v_traction, True
        return (v1 + v2) / 2.0, False

    sorted_v = sorted(valid_v, key=abs)
    mid = len(sorted_v) // 2
    slip_flag = (abs(sorted_v[-1]) - abs(sorted_v[0])) > slip_diff_threshold or (len(valid_v) < len(velocities))
    return sorted_v[mid], slip_flag


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

    WHEEL_NAME_ALIASES: Dict[str, str] = {
        "front_left": "left_front",
        "left_front": "front_left",
        "front_right": "right_front",
        "right_front": "front_right",
        "rear_left": "left_rear",
        "left_rear": "rear_left",
        "rear_right": "right_rear",
        "right_rear": "rear_right",
    }

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

        # CUSTOMIZE WITH MECHANICAL TEAM: Left-to-right wheel baseline distance in meters (matches URDF & Gazebo diff-drive 0.49m)
        self.declare_parameter("track_width", 0.49)

        # Threshold to flag single wheel slip (m/s difference between wheels on same side)
        self.declare_parameter("single_wheel_slip_threshold", 0.15)

        # CUSTOMIZE WITH HARDWARE/SLAM TEAM: Set True if node should publish odom -> base_link TF
        self.declare_parameter("publish_tf", False)

        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")

        # CUSTOMIZE WITH HARDWARE/MECHANICAL TEAM: Naming of the 4 wheels (matches STM32/URDF)
        self.declare_parameter(
            "wheel_names",
            [
                "left_front",
                "right_front",
                "left_rear",
                "right_rear",
            ],
        )

        # Retrieve parameter values
        self.ticks_per_rev: int = self.get_parameter("ticks_per_revolution").get_parameter_value().integer_value
        self.wheel_radius: float = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.track_width: float = self.get_parameter("track_width").get_parameter_value().double_value
        self.single_wheel_slip_threshold: float = (
            self.get_parameter("single_wheel_slip_threshold").get_parameter_value().double_value
        )
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
        self.last_odom_ticks: Dict[str, Optional[int]] = {name: None for name in self.wheel_names}

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
        self.pub_single_wheel_slip = self.create_publisher(Bool, "/wheel/single_wheel_slip", 10)

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

    @classmethod
    def _is_joint_matching_wheel(cls, joint_name: str, wheel_name: str, alias: str = "") -> bool:
        """
        Check if a joint_name strictly corresponds to the wheel_joint of a given wheel,
        preventing false positive matches with arm joints (e.g., left_front_arm_joint)
        or steering joints (e.g., left_front_steer_joint).

        Args:
            joint_name (str): Name of joint in JointState message.
            wheel_name (str): Primary wheel identifier name.
            alias (str): Optional alternate alias name (e.g. front_left for left_front).

        Returns:
            bool: True if joint_name is the continuous wheel joint for this wheel.
        """
        clean_name: str = joint_name.lower()
        if "arm" in clean_name or "steer" in clean_name:
            return False

        urdf_target: str = f"{wheel_name.lower()}_wheel_joint"
        alias_target: str = f"{alias.lower()}_wheel_joint" if alias else ""

        # Exact match or namespace-prefixed match (e.g. /my_robot/left_front_wheel_joint)
        if clean_name == urdf_target or clean_name.endswith(f"/{urdf_target}"):
            return True
        if alias_target and (clean_name == alias_target or clean_name.endswith(f"/{alias_target}")):
            return True

        # Bare wheel identifier matches
        if clean_name == wheel_name.lower() or (alias and clean_name == alias.lower()):
            return True

        # General pattern: must include wheel identifier AND 'wheel'
        if (wheel_name.lower() in clean_name or (alias and alias.lower() in clean_name)) and "wheel" in clean_name:
            return True

        return False

    def _ticks_array_callback(self, msg: Int64MultiArray) -> None:
        """
        Callback for receiving an array of raw encoder ticks from all wheels.
        Handles standard 4-wheel arrays as well as legacy 6-wheel arrays.

        Args:
            msg (Int64MultiArray): Message containing raw tick counts for each wheel in order.
        """
        data: List[int] = list(msg.data)
        if len(data) == 6 and len(self.wheel_names) == 4:
            # Handle legacy 6-wheel array layout: [LF, LM, LR, RF, RM, RR]
            # Mapping 4 active wheels: LF -> data[0], LR -> data[2], RF -> data[3], RR -> data[5]
            legacy_map = {
                "left_front": data[0],
                "front_left": data[0],
                "left_rear": data[2],
                "rear_left": data[2],
                "right_front": data[3],
                "front_right": data[3],
                "right_rear": data[5],
                "rear_right": data[5],
            }
            for name in self.wheel_names:
                if name in legacy_map:
                    self._update_single_wheel_tick(name, legacy_map[name])
            return

        for idx, name in enumerate(self.wheel_names):
            if idx < len(data):
                self._update_single_wheel_tick(name, data[idx])

    def _joint_states_callback(self, msg: JointState) -> None:
        """
        Callback for receiving joint states from simulation or hardware drivers.
        Strictly matches continuous wheel joints while ignoring fixed suspension arm joints.

        Args:
            msg (JointState): Standard ROS 2 JointState message.
        """
        for idx, joint_name in enumerate(msg.name):
            for wheel_name in self.wheel_names:
                alias = self.WHEEL_NAME_ALIASES.get(wheel_name, "")
                if self._is_joint_matching_wheel(joint_name, wheel_name, alias) and idx < len(msg.position):
                    rad_pos: float = msg.position[idx]
                    if not math.isfinite(rad_pos):
                        break
                    ticks: int = int(round((rad_pos / (2.0 * math.pi)) * self.ticks_per_rev))
                    self._update_single_wheel_tick(wheel_name, ticks)
                    break

    def _update_single_wheel_tick(self, wheel_name: str, new_tick_count: int) -> None:
        """
        Updates internal tick count state and delta ticks for a specific wheel.

        Args:
            wheel_name (str): Name of the wheel to update.
            new_tick_count (int): Latest raw encoder tick count reading.
        """
        if self.prev_ticks.get(wheel_name) is None:
            self.prev_ticks[wheel_name] = new_tick_count
            self.current_ticks[wheel_name] = new_tick_count
            self.wheel_delta_ticks[wheel_name] = 0
            self.last_odom_ticks[wheel_name] = new_tick_count
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

        # Calculate linear velocity for every individual wheel based on delta since last odometry cycle
        for wheel_name in self.wheel_names:
            if self.prev_ticks.get(wheel_name) is None:
                # No encoder data received yet for this wheel; hold at zero
                delta_ticks = 0
            else:
                current: int = self.current_ticks.get(wheel_name, 0)
                last_odom: Optional[int] = self.last_odom_ticks.get(wheel_name)

                if last_odom is not None:
                    delta_ticks = current - last_odom
                else:
                    delta_ticks = 0

                self.last_odom_ticks[wheel_name] = current

            self.wheel_delta_ticks[wheel_name] = delta_ticks
            dist: float = delta_ticks * self.meters_per_tick
            velocity: float = dist / dt
            self.wheel_velocities[wheel_name] = velocity

            if "left" in wheel_name.lower():
                left_velocities.append(velocity)
            elif "right" in wheel_name.lower():
                right_velocities.append(velocity)

        # Robust per-side wheel velocity calculations rejecting single-wheel slippage
        v_left, slip_left = compute_robust_side_velocity(left_velocities, self.single_wheel_slip_threshold)
        v_right, slip_right = compute_robust_side_velocity(right_velocities, self.single_wheel_slip_threshold)
        single_wheel_slip_active: bool = slip_left or slip_right

        # Kinematics calculations
        v_x: float = (v_right + v_left) / 2.0
        v_y: float = 0.0
        safe_track_width: float = max(1e-6, self.track_width)
        omega_z: float = (v_right - v_left) / safe_track_width

        # Integrate pose
        delta_x: float = (v_x * math.cos(self.yaw)) * dt
        delta_y: float = (v_x * math.sin(self.yaw)) * dt
        delta_yaw: float = omega_z * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw
        # Normalize yaw to [-pi, pi] per REP-103
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

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

        # Publish single-wheel slip telemetry flag
        slip_msg = Bool()
        slip_msg.data = single_wheel_slip_active
        self.pub_single_wheel_slip.publish(slip_msg)

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

