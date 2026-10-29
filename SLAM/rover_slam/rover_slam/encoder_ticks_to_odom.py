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
    """Consume one selected source and publish only new complete measurements."""
    WHEEL_NAME_ALIASES = {'front_left':'left_front', 'left_front':'front_left',
        'front_right':'right_front', 'right_front':'front_right',
        'rear_left':'left_rear', 'left_rear':'rear_left',
        'rear_right':'right_rear', 'right_rear':'rear_right'}

    def __init__(self):
        super().__init__('encoder_ticks_to_odom')
        from rover_slam.wheel_odometry import WheelOdometry
        from diagnostic_msgs.msg import DiagnosticArray
        defaults = {'ticks_per_revolution':1024, 'wheel_radius':0.06, 'track_width':0.49,
            'single_wheel_slip_threshold':0.15, 'publish_tf':False, 'input_mode':'joint_states',
            'input_timeout_sec':0.5, 'max_wheel_speed_mps':2.0, 'odom_frame_id':'odom',
            'base_frame_id':'base_link', 'wheel_names':['left_front','right_front','left_rear','right_rear']}
        for key,value in defaults.items(): self.declare_parameter(key,value)
        def value(key): return self.get_parameter(key).value
        self.wheel_names = list(value('wheel_names'))
        self.ticks_per_rev = value('ticks_per_revolution')
        if self.ticks_per_rev <= 0 or not math.isfinite(value('wheel_radius')) or value('wheel_radius') <= 0:
            raise ValueError('Positive encoder resolution and radius required')
        self.core = WheelOdometry(self.wheel_names, 2*math.pi*value('wheel_radius')/self.ticks_per_rev,
            value('track_width'), value('input_timeout_sec'), value('max_wheel_speed_mps'),
            value('single_wheel_slip_threshold'))
        self.odom_frame_id, self.base_frame_id = value('odom_frame_id'), value('base_frame_id')
        self.current_ticks, self._wheel_receipts, self._pending_wheels = {}, {}, set()
        self._last_receipt = None
        self._last_clock = self.get_clock().now().nanoseconds/1e9
        self._last_valid_stamp = None
        mode = value('input_mode')
        self.subscriptions_owned = []
        if mode == 'joint_states':
            from rclpy.qos import qos_profile_sensor_data
            self.subscriptions_owned.append(self.create_subscription(JointState, '/joint_states', self._joint_states_callback, qos_profile_sensor_data))
        elif mode == 'ticks_array':
            self.subscriptions_owned.append(self.create_subscription(Int64MultiArray, '/wheel/ticks', self._ticks_array_callback, 10))
        elif mode == 'individual_ticks':
            for wheel in self.wheel_names:
                self.subscriptions_owned.append(self.create_subscription(Int64, '/wheel/ticks/'+wheel,
                    lambda msg, name=wheel: self._individual(name,msg.data), 10))
        else:
            raise ValueError('input_mode must be joint_states, ticks_array or individual_ticks')
        self.pub_odom = self.create_publisher(Odometry, '/wheel/odom_raw', 10)
        self.pub_wheel_speeds = self.create_publisher(Float64MultiArray, '/wheel/per_wheel_speeds', 10)
        self.pub_single_wheel_slip = self.create_publisher(Bool, '/wheel/single_wheel_slip', 10)
        self.pub_diagnostics = self.create_publisher(DiagnosticArray, '/slam/encoder_diagnostics', 10)
        self.publish_tf = value('publish_tf')
        if self.publish_tf: self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self._watchdog)

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

    def _now(self):
        now = self.get_clock().now().nanoseconds/1e9
        if now < self._last_clock:
            self.core.reset_measurements()
            self.current_ticks.clear()
            self._wheel_receipts.clear()
            self._pending_wheels.clear()
            self._last_valid_stamp = self._last_receipt = None
        self._last_clock = now
        return now

    def _ticks_array_callback(self, msg):
        self._now()
        data = list(msg.data)
        if len(data) == 6 and len(self.wheel_names) == 4:
            legacy = dict(zip(('left_front','left_rear','right_front','right_rear'),
                              (data[0],data[2],data[3],data[5])))
            counts = {n:legacy.get(n,legacy.get(self.WHEEL_NAME_ALIASES.get(n))) for n in self.wheel_names}
        elif len(data) == len(self.wheel_names):
            counts = dict(zip(self.wheel_names,data))
        else:
            self.core.reason = 'incorrect wheel array length'
            return
        self._observe(counts, self.get_clock().now().to_msg())

    def _joint_states_callback(self, msg):
        self._now()
        counts = {}
        for name in self.wheel_names:
            for i,joint in enumerate(msg.name):
                if self._is_joint_matching_wheel(joint,name,self.WHEEL_NAME_ALIASES.get(name,'')):
                    if i < len(msg.position) and math.isfinite(msg.position[i]):
                        counts[name] = msg.position[i]*self.ticks_per_rev/(2*math.pi)
                    break
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
        self._observe(counts, stamp)

    def _individual(self, name, count):
        now = self._now()
        self.current_ticks[name], self._wheel_receipts[name] = count, now
        self._pending_wheels.add(name)
        if len(self._pending_wheels) != len(self.wheel_names): return
        self._pending_wheels.clear()
        if now-min(self._wheel_receipts.values()) > self.core.timeout:
            self.core.reason = 'individual wheel inputs not coherent'
            return
        self._observe(self.current_ticks, self.get_clock().now().to_msg())

    def _observe(self, counts, stamp):
        now = self._now()
        seconds = stamp.sec+stamp.nanosec/1e9
        if not 0 <= now-seconds <= self.core.timeout:
            self.core.reason = 'stale or future wheel timestamp'
            return
        result = self.core.observe(counts, seconds)
        if result is None: return
        self._last_receipt, self._last_valid_stamp = now, seconds
        velocity, yaw_rate, speeds, slip = result
        msg = Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, self.odom_frame_id, self.base_frame_id
        msg.pose.pose.position.x, msg.pose.pose.position.y = self.core.x, self.core.y
        msg.pose.pose.orientation = euler_to_quaternion(0.,0.,self.core.yaw)
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = velocity, yaw_rate
        msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35] = .01,.01,.05
        msg.twist.covariance[0], msg.twist.covariance[7], msg.twist.covariance[35] = .02,.02,.05
        self.pub_odom.publish(msg)
        self.pub_wheel_speeds.publish(Float64MultiArray(data=[speeds[n] for n in self.wheel_names]))
        self.pub_single_wheel_slip.publish(Bool(data=slip))
        if self.publish_tf:
            tf = TransformStamped()
            tf.header, tf.child_frame_id = msg.header, msg.child_frame_id
            tf.transform.translation.x, tf.transform.translation.y = self.core.x,self.core.y
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def _watchdog(self):
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
        now = self._now()
        healthy = (self._last_receipt is not None and self._last_valid_stamp is not None
                   and 0 <= now-self._last_receipt <= self.core.timeout
                   and 0 <= now-self._last_valid_stamp <= self.core.timeout
                   and self.core.reason == 'healthy')
        status = DiagnosticStatus(name='slam/wheel_input', hardware_id='wheel_encoders',
            level=DiagnosticStatus.OK if healthy else DiagnosticStatus.WARN,
            message='healthy' if healthy else 'wheel input unavailable: '+self.core.reason)
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.pub_diagnostics.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderTicksToOdomNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
