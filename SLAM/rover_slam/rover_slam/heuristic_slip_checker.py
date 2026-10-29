#!/usr/bin/env python3
"""
heuristic_slip_checker.py
=========================
ROS 2 Node that monitors wheel velocity against IMU motion telemetry to detect wheel slip
(e.g., in loose sand or steep terrain).

When wheel slippage is detected (|V_wheels - V_imu| > threshold or |W_wheels - W_imu| > threshold),
it dynamically inflates the covariance matrix of wheel odometry before passing it to EKF,
reducing trust in wheels in the local EKF. RTAB-Map remains a separate global
mapping backend and does not supply local EKF velocity.

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
        covariance_scale: float = 100.0,
        nominal_linear_cov: float = 0.02,
        nominal_angular_cov: float = 0.05,
        slip_hold_time: float = 0.4,
    ) -> None:
        self.slip_rot_threshold = slip_rot_threshold
        self.slip_accel_threshold = slip_accel_threshold
        self.covariance_scale = covariance_scale
        self.nominal_linear_cov = nominal_linear_cov
        self.nominal_angular_cov = nominal_angular_cov
        self.slip_hold_time = slip_hold_time

        if not all(math.isfinite(v) and v > 0 for v in (slip_rot_threshold, slip_accel_threshold,
                covariance_scale, nominal_linear_cov, nominal_angular_cov, slip_hold_time)):
            raise ValueError('Positive finite slip thresholds, covariance and hold time required')
        self.last_evaluation_time = None
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
        values = (v_wheel, w_wheel, w_imu, a_imu_x, current_time_sec, w_imu_y, *orientation_q)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Non-finite slip input')
        if self.last_evaluation_time is not None and current_time_sec < self.last_evaluation_time:
            self.last_slip_time = -float('inf')
            self.last_v_wheel = None
            self.obstacle_blocked = False
            self.takeoff_start_time = None
            self.takeoff_accel_seen = True
        self.last_evaluation_time = current_time_sec
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
            if (abs(w_wheel-w_imu) > self.slip_rot_threshold and abs(w_wheel) > 0.25
                    and (abs(w_imu) < 0.10 or w_wheel*w_imu < -0.05)):
                raw_slip = True
                reason = f"YawBlocked (w_wheel={w_wheel:.2f}, w_imu={w_imu:.2f})"
            elif (abs(w_wheel-w_imu) > self.slip_rot_threshold
                  and abs(w_wheel) < 0.10 and abs(w_imu) > 0.40):
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


def quaternion_product(a, b):
    x,y,z,w = a; X,Y,Z,W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def normalize_quaternion(q):
    norm = math.hypot(*q)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError('Invalid quaternion')
    return tuple(v/norm for v in q)


def rotate_vector(q, vector):
    q = normalize_quaternion(q)
    return quaternion_product(quaternion_product(q, (*vector,0.)), (-q[0],-q[1],-q[2],q[3]))[:3]


def inflate_covariance(covariance, multiplier, linear_floor, angular_floor):
    if (len(covariance) != 36 or not all(math.isfinite(v) for v in covariance)
            or any(covariance[i*6+i] < 0 for i in range(6))):
        raise ValueError('Invalid odometry covariance')
    result = [v*multiplier for v in covariance]
    for index, floor in ((0,linear_floor),(35,angular_floor)):
        result[index] = max(result[index],floor*multiplier)
    return result


class HeuristicSlipCheckerNode(Node):
    """Fresh, frame-correct slip assessment; uncertainty changes, twist does not."""
    def __init__(self):
        super().__init__('heuristic_slip_checker')
        from tf2_ros import Buffer, TransformListener
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import qos_profile_sensor_data
        defaults = {'slip_angular_threshold':.35, 'slip_accel_threshold':.18,
            'covariance_inflation_factor':100., 'nominal_linear_covariance':.02,
            'nominal_angular_covariance':.05, 'slip_hold_time':.4,
            'raw_odom_topic':'/wheel/odom_raw', 'imu_topic':'/imu/data',
            'imu_timeout_sec':.2, 'odom_timeout_sec':.5, 'slip_flag_timeout_sec':.5,
            'imu_includes_gravity':True}
        for key,val in defaults.items(): self.declare_parameter(key,val)
        self.settings = {key:self.get_parameter(key).value for key in defaults}
        for key in ('imu_timeout_sec','odom_timeout_sec','slip_flag_timeout_sec'):
            if not math.isfinite(self.settings[key]) or self.settings[key] <= 0:
                raise ValueError('Positive finite sensor timeout required')
        if self.settings['covariance_inflation_factor'] < 1:
            raise ValueError('Covariance inflation must be at least one')
        self.core = SlipCheckerCore(slip_rot_threshold=self.settings['slip_angular_threshold'],
            slip_accel_threshold=self.settings['slip_accel_threshold'],
            covariance_scale=self.settings['covariance_inflation_factor'],
            nominal_linear_cov=self.settings['nominal_linear_covariance'],
            nominal_angular_cov=self.settings['nominal_angular_covariance'],
            slip_hold_time=self.settings['slip_hold_time'])
        self.latest_imu = None
        self._imu_receipt = self._slip_receipt = self._odom_receipt = None
        self._last_processed_stamp = None
        self._last_clock = self.get_clock().now().nanoseconds/1e9
        self.single_wheel_slip = False
        self.health = 'waiting for wheel odometry'
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer,self)
        self.sub_wheel_odom = self.create_subscription(Odometry,self.settings['raw_odom_topic'],self._wheel_odom_callback,10)
        self.sub_imu = self.create_subscription(Imu,self.settings['imu_topic'],self._imu_callback,qos_profile_sensor_data)
        self.sub_single_wheel_slip = self.create_subscription(Bool,'/wheel/single_wheel_slip',self._single_wheel_slip_callback,10)
        self.pub_filtered_odom = self.create_publisher(Odometry,'/wheel/odom_filtered',10)
        self.pub_slip_flag = self.create_publisher(Bool,'/wheel/slip_detected',10)
        self.pub_diagnostics = self.create_publisher(DiagnosticArray,'/slam/slip_diagnostics',10)
        self.timer = self.create_timer(.1,self._watchdog)

    @staticmethod
    def _stamp(msg): return msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def _now(self):
        now = self.get_clock().now().nanoseconds/1e9
        if now < self._last_clock:
            self.latest_imu = None
            self._imu_receipt = self._slip_receipt = self._odom_receipt = None
            self._last_processed_stamp = None
            self.single_wheel_slip = False
            self.core.last_evaluation_time = float('inf')
            self.health = 'clock reset; waiting for new measurements'
        self._last_clock = now
        return now

    def _single_wheel_slip_callback(self,msg):
        self._slip_receipt = self._now()
        self.single_wheel_slip = msg.data

    def _imu_callback(self,msg):
        self._imu_receipt = self._now()
        self.latest_imu = msg

    def _body_imu(self, target, odom_stamp, now):
        from rclpy.time import Time
        imu = self.latest_imu
        if (imu is None or self._imu_receipt is None
                or not 0 <= now-self._imu_receipt <= self.settings['imu_timeout_sec']
                or not 0 <= now-self._stamp(imu) <= self.settings['imu_timeout_sec']
                or abs(odom_stamp-self._stamp(imu)) > self.settings['imu_timeout_sec']):
            raise ValueError('IMU missing, stale or not aligned in time')
        if not target or not imu.header.frame_id:
            raise ValueError('Missing IMU or wheel body frame')
        if imu.angular_velocity_covariance[0] == -1 or imu.linear_acceleration_covariance[0] == -1:
            raise ValueError('IMU rate or acceleration unavailable')
        gyro=(imu.angular_velocity.x,imu.angular_velocity.y,imu.angular_velocity.z)
        accel=(imu.linear_acceleration.x,imu.linear_acceleration.y,imu.linear_acceleration.z)
        if not all(math.isfinite(v) for v in (*gyro,*accel)):
            raise ValueError('Non-finite IMU data')
        q_bs=(0.,0.,0.,1.)
        if imu.header.frame_id != target:
            rotation=self.buffer.lookup_transform(target,imu.header.frame_id,Time.from_msg(imu.header.stamp)).transform.rotation
            q_bs=normalize_quaternion((rotation.x,rotation.y,rotation.z,rotation.w))
        gyro,accel=rotate_vector(q_bs,gyro),rotate_vector(q_bs,accel)
        q=(imu.orientation.x,imu.orientation.y,imu.orientation.z,imu.orientation.w)
        if self.settings['imu_includes_gravity']:
            if imu.orientation_covariance[0] == -1:
                raise ValueError('Gravity compensation requires valid orientation')
            q=normalize_quaternion(q)
            q=quaternion_product(q,(-q_bs[0],-q_bs[1],-q_bs[2],q_bs[3]))
        else:
            q=(0.,0.,0.,1.)
        return gyro,accel,q

    def _wheel_odom_callback(self,msg):
        import copy
        from tf2_ros import TransformException
        now = self._now()
        stamp = self._stamp(msg)
        if self._last_processed_stamp is not None and stamp <= self._last_processed_stamp:
            self.health = 'duplicate or out-of-order odometry'
            return
        twist=msg.twist.twist
        if (not 0 <= now-stamp <= self.settings['odom_timeout_sec']
                or not msg.header.frame_id or not msg.child_frame_id
                or not all(math.isfinite(v) for v in (twist.linear.x,twist.linear.y,twist.linear.z,
                    twist.angular.x,twist.angular.y,twist.angular.z))):
            self.health = 'invalid or stale wheel odometry'
            return
        slip = (self.single_wheel_slip and self._slip_receipt is not None
                and 0 <= now-self._slip_receipt <= self.settings['slip_flag_timeout_sec'])
        imu_valid = False
        self.health = 'healthy'
        try:
            gyro,accel,orientation = self._body_imu(msg.child_frame_id,stamp,now)
            imu_valid = True
            slip,_,_ = self.core.evaluate(twist.linear.x,twist.angular.z,gyro[2],accel[0],stamp,
                orientation_q=orientation,w_imu_y=gyro[1],single_wheel_slip=slip)
            if slip: self.health='suspected slip; wheel covariance inflated'
        except (ValueError,TransformException) as error:
            self.health='degraded: '+str(error)
        multiplier=self.settings['covariance_inflation_factor'] if slip or not imu_valid else 1.
        filtered = copy.deepcopy(msg)
        try:
            filtered.twist.covariance = inflate_covariance(msg.twist.covariance,multiplier,
                self.settings['nominal_linear_covariance'],self.settings['nominal_angular_covariance'])
        except ValueError as error:
            self.health=str(error)
            return
        self._last_processed_stamp, self._odom_receipt = stamp,now
        self.pub_filtered_odom.publish(filtered)
        self.pub_slip_flag.publish(Bool(data=slip))

    def _watchdog(self):
        from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus
        now=self._now()
        if self._odom_receipt is None or now-self._odom_receipt > self.settings['odom_timeout_sec']:
            self.health='wheel odometry unavailable or stale'
        status=DiagnosticStatus(name='slam/slip_filter',hardware_id='wheel_imu',
            level=DiagnosticStatus.OK if self.health=='healthy' else DiagnosticStatus.WARN,
            message=self.health)
        array=DiagnosticArray(status=[status]); array.header.stamp=self.get_clock().now().to_msg()
        self.pub_diagnostics.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node=HeuristicSlipCheckerNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__': main()
