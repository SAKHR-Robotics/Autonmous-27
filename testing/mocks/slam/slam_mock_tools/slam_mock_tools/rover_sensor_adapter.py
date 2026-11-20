"""Test-only sensors from the shared mock rover; never publishes estimated TF."""
import json
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Int64, Int64MultiArray, String


def _rot_xyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


class RoverSensorAdapter(Node):
    WHEEL_ORDER = ('left_front', 'right_front', 'left_rear', 'right_rear')

    def __init__(self):
        super().__init__('slam_sensor_adapter')
        self.declare_parameter('drop_imu', False)
        self.declare_parameter('drop_wheels', False)
        self.declare_parameter('seed', 0)
        self.declare_parameter('imu_accel_noise_std', 0.0)
        self.declare_parameter('imu_gyro_noise_std', 0.0)
        self.declare_parameter('imu_gyro_bias_z', 0.0)
        self.declare_parameter('imu_frame_id', '')
        self.declare_parameter('imu_mount_roll_rad', 0.0)
        self.declare_parameter('imu_mount_pitch_rad', 0.0)
        self.declare_parameter('imu_mount_yaw_rad', 0.0)
        self.declare_parameter('gravity_sign', 1.0)
        self.declare_parameter('wheel_output_mode', 'joint_states')
        self.declare_parameter('ticks_per_revolution', 1024)
        self.declare_parameter('wheel_slip_scale', 1.0)
        self.declare_parameter('single_wheel_slip_index', -1)

        mode = str(self.get_parameter('wheel_output_mode').value)
        if mode not in ('joint_states', 'ticks_array', 'individual_ticks'):
            raise ValueError(f'Unsupported wheel_output_mode: {mode!r}')
        self.wheel_output_mode = mode
        self._rng = np.random.default_rng(int(self.get_parameter('seed').value))

        self.previous = None
        self._prev_side_pos = None
        self._wheel_accum = [0.0, 0.0, 0.0, 0.0]

        self.imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.wheels = self.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
        self.ticks_array_pub = self.create_publisher(Int64MultiArray, '/wheel/ticks', 10)
        self.individual_tick_pubs = {
            name: self.create_publisher(Int64, f'/wheel/ticks/{name}', 10)
            for name in self.WHEEL_ORDER
        }
        self.fault_state_pub = self.create_publisher(String, '/testing/slam/fault_state', 10)

        self.create_subscription(Odometry, '/testing/slam/truth', self.observe_truth, 10)
        self.create_subscription(JointState, '/testing/slam/side_joints', self.observe_wheels, 10)

    def _publish_fault_state(self):
        payload = {
            'drop_imu': bool(self.get_parameter('drop_imu').value),
            'drop_wheels': bool(self.get_parameter('drop_wheels').value),
            'wheel_slip_scale': float(self.get_parameter('wheel_slip_scale').value),
            'single_wheel_slip_index': int(self.get_parameter('single_wheel_slip_index').value),
            'wheel_output_mode': str(self.get_parameter('wheel_output_mode').value),
        }
        self.fault_state_pub.publish(String(data=json.dumps(payload)))

    def observe_truth(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        velocity, yaw_rate = msg.twist.twist.linear.x, msg.twist.twist.angular.z
        if not all(math.isfinite(v) for v in (stamp, velocity, yaw_rate)):
            self.previous = None
            return
        previous, self.previous = self.previous, (stamp, velocity)
        if previous is None or not 0 < stamp - previous[0] <= .5:
            return
        if self.get_parameter('drop_imu').value:
            self._publish_fault_state()
            return

        ax_body = (velocity - previous[1]) / (stamp - previous[0])
        ay_body = velocity * yaw_rate
        az_body = float(self.get_parameter('gravity_sign').value) * 9.81
        accel_body = np.array([ax_body, ay_body, az_body], dtype=float)
        gyro_body = np.array([0.0, 0.0, yaw_rate], dtype=float)

        roll = float(self.get_parameter('imu_mount_roll_rad').value)
        pitch = float(self.get_parameter('imu_mount_pitch_rad').value)
        mount_yaw = float(self.get_parameter('imu_mount_yaw_rad').value)
        if roll != 0.0 or pitch != 0.0 or mount_yaw != 0.0:
            r_imu_from_body = _rot_xyz(roll, pitch, mount_yaw).T
            accel_imu = r_imu_from_body @ accel_body
            gyro_imu = r_imu_from_body @ gyro_body
        else:
            accel_imu = accel_body
            gyro_imu = gyro_body

        accel_std = float(self.get_parameter('imu_accel_noise_std').value)
        gyro_std = float(self.get_parameter('imu_gyro_noise_std').value)
        gyro_bias_z = float(self.get_parameter('imu_gyro_bias_z').value)
        if accel_std > 0.0:
            accel_imu = accel_imu + self._rng.normal(0.0, accel_std, size=3)
        if gyro_std > 0.0:
            gyro_imu = gyro_imu + self._rng.normal(0.0, gyro_std, size=3)
        gyro_imu[2] += gyro_bias_z

        imu = Imu()
        imu.header.stamp = msg.header.stamp
        custom_frame = str(self.get_parameter('imu_frame_id').value)
        imu.header.frame_id = custom_frame if custom_frame else msg.child_frame_id
        imu.orientation = msg.pose.pose.orientation
        imu.angular_velocity.x = float(gyro_imu[0])
        imu.angular_velocity.y = float(gyro_imu[1])
        imu.angular_velocity.z = float(gyro_imu[2])
        imu.linear_acceleration.x = float(accel_imu[0])
        imu.linear_acceleration.y = float(accel_imu[1])
        imu.linear_acceleration.z = float(accel_imu[2])
        for covariance in (
            imu.orientation_covariance,
            imu.angular_velocity_covariance,
            imu.linear_acceleration_covariance,
        ):
            for index in (0, 4, 8):
                covariance[index] = 1e-4
        self.imu.publish(imu)
        self._publish_fault_state()

    def observe_wheels(self, msg):
        if self.get_parameter('drop_wheels').value:
            self._publish_fault_state()
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in ('left_wheel_joint', 'right_wheel_joint')):
            return
        pos_l = float(positions['left_wheel_joint'])
        pos_r = float(positions['right_wheel_joint'])
        if not (math.isfinite(pos_l) and math.isfinite(pos_r)):
            return

        slip_scale = float(self.get_parameter('wheel_slip_scale').value)
        single_idx = int(self.get_parameter('single_wheel_slip_index').value)

        if slip_scale == 1.0 and single_idx < 0 and self._prev_side_pos is None:
            values = [pos_l, pos_r, pos_l, pos_r]
            self._wheel_accum = list(values)
            self._prev_side_pos = (pos_l, pos_r)
        elif self._prev_side_pos is None:
            values = [pos_l, pos_r, pos_l, pos_r]
            self._wheel_accum = list(values)
            self._prev_side_pos = (pos_l, pos_r)
        else:
            dl = pos_l - self._prev_side_pos[0]
            dr = pos_r - self._prev_side_pos[1]
            self._prev_side_pos = (pos_l, pos_r)
            deltas = [dl, dr, dl, dr]
            for idx in range(4):
                if single_idx < 0:
                    self._wheel_accum[idx] += deltas[idx] * slip_scale
                elif idx == single_idx:
                    self._wheel_accum[idx] += deltas[idx] * slip_scale
                else:
                    self._wheel_accum[idx] += deltas[idx]
            values = list(self._wheel_accum)

        mode = str(self.get_parameter('wheel_output_mode').value)
        if mode == 'joint_states':
            wheels = JointState()
            wheels.header = msg.header
            wheels.name = [name + '_wheel_joint' for name in self.WHEEL_ORDER]
            wheels.position = values
            self.wheels.publish(wheels)
        elif mode == 'ticks_array':
            tpr = int(self.get_parameter('ticks_per_revolution').value)
            ticks = [int(round(v * tpr / (2.0 * math.pi))) for v in values]
            self.ticks_array_pub.publish(Int64MultiArray(data=ticks))
        elif mode == 'individual_ticks':
            tpr = int(self.get_parameter('ticks_per_revolution').value)
            for name, v in zip(self.WHEEL_ORDER, values):
                self.individual_tick_pubs[name].publish(Int64(data=int(round(v * tpr / (2.0 * math.pi)))))
        else:
            raise ValueError(f'Unsupported wheel_output_mode: {mode!r}')
        self._publish_fault_state()


def main(args=None):
    rclpy.init(args=args)
    node = RoverSensorAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
