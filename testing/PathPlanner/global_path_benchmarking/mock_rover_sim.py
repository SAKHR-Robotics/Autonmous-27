#!/usr/bin/env python3
"""
mock_rover_sim.py

Universal Real-Time Rover Kinematics & Actuator Physics Simulator.
Simulates a differential/skid-steer rover in response to velocity commands (/cmd_vel)
or downstream motor commands (/rover/motor_rpm).

Features:
- Dual Twist & TwistStamped auto-handling
- Slew-rate acceleration limiting (linear & angular)
- Skid-steer velocity splitting and wheel RPM conversion
- Motor deadband and RPM saturation clamping
- Realistic Martian terrain wheel slip and Gaussian sensor noise
- Two-tier control testing (Trajectory controller vs Motor driver)
- Publishes /odometry/filtered (50 Hz), TF (map -> odom -> base_link),
  /rover/motor_rpm, /rover/joint_states, and visual 3D markers for RViz2.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import (
    Twist,
    TwistStamped,
    TransformStamped,
    PoseStamped,
    PoseWithCovarianceStamped
)
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class MockRoverSim(Node):
    def __init__(self):
        super().__init__('mock_rover_sim')

        # Spatial & Frame Parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        # Vehicle Physical Dimensions
        self.declare_parameter('rover_length', 0.8)
        self.declare_parameter('rover_width', 0.65)
        self.declare_parameter('rover_height', 0.3)
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('track_width', 0.65)

        # Actuator Dynamics & Limits
        self.declare_parameter('max_linear_accel', 1.2)      # m/s^2
        self.declare_parameter('max_angular_accel', 2.0)     # rad/s^2
        self.declare_parameter('max_rpm', 120.0)             # RPM
        self.declare_parameter('deadband_linear', 0.02)      # m/s
        self.declare_parameter('deadband_angular', 0.03)     # rad/s

        # Terrain Interaction (Slip & Noise)
        self.declare_parameter('slip_ratio_linear', 0.15)    # 15% slip
        self.declare_parameter('slip_ratio_angular', 0.10)   # 10% angular slip
        self.declare_parameter('noise_std_v', 0.02)          # m/s
        self.declare_parameter('noise_std_w', 0.03)          # rad/s

        # Interface Options
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('use_stamped_cmd_vel', False)
        self.declare_parameter('control_mode', 'cmd_vel')    # 'cmd_vel' or 'motor_rpm'

        # Fetch Parameter Values
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.update_rate = float(self.get_parameter('update_rate').value)

        self.rover_length = float(self.get_parameter('rover_length').value)
        self.rover_width = float(self.get_parameter('rover_width').value)
        self.rover_height = float(self.get_parameter('rover_height').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.track_width = float(self.get_parameter('track_width').value)

        self.max_linear_accel = float(self.get_parameter('max_linear_accel').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.deadband_linear = float(self.get_parameter('deadband_linear').value)
        self.deadband_angular = float(self.get_parameter('deadband_angular').value)

        self.slip_ratio_linear = float(self.get_parameter('slip_ratio_linear').value)
        self.slip_ratio_angular = float(self.get_parameter('slip_ratio_angular').value)
        self.noise_std_v = float(self.get_parameter('noise_std_v').value)
        self.noise_std_w = float(self.get_parameter('noise_std_w').value)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.use_stamped_cmd_vel = self.get_parameter('use_stamped_cmd_vel').value
        self.control_mode = self.get_parameter('control_mode').value

        # Robot State (in odom / map coordinates)
        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)

        # Commanded targets (raw from planner)
        self.target_vx = 0.0
        self.target_wz = 0.0

        # Slewed velocities (after acceleration limiter)
        self.slewed_vx = 0.0
        self.slewed_wz = 0.0

        # Actual velocities (after actuator limits & slip)
        self.actual_vx = 0.0
        self.actual_wz = 0.0

        # Wheel speeds
        self.rpm_left = 0.0
        self.rpm_right = 0.0
        self.wheel_pos_left = 0.0
        self.wheel_pos_right = 0.0

        # Command timeout handling
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/rover_marker', 10)
        self.rpm_pub = self.create_publisher(Float32MultiArray, '/rover/motor_rpm', 10)
        self.joint_pub = self.create_publisher(JointState, '/rover/joint_states', 10)

        # Subscribers
        if self.use_stamped_cmd_vel:
            self.cmd_sub = self.create_subscription(
                TwistStamped,
                self.cmd_vel_topic,
                self.cmd_vel_stamped_callback,
                10
            )
        else:
            self.cmd_sub = self.create_subscription(
                Twist,
                self.cmd_vel_topic,
                self.cmd_vel_callback,
                10
            )
            # Also listen to stamped version if published on topic/stamped
            self.cmd_stamped_aux_sub = self.create_subscription(
                TwistStamped,
                self.cmd_vel_topic + '/stamped',
                self.cmd_vel_stamped_callback,
                10
            )

        # Motor driver input subscriber (Tier 2 Control mode)
        self.motor_rpm_sub = self.create_subscription(
            Float32MultiArray,
            '/motor_driver/rpm_feedback',
            self.motor_rpm_callback,
            10
        )

        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10
        )

        self.set_pose_sub = self.create_subscription(
            PoseStamped,
            '/start_pose',
            self.set_pose_callback,
            10
        )

        # Simulation Timer (e.g. 50 Hz)
        dt = 1.0 / self.update_rate
        self.timer = self.create_timer(dt, self.update_physics)
        self.last_time = self.get_clock().now()

        self.get_logger().info(
            f"Universal Mock Rover Sim initialized at ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f} rad). "
            f"Rate: {self.update_rate}Hz, Mode: {self.control_mode}, Topic: {self.cmd_vel_topic}"
        )

    def cmd_vel_callback(self, msg: Twist):
        self.target_vx = msg.linear.x
        self.target_wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def cmd_vel_stamped_callback(self, msg: TwistStamped):
        self.target_vx = msg.twist.linear.x
        self.target_wz = msg.twist.angular.z
        self.last_cmd_time = self.get_clock().now()

    def motor_rpm_callback(self, msg: Float32MultiArray):
        """Tier 2 testing: directly receives RPM commands from user's motor driver."""
        if self.control_mode == 'motor_rpm' and len(msg.data) >= 2:
            self.rpm_left = float(msg.data[0])
            self.rpm_right = float(msg.data[1])
            self.last_cmd_time = self.get_clock().now()

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.reset_velocities()
        self.get_logger().info(f"Rover reset via /initialpose to ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f})")

    def set_pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.reset_velocities()
        self.get_logger().info(f"Rover reset via /start_pose to ({self.x:.2f}, {self.y:.2f}, yaw={self.yaw:.2f})")

    def reset_velocities(self):
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.slewed_vx = 0.0
        self.slewed_wz = 0.0
        self.actual_vx = 0.0
        self.actual_wz = 0.0
        self.rpm_left = 0.0
        self.rpm_right = 0.0

    def update_physics(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time

        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.update_rate

        # Stop robot if command has timed out
        cmd_age = (current_time - self.last_cmd_time).nanoseconds * 1e-9
        if cmd_age > self.cmd_timeout:
            self.target_vx = 0.0
            self.target_wz = 0.0
            if self.control_mode == 'motor_rpm':
                self.rpm_left = 0.0
                self.rpm_right = 0.0

        if self.control_mode == 'cmd_vel':
            # 1. Apply Actuator Deadband
            target_v = self.target_vx if abs(self.target_vx) >= self.deadband_linear else 0.0
            target_w = self.target_wz if abs(self.target_wz) >= self.deadband_angular else 0.0

            # 2. Slew-Rate Acceleration Limiter
            max_dv = self.max_linear_accel * dt
            max_dw = self.max_angular_accel * dt
            self.slewed_vx = float(np.clip(target_v, self.slewed_vx - max_dv, self.slewed_vx + max_dv))
            self.slewed_wz = float(np.clip(target_w, self.slewed_wz - max_dw, self.slewed_wz + max_dw))

            # 3. Skid-Steer Kinematic Decomposition
            v_left = self.slewed_vx - (self.slewed_wz * self.track_width / 2.0)
            v_right = self.slewed_vx + (self.slewed_wz * self.track_width / 2.0)

            # Convert linear velocity to RPM
            rpm_l = (v_left * 60.0) / (2.0 * math.pi * self.wheel_radius)
            rpm_r = (v_right * 60.0) / (2.0 * math.pi * self.wheel_radius)

            # Clamp to max RPM
            self.rpm_left = float(np.clip(rpm_l, -self.max_rpm, self.max_rpm))
            self.rpm_right = float(np.clip(rpm_r, -self.max_rpm, self.max_rpm))

            # Recalculate achievable velocities from clamped RPM
            v_left_achieved = (self.rpm_left * 2.0 * math.pi * self.wheel_radius) / 60.0
            v_right_achieved = (self.rpm_right * 2.0 * math.pi * self.wheel_radius) / 60.0
            achieved_vx = (v_right_achieved + v_left_achieved) / 2.0
            achieved_wz = (v_right_achieved - v_left_achieved) / self.track_width

        else:
            # Mode: motor_rpm (direct wheel input)
            self.rpm_left = float(np.clip(self.rpm_left, -self.max_rpm, self.max_rpm))
            self.rpm_right = float(np.clip(self.rpm_right, -self.max_rpm, self.max_rpm))
            v_left_achieved = (self.rpm_left * 2.0 * math.pi * self.wheel_radius) / 60.0
            v_right_achieved = (self.rpm_right * 2.0 * math.pi * self.wheel_radius) / 60.0
            achieved_vx = (v_right_achieved + v_left_achieved) / 2.0
            achieved_wz = (v_right_achieved - v_left_achieved) / self.track_width

        # 4. Realistic Terrain Interaction (Longitudinal & Rotational Slip + Sensor Noise)
        slip_v_factor = max(0.0, 1.0 - self.slip_ratio_linear)
        slip_w_factor = max(0.0, 1.0 - self.slip_ratio_angular)

        noise_v = float(np.random.normal(0.0, self.noise_std_v)) if (abs(achieved_vx) > 0.01) else 0.0
        noise_w = float(np.random.normal(0.0, self.noise_std_w)) if (abs(achieved_wz) > 0.01) else 0.0

        self.actual_vx = achieved_vx * slip_v_factor + noise_v
        self.actual_wz = achieved_wz * slip_w_factor + noise_w

        # 5. Kinematic Integration (Position Update)
        self.yaw += self.actual_wz * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        dx = (self.actual_vx * math.cos(self.yaw)) * dt
        dy = (self.actual_vx * math.sin(self.yaw)) * dt
        self.x += dx
        self.y += dy

        # Wheel positions
        self.wheel_pos_left += (self.rpm_left * 2.0 * math.pi / 60.0) * dt
        self.wheel_pos_right += (self.rpm_right * 2.0 * math.pi / 60.0) * dt

        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, self.yaw)

        # 6. Publish Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.actual_vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.actual_wz

        self.odom_pub.publish(odom)

        # 7. Publish Diagnostics (Motor RPM & JointStates)
        rpm_msg = Float32MultiArray()
        rpm_msg.data = [self.rpm_left, self.rpm_right]
        self.rpm_pub.publish(rpm_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_msg.position = [self.wheel_pos_left, self.wheel_pos_right]
        joint_msg.velocity = [
            (self.rpm_left * 2.0 * math.pi) / 60.0,
            (self.rpm_right * 2.0 * math.pi) / 60.0
        ]
        self.joint_pub.publish(joint_msg)

        # 8. Broadcast Dynamic TF Transforms
        if self.publish_tf:
            t_map_odom = TransformStamped()
            t_map_odom.header.stamp = current_time.to_msg()
            t_map_odom.header.frame_id = self.map_frame
            t_map_odom.child_frame_id = self.odom_frame
            t_map_odom.transform.translation.x = 0.0
            t_map_odom.transform.translation.y = 0.0
            t_map_odom.transform.translation.z = 0.0
            t_map_odom.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t_map_odom)

            t_odom_base = TransformStamped()
            t_odom_base.header.stamp = current_time.to_msg()
            t_odom_base.header.frame_id = self.odom_frame
            t_odom_base.child_frame_id = self.base_frame
            t_odom_base.transform.translation.x = self.x
            t_odom_base.transform.translation.y = self.y
            t_odom_base.transform.translation.z = 0.15
            t_odom_base.transform.rotation.x = qx
            t_odom_base.transform.rotation.y = qy
            t_odom_base.transform.rotation.z = qz
            t_odom_base.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t_odom_base)

            # base_link -> base_footprint (for AMCL / Nav2 compatibility)
            t_base_footprint = TransformStamped()
            t_base_footprint.header.stamp = current_time.to_msg()
            t_base_footprint.header.frame_id = self.base_frame
            t_base_footprint.child_frame_id = 'base_footprint'
            t_base_footprint.transform.translation.x = 0.0
            t_base_footprint.transform.translation.y = 0.0
            t_base_footprint.transform.translation.z = -0.15
            t_base_footprint.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t_base_footprint)

        # 9. Publish Visual Markers for RViz2
        self.publish_rover_markers(current_time)

    def publish_rover_markers(self, stamp: Time):
        markers = MarkerArray()

        # Chassis Box
        chassis = Marker()
        chassis.header.stamp = stamp.to_msg()
        chassis.header.frame_id = self.base_frame
        chassis.ns = "rover_chassis"
        chassis.id = 0
        chassis.type = Marker.CUBE
        chassis.action = Marker.ADD
        chassis.pose.position.x = 0.0
        chassis.pose.position.y = 0.0
        chassis.pose.position.z = self.rover_height / 2.0
        chassis.pose.orientation.w = 1.0
        chassis.scale.x = self.rover_length
        chassis.scale.y = self.rover_width
        chassis.scale.z = self.rover_height
        chassis.color.r = 0.85
        chassis.color.g = 0.35
        chassis.color.b = 0.15
        chassis.color.a = 0.85
        markers.markers.append(chassis)

        # Heading Arrow
        arrow = Marker()
        arrow.header.stamp = stamp.to_msg()
        arrow.header.frame_id = self.base_frame
        arrow.ns = "heading_arrow"
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = self.rover_length / 2.0
        arrow.pose.position.y = 0.0
        arrow.pose.position.z = self.rover_height / 2.0
        arrow.pose.orientation.w = 1.0
        arrow.scale.x = 0.4
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color.r = 0.1
        arrow.color.g = 0.9
        arrow.color.b = 0.2
        arrow.color.a = 0.95
        markers.markers.append(arrow)

        # Left Wheels Marker
        wheel_l = Marker()
        wheel_l.header.stamp = stamp.to_msg()
        wheel_l.header.frame_id = self.base_frame
        wheel_l.ns = "wheels_left"
        wheel_l.id = 2
        wheel_l.type = Marker.CYLINDER
        wheel_l.action = Marker.ADD
        wheel_l.pose.position.x = 0.0
        wheel_l.pose.position.y = self.track_width / 2.0
        wheel_l.pose.position.z = 0.0
        qx, qy, qz, qw = euler_to_quaternion(math.pi / 2.0, 0.0, 0.0)
        wheel_l.pose.orientation.x = qx
        wheel_l.pose.orientation.y = qy
        wheel_l.pose.orientation.z = qz
        wheel_l.pose.orientation.w = qw
        wheel_l.scale.x = self.wheel_radius * 2.0
        wheel_l.scale.y = self.wheel_radius * 2.0
        wheel_l.scale.z = 0.1
        wheel_l.color.r = 0.2
        wheel_l.color.g = 0.2
        wheel_l.color.b = 0.2
        wheel_l.color.a = 1.0
        markers.markers.append(wheel_l)

        # Right Wheels Marker
        wheel_r = Marker()
        wheel_r.header.stamp = stamp.to_msg()
        wheel_r.header.frame_id = self.base_frame
        wheel_r.ns = "wheels_right"
        wheel_r.id = 3
        wheel_r.type = Marker.CYLINDER
        wheel_r.action = Marker.ADD
        wheel_r.pose.position.x = 0.0
        wheel_r.pose.position.y = -self.track_width / 2.0
        wheel_r.pose.position.z = 0.0
        wheel_r.pose.orientation.x = qx
        wheel_r.pose.orientation.y = qy
        wheel_r.pose.orientation.z = qz
        wheel_r.pose.orientation.w = qw
        wheel_r.scale.x = self.wheel_radius * 2.0
        wheel_r.scale.y = self.wheel_radius * 2.0
        wheel_r.scale.z = 0.1
        wheel_r.color.r = 0.2
        wheel_r.color.g = 0.2
        wheel_r.color.b = 0.2
        wheel_r.color.a = 1.0
        markers.markers.append(wheel_r)

        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = MockRoverSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
