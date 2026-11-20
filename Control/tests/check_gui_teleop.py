#!/usr/bin/env python3
"""ROS/Qt component integration with synthetic sensors; does not run Gazebo."""

import argparse
import importlib.util
import math
import os
from pathlib import Path
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    control = Path(__file__).resolve().parents[1]
    parser.add_argument('--gui-dir', type=Path, default=control.parents[2] / 'GUI')
    args = parser.parse_args()
    sys.path.insert(0, str(args.gui_dir))
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from PySide6.QtCore import QSettings
    from PySide6.QtWidgets import QApplication
    from sensor_msgs.msg import Image, Imu, JointState
    from rover_dashboard import RoverDashboard

    spec = importlib.util.spec_from_file_location('teleop_guard', control / 'scripts' / 'teleop_guard.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    node = rclpy.create_node('gui_component_check')
    guard = module.TeleopGuard()
    samples = []
    node.create_subscription(Twist, '/cmd_vel',
        lambda msg: samples.append((time.monotonic(), msg.linear.x, msg.angular.z)), 10)
    command_pub = node.create_publisher(Twist, '/cmd_vel_manual', 1)
    joint_pub = node.create_publisher(JointState, '/joint_states_gz', 10)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    imu_pub = node.create_publisher(Imu, '/imu/data', 10)
    image_pub = node.create_publisher(Image, '/camera/image_raw', 10)
    depth_pub = node.create_publisher(Image, '/camera/depth/image_raw', 10)
    app = QApplication([])
    window = None

    def pump(seconds, publish=None):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if publish:
                publish()
            rclpy.spin_once(node, timeout_sec=0.005)
            rclpy.spin_once(guard, timeout_sec=0.005)
            app.processEvents()

    try:
        with tempfile.TemporaryDirectory() as settings_dir:
            QSettings.setDefaultFormat(QSettings.Format.IniFormat)
            QSettings.setPath(QSettings.Format.IniFormat, QSettings.Scope.UserScope, settings_dir)
            window = RoverDashboard()
            window.configure_gazebo_teleop()
            # Disable operator polling only for controlled component inputs below.
            window.preview_timer.stop()
            assert window.topic_fields['cmd_vel'].text() == '/cmd_vel_manual'
            assert window.topic_fields['pose'].text() == '/odom'
            assert not window.safety.can_publish_ros()
            assert window.rover_view.fixed_steering
            window.toggle_ros()
            status_messages = []
            window.ros_link.status.connect(lambda _ok, detail: status_messages.append(detail))
            pump(1.5)
            assert window.ready, 'GUI ROS worker did not start'
            assert window.ros_link.use_sim_time

            def sensors():
                stamp = node.get_clock().now().to_msg()
                joints = JointState()
                joints.header.stamp = stamp
                joints.name = [f'{w}_wheel_joint' for w in
                    ('left_front', 'right_front', 'left_rear', 'right_rear')]
                joints.position = [0.0] * 4
                joints.velocity = [2.0] * 4
                joint_pub.publish(joints)
                odom = Odometry()
                odom.header.stamp = stamp
                odom.pose.pose.orientation.w = 1.0
                odom.pose.pose.position.x = 1.25
                odom.twist.twist.linear.x = 0.12
                odom_pub.publish(odom)
                imu = Imu()
                imu.header.stamp = stamp
                imu.orientation.w = 1.0
                imu.angular_velocity.z = 0.2
                imu_pub.publish(imu)
                image = Image()
                image.header.stamp = stamp
                image.height, image.width, image.step = 2, 2, 6
                image.encoding = 'rgb8'
                image.data = bytes([255, 0, 0] * 4)
                image_pub.publish(image)
                image.encoding, image.step = '16UC1', 4
                image.data = bytes([232, 3] * 4)  # 1000 mm depth.
                depth_pub.publish(image)

            window.camera.source_combo.setCurrentIndex(window.camera.source_combo.findData('rgb'))
            window.camera.start()
            pump(2.0, sensors)
            assert len(window.readings) == 4, 'Four wheel readings did not reach GUI'
            assert math.isclose(window.readings['D_FL']['value'], 60 / math.pi, rel_tol=1e-5)
            assert window.latest_odom and window.latest_odom['x'] == 1.25, (
                window.latest_odom, window.topic_rates, status_messages)
            assert window.camera.image is not None, 'ROS image did not reach camera widget'
            assert time.monotonic() - window.last_imu < 1.0, 'IMU callback stopped telemetry'
            window.camera.source_combo.setCurrentIndex(window.camera.source_combo.findData('depth'))
            pump(1.2, sensors)
            assert window.camera.image is not None and window.ros_link._rate_buckets.get('depth')
            print('PASS: GUI standalone profile, four RPM readings, IMU, pose, RGB and depth delivery')

            window.output_mode_combo.setCurrentIndex(1)
            window.toggle_arm(True)
            assert window.safety.can_publish_ros()
            def drive():
                window.ros_link.set_cmd_vel(0.2, 0.0, -0.3, True)
            pump(1.0, drive)
            assert any(x > 0.19 and z < -0.29 for _, x, z in samples[-10:])
            pump(0.8)
            assert samples[-1][1:] == (0.0, 0.0), 'Stale GUI setpoint was not stopped'
            print('PASS: GUI -> manual topic -> guard -> final Twist; stale UI setpoint stops')

            window.ros_link.set_cmd_vel(0.0, 0.0, 0.0, False)
            def manual(x, y=0.0):
                msg = Twist()
                msg.linear.x, msg.linear.y = x, y
                command_pub.publish(msg)
            pump(0.6, lambda: manual(2.0))
            assert samples[-1][1] == 0.5, 'Guard speed limit failed'
            pump(0.6)
            assert samples[-1][1:] == (0.0, 0.0), 'Lost manual input was not stopped'
            pump(0.3, lambda: manual(float('nan')))
            assert samples[-1][1:] == (0.0, 0.0)
            pump(0.3, lambda: manual(0.2, 0.1))
            assert samples[-1][1:] == (0.0, 0.0)
            window.stop_action()
            assert not window.safety.can_publish_ros()
            print('PASS: guard clamp, missing-input timeout, invalid/lateral rejection, GUI latched stop')
    finally:
        if window:
            window.final_shutdown()
        guard.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
