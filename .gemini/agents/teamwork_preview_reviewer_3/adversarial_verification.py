#!/usr/bin/env python3
"""
adversarial_verification.py - Reviewer 3 Independent Verification Suite
Tests the complete 4-wheel rover refactoring against all edge cases.
"""

import math
import subprocess
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray
from rover_slam.encoder_ticks_to_odom import (
    EncoderTicksToOdomNode,
    compute_robust_side_velocity,
    euler_to_quaternion,
)


def test_stationary_zero_drift():
    print("\n--- Suite 1: Stationary Odometry Zero Drift ---")
    node = EncoderTicksToOdomNode()

    # Step 0: establish baseline at 0 rad
    js0 = JointState()
    js0.name = ["left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint"]
    js0.position = [0.0, 0.0, 0.0, 0.0]
    node._joint_states_callback(js0)

    # Step 1: rotate wheels by 1 radian
    js1 = JointState()
    js1.name = js0.name
    js1.position = [1.0, 1.0, 1.0, 1.0]
    node._joint_states_callback(js1)

    node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
    node._update_odometry()
    x1 = node.x
    assert x1 > 0.0, f"Expected positive movement, got {x1}"

    # Step 2: robot sitting still for 10 consecutive timer steps with NO new callbacks
    for step in range(10):
        node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
        node._update_odometry()
        assert abs(node.x - x1) < 1e-12, f"Drift detected at step {step}: x moved to {node.x} from {x1}"
        for w in node.wheel_names:
            assert abs(node.wheel_velocities[w]) < 1e-12, f"Wheel velocity non-zero: {node.wheel_velocities[w]}"
            assert node.wheel_delta_ticks[w] == 0, f"Wheel delta ticks non-zero: {node.wheel_delta_ticks[w]}"

    node.destroy_node()
    print("✓ PASSED: Odometry produces 0.0 velocity and exactly zero drift when stationary!")


def test_multi_message_accumulation():
    print("\n--- Suite 2: Multi-Message Tick Accumulation Between Odom Cycles ---")
    node = EncoderTicksToOdomNode()

    # Initial ticks
    msg0 = Int64MultiArray()
    msg0.data = [100, 100, 100, 100]
    node._ticks_array_callback(msg0)
    node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
    node._update_odometry()

    # Deliver 4 intermediate burst messages before next timer cycle
    deltas = [20, 30, 50, 100]
    cumulative = 100
    for d in deltas:
        cumulative += d
        msg = Int64MultiArray()
        msg.data = [cumulative, cumulative, cumulative, cumulative]
        node._ticks_array_callback(msg)

    # Step timer
    node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
    node._update_odometry()

    expected_delta = sum(deltas)  # 200
    for w in node.wheel_names:
        assert node.wheel_delta_ticks[w] == expected_delta, (
            f"Expected {expected_delta} total delta ticks, got {node.wheel_delta_ticks[w]}"
        )

    node.destroy_node()
    print("✓ PASSED: All intermediate tick bursts accumulated without tick loss!")


def test_nan_inf_and_steering_safety():
    print("\n--- Suite 3: NaN/Inf Safety & Steering Joint Rejection ---")
    node = EncoderTicksToOdomNode()

    # Non-finite values in JointState
    js_bad = JointState()
    js_bad.name = ["left_front_wheel_joint", "right_front_wheel_joint"]
    js_bad.position = [float('nan'), float('inf')]
    node._joint_states_callback(js_bad)
    assert node.current_ticks["left_front"] == 0
    assert node.current_ticks["right_front"] == 0

    # Non-finite values in compute_robust_side_velocity
    v, slip = compute_robust_side_velocity([float('nan'), 0.40])
    assert v == 0.40
    assert slip is True

    v_all_nan, slip_all_nan = compute_robust_side_velocity([float('nan'), float('nan')])
    assert v_all_nan == 0.0
    assert slip_all_nan is False

    # Steering joints ignored
    js_steer = JointState()
    js_steer.name = [
        "left_front_steer_joint",
        "right_front_steering_joint",
        "left_rear_steering_wheel_joint",
        "right_rear_steer_joint",
    ]
    js_steer.position = [3.14, 3.14, 3.14, 3.14]
    node._joint_states_callback(js_steer)
    for w in node.wheel_names:
        assert node.current_ticks[w] == 0, f"Steering joint incorrectly updated {w}"

    # Zero track width protection
    node.track_width = 0.0
    node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
    node._update_odometry()  # Must not raise ZeroDivisionError

    # Yaw normalization
    node.yaw = 25.0 * math.pi + 0.1
    node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
    node._update_odometry()
    assert -math.pi <= node.yaw <= math.pi, f"Yaw not normalized: {node.yaw}"

    node.destroy_node()
    print("✓ PASSED: Non-finite values, steering joints, track width, and yaw handled safely!")


def test_full_system_verification():
    print("\n--- Suite 4: URDF, Gazebo & State Publisher Coherence ---")
    urdf = subprocess.check_output(
        ["xacro", "Rover/my_robot_description/urdf/my_robot.urdf.xacro"],
        text=True
    )
    assert urdf.lower().count("middle") == 0, "Found 'middle' in URDF!"
    root = ET.fromstring(urdf)

    # Verify active links
    wheel_links = [l.attrib["name"] for l in root.findall("link") if "wheel" in l.attrib["name"]]
    assert len(wheel_links) == 4, f"Expected 4 wheel links, got {len(wheel_links)}: {wheel_links}"
    assert set(wheel_links) == {
        "left_front_wheel_link",
        "left_rear_wheel_link",
        "right_front_wheel_link",
        "right_rear_wheel_link"
    }

    # Verify active joints
    wheel_joints = [j.attrib["name"] for j in root.findall("joint") if "wheel" in j.attrib["name"]]
    assert len(wheel_joints) == 4, f"Expected 4 wheel joints, got {len(wheel_joints)}: {wheel_joints}"
    assert set(wheel_joints) == {
        "left_front_wheel_joint",
        "left_rear_wheel_joint",
        "right_front_wheel_joint",
        "right_rear_wheel_joint"
    }

    # Verify DiffDrive plugin
    diff_plugin = None
    for p in root.findall(".//plugin"):
        if p.attrib.get("name") == "gz::sim::systems::DiffDrive":
            diff_plugin = p
            break
    assert diff_plugin is not None, "DiffDrive plugin missing!"

    joints = [c.text for c in diff_plugin.findall("left_joint")] + [c.text for c in diff_plugin.findall("right_joint")]
    assert len(joints) == 4
    assert set(joints) == {
        "left_front_wheel_joint",
        "left_rear_wheel_joint",
        "right_front_wheel_joint",
        "right_rear_wheel_joint",
    }
    assert float(diff_plugin.find("wheel_separation").text) == 0.49
    assert float(diff_plugin.find("wheel_radius").text) == 0.06

    print("✓ PASSED: URDF, 4-wheel topology, and Gazebo DiffDrive plugin fully verified!")


if __name__ == "__main__":
    rclpy.init()
    test_stationary_zero_drift()
    test_multi_message_accumulation()
    test_nan_inf_and_steering_safety()
    test_full_system_verification()
    rclpy.shutdown()
    print("\n🎉 ALL REVIEWER 3 ADVERSARIAL VERIFICATIONS PASSED!")
