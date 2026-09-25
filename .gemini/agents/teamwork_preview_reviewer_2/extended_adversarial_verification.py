#!/usr/bin/env python3
"""
extended_adversarial_verification.py
Deep adversarial verification suite for Mars rover 4-wheel refactoring (Reviewer 2).
"""

import math
import subprocess
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray
from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode


def test_stationary_zero_drift():
    print("\n--- Test 1: Stationary Odometry Zero Drift ---")
    node = EncoderTicksToOdomNode()

    # Step 0: establish baseline at 0 rad
    js0 = JointState()
    js0.name = ["left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint"]
    js0.position = [0.0, 0.0, 0.0, 0.0]
    node._joint_states_callback(js0)

    # Step 1: rotate wheels
    js1 = JointState()
    js1.name = js0.name
    js1.position = [1.0, 1.0, 1.0, 1.0]
    node._joint_states_callback(js1)

    node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
    node._update_odometry()
    x1 = node.x
    assert x1 > 0.0, f"Expected positive movement, got {x1}"

    # Step 2: robot sitting still for 5 consecutive 50Hz timer ticks with NO new callbacks
    for step in range(5):
        node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
        node._update_odometry()
        assert abs(node.x - x1) < 1e-9, f"Drift detected at step {step}: x moved to {node.x} from {x1}"
        for w in node.wheel_names:
            assert abs(node.wheel_velocities[w]) < 1e-9, f"Wheel velocity non-zero: {node.wheel_velocities[w]}"
            assert node.wheel_delta_ticks[w] == 0, f"Wheel delta ticks non-zero: {node.wheel_delta_ticks[w]}"

    node.destroy_node()
    print("✓ PASSED: Odometry produces 0.0 velocity and exactly zero drift when stationary!")


def test_multi_message_accumulation():
    print("\n--- Test 2: Multi-Message Tick Accumulation Between Odom Cycles ---")
    node = EncoderTicksToOdomNode()

    # Initial ticks
    msg0 = Int64MultiArray()
    msg0.data = [100, 100, 100, 100]
    node._ticks_array_callback(msg0)
    node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
    node._update_odometry()

    # Deliver 3 messages before next timer cycle
    msg1 = Int64MultiArray()
    msg1.data = [120, 120, 120, 120]  # +20
    node._ticks_array_callback(msg1)

    msg2 = Int64MultiArray()
    msg2.data = [150, 150, 150, 150]  # +30
    node._ticks_array_callback(msg2)

    msg3 = Int64MultiArray()
    msg3.data = [200, 200, 200, 200]  # +50
    node._ticks_array_callback(msg3)

    # Step timer
    node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
    node._update_odometry()

    for w in node.wheel_names:
        assert node.wheel_delta_ticks[w] == 100, f"Expected 100 total delta ticks, got {node.wheel_delta_ticks[w]}"

    node.destroy_node()
    print("✓ PASSED: All intermediate tick bursts accumulated without tick loss!")


def test_nan_inf_and_steering_safety():
    print("\n--- Test 3: NaN/Inf Safety & Steering Joint Rejection ---")
    node = EncoderTicksToOdomNode()

    # Non-finite values
    js_bad = JointState()
    js_bad.name = ["left_front_wheel_joint", "right_front_wheel_joint"]
    js_bad.position = [float('nan'), float('inf')]
    node._joint_states_callback(js_bad)
    assert node.current_ticks["left_front"] == 0
    assert node.current_ticks["right_front"] == 0

    # Steering joints
    js_steer = JointState()
    js_steer.name = ["left_front_steer_joint", "right_front_steering_joint", "left_rear_steering_wheel_joint"]
    js_steer.position = [3.14, 3.14, 3.14]
    node._joint_states_callback(js_steer)
    for w in node.wheel_names:
        assert node.current_ticks[w] == 0, f"Steering joint incorrectly updated {w}"

    node.destroy_node()
    print("✓ PASSED: Non-finite values and steering joints handled safely!")


def test_full_system_verification():
    print("\n--- Test 4: URDF, Gazebo & State Publisher Coherence ---")
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
    assert float(diff_plugin.find("wheel_separation").text) == 0.49

    print("✓ PASSED: URDF, 4-wheel topology, and Gazebo DiffDrive plugin fully verified!")


if __name__ == "__main__":
    rclpy.init()
    test_stationary_zero_drift()
    test_multi_message_accumulation()
    test_nan_inf_and_steering_safety()
    test_full_system_verification()
    rclpy.shutdown()
    print("\n🎉 ALL REVIEWER 2 ADVERSARIAL VERIFICATIONS PASSED!")
