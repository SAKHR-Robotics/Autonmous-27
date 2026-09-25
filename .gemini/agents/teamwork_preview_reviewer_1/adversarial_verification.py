#!/usr/bin/env python3
"""
adversarial_verification.py
Deep adversarial verification suite for Mars rover 4-wheel refactoring.
Tests:
1. Rejection of arm and non-wheel joints in JointState messages.
2. Kinematic calculation with 0.49m track width.
3. Legacy 6-wheel array ingestion on /wheel/ticks.
4. URDF structure and Gazebo plugin parameters.
"""

import math
import subprocess
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray
from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode


def test_adversarial_joint_states():
    print("\n--- Test 1: Adversarial JointState matching (arm joints rejection) ---")
    if not rclpy.ok():
        rclpy.init()

    node = EncoderTicksToOdomNode()
    
    # Send JointState with arm joints BEFORE wheel joints
    js1 = JointState()
    js1.name = [
        "base_footprint_joint",
        "left_front_arm_joint",
        "left_front_wheel_joint",
        "right_front_arm_joint",
        "right_front_wheel_joint",
        "left_rear_arm_joint",
        "left_rear_wheel_joint",
        "right_rear_arm_joint",
        "right_rear_wheel_joint",
        "imu_joint",
        "camera_joint",
    ]
    # Arm joints are at 0.0, wheels at 2*pi (1024 ticks)
    js1.position = [0.0, 0.0, 2.0 * math.pi, 0.0, 2.0 * math.pi, 0.0, 2.0 * math.pi, 0.0, 2.0 * math.pi, 0.0, 0.0]
    node._joint_states_callback(js1)

    for w in ["left_front", "right_front", "left_rear", "right_rear"]:
        assert node.current_ticks[w] == 1024, f"{w} current_ticks expected 1024, got {node.current_ticks[w]}"
        assert node.prev_ticks[w] == 1024, f"{w} prev_ticks expected 1024, got {node.prev_ticks[w]}"
        assert node.wheel_delta_ticks[w] == 0, f"{w} delta_ticks expected 0 on init, got {node.wheel_delta_ticks[w]}"

    # Send JointState with arm joints AFTER wheel joints (reversed order), wheels at 4*pi (2048 ticks)
    js2 = JointState()
    js2.name = list(reversed(js1.name))
    js2.position = [0.0, 0.0, 4.0 * math.pi, 0.0, 4.0 * math.pi, 0.0, 4.0 * math.pi, 0.0, 4.0 * math.pi, 0.0, 0.0]
    node._joint_states_callback(js2)

    for w in ["left_front", "right_front", "left_rear", "right_rear"]:
        assert node.current_ticks[w] == 2048, f"{w} current_ticks expected 2048, got {node.current_ticks[w]}"
        assert node.prev_ticks[w] == 1024, f"{w} prev_ticks expected 1024, got {node.prev_ticks[w]}"
        assert node.wheel_delta_ticks[w] == 1024, f"{w} delta_ticks expected 1024, got {node.wheel_delta_ticks[w]}"

    node.destroy_node()
    print("✓ PASSED: Arm joints completely ignored, no tick corruption or direction reversal!")


def test_adversarial_track_width_kinematics():
    print("\n--- Test 2: Kinematic calculation with 0.49m baseline ---")
    if not rclpy.ok():
        rclpy.init()

    node = EncoderTicksToOdomNode()
    assert node.track_width == 0.49, f"Default track_width expected 0.49, got {node.track_width}"

    # Set delta ticks corresponding to left=0.1 m and right=0.3 m in dt=1.0s
    dt = 1.0
    dist_left = 0.1
    dist_right = 0.3
    ticks_left = int(dist_left / node.meters_per_tick)
    ticks_right = int(dist_right / node.meters_per_tick)

    node.wheel_delta_ticks["left_front"] = ticks_left
    node.wheel_delta_ticks["left_rear"] = ticks_left
    node.wheel_delta_ticks["right_front"] = ticks_right
    node.wheel_delta_ticks["right_rear"] = ticks_right

    # Simulate odometry step
    from rclpy.time import Duration
    node.last_time = node.get_clock().now() - Duration(seconds=1, nanoseconds=0)
    node._update_odometry()

    expected_omega = (dist_right - dist_left) / 0.49
    computed_omega = (node.wheel_velocities["right_front"] - node.wheel_velocities["left_front"]) / node.track_width
    print(f"Computed omega_z: {computed_omega:.4f} rad/s, Expected: {expected_omega:.4f} rad/s")
    assert abs(computed_omega - expected_omega) < 0.01, f"Omega mismatch: {computed_omega} vs {expected_omega}"

    node.destroy_node()
    print("✓ PASSED: Kinematic angular rate precisely matches 0.49m baseline!")


def test_adversarial_legacy_6wheel_payload():
    print("\n--- Test 3: Legacy 6-wheel array payload ingestion ---")
    if not rclpy.ok():
        rclpy.init()

    node = EncoderTicksToOdomNode()
    msg = Int64MultiArray()
    # Layout [LF, LM, LR, RF, RM, RR]
    msg.data = [500, 9999, 600, 700, 8888, 800]
    node._ticks_array_callback(msg)

    assert node.current_ticks["left_front"] == 500
    assert node.current_ticks["left_rear"] == 600
    assert node.current_ticks["right_front"] == 700
    assert node.current_ticks["right_rear"] == 800
    print(f"Mapped ticks: LF={node.current_ticks['left_front']}, LR={node.current_ticks['left_rear']}, RF={node.current_ticks['right_front']}, RR={node.current_ticks['right_rear']}")

    node.destroy_node()
    print("✓ PASSED: Legacy 6-wheel payload correctly extracted without ingesting middle wheels!")


def test_adversarial_urdf_gazebo():
    print("\n--- Test 4: URDF & Gazebo DiffDrive separation parameter check ---")
    urdf = subprocess.check_output(
        ["xacro", "Rover/my_robot_description/urdf/my_robot.urdf.xacro"],
        text=True
    )
    assert urdf.lower().count("middle") == 0, "Found 'middle' in URDF!"
    root = ET.fromstring(urdf)

    diff_plugin = None
    for p in root.findall(".//plugin"):
        if p.attrib.get("name") == "gz::sim::systems::DiffDrive":
            diff_plugin = p
            break
    assert diff_plugin is not None, "DiffDrive plugin not found in URDF!"

    wheel_sep = diff_plugin.find("wheel_separation").text
    print(f"Gazebo DiffDrive wheel_separation: {wheel_sep}")
    assert float(wheel_sep) == 0.49, f"Expected wheel_separation 0.49, got {wheel_sep}"

    joints = [c.text for c in diff_plugin.findall("left_joint")] + [c.text for c in diff_plugin.findall("right_joint")]
    print(f"DiffDrive controlled joints ({len(joints)}): {joints}")
    assert len(joints) == 4
    assert set(joints) == {
        "left_front_wheel_joint",
        "left_rear_wheel_joint",
        "right_front_wheel_joint",
        "right_rear_wheel_joint",
    }
    print("✓ PASSED: URDF and Gazebo DiffDrive parameters fully consistent!")


if __name__ == "__main__":
    test_adversarial_joint_states()
    test_adversarial_track_width_kinematics()
    test_adversarial_legacy_6wheel_payload()
    test_adversarial_urdf_gazebo()
    print("\n🎉 ALL ADVERSARIAL VERIFICATION TESTS PASSED SUCCESSFULLY!")
