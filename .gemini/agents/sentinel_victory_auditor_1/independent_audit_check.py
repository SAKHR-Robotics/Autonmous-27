#!/usr/bin/env python3
"""
Sentinel Victory Auditor Independent Check Script
Validates:
1. R1: URDF Kinematics & Structure Refactoring (links, joints, tree, coordinates)
2. R2: Gazebo Physics & DiffDrive Plugin Update
3. R3: Telemetry, Odometry & RViz Configuration Alignment
4. Code Quality & Non-cheating / No facade implementations
"""

import sys
import os
import math
import subprocess
import xml.etree.ElementTree as ET
import yaml

WORKSPACE = "/workspace" if os.path.exists("/workspace/Rover") else os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))

def test_urdf_xacro_and_tree():
    print(">>> 1. Validating URDF Xacro generation and tree structure...")
    xacro_file = os.path.join(WORKSPACE, "Rover/my_robot_description/urdf/my_robot.urdf.xacro")
    assert os.path.isfile(xacro_file), f"File not found: {xacro_file}"

    # Run xacro
    cmd = ["xacro", xacro_file]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert res.returncode == 0, f"Xacro execution failed: {res.stderr}"
    urdf_content = res.stdout

    # Check that middle is nowhere in the generated URDF
    assert "middle" not in urdf_content.lower(), "Found 'middle' in generated URDF!"

    root = ET.fromstring(urdf_content)
    links = [l.attrib["name"] for l in root.findall("link")]
    joints = [j.attrib["name"] for j in root.findall("joint")]

    print(f"    Links count: {len(links)}")
    print(f"    Joints count: {len(joints)}")

    expected_links = [
        "base_footprint",
        "base_link",
        "imu_link",
        "camera_link",
        "my_robot/camera_link/camera",
        "left_front_arm_link",
        "left_front_wheel_link",
        "left_rear_arm_link",
        "left_rear_wheel_link",
        "right_front_arm_link",
        "right_front_wheel_link",
        "right_rear_arm_link",
        "right_rear_wheel_link",
    ]
    assert sorted(links) == sorted(expected_links), f"Links mismatch: {set(links) ^ set(expected_links)}"

    expected_wheel_links = [
        "left_front_wheel_link",
        "left_rear_wheel_link",
        "right_front_wheel_link",
        "right_rear_wheel_link",
    ]
    actual_wheel_links = [l for l in links if "wheel_link" in l]
    assert sorted(actual_wheel_links) == sorted(expected_wheel_links), f"Wheel links mismatch: {actual_wheel_links}"

    # Verify positions: front at +0.15, rear at -0.15
    joint_origins = {}
    for j in root.findall("joint"):
        j_name = j.attrib["name"]
        origin = j.find("origin")
        if origin is not None:
            xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
            joint_origins[j_name] = xyz

    assert math.isclose(joint_origins["left_front_arm_joint"][0], 0.15, abs_tol=1e-4)
    assert math.isclose(joint_origins["left_rear_arm_joint"][0], -0.15, abs_tol=1e-4)
    assert math.isclose(joint_origins["right_front_arm_joint"][0], 0.15, abs_tol=1e-4)
    assert math.isclose(joint_origins["right_rear_arm_joint"][0], -0.15, abs_tol=1e-4)
    print("    [PASS] URDF tree & positions verified.")

def test_gazebo_diffdrive():
    print(">>> 2. Validating Gazebo DiffDrive plugin...")
    gazebo_file = os.path.join(WORKSPACE, "Rover/my_robot_description/urdf/gazebo.xacro")
    with open(gazebo_file, "r") as f:
        content = f.read()

    assert "left_middle" not in content, "Found left_middle in gazebo.xacro"
    assert "right_middle" not in content, "Found right_middle in gazebo.xacro"

    # Check compiled diff drive plugin in URDF
    xacro_file = os.path.join(WORKSPACE, "Rover/my_robot_description/urdf/my_robot.urdf.xacro")
    cmd = ["xacro", xacro_file]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    root = ET.fromstring(res.stdout)

    diff_plugin = None
    for p in root.iter("plugin"):
        if p.attrib.get("filename") == "gz-sim-diff-drive-system":
            diff_plugin = p
            break
    assert diff_plugin is not None, "DiffDrive plugin not found in compiled URDF!"

    left_joints = [j.text.strip() for j in diff_plugin.findall("left_joint")]
    right_joints = [j.text.strip() for j in diff_plugin.findall("right_joint")]

    assert left_joints == ["left_front_wheel_joint", "left_rear_wheel_joint"], f"Left joints: {left_joints}"
    assert right_joints == ["right_front_wheel_joint", "right_rear_wheel_joint"], f"Right joints: {right_joints}"

    sep = float(diff_plugin.find("wheel_separation").text.strip())
    assert math.isclose(sep, 0.49, abs_tol=1e-3), f"wheel_separation: {sep}"
    print("    [PASS] DiffDrive plugin verified.")

def test_rviz_and_docs():
    print(">>> 3. Validating RViz configurations and documentation...")
    rviz_paths = [
        "Rover/my_robot_description/rviz/robot_view.rviz",
        "SLAM/rover_slam/config/slam_visualization.rviz",
        "Perception/marker_detection/rviz/marker_detection_view.rviz",
        "Perception/terrain_geometry/rviz/terrain_geometry_view.rviz",
    ]
    for rel_path in rviz_paths:
        full_path = os.path.join(WORKSPACE, rel_path)
        with open(full_path, "r") as f:
            text = f.read()
        assert "middle" not in text.lower(), f"Found middle in {rel_path}"
        data = yaml.safe_load(text)
        assert isinstance(data, dict), f"Failed to parse {rel_path} as YAML"
        print(f"    [PASS] RViz file valid & clean: {rel_path}")

    # Check READMEs
    rover_readme = os.path.join(WORKSPACE, "Rover/my_robot_description/README.md")
    with open(rover_readme, "r") as f:
        r_text = f.read()
    assert "4-wheeled Mars rover" in r_text or "4 wheels" in r_text
    assert "Total Links**: 13" in r_text
    assert "Total Joints**: 12" in r_text

    slam_readme = os.path.join(WORKSPACE, "SLAM/README.md")
    with open(slam_readme, "r") as f:
        s_text = f.read()
    assert "4-wheel" in s_text

    print("    [PASS] Documentation verified.")

def test_odometry_kinematics():
    print(">>> 4. Validating Odometry Node and Kinematics...")
    import rclpy
    from sensor_msgs.msg import JointState
    from rclpy.time import Duration
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode

    if not rclpy.ok():
        rclpy.init()

    node = EncoderTicksToOdomNode()
    assert node.wheel_names == ["left_front", "right_front", "left_rear", "right_rear"]
    assert math.isclose(node.track_width, 0.49, abs_tol=1e-4)

    # Drive 1 meter forward
    # ticks = 1.0 / (2*pi*0.06 / 1024) = 2715.4 ticks
    cpr = 1024
    radius = 0.06
    circ = 2.0 * math.pi * radius
    revolutions = 2.0
    expected_dist = revolutions * circ
    rads = revolutions * 2.0 * math.pi

    # Baseline
    js0 = JointState()
    js0.name = ["left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint"]
    js0.position = [0.0, 0.0, 0.0, 0.0]
    node._joint_states_callback(js0)

    # Step 1
    js1 = JointState()
    js1.name = js0.name
    js1.position = [rads, rads, rads, rads]
    node._joint_states_callback(js1)

    node.last_time = node.get_clock().now() - Duration(seconds=1, nanoseconds=0)
    node._update_odometry()

    assert math.isclose(node.x, expected_dist, rel_tol=1e-3), f"Displacement {node.x} != {expected_dist}"
    assert math.isclose(node.y, 0.0, abs_tol=1e-5)
    assert math.isclose(node.yaw, 0.0, abs_tol=1e-5)

    node.destroy_node()
    rclpy.shutdown()
    print("    [PASS] Odometry kinematics verified.")

if __name__ == "__main__":
    test_urdf_xacro_and_tree()
    test_gazebo_diffdrive()
    test_rviz_and_docs()
    test_odometry_kinematics()
    print("\nALL SENTINEL POST-VICTORY AUDIT CHECKS PASSED SUCCESSFULLY!")
