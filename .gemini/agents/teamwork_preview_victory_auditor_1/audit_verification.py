#!/usr/bin/env python3
"""
Independent Victory Auditor Verification Script
================================================
Audits URDF Xacro expansion, link/joint counts, Gazebo DiffDrive plugin,
RViz configs, documentation, and executes odometry node kinematics independently.
"""

import sys
import os
import math
import subprocess
import xml.etree.ElementTree as ET
import yaml

WORKSPACE = "/workspace"

def audit_urdf_and_gazebo():
    print("=== AUDIT 1: URDF & GAZEBO DIFFDRIVE CONFIGURATION ===")
    
    # 1. Expand Xacro
    urdf_xacro = os.path.join(WORKSPACE, "Rover/my_robot_description/urdf/my_robot.urdf.xacro")
    assert os.path.exists(urdf_xacro), f"Missing {urdf_xacro}"
    
    cmd = ["xacro", urdf_xacro]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert res.returncode == 0, f"xacro failed: {res.stderr}"
    urdf_xml = res.stdout
    
    # 2. Assert no references to middle wheels or middle arms in compiled URDF
    assert "middle" not in urdf_xml.lower(), "Found 'middle' in compiled URDF output!"
    
    root = ET.fromstring(urdf_xml)
    links = [l.attrib["name"] for l in root.findall("link")]
    joints = [j.attrib["name"] for j in root.findall("joint")]
    
    print(f"Total links: {len(links)}")
    print(f"Total joints: {len(joints)}")
    
    expected_links = {
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
    }
    assert set(links) == expected_links, f"Link mismatch: {set(links) ^ expected_links}"
    assert len(links) == 13, f"Expected 13 links, got {len(links)}"
    assert len(joints) == 12, f"Expected 12 joints, got {len(joints)}"
    
    wheel_links = [l for l in links if "wheel_link" in l]
    assert len(wheel_links) == 4, f"Expected 4 wheel links, got {wheel_links}"
    
    arm_links = [l for l in links if "arm_link" in l]
    assert len(arm_links) == 4, f"Expected 4 arm links, got {arm_links}"
    
    # 3. Check wheel joint kinematics in URDF
    # front wheel arm joints at +0.15, rear wheel arm joints at -0.15
    for j in root.findall("joint"):
        j_name = j.attrib["name"]
        origin = j.find("origin")
        if origin is not None:
            xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
            if j_name == "left_front_arm_joint":
                assert math.isclose(xyz[0], 0.15, abs_tol=1e-4), f"left_front_arm_joint X position {xyz[0]} != 0.15"
                assert math.isclose(xyz[1], 0.21, abs_tol=1e-4), f"left_front_arm_joint Y position {xyz[1]} != 0.21"
            elif j_name == "left_rear_arm_joint":
                assert math.isclose(xyz[0], -0.15, abs_tol=1e-4), f"left_rear_arm_joint X position {xyz[0]} != -0.15"
                assert math.isclose(xyz[1], 0.21, abs_tol=1e-4), f"left_rear_arm_joint Y position {xyz[1]} != 0.21"
            elif j_name == "right_front_arm_joint":
                assert math.isclose(xyz[0], 0.15, abs_tol=1e-4), f"right_front_arm_joint X position {xyz[0]} != 0.15"
                assert math.isclose(xyz[1], -0.21, abs_tol=1e-4), f"right_front_arm_joint Y position {xyz[1]} != -0.21"
            elif j_name == "right_rear_arm_joint":
                assert math.isclose(xyz[0], -0.15, abs_tol=1e-4), f"right_rear_arm_joint X position {xyz[0]} != -0.15"
                assert math.isclose(xyz[1], -0.21, abs_tol=1e-4), f"right_rear_arm_joint Y position {xyz[1]} != -0.21"
    
    # 4. Check gazebo.xacro
    gazebo_file = os.path.join(WORKSPACE, "Rover/my_robot_description/urdf/gazebo.xacro")
    with open(gazebo_file, "r") as f:
        gazebo_content = f.read()
    
    # Assert no gazebo reference tags for middle
    assert 'reference="left_middle' not in gazebo_content
    assert 'reference="right_middle' not in gazebo_content
    
    # Find DiffDrive plugin
    assert "gz-sim-diff-drive-system" in gazebo_content
    diff_tree = ET.fromstring(urdf_xml)
    diff_plugin = None
    for p in diff_tree.iter("plugin"):
        if p.attrib.get("filename") == "gz-sim-diff-drive-system":
            diff_plugin = p
            break
    assert diff_plugin is not None, "gz-sim-diff-drive-system plugin not found in compiled URDF"
    
    left_joints = [lj.text.strip() for lj in diff_plugin.findall("left_joint")]
    right_joints = [rj.text.strip() for rj in diff_plugin.findall("right_joint")]
    
    assert left_joints == ["left_front_wheel_joint", "left_rear_wheel_joint"], f"Unexpected left joints: {left_joints}"
    assert right_joints == ["right_front_wheel_joint", "right_rear_wheel_joint"], f"Unexpected right joints: {right_joints}"
    
    wheel_sep = float(diff_plugin.find("wheel_separation").text.strip())
    assert math.isclose(wheel_sep, 0.49, abs_tol=1e-3), f"wheel_separation {wheel_sep} != 0.49"
    
    wheel_rad = float(diff_plugin.find("wheel_radius").text.strip())
    assert math.isclose(wheel_rad, 0.06, abs_tol=1e-3), f"wheel_radius {wheel_rad} != 0.06"
    
    print("✓ AUDIT 1 PASSED: URDF and Gazebo DiffDrive verified.")


def audit_rviz_configs():
    print("=== AUDIT 2: RVIZ CONFIGURATIONS ===")
    rviz_files = [
        os.path.join(WORKSPACE, "Rover/my_robot_description/rviz/robot_view.rviz"),
        os.path.join(WORKSPACE, "SLAM/rover_slam/config/slam_visualization.rviz"),
        os.path.join(WORKSPACE, "Perception/marker_detection/rviz/marker_detection_view.rviz"),
        os.path.join(WORKSPACE, "Perception/terrain_geometry/rviz/terrain_geometry_view.rviz"),
    ]
    for rf in rviz_files:
        assert os.path.exists(rf), f"RViz config file {rf} does not exist"
        with open(rf, "r") as f:
            content = f.read()
        
        # Check that middle wheel and middle arm are completely absent
        assert "left_middle" not in content, f"Found left_middle in {rf}"
        assert "right_middle" not in content, f"Found right_middle in {rf}"
        
        # Verify valid YAML
        parsed = yaml.safe_load(content)
        assert isinstance(parsed, dict), f"Failed to parse YAML in {rf}"
        print(f"✓ {os.path.basename(rf)}: Valid YAML with 0 middle wheel/arm references")
    print("✓ AUDIT 2 PASSED: All RViz configurations clean.")


def audit_documentation():
    print("=== AUDIT 3: DOCUMENTATION ===")
    rover_readme = os.path.join(WORKSPACE, "Rover/my_robot_description/README.md")
    with open(rover_readme, "r") as f:
        r_text = f.read()
    assert "4" in r_text and ("4-wheel" in r_text or "4 wheels" in r_text or "4-wheeled" in r_text)
    assert "Total Links**: 13" in r_text and "Total Joints**: 12" in r_text
    
    slam_readme = os.path.join(WORKSPACE, "SLAM/README.md")
    with open(slam_readme, "r") as f:
        s_text = f.read()
    assert "4-wheel" in s_text or "4 wheels" in s_text or "4-wheeled" in s_text
    print("✓ AUDIT 3 PASSED: Documentation correctly reflects 4-wheel specification.")


def audit_odometry_node_execution():
    print("=== AUDIT 4: ODOMETRY NODE INDEPENDENT EXECUTION ===")
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int64MultiArray
    from rclpy.time import Duration
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode, compute_robust_side_velocity
    
    if not rclpy.ok():
        rclpy.init()
    
    node = EncoderTicksToOdomNode()
    
    # 1. Parameter checks
    assert node.wheel_names == ["left_front", "right_front", "left_rear", "right_rear"]
    assert math.isclose(node.track_width, 0.49, abs_tol=1e-4)
    assert math.isclose(node.wheel_radius, 0.06, abs_tol=1e-4)
    assert node.ticks_per_rev == 1024
    
    # 2. JointState parsing with URDF joint names + ignoring arm & steer joints
    js = JointState()
    js.name = [
        "left_front_arm_joint",          # MUST BE IGNORED
        "left_front_wheel_joint",        # 2*pi rad = 1 rev = 1024 ticks
        "right_front_steer_joint",       # MUST BE IGNORED
        "right_front_wheel_joint",       # 2*pi rad = 1 rev = 1024 ticks
        "left_rear_wheel_joint",         # 2*pi rad = 1 rev = 1024 ticks
        "right_rear_wheel_joint",        # 2*pi rad = 1 rev = 1024 ticks
        "left_rear_arm_joint",           # MUST BE IGNORED
    ]
    js.position = [0.0, 2.0 * math.pi, 0.5, 2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi, 0.0]
    
    # Initialize ticks baseline
    node._joint_states_callback(js)
    for w in node.wheel_names:
        assert node.current_ticks[w] == 1024, f"Wheel {w} ticks {node.current_ticks[w]} != 1024"
    
    # 3. Forward straight displacement kinematics
    # Advance wheels by another 1024 ticks (1 revolution = 2 * pi * 0.06 = 0.37699m) over dt = 1.0s
    js2 = JointState()
    js2.name = js.name
    js2.position = [0.0, 4.0 * math.pi, 0.5, 4.0 * math.pi, 4.0 * math.pi, 4.0 * math.pi, 0.0]
    node._joint_states_callback(js2)
    
    node.last_time = node.get_clock().now() - Duration(seconds=1, nanoseconds=0)
    node._update_odometry()
    
    expected_dist = 2.0 * math.pi * 0.06
    print(f"Computed displacement x: {node.x:.5f}, Expected: {expected_dist:.5f}")
    assert math.isclose(node.x, expected_dist, rel_tol=1e-3), f"Displacement {node.x} != {expected_dist}"
    assert math.isclose(node.y, 0.0, abs_tol=1e-5), f"Lateral displacement {node.y} != 0"
    assert math.isclose(node.yaw, 0.0, abs_tol=1e-5), f"Heading yaw {node.yaw} != 0"
    
    # 4. Pure skid-steer rotation in place
    # Rotate in place: left wheels reverse 100 ticks (-0.0368m), right wheels forward 100 ticks (+0.0368m)
    current_x = node.x
    current_y = node.y
    initial_yaw = node.yaw
    
    node.current_ticks["left_front"] -= 100
    node.current_ticks["left_rear"] -= 100
    node.current_ticks["right_front"] += 100
    node.current_ticks["right_rear"] += 100
    
    dt = 1.0
    node.last_time = node.get_clock().now() - Duration(seconds=1, nanoseconds=0)
    node._update_odometry()
    
    wheel_circ = 2.0 * math.pi * 0.06
    v_left = (-100 * (wheel_circ / 1024)) / dt
    v_right = (+100 * (wheel_circ / 1024)) / dt
    expected_omega = (v_right - v_left) / 0.49
    expected_yaw = initial_yaw + expected_omega * dt
    
    print(f"In-place rotation yaw: {node.yaw:.5f}, Expected: {expected_yaw:.5f}")
    assert math.isclose(node.yaw, expected_yaw, rel_tol=1e-3)
    assert math.isclose(node.x, current_x, rel_tol=1e-3)
    assert math.isclose(node.y, current_y, abs_tol=1e-4)
    
    # 5. Stationary zero-drift verification
    x_before = node.x
    y_before = node.y
    yaw_before = node.yaw
    
    for _ in range(10):
        node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
        node._update_odometry()
        assert math.isclose(node.x, x_before, abs_tol=1e-12)
        assert math.isclose(node.y, y_before, abs_tol=1e-12)
        assert math.isclose(node.yaw, yaw_before, abs_tol=1e-12)
        for w in node.wheel_names:
            assert node.wheel_velocities[w] == 0.0
            assert node.wheel_delta_ticks[w] == 0
    print("✓ Stationary zero-drift verified across 10 odometry cycles.")
    
    # 6. Single wheel slip handling
    # Left front slipping at 1.0 m/s, Left rear traction at 0.2 m/s -> selects traction wheel (0.2)
    v_side, slip = compute_robust_side_velocity([1.0, 0.2], slip_diff_threshold=0.15)
    assert math.isclose(v_side, 0.2, abs_tol=1e-4), f"Expected 0.2 m/s, got {v_side}"
    assert slip is True, "Expected slip flag True"
    
    # NaN and Inf safety in side velocity
    v_side_nan, slip_nan = compute_robust_side_velocity([float('nan'), 0.3])
    assert math.isclose(v_side_nan, 0.3, abs_tol=1e-4)
    assert slip_nan is True
    
    node.destroy_node()
    rclpy.shutdown()
    print("✓ AUDIT 4 PASSED: Odometry node kinematics, drift, and slip verified.")

if __name__ == "__main__":
    audit_urdf_and_gazebo()
    audit_rviz_configs()
    audit_documentation()
    audit_odometry_node_execution()
    print("\n=======================================================")
    print("ALL INDEPENDENT VICTORY AUDITOR CHECKS PASSED (100%)!")
    print("=======================================================")
