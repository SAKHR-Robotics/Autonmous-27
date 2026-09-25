import subprocess
import xml.etree.ElementTree as ET
import sys

def verify():
    # 1. Process Xacro
    print("Running xacro on Rover/my_robot_description/urdf/my_robot.urdf.xacro...")
    urdf = subprocess.check_output(
        ["xacro", "Rover/my_robot_description/urdf/my_robot.urdf.xacro"],
        text=True
    )
    
    # 2. Check for middle
    middle_count = urdf.lower().count("middle")
    print(f"Occurrences of 'middle' in generated URDF: {middle_count}")
    assert middle_count == 0, f"Found {middle_count} occurrences of 'middle' in URDF!"

    # 3. Parse XML
    root = ET.fromstring(urdf)
    links = [l.attrib["name"] for l in root.findall("link")]
    wheel_links = [l for l in links if "wheel_link" in l]
    arm_links = [l for l in links if "arm_link" in l]
    joints = [j.attrib["name"] for j in root.findall("joint")]
    wheel_joints = [j for j in joints if "wheel_joint" in j]
    arm_joints = [j for j in joints if "arm_joint" in j]

    print(f"Total links ({len(links)}): {links}")
    print(f"Wheel links ({len(wheel_links)}): {wheel_links}")
    print(f"Arm links ({len(arm_links)}): {arm_links}")
    print(f"Total joints ({len(joints)}): {joints}")
    print(f"Wheel joints ({len(wheel_joints)}): {wheel_joints}")
    print(f"Arm joints ({len(arm_joints)}): {arm_joints}")

    expected_wheel_links = {
        "left_front_wheel_link",
        "left_rear_wheel_link",
        "right_front_wheel_link",
        "right_rear_wheel_link",
    }
    expected_arm_links = {
        "left_front_arm_link",
        "left_rear_arm_link",
        "right_front_arm_link",
        "right_rear_arm_link",
    }
    expected_wheel_joints = {
        "left_front_wheel_joint",
        "left_rear_wheel_joint",
        "right_front_wheel_joint",
        "right_rear_wheel_joint",
    }

    assert set(wheel_links) == expected_wheel_links, f"Wheel links mismatch: {set(wheel_links)}"
    assert set(arm_links) == expected_arm_links, f"Arm links mismatch: {set(arm_links)}"
    assert set(wheel_joints) == expected_wheel_joints, f"Wheel joints mismatch: {set(wheel_joints)}"
    assert len(links) == 13, f"Expected 13 total links, got {len(links)}"
    assert len(joints) == 12, f"Expected 12 total joints, got {len(joints)}"

    # 4. Check DiffDrive plugin
    diff_joints = []
    for p in root.findall(".//plugin"):
        if p.attrib.get("name") == "gz::sim::systems::DiffDrive":
            for child in p:
                if child.tag in ("left_joint", "right_joint"):
                    diff_joints.append((child.tag, child.text))
    print(f"DiffDrive plugin joints ({len(diff_joints)}): {diff_joints}")
    expected_diff_joints = [
        ("left_joint", "left_front_wheel_joint"),
        ("left_joint", "left_rear_wheel_joint"),
        ("right_joint", "right_front_wheel_joint"),
        ("right_joint", "right_rear_wheel_joint"),
    ]
    assert diff_joints == expected_diff_joints, f"DiffDrive joints mismatch: {diff_joints}"

    print("\n✓ ALL URDF & GAZEBO CHECKS PASSED PERFECTLY!")

if __name__ == "__main__":
    verify()
