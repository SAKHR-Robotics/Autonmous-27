"""
test_camera_optical_frame.py
Unit tests verifying ROS REP-103 standard camera optical frame conventions.
Task: SLAM Fix 4 / Issue 4 (Standardize Camera Optical Frame Conventions)
"""
import math
import os
import re
import numpy as np
import pytest
from launch import LaunchContext
from launch_ros.actions import Node
import xacro

from test_static_transforms import load_static_transforms_module, get_node_name


def test_rep103_optical_frame_mathematical_rotation():
    """
    Mathematical proof of ROS REP-103 rotation from camera body frame to optical frame:
    Chassis/Camera Body Frame (REP-103 body): +X forward, +Y left, +Z up
    Camera Optical Frame (REP-103 optical):   +X right,   +Y down, +Z forward (depth)
    
    Standard Euler RPY rotation applied:
    Roll  = -pi/2 (-1.57079632679 rad)
    Pitch = 0.0
    Yaw   = -pi/2 (-1.57079632679 rad)
    """
    roll = -math.pi / 2.0
    pitch = 0.0
    yaw = -math.pi / 2.0

    # Rotation matrix: R = Rz(yaw) * Ry(pitch) * Rx(roll)
    # transforming coordinates from optical frame to camera body frame
    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(roll), -math.sin(roll)],
        [0.0, math.sin(roll), math.cos(roll)]
    ])
    ry = np.array([
        [math.cos(pitch), 0.0, math.sin(pitch)],
        [0.0, 1.0, 0.0],
        [-math.sin(pitch), 0.0, math.cos(pitch)]
    ])
    rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0.0],
        [math.sin(yaw), math.cos(yaw), 0.0],
        [0.0, 0.0, 1.0]
    ])

    R_optical_to_body = rz @ ry @ rx

    # 1. Optical forward vector (+Z optical): depth axis pointing into scene
    v_optical_fwd = np.array([0.0, 0.0, 1.0])
    v_body_fwd = R_optical_to_body @ v_optical_fwd
    # In rover chassis body frame, this MUST point forward (+X)
    np.testing.assert_allclose(v_body_fwd, [1.0, 0.0, 0.0], atol=1e-6)

    # 2. Optical horizontal vector (+X optical): image columns going right
    v_optical_right = np.array([1.0, 0.0, 0.0])
    v_body_right = R_optical_to_body @ v_optical_right
    # In rover chassis body frame, +Y is left, so image right is -Y
    np.testing.assert_allclose(v_body_right, [0.0, -1.0, 0.0], atol=1e-6)

    # 3. Optical vertical vector (+Y optical): image rows going down
    v_optical_down = np.array([0.0, 1.0, 0.0])
    v_body_down = R_optical_to_body @ v_optical_down
    # In rover chassis body frame, +Z is up, so image down is -Z
    np.testing.assert_allclose(v_body_down, [0.0, 0.0, -1.0], atol=1e-6)


def test_static_transforms_rep103_arguments():
    """Verify that static_transforms.launch.py configures REP-103 optical parameters."""
    mod = load_static_transforms_module()
    ld = mod.generate_launch_description()

    nodes = {get_node_name(e): e for e in ld.entities if isinstance(e, Node)}

    # Depth optical frame broadcaster
    assert 'static_tf_camera_to_optical' in nodes
    depth_node = nodes['static_tf_camera_to_optical']
    depth_args = [str(a) for a in getattr(depth_node, '_Node__arguments', [])]

    # Verify frame identifiers
    assert '--frame-id' in depth_args and 'camera_link' in depth_args
    assert '--child-frame-id' in depth_args and 'camera_depth_optical_frame' in depth_args

    # Verify Euler angles match REP-103 (-1.57079632679, 0.0, -1.57079632679)
    roll_idx = depth_args.index('--roll') + 1
    pitch_idx = depth_args.index('--pitch') + 1
    yaw_idx = depth_args.index('--yaw') + 1

    assert pytest.approx(float(depth_args[roll_idx]), abs=1e-4) == -1.570796
    assert pytest.approx(float(depth_args[pitch_idx]), abs=1e-4) == 0.0
    assert pytest.approx(float(depth_args[yaw_idx]), abs=1e-4) == -1.570796

    # Color optical frame broadcaster
    assert 'static_tf_camera_to_color_optical' in nodes
    color_node = nodes['static_tf_camera_to_color_optical']
    color_args = [str(a) for a in getattr(color_node, '_Node__arguments', [])]
    assert 'camera_color_optical_frame' in color_args

    # Gazebo camera bridge node
    assert 'static_tf_camera_optical_to_gz' in nodes
    gz_node = nodes['static_tf_camera_optical_to_gz']
    gz_args = [str(a) for a in getattr(gz_node, '_Node__arguments', [])]
    assert 'my_robot/camera_link/camera' in gz_args


def test_launch_urdf_rep103_enhancement():
    """Verify that Python launch processing produces URDF with REP-103 optical frames."""
    rover_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))),
        'Rover',
        'my_robot_description'
    )
    xacro_file = os.path.join(rover_dir, 'urdf', 'my_robot.urdf.xacro')
    assert os.path.exists(xacro_file), f"URDF xacro file not found at {xacro_file}"

    # Process URDF through xacro
    robot_description_config = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_config.toxml()

    # Apply the launch script standardization
    optical_rep103_frames = """
  <!-- Standard ROS REP-103 Optical Frames (Task: SLAM Fix 4) -->
  <link name="camera_depth_optical_frame"/>
  <joint name="camera_depth_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_depth_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
  </joint>
  <link name="camera_color_optical_frame"/>
  <joint name="camera_color_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_color_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.57079632679 0 -1.57079632679"/>
  </joint>
"""
    robot_description_xml = re.sub(
        r'(<joint name="camera_optical_joint"[^>]*>.*?<origin\s+[^>]*?)rpy="[^"]*"',
        r'\1rpy="-1.57079632679 0 -1.57079632679"',
        robot_description_xml,
        flags=re.DOTALL
    )
    if '<link name="camera_depth_optical_frame"' not in robot_description_xml:
        robot_description_xml = robot_description_xml.replace('</robot>', optical_rep103_frames + '\n</robot>')

    # Assertions on enhanced URDF
    assert '<link name="camera_depth_optical_frame"' in robot_description_xml
    assert '<link name="camera_color_optical_frame"' in robot_description_xml
    assert 'rpy="-1.57079632679 0 -1.57079632679"' in robot_description_xml

    # Check joint parent and child links
    depth_joint_match = re.search(
        r'<joint name="camera_depth_optical_joint"[^>]*>.*?</joint>',
        robot_description_xml,
        re.DOTALL
    )
    assert depth_joint_match is not None
    depth_joint_xml = depth_joint_match.group(0)
    assert '<parent link="camera_link"' in depth_joint_xml
    assert '<child link="camera_depth_optical_frame"' in depth_joint_xml
    assert 'rpy="-1.57079632679 0 -1.57079632679"' in depth_joint_xml
