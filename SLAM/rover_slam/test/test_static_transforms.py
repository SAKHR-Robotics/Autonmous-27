"""
test_static_transforms.py
Unit tests verifying static transforms launch structure and conditions.
Task: SLAM Fix 3 / Issue 3 (Resolve duplicate static TF broadcasters)
"""
import os
import importlib.util
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def load_static_transforms_module():
    launch_path = os.path.join(
        os.path.dirname(os.path.dirname(__file__)),
        'launch',
        'static_transforms.launch.py'
    )
    spec = importlib.util.spec_from_file_location('static_transforms', launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def get_node_name(node: Node) -> str:
    # Safely retrieve configured name prior to action execution
    name_attr = getattr(node, '_Node__node_name', None)
    if isinstance(name_attr, list):
        return ''.join(str(item) for item in name_attr)
    return str(name_attr)


def test_static_transforms_launch_description_structure():
    """Verify that static_transforms.launch.py declares required arguments and configures conditions."""
    mod = load_static_transforms_module()
    ld = mod.generate_launch_description()

    entities = ld.entities
    declared_args = {
        e.name: e.default_value[0].text
        for e in entities
        if isinstance(e, DeclareLaunchArgument) and e.default_value is not None
    }

    assert 'standalone' in declared_args
    assert declared_args['standalone'] == 'false'
    assert 'publish_optical_tf' in declared_args
    assert declared_args['publish_optical_tf'] == 'true'

    nodes = [e for e in entities if isinstance(e, Node)]
    node_names = [get_node_name(n) for n in nodes]

    assert 'static_tf_base_to_camera' in node_names
    assert 'static_tf_base_to_imu' in node_names
    assert 'static_tf_camera_to_optical' in node_names
    assert 'static_tf_camera_to_color_optical' in node_names
    assert 'static_tf_camera_optical_to_gz' in node_names

    # Check conditions
    for node in nodes:
        assert node.condition is not None


def test_static_transforms_conditions_evaluation():
    """Verify that redundant transforms are inactive by default (standalone=false) and active when standalone=true."""
    mod = load_static_transforms_module()
    ld = mod.generate_launch_description()

    nodes = {get_node_name(e): e for e in ld.entities if isinstance(e, Node)}

    # Context 1: Integrated mode (standalone=false, robot_state_publisher active)
    ctx_integrated = LaunchContext()
    ctx_integrated.launch_configurations['standalone'] = 'false'
    ctx_integrated.launch_configurations['publish_optical_tf'] = 'true'

    assert not nodes['static_tf_base_to_camera'].condition.evaluate(ctx_integrated)
    assert not nodes['static_tf_base_to_imu'].condition.evaluate(ctx_integrated)
    assert nodes['static_tf_camera_to_optical'].condition.evaluate(ctx_integrated)
    assert nodes['static_tf_camera_to_color_optical'].condition.evaluate(ctx_integrated)
    assert nodes['static_tf_camera_optical_to_gz'].condition.evaluate(ctx_integrated)

    # Context 2: Standalone mode (isolated sensor testing, no robot_state_publisher)
    ctx_standalone = LaunchContext()
    ctx_standalone.launch_configurations['standalone'] = 'true'
    ctx_standalone.launch_configurations['publish_optical_tf'] = 'true'

    assert nodes['static_tf_base_to_camera'].condition.evaluate(ctx_standalone)
    assert nodes['static_tf_base_to_imu'].condition.evaluate(ctx_standalone)
    assert nodes['static_tf_camera_to_optical'].condition.evaluate(ctx_standalone)
    assert nodes['static_tf_camera_to_color_optical'].condition.evaluate(ctx_standalone)
    assert nodes['static_tf_camera_optical_to_gz'].condition.evaluate(ctx_standalone)
