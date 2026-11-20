"""Unit checks for pathplanner_mock_tools geometry presets, seeding, and fault injection."""
import json
from unittest.mock import Mock

import pytest
rclpy = pytest.importorskip("rclpy", reason="ROS 2 Jazzy required")
from rclpy.parameter import Parameter
from std_msgs.msg import Empty, String

from pathplanner_mock_tools.mock_perception import MockPerception
from pathplanner_mock_tools.mock_rover_sim import MockRoverSim


@pytest.fixture
def ros():
    owned = not rclpy.ok()
    if owned:
        rclpy.init(args=[])
    yield
    if owned:
        rclpy.try_shutdown()


def test_rover_geometry_profile_and_fault_injection(ros):
    node = MockRoverSim(
        parameter_overrides=[
            Parameter("geometry_profile", value="production_rover"),
            Parameter("seed", value=123),
        ]
    )
    try:
        assert node.rover_length == pytest.approx(0.90)
        assert node.rover_width == pytest.approx(0.70)
        assert node.wheel_radius == pytest.approx(0.06)
        assert node.track_width == pytest.approx(0.49)

        node.odom_pub = Mock()
        node.truth_odom_pub = Mock()
        node.joint_pub = Mock()
        node.tf_broadcaster = Mock()

        node.update_physics()
        assert node.odom_pub.publish.called
        assert node.truth_odom_pub.publish.called
        assert node.joint_pub.publish.called
        assert node.tf_broadcaster.sendTransform.called

        node.odom_pub.reset_mock()
        node.truth_odom_pub.reset_mock()
        node.joint_pub.reset_mock()
        node.tf_broadcaster.reset_mock()

        node.set_parameters([
            Parameter("freeze_odom", value=True),
            Parameter("drop_tf", value=True),
            Parameter("drop_joints", value=True),
        ])
        node.update_physics()
        assert node.truth_odom_pub.publish.called
        assert not node.odom_pub.publish.called
        assert not node.joint_pub.publish.called
        assert not node.tf_broadcaster.sendTransform.called
    finally:
        node.destroy_node()


def test_mock_perception_clear_spawn_and_freeze(ros):
    node = MockPerception(
        parameter_overrides=[
            Parameter("clear_default_obstacles", value=True),
            Parameter("publish_detections", value=True),
        ]
    )
    try:
        assert node.obstacles == []
        node.spawn_full_callback(String(data=json.dumps({"x": 2.0, "y": -1.0, "radius": 0.5})))
        assert len(node.obstacles) == 1
        assert node.obstacles[0]["radius"] == pytest.approx(0.5)

        node.detection_pub = Mock()
        node.feature_pub = Mock()
        node.scan_pub = Mock()
        node.marker_pub = Mock()

        node.set_parameters([Parameter("freeze_perception", value=True)])
        node.publish_perception_data()
        assert not node.detection_pub.publish.called
        assert not node.scan_pub.publish.called

        node.clear_obstacles_callback(Empty())
        assert node.obstacles == []
        assert node.detection_pub.publish.called
    finally:
        node.destroy_node()
