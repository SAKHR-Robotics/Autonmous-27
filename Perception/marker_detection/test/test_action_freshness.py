import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from marker_detection.marker_action_interface_node import MarkerActionInterfaceNode
from marker_detection.quality_flags import QualityFlag
from marker_detection_msgs.msg import MarkerTrackedPose, MarkerTrackedPoseArray


@pytest.fixture
def node():
    rclpy.init()
    value = MarkerActionInterfaceNode()
    transform = TransformStamped()
    transform.header.frame_id = 'base_link'
    transform.child_frame_id = 'camera_optical'
    transform.transform.rotation.w = 1.
    value.tf_buffer.set_transform_static(transform, 'unit_test')
    yield value
    value.destroy_node()
    rclpy.try_shutdown()


def observation(node, delay=0., age=0.):
    pose = MarkerTrackedPose()
    stamp = node.get_clock().now().nanoseconds - int(delay * 1e9)
    pose.header.stamp.sec, pose.header.stamp.nanosec = divmod(stamp, 10**9)
    pose.header.frame_id = 'camera_optical'
    pose.id = 1
    pose.position.z = 2.
    pose.orientation.w = 1.
    pose.detection_valid = pose.tracking_valid = True
    pose.track_state = 'CONFIRMED'
    pose.confidence = .9
    pose.position_covariance = (np.eye(3) * 1e-4).ravel().tolist()
    pose.age_seconds = age
    return pose


def test_fresh_observation_is_actionable_with_real_tf_composition(node):
    result = node._build_message(observation(node, delay=.05))
    assert result.usable_for_action
    assert .05 <= result.age_seconds < .5
    assert result.vertical_distance == pytest.approx(2.)


@pytest.mark.parametrize('delay,age', [(10., 0.), (-1., 0.), (0., -1.), (0., float('nan')), (.3, .3)])
def test_delayed_future_or_invalid_age_is_never_actionable(node, delay, age):
    result = node._build_message(observation(node, delay, age))
    assert not result.usable_for_action
    assert result.quality_flags & int(QualityFlag.STALE_DATA)


def test_stale_observation_is_not_forwarded_on_legacy_pose_output(node):
    targets, legacy = [], []
    node.targets_pub = type('Capture', (), {'publish': lambda self, value: targets.append(value)})()
    node.slam_landmark_pub = type('Capture', (), {'publish': lambda self, value: legacy.append(value)})()
    pose = observation(node, delay=10.)
    node._poses_callback(MarkerTrackedPoseArray(header=pose.header, markers=[pose]))
    assert not legacy and len(targets) == 1
    assert not targets[0].markers[0].usable_for_action
