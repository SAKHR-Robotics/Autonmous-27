"""Step 5 TF message construction and invalid-input rejection, without a live TF tree."""
import pytest
import rclpy
from builtin_interfaces.msg import Time
from marker_detection.marker_tf_broadcaster import MarkerTFBroadcaster
from marker_detection_msgs.msg import MarkerTrackedPose


def _pose(id_=17, frame="camera_color_optical_frame", stamp=(1, 0), tracking_valid=True,
          detection_valid=True, state="CONFIRMED", position=(0.1, 0.2, 1.0),
          orientation=(0.0, 0.0, 0.0, 1.0)) -> MarkerTrackedPose:
    message = MarkerTrackedPose()
    message.id = id_
    message.header.frame_id = frame
    message.header.stamp = Time(sec=stamp[0], nanosec=stamp[1])
    message.tracking_valid = tracking_valid
    message.detection_valid = detection_valid
    message.track_state = state
    message.position.x, message.position.y, message.position.z = position
    (message.orientation.x, message.orientation.y,
     message.orientation.z, message.orientation.w) = orientation
    return message


@pytest.fixture(scope="module", autouse=True)
def _ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    instance = MarkerTFBroadcaster()
    yield instance
    instance.destroy_node()


def test_valid_confirmed_pose_produces_camera_to_marker_transform(node):
    transform = node._to_transform(_pose())
    assert transform is not None
    assert transform.header.frame_id == "camera_color_optical_frame"
    assert transform.child_frame_id == "aruco_marker_17"
    assert transform.transform.translation.x == pytest.approx(0.1)
    assert transform.transform.translation.z == pytest.approx(1.0)
    assert transform.transform.rotation.w == pytest.approx(1.0)


def test_empty_parent_frame_is_rejected(node):
    assert node._to_transform(_pose(frame="")) is None


def test_zero_timestamp_is_rejected(node):
    assert node._to_transform(_pose(stamp=(0, 0))) is None


def test_invalid_zero_quaternion_is_rejected(node):
    assert node._to_transform(_pose(orientation=(0.0, 0.0, 0.0, 0.0))) is None


def test_non_finite_position_is_rejected(node):
    assert node._to_transform(_pose(position=(float("nan"), 0.0, 1.0))) is None


def test_only_confirmed_tracks_are_publishable_by_default(node):
    assert node._publishable(_pose(state="CONFIRMED", tracking_valid=True, detection_valid=True))
    assert not node._publishable(_pose(state="NEW", tracking_valid=True, detection_valid=True))
    assert not node._publishable(_pose(state="LOST", tracking_valid=True, detection_valid=True))
    assert not node._publishable(_pose(state="DEGRADED", tracking_valid=True, detection_valid=True))
    assert not node._publishable(_pose(state="CONFIRMED", tracking_valid=True, detection_valid=False))
    assert not node._publishable(_pose(state="CONFIRMED", tracking_valid=False, detection_valid=True))
