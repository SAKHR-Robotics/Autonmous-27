import time
import pytest
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from diagnostic_msgs.msg import DiagnosticStatus
from marker_detection.marker_detection_node import MarkerDetectionNode


@pytest.fixture
def node():
    rclpy.init()
    value = MarkerDetectionNode()
    yield value
    value.destroy_node()
    rclpy.try_shutdown()


def calibration():
    return CameraInfo(width=640, height=480, k=[600., 0., 320., 0., 600., 240., 0., 0., 1.])


def test_invalid_calibration_cannot_leave_previous_intrinsics_active(node):
    node._camera_info_callback(calibration())
    assert node._usable_camera_info((480, 640)) is not None
    invalid = calibration()
    invalid.k[0] = float('nan')
    node._camera_info_callback(invalid)
    assert node.camera_info is None


def test_stale_and_wrong_resolution_intrinsics_are_not_used(node):
    node._camera_info_callback(calibration())
    assert node._usable_camera_info((240, 320)) is None
    node._camera_info_received_monotonic = time.monotonic() - 10.
    assert node._usable_camera_info((480, 640)) is None


@pytest.mark.parametrize('delay', [10., -1.])
def test_old_or_future_rgb_is_rejected_before_detection(node, delay):
    frame = Image()
    stamp = node.get_clock().now().nanoseconds - int(delay * 1e9)
    frame.header.stamp.sec, frame.header.stamp.nanosec = divmod(stamp, 10**9)
    node._to_bgr = lambda msg: pytest.fail('Rejected timestamp reached image processing')
    node._rgb_callback(frame)
    assert 'timestamp' in node._input_error


def test_missing_calibration_and_invalid_input_are_visible_in_health(node):
    output = []
    node.diagnostics_pub = type('Capture', (), {'publish': lambda self, msg: output.append(msg)})()
    node._publish_diagnostics()
    assert output[-1].status[0].level in (DiagnosticStatus.WARN, DiagnosticStatus.ERROR)
    assert {v.key:v.value for v in output[-1].status[0].values}['calibration_usable'] == 'False'
    node._input_error = 'Invalid input'
    node._publish_diagnostics()
    assert output[-1].status[0].level == DiagnosticStatus.ERROR


def test_diagnostic_rate_counts_only_frames_in_its_reporting_window(node):
    output = []
    node.diagnostics_pub = type('Capture', (), {'publish': lambda self, msg: output.append(msg)})()
    node._processing_times.extend([.01] * 100)
    node._diagnostic_processing_times.extend([.01, .02])
    node._diag_window_start = time.monotonic() - 1.
    node._publish_diagnostics()
    values = {value.key:value.value for value in output[-1].status[0].values}
    assert 1.9 <= float(values['processing_fps']) <= 2.01
    node._publish_diagnostics()
    values = {value.key:value.value for value in output[-1].status[0].values}
    assert float(values['processing_fps']) == 0.


def test_clock_reversal_discards_tracks_and_buffered_depth(node):
    now = node.get_clock().now().nanoseconds * 1e-9
    node._last_ros_time = now + 10.
    node.tracker_manager.tracks[1] = object()
    node._depth_buffer.push(now, object())
    node._rgb_callback(Image())
    assert not node.tracker_manager.tracks
    assert len(node._depth_buffer) == 0
