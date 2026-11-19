"""Exercise the installed production adapter over isolated test ROS topics."""
import signal
import subprocess
import time
from pathlib import Path
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.node import Node
from marker_detection_msgs.msg import MarkerTrackedPose, MarkerTrackedPoseArray
from rtabmap_msgs.msg import LandmarkDetections


def test_installed_adapter_preserves_array_header_and_rejects_future_observations(tmp_path):
    rclpy.init()
    node = Node('landmark_contract_check')
    received = []
    topic = '/testing/perception/landmark_contract/'
    publisher = node.create_publisher(MarkerTrackedPoseArray, topic + 'tracks', 10)
    node.create_subscription(LandmarkDetections, topic + 'landmarks', received.append, 10)
    with (tmp_path / 'adapter.log').open('w+') as log:
        process = subprocess.Popen([
            str(Path(get_package_prefix('marker_detection')) / 'lib/marker_detection/marker_landmark_adapter'), '--ros-args',
            '-r', '/marker_poses:=' + topic + 'tracks',
            '-r', '/perception/landmark_detections:=' + topic + 'landmarks',
        ], stdout=log, stderr=subprocess.STDOUT)
        try:
            pose = MarkerTrackedPose()
            pose.id = 0
            pose.header.frame_id = 'contract_camera_optical'
            pose.position.z = 2.
            pose.orientation.w = 1.
            pose.detection_valid = pose.tracking_valid = True
            pose.track_state = 'CONFIRMED'
            pose.tracking_confidence = .9
            pose.position_covariance = (np.eye(3) * .01).ravel().tolist()
            deadline = time.monotonic() + 8.
            while time.monotonic() < deadline and not any(msg.landmarks for msg in received):
                assert process.poll() is None
                pose.header.stamp = node.get_clock().now().to_msg()
                publisher.publish(MarkerTrackedPoseArray(header=pose.header, markers=[pose]))
                rclpy.spin_once(node, timeout_sec=.1)
            valid = next(msg for msg in received if msg.landmarks)
            assert valid.header.frame_id == 'contract_camera_optical'
            assert valid.header.stamp == valid.landmarks[0].header.stamp
            assert valid.landmarks[0].id == 1
            assert valid.landmarks[0].pose.covariance[0] == .01

            future = node.get_clock().now().to_msg()
            future.sec += 10
            pose.header.stamp = future
            received.clear()
            deadline = time.monotonic() + 3.
            while time.monotonic() < deadline and not any(msg.header.stamp == future for msg in received):
                publisher.publish(MarkerTrackedPoseArray(header=pose.header, markers=[pose]))
                rclpy.spin_once(node, timeout_sec=.1)
            invalid = next(msg for msg in received if msg.header.stamp == future)
            assert not invalid.landmarks
        finally:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=10)
            node.destroy_node()
            rclpy.try_shutdown()
        log.seek(0)
        assert process.returncode == 0, log.read()
