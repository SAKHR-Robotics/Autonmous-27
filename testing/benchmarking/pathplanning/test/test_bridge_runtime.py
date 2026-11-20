"""Opt-in integration tests of the installed production C++ bridge."""
import math
import os
import signal
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('PATHPLANNER_RUNTIME_TESTS') != '1',
                                reason='Set PATHPLANNER_RUNTIME_TESTS=1 with the Jazzy overlay sourced')
rclpy = pytest.importorskip('rclpy')
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from vision_msgs.msg import Detection3D, Detection3DArray


@pytest.fixture(scope='module')
def bridge():
    rclpy.init()
    node = Node('bridge_geometry_probe')
    received = []
    node.create_subscription(PointCloud2, '/testing/pathplanner/bridge_output', received.append, 10)
    pub = node.create_publisher(Detection3DArray, '/testing/pathplanner/bridge_input', 10)
    process = subprocess.Popen(['ros2', 'run', 'erc_path_planner', 'costmap_bridge_node',
        '--ros-args', '-r', '__node:=geometry_test_bridge',
        '-p', 'input_topic:=/testing/pathplanner/bridge_input',
        '-p', 'output_topic:=/testing/pathplanner/bridge_output', '-p', 'max_points:=1000'],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
    sequence = 0
    def observe(detection):
        nonlocal sequence
        sequence += 1
        message = Detection3DArray()
        message.header.frame_id = 'test_frame'
        message.header.stamp.sec = 123
        message.header.stamp.nanosec = sequence
        message.detections = [] if detection is None else [detection]
        end = time.monotonic()+8
        received.clear()
        while time.monotonic() < end:
            pub.publish(message)
            rclpy.spin_once(node, timeout_sec=.1)
            matching = [cloud for cloud in received if cloud.header == message.header]
            if matching:
                cloud = matching[-1]
                return cloud, list(read_points(cloud, field_names=('x', 'y', 'z')))
        raise AssertionError('Production bridge did not respond')
    yield observe
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()


def box():
    d = Detection3D()
    d.bbox.center.orientation.w = 1.0
    d.bbox.size.x, d.bbox.size.y, d.bbox.size.z = 1.0, 0.2, 0.4
    return d


def test_yaw_rotates_footprint(bridge):
    d = box()
    d.bbox.center.orientation.z = math.sin(math.pi/4)
    d.bbox.center.orientation.w = math.cos(math.pi/4)
    cloud, points = bridge(d)
    assert cloud.width > 0
    assert max(abs(float(p[0])) for p in points) == pytest.approx(.1, abs=1e-4)
    assert max(abs(float(p[1])) for p in points) == pytest.approx(.5, abs=1e-4)


def test_tilt_includes_vertical_extent(bridge):
    d = box()
    d.bbox.size.x, d.bbox.size.z = .2, 2.0
    d.bbox.center.orientation.y = math.sin(math.pi/4)
    d.bbox.center.orientation.w = math.cos(math.pi/4)
    _, points = bridge(d)
    assert max(abs(float(p[0])) for p in points) == pytest.approx(1., abs=1e-4)


@pytest.mark.parametrize('kind', ['empty', 'nan', 'zero_quaternion'])
def test_empty_and_invalid_inputs_produce_empty_update(bridge, kind):
    d = box()
    if kind == 'empty':
        d = None
    elif kind == 'nan':
        d.bbox.center.position.x = float('nan')
    else:
        d.bbox.center.orientation.w = 0.0
    cloud, points = bridge(d)
    assert cloud.width == 0 and not points


def test_sampling_is_bounded(bridge):
    d = box()
    d.bbox.size.x = d.bbox.size.y = 100.0
    cloud, _ = bridge(d)
    assert cloud.width == 1000
