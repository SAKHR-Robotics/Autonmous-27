"""Sensor physics, source timestamps, slip/mount injection and reset/dropout checks."""
import math
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from slam_mock_tools.mock_aruco_publisher import MockArucoPublisherNode
from slam_mock_tools.rover_sensor_adapter import RoverSensorAdapter


@pytest.fixture
def adapter():
    rclpy.init(args=[])
    node = RoverSensorAdapter()
    node.imu = Mock()
    node.wheels = Mock()
    node.ticks_array_pub = Mock()
    node.individual_tick_pubs = {k: Mock() for k in RoverSensorAdapter.WHEEL_ORDER}
    node.fault_state_pub = Mock()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def truth(seconds, velocity=.2, yaw_rate=.3):
    message = Odometry()
    message.header.stamp.sec = int(seconds)
    message.header.stamp.nanosec = round((seconds - int(seconds)) * 1e9)
    message.child_frame_id = 'base_link'
    message.pose.pose.orientation.w = 1.
    message.twist.twist.linear.x = velocity
    message.twist.twist.angular.z = yaw_rate
    return message


def test_turn_imu_has_centripetal_acceleration_and_source_stamp(adapter):
    adapter.observe_truth(truth(1.))
    adapter.imu.publish.assert_not_called()
    message = truth(1.02)
    adapter.observe_truth(message)
    imu = adapter.imu.publish.call_args.args[0]
    assert imu.header.stamp == message.header.stamp
    assert imu.header.frame_id == 'base_link'
    assert imu.linear_acceleration.x == 0.
    assert imu.linear_acceleration.y == pytest.approx(.06)
    assert imu.linear_acceleration.z == 9.81
    assert imu.angular_velocity.z == .3
    assert all(imu.orientation_covariance[i] > 0 for i in (0, 4, 8))


def test_clock_reversal_rebaselines_acceleration(adapter):
    adapter.observe_truth(truth(10., 0.))
    adapter.observe_truth(truth(10.1, .1))
    assert adapter.imu.publish.call_args.args[0].linear_acceleration.x == pytest.approx(1.)
    adapter.imu.publish.reset_mock()
    adapter.observe_truth(truth(1., 1.))
    adapter.imu.publish.assert_not_called()
    adapter.observe_truth(truth(1.1, 1.))
    assert adapter.imu.publish.call_args.args[0].linear_acceleration.x == 0.


def test_imu_dropout_does_not_accumulate_acceleration(adapter):
    adapter.observe_truth(truth(1., 0.))
    adapter.set_parameters([Parameter('drop_imu', value=True)])
    adapter.observe_truth(truth(1.02, .2))
    adapter.imu.publish.assert_not_called()
    adapter.set_parameters([Parameter('drop_imu', value=False)])
    adapter.observe_truth(truth(1.04, .2))
    assert adapter.imu.publish.call_args.args[0].linear_acceleration.x == 0.


def test_complete_sides_expand_and_wheel_dropout_stops_output(adapter):
    message = JointState()
    message.header.stamp.sec = 1
    message.name = ['left_wheel_joint']
    message.position = [.1]
    adapter.observe_wheels(message)
    adapter.wheels.publish.assert_not_called()
    message.name.append('right_wheel_joint')
    message.position.append(.2)
    adapter.observe_wheels(message)
    wheels = adapter.wheels.publish.call_args.args[0]
    assert wheels.header == message.header
    assert list(wheels.position) == [.1, .2, .1, .2]
    assert wheels.name == ['left_front_wheel_joint', 'right_front_wheel_joint',
                          'left_rear_wheel_joint', 'right_rear_wheel_joint']
    adapter.wheels.publish.reset_mock()
    adapter.set_parameters([Parameter('drop_wheels', value=True)])
    adapter.observe_wheels(message)
    adapter.wheels.publish.assert_not_called()


def test_rotated_imu_mount_and_gravity_sign(adapter):
    adapter.set_parameters([
        Parameter('imu_mount_yaw_rad', value=math.pi / 2),
        Parameter('gravity_sign', value=-1.0),
        Parameter('imu_frame_id', value='imu_link'),
    ])
    adapter.observe_truth(truth(1.0, velocity=0.2, yaw_rate=0.3))
    adapter.observe_truth(truth(1.02, velocity=0.2, yaw_rate=0.3))
    imu = adapter.imu.publish.call_args.args[0]
    assert imu.header.frame_id == 'imu_link'
    # Body ay=0.06 rotated by -90 deg about Z maps to +ax in IMU frame
    assert imu.linear_acceleration.x == pytest.approx(0.06, abs=1e-6)
    assert imu.linear_acceleration.y == pytest.approx(0.0, abs=1e-6)
    assert imu.linear_acceleration.z == pytest.approx(-9.81, abs=1e-6)


def test_single_wheel_slip_and_tick_output_modes(adapter):
    adapter.set_parameters([
        Parameter('wheel_output_mode', value='ticks_array'),
        Parameter('single_wheel_slip_index', value=0),
        Parameter('wheel_slip_scale', value=2.0),
    ])
    m1 = JointState()
    m1.header.stamp.sec = 1
    m1.name = ['left_wheel_joint', 'right_wheel_joint']
    m1.position = [0.0, 0.0]
    adapter.observe_wheels(m1)

    m2 = JointState()
    m2.header.stamp.sec = 2
    m2.name = ['left_wheel_joint', 'right_wheel_joint']
    m2.position = [2.0 * math.pi, 2.0 * math.pi]
    adapter.observe_wheels(m2)
    arr = adapter.ticks_array_pub.publish.call_args.args[0]
    assert list(arr.data) == [2048, 1024, 1024, 1024]

    adapter.set_parameters([Parameter('wheel_output_mode', value='individual_ticks')])
    m3 = JointState()
    m3.header.stamp.sec = 3
    m3.name = ['left_wheel_joint', 'right_wheel_joint']
    m3.position = [4.0 * math.pi, 4.0 * math.pi]
    adapter.observe_wheels(m3)
    assert adapter.individual_tick_pubs['left_front'].publish.call_args.args[0].data == 4096
    assert adapter.individual_tick_pubs['right_front'].publish.call_args.args[0].data == 2048


def test_mock_aruco_publisher_emits_nonzero_stamp_and_psd_covariance():
    rclpy.init(args=[])
    node = MockArucoPublisherNode(
        parameter_overrides=[Parameter('use_sim_time', value=False)]
    )
    try:
        msg = node.build_message()
        assert (msg.header.stamp.sec > 0 or msg.header.stamp.nanosec > 0)
        assert msg.pose.covariance[0] > 0.0
        assert msg.pose.covariance[21] > 0.0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
