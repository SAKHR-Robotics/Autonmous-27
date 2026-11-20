"""Sensor physics, source timestamps and reset/dropout behavior of the fixture."""
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from slam_mock_tools.rover_sensor_adapter import RoverSensorAdapter


@pytest.fixture
def adapter():
    rclpy.init(args=[])
    node = RoverSensorAdapter()
    node.imu = Mock()
    node.wheels = Mock()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def truth(seconds, velocity=.2, yaw_rate=.3):
    message = Odometry()
    message.header.stamp.sec = int(seconds)
    message.header.stamp.nanosec = round((seconds-int(seconds))*1e9)
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
