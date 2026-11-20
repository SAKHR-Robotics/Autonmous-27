"""Reject finite covariance matrices that cannot represent uncertainty."""
import pytest

from rover_slam.heuristic_slip_checker import inflate_covariance


def covariance():
    values = [0.] * 36
    values[0] = values[35] = 1.
    return values


def test_asymmetric_covariance_is_rejected():
    values = covariance()
    values[5] = .2
    with pytest.raises(ValueError, match='covariance'):
        inflate_covariance(values, 100., .02, .05)


def test_indefinite_covariance_is_rejected():
    values = covariance()
    values[5] = values[30] = 2.
    with pytest.raises(ValueError, match='covariance'):
        inflate_covariance(values, 100., .02, .05)


def test_valid_rank_deficient_covariance_preserves_cross_terms():
    values = covariance()
    values[5] = values[30] = 1.
    result = inflate_covariance(values, 100., .02, .05)
    assert result[0] == result[35] == result[5] == result[30] == 100.


def test_rejected_covariance_cannot_latch_collision_state():
    from unittest.mock import Mock
    import rclpy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from rover_slam.heuristic_slip_checker import HeuristicSlipCheckerNode

    rclpy.init(args=[])
    node = HeuristicSlipCheckerNode()
    node.pub_filtered_odom = Mock()
    try:
        message = Odometry()
        message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
        message.twist.twist.linear.x = .3
        message.twist.covariance = covariance()
        for impact in (False, True):
            message.header.stamp = node.get_clock().now().to_msg()
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = message.header.stamp, 'base_link'
            imu.orientation.w = 1.
            imu.linear_acceleration.z = 9.81
            if impact:
                imu.linear_acceleration.x = -3.
                message.twist.covariance[5] = message.twist.covariance[30] = 2.
                node.pub_filtered_odom.publish.reset_mock()
            node._imu_callback(imu)
            node._wheel_odom_callback(message)
        node.pub_filtered_odom.publish.assert_not_called()
        assert not node.core.obstacle_blocked
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
