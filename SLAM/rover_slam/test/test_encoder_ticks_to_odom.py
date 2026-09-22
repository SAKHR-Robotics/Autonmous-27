import pytest
import math
from rover_slam.encoder_ticks_to_odom import compute_robust_side_velocity, euler_to_quaternion


def test_side_velocity_empty_and_single():
    v, slip = compute_robust_side_velocity([])
    assert v == 0.0
    assert not slip

    v, slip = compute_robust_side_velocity([0.4])
    assert v == pytest.approx(0.4)
    assert not slip


def test_side_velocity_matched():
    v, slip = compute_robust_side_velocity([0.30, 0.32], slip_diff_threshold=0.15)
    assert v == pytest.approx(0.31)
    assert not slip


def test_side_velocity_single_wheel_slip_forward():
    # Front wheel slips at 0.90 m/s, rear wheel has traction at 0.30 m/s
    v, slip = compute_robust_side_velocity([0.90, 0.30], slip_diff_threshold=0.15)
    assert v == pytest.approx(0.30)
    assert slip


def test_side_velocity_single_wheel_slip_reverse():
    # Front wheel slips in reverse at -0.80 m/s, rear wheel has traction at -0.20 m/s
    v, slip = compute_robust_side_velocity([-0.80, -0.20], slip_diff_threshold=0.15)
    assert v == pytest.approx(-0.20)
    assert slip


def test_side_velocity_stationary_one_wheel_spinning():
    # Rover stopped, but 1 wheel free spins at 0.60 m/s
    v, slip = compute_robust_side_velocity([0.60, 0.0], slip_diff_threshold=0.15)
    assert v == pytest.approx(0.0)
    assert slip


def test_side_velocity_median_multi_wheel():
    # 3 wheels: 0.30, 0.31, 0.95 (one wheel slipping)
    v, slip = compute_robust_side_velocity([0.30, 0.95, 0.31], slip_diff_threshold=0.15)
    assert v == pytest.approx(0.31)
    assert slip


def test_euler_to_quaternion_planar():
    q = euler_to_quaternion(0.0, 0.0, math.pi / 2.0)
    assert q.x == pytest.approx(0.0)
    assert q.y == pytest.approx(0.0)
    assert q.z == pytest.approx(math.sin(math.pi / 4.0))
    assert q.w == pytest.approx(math.cos(math.pi / 4.0))
