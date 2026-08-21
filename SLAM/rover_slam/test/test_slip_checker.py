"""
test_slip_checker.py
Unit tests for heuristic slip checker pre-filter node logic.
Developer Track: Person 1 (Task 3A.3)
"""
import pytest
from rover_slam.heuristic_slip_checker import SlipCheckerCore


def test_no_slip_scenario():
    """Verify normal driving condition where wheels and IMU match."""
    checker = SlipCheckerCore(
        linear_threshold=0.15,
        angular_threshold=0.20,
        base_linear_cov=0.01,
        base_angular_cov=0.01,
        slip_cov_multiplier=1000.0
    )

    # Simulate matched state
    checker.v_imu = 0.50
    checker.latest_imu_yaw_rate = 0.10

    is_slipping, lin_cov, ang_cov = checker.check_slip(v_wheel=0.52, omega_wheel=0.11)
    assert not is_slipping
    assert lin_cov == pytest.approx(0.01)
    assert ang_cov == pytest.approx(0.01)


def test_linear_slip_detection():
    """Verify wheel spin detection (e.g. wheels spinning on sand with no rover motion)."""
    checker = SlipCheckerCore(
        linear_threshold=0.15,
        angular_threshold=0.20,
        base_linear_cov=0.01,
        base_angular_cov=0.01,
        slip_cov_multiplier=1000.0
    )

    # Rover is stuck (IMU v=0.0), but wheels spin at 0.5 m/s (|0.5 - 0.0| = 0.5 > 0.15)
    checker.v_imu = 0.0
    checker.latest_imu_yaw_rate = 0.0

    is_slipping, lin_cov, ang_cov = checker.check_slip(v_wheel=0.50, omega_wheel=0.0)
    assert is_slipping
    assert lin_cov == pytest.approx(10.0)    # 0.01 * 1000.0
    assert ang_cov == pytest.approx(10.0)


def test_angular_slip_detection():
    """Verify yaw rate discrepancy detection."""
    checker = SlipCheckerCore(
        linear_threshold=0.15,
        angular_threshold=0.20,
        base_linear_cov=0.01,
        base_angular_cov=0.01,
        slip_cov_multiplier=500.0
    )

    # Wheels command high yaw rotation, but IMU reports near zero rotation
    checker.v_imu = 0.20
    checker.latest_imu_yaw_rate = 0.05

    is_slipping, lin_cov, ang_cov = checker.check_slip(v_wheel=0.20, omega_wheel=0.60)
    assert is_slipping
    assert lin_cov == pytest.approx(5.0)     # 0.01 * 500.0
    assert ang_cov == pytest.approx(5.0)


def test_imu_kinematic_integration():
    """Verify IMU acceleration integration tracking."""
    checker = SlipCheckerCore()

    # Initial timestamp
    checker.update_imu(ax=1.0, yaw_rate=0.0, current_time_sec=100.0)
    assert checker.v_imu == 0.0

    # 0.5 seconds later with 1.0 m/s^2 forward acceleration
    checker.update_imu(ax=1.0, yaw_rate=0.0, current_time_sec=100.5)
    # v_imu should be approximately (0 + 1.0 * 0.5) * 0.98 = 0.49
    assert checker.v_imu == pytest.approx(0.49, rel=1e-2)


def test_slip_recovery():
    """Verify system clears slip state once wheel speed aligns with IMU."""
    checker = SlipCheckerCore(linear_threshold=0.15)
    checker.v_imu = 0.20

    # 1. Slipping
    slipping, lin_cov, _ = checker.check_slip(v_wheel=0.80, omega_wheel=0.0)
    assert slipping
    assert lin_cov > checker.base_linear_cov

    # 2. Recovered
    slipping, lin_cov, _ = checker.check_slip(v_wheel=0.22, omega_wheel=0.0)
    assert not slipping
    assert lin_cov == pytest.approx(checker.base_linear_cov)

