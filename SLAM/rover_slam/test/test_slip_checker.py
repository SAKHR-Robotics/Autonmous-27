"""
test_slip_checker.py
Unit tests for heuristic slip checker pre-filter node logic.
Developer Track: Person 1 (Task 3A.3 / SLAM Fix 1)
"""
import pytest
from rover_slam.heuristic_slip_checker import SlipCheckerCore


def test_no_slip_stationary_no_drift():
    """Verify that stationary state with typical MEMS accelerometer bias never drifts into slip."""
    core = SlipCheckerCore(
        slip_rot_threshold=0.20,
        slip_accel_threshold=1.5,
        covariance_scale=100.0,
        nominal_linear_cov=0.02,
        nominal_angular_cov=0.05,
    )

    current_time = 0.0
    dt = 0.02  # 50 Hz updates
    # Simulate 60 seconds (3000 cycles) of stationary rover with accelerometer bias noise (0.08 m/s^2)
    for _ in range(3000):
        current_time += dt
        is_slipping, lin_cov, ang_cov = core.evaluate(
            v_wheel=0.0,
            w_wheel=0.0,
            w_imu=0.0,
            a_imu_x=0.08,
            current_time_sec=current_time,
        )
        assert not is_slipping
        assert lin_cov == pytest.approx(0.02)
        assert ang_cov == pytest.approx(0.05)


def test_nominal_driving_matched_rates():
    """Verify that normal driving with matching wheel and IMU rates flags no slip."""
    core = SlipCheckerCore()

    current_time = 0.0
    dt = 0.02
    v = 0.5
    w = 0.1

    for _ in range(100):
        current_time += dt
        is_slipping, lin_cov, ang_cov = core.evaluate(
            v_wheel=v,
            w_wheel=w,
            w_imu=w,
            a_imu_x=0.0,
            current_time_sec=current_time,
        )
        assert not is_slipping
        assert lin_cov == pytest.approx(0.02)
        assert ang_cov == pytest.approx(0.05)


def test_angular_slip_detection():
    """Verify that yaw rate discrepancy (wheel spinning vs stationary chassis) flags slip."""
    core = SlipCheckerCore(slip_rot_threshold=0.20, covariance_scale=100.0)

    # Wheel commanded yaw 0.8 rad/s, but IMU reports only 0.05 rad/s (|0.8 - 0.05| = 0.75 > 0.20)
    is_slipping, lin_cov, ang_cov = core.evaluate(
        v_wheel=0.2,
        w_wheel=0.8,
        w_imu=0.05,
        a_imu_x=0.0,
        current_time_sec=1.0,
    )
    assert is_slipping
    assert lin_cov == pytest.approx(2.0)    # 0.02 * 100
    assert ang_cov == pytest.approx(5.0)    # 0.05 * 100


def test_linear_acceleration_slip_detection():
    """Verify wheel spin-up from rest without chassis acceleration triggers slip after debounce window."""
    core = SlipCheckerCore(slip_accel_threshold=1.5, covariance_scale=100.0)

    # Initial state at t = 1.0s, v = 0.0
    core.evaluate(v_wheel=0.0, w_wheel=0.0, w_imu=0.0, a_imu_x=0.0, current_time_sec=1.0)

    # Wheels spin rapidly to 0.5 m/s, but chassis remains stalled (a_imu_x = 0.0)
    current_time = 1.0
    for _ in range(15):  # 300ms of spinning wheels with zero body acceleration
        current_time += 0.02
        is_slipping, lin_cov, ang_cov = core.evaluate(
            v_wheel=0.5,
            w_wheel=0.0,
            w_imu=0.0,
            a_imu_x=0.0,
            current_time_sec=current_time,
        )

    assert is_slipping
    assert lin_cov == pytest.approx(2.0)
    assert ang_cov == pytest.approx(5.0)


def test_slip_recovery():
    """Verify that covariance returns to nominal once slip ceases."""
    core = SlipCheckerCore(slip_rot_threshold=0.20, covariance_scale=100.0)

    # Step 1: Slip occurring
    is_slipping, lin_cov, _ = core.evaluate(
        v_wheel=0.0,
        w_wheel=0.6,
        w_imu=0.0,
        a_imu_x=0.0,
        current_time_sec=1.0,
    )
    assert is_slipping
    assert lin_cov == pytest.approx(2.0)

    # Step 2: Wheels realign with IMU and hold time expires
    is_slipping, lin_cov, _ = core.evaluate(
        v_wheel=0.0,
        w_wheel=0.05,
        w_imu=0.05,
        a_imu_x=0.0,
        current_time_sec=2.0,  # 1.0 second later, exceeding hold time
    )
    assert not is_slipping
    assert lin_cov == pytest.approx(0.02)


def test_slope_driving_gravity_compensation():
    """Verify that driving on a 20-degree slope does not falsely trigger slip due to gravity leakage."""
    import math
    core = SlipCheckerCore(slip_accel_threshold=2.5)

    # Pitch angle theta = 20 degrees (REP 103 nose up: negative rotation around Y)
    theta = math.radians(20.0)
    qy = math.sin(-theta / 2.0)
    qw = math.cos(-theta / 2.0)
    orientation_q = (0.0, qy, 0.0, qw)

    # Gravity leakage on accelerometer = 9.81 * sin(20 deg) = 3.355 m/s^2
    ax_with_gravity = 9.81 * math.sin(theta)

    current_time = 1.0
    for _ in range(50):
        current_time += 0.02
        is_slipping, lin_cov, _ = core.evaluate(
            v_wheel=0.5,
            w_wheel=0.0,
            w_imu=0.0,
            a_imu_x=ax_with_gravity,
            current_time_sec=current_time,
            orientation_q=orientation_q,
        )
        assert not is_slipping
        assert lin_cov == pytest.approx(0.02)


def test_nominal_driving_rapid_acceleration():
    """Verify that rapid forward acceleration from rest does not falsely trigger slip when body accelerates."""
    core = SlipCheckerCore()

    # Step 1: Stationary at t=1.0s
    core.evaluate(v_wheel=0.0, w_wheel=0.0, w_imu=0.0, a_imu_x=0.0, current_time_sec=1.0)

    # Step 2: Wheels accelerate rapidly to 0.5 m/s, and body accelerates forward at 3.0 m/s^2 (traction)
    is_slipping, lin_cov, _ = core.evaluate(
        v_wheel=0.5,
        w_wheel=0.0,
        w_imu=0.0,
        a_imu_x=3.0,
        current_time_sec=1.02,
    )
    assert not is_slipping
    assert lin_cov == pytest.approx(0.02)


def test_collision_impact_slip():
    """Verify that colliding with an obstacle (body deceleration while wheels driven forward) triggers slip."""
    core = SlipCheckerCore()

    # Step 1: Cruising forward at 0.5 m/s
    core.evaluate(v_wheel=0.5, w_wheel=0.0, w_imu=0.0, a_imu_x=0.0, current_time_sec=1.0)

    # Step 2: Collides with rock - sharp negative deceleration spike on IMU while wheels are still rolling forward
    is_slipping, lin_cov, _ = core.evaluate(
        v_wheel=0.5,
        w_wheel=0.0,
        w_imu=0.0,
        a_imu_x=-2.5,
        current_time_sec=1.02,
    )
    assert is_slipping
    assert lin_cov == pytest.approx(2.0)


def test_reverse_collision_impact_slip():
    """Verify that colliding with an obstacle in reverse triggers slip."""
    core = SlipCheckerCore()

    # Step 1: Reversing at -0.5 m/s
    core.evaluate(v_wheel=-0.5, w_wheel=0.0, w_imu=0.0, a_imu_x=0.0, current_time_sec=1.0)

    # Step 2: Collides with rock behind - forward deceleration impact while wheels driven backward
    is_slipping, lin_cov, _ = core.evaluate(
        v_wheel=-0.5,
        w_wheel=0.0,
        w_imu=0.0,
        a_imu_x=2.5,
        current_time_sec=1.02,
    )
    assert is_slipping
    assert lin_cov == pytest.approx(2.0)

    # Step 3: Hold time expires and wheels stop
    core.evaluate(v_wheel=0.0, w_wheel=0.0, w_imu=0.0, a_imu_x=0.0, current_time_sec=2.0)
    is_slipping, lin_cov, _ = core.evaluate(
        v_wheel=0.0,
        w_wheel=0.0,
        w_imu=0.0,
        a_imu_x=0.0,
        current_time_sec=3.5,
    )
    assert not is_slipping
    assert lin_cov == pytest.approx(0.02)


def test_sustained_rock_pushing():
    """Verify that when colliding with a rock and pushing into it, slip stays True continuously."""
    core = SlipCheckerCore()

    # Step 1: Cruising forward
    core.evaluate(v_wheel=0.5, w_wheel=0.0, w_imu=0.0, a_imu_x=0.0, current_time_sec=1.0)

    # Step 2: Hits rock
    is_slipping, _, _ = core.evaluate(v_wheel=0.5, w_wheel=0.0, w_imu=0.0, a_imu_x=-2.5, current_time_sec=1.02)
    assert is_slipping

    # Step 3: Pushing against the rock for 5 seconds (chassis stopped ax=0, wheels spinning at 0.5)
    current_time = 1.02
    for _ in range(250):  # 5 seconds
        current_time += 0.02
        is_slipping, lin_cov, _ = core.evaluate(
            v_wheel=0.5,
            w_wheel=0.0,
            w_imu=0.0,
            a_imu_x=0.0,
            current_time_sec=current_time,
        )
        assert is_slipping
        assert lin_cov == pytest.approx(2.0)

    # Step 4: Driver releases key (v_wheel drops to 0) -> recovers to nominal
    for _ in range(25):  # 500ms (exceeding 400ms hold time)
        current_time += 0.02
        is_slipping, lin_cov, _ = core.evaluate(
            v_wheel=0.0,
            w_wheel=0.0,
            w_imu=0.0,
            a_imu_x=0.0,
            current_time_sec=current_time,
        )
    assert not is_slipping
    assert lin_cov == pytest.approx(0.02)


def test_reverse_sustained_obstacle_pushing():
    """Verify that reversing into an obstacle and continuing to push backward flags slip continuously."""
    core = SlipCheckerCore()

    # Step 1: Reversing
    core.evaluate(v_wheel=-0.5, w_wheel=0.0, w_imu=0.0, a_imu_x=0.0, current_time_sec=1.0)

    # Step 2: Collision impact behind
    is_slipping, _, _ = core.evaluate(v_wheel=-0.5, w_wheel=0.0, w_imu=0.0, a_imu_x=2.0, current_time_sec=1.02)
    assert is_slipping

    # Step 3: Pushing backward for 3 seconds
    current_time = 1.02
    for _ in range(150):
        current_time += 0.02
        is_slipping, lin_cov, _ = core.evaluate(
            v_wheel=-0.5,
            w_wheel=0.0,
            w_imu=0.0,
            a_imu_x=0.0,
            current_time_sec=current_time,
        )
        assert is_slipping

    # Step 4: Driver backs away in opposite direction (forward) -> obstacle block immediately clears
    current_time += 0.02
    is_slipping, _, _ = core.evaluate(
        v_wheel=0.5,
        w_wheel=0.0,
        w_imu=0.0,
        a_imu_x=1.5,
        current_time_sec=current_time,
    )
    assert not core.obstacle_blocked


def test_normal_left_and_right_turns_without_obstacles():
    """Verify that turning left or right on open ground (skid-steer scrubbing) stays False."""
    core = SlipCheckerCore()
    t = 0.0

    # Normal left turn: wheels commanded 0.8 rad/s, chassis turns 0.4 rad/s in same direction
    for _ in range(50):
        t += 0.02
        is_slipping, lin_cov, ang_cov = core.evaluate(
            v_wheel=0.0,
            w_wheel=0.8,
            w_imu=0.4,
            a_imu_x=0.0,
            current_time_sec=t,
        )
        assert not is_slipping
        assert lin_cov == pytest.approx(0.02)
        assert ang_cov == pytest.approx(0.05)

    # Normal right turn: wheels commanded -0.8 rad/s, chassis turns -0.4 rad/s
    for _ in range(50):
        t += 0.02
        is_slipping, lin_cov, ang_cov = core.evaluate(
            v_wheel=0.0,
            w_wheel=-0.8,
            w_imu=-0.4,
            a_imu_x=0.0,
            current_time_sec=t,
        )
        assert not is_slipping
        assert lin_cov == pytest.approx(0.02)
        assert ang_cov == pytest.approx(0.05)


def test_turn_blocked_by_obstacle():
    """Verify that attempting to turn against an obstacle (wheels spin but chassis cannot rotate) flags slip."""
    core = SlipCheckerCore()

    # Wheels commanded to turn 0.8 rad/s, but rock blocks chassis (w_imu = 0.02 rad/s)
    is_slipping, lin_cov, ang_cov = core.evaluate(
        v_wheel=0.0,
        w_wheel=0.8,
        w_imu=0.02,
        a_imu_x=0.0,
        current_time_sec=1.0,
    )
    assert is_slipping
    assert lin_cov == pytest.approx(2.0)
    assert ang_cov == pytest.approx(5.0)





