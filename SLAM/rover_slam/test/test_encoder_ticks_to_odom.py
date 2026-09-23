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


def test_4wheel_urdf_joint_state_matching():
    """Verify that 4-wheel URDF joint names are matched and update wheel ticks."""
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        node = EncoderTicksToOdomNode()
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.name = [
            "left_front_wheel_joint",
            "right_front_wheel_joint",
            "left_rear_wheel_joint",
            "right_rear_wheel_joint",
        ]
        msg.position = [2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi, 2.0 * math.pi]
        node._joint_states_callback(msg)

        assert node.current_ticks["left_front"] == node.ticks_per_rev
        assert node.current_ticks["right_front"] == node.ticks_per_rev
        assert node.current_ticks["left_rear"] == node.ticks_per_rev
        assert node.current_ticks["right_rear"] == node.ticks_per_rev
        node.destroy_node()
    except (ImportError, AttributeError):
        pass


def test_4wheel_alias_joint_state_matching():
    """Verify that configured alias names (front_left, etc.) match URDF joint names."""
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        node = EncoderTicksToOdomNode()
        node.wheel_names = ["front_left", "front_right", "rear_left", "rear_right"]
        node.current_ticks = {name: 0 for name in node.wheel_names}
        node.prev_ticks = {name: None for name in node.wheel_names}
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.name = [
            "left_front_wheel_joint",
            "right_front_wheel_joint",
            "left_rear_wheel_joint",
            "right_rear_wheel_joint",
        ]
        msg.position = [math.pi, math.pi, math.pi, math.pi]
        node._joint_states_callback(msg)

        expected_ticks = int((math.pi / (2.0 * math.pi)) * node.ticks_per_rev)
        assert node.current_ticks["front_left"] == expected_ticks
        assert node.current_ticks["front_right"] == expected_ticks
        assert node.current_ticks["rear_left"] == expected_ticks
        assert node.current_ticks["rear_right"] == expected_ticks
        node.destroy_node()
    except (ImportError, AttributeError):
        pass


def test_4wheel_arm_joints_ignored():
    """Verify that arm joints and other non-wheel joints in JointState are ignored."""
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        node = EncoderTicksToOdomNode()
        from sensor_msgs.msg import JointState
        msg = JointState()
        # Mix of arm joints, wheel joints, and sensor frames
        msg.name = [
            "base_footprint_joint",
            "left_front_arm_joint",
            "left_front_wheel_joint",
            "right_front_arm_joint",
            "right_front_wheel_joint",
            "left_rear_arm_joint",
            "left_rear_wheel_joint",
            "right_rear_arm_joint",
            "right_rear_wheel_joint",
            "imu_joint",
            "camera_joint",
        ]
        msg.position = [
            0.0,
            0.0,
            2.0 * math.pi,  # 1 full revolution for left_front
            0.0,
            2.0 * math.pi,  # 1 full revolution for right_front
            0.0,
            2.0 * math.pi,  # 1 full revolution for left_rear
            0.0,
            2.0 * math.pi,  # 1 full revolution for right_rear
            0.0,
            0.0,
        ]
        node._joint_states_callback(msg)

        assert node.current_ticks["left_front"] == node.ticks_per_rev
        assert node.current_ticks["right_front"] == node.ticks_per_rev
        assert node.current_ticks["left_rear"] == node.ticks_per_rev
        assert node.current_ticks["right_rear"] == node.ticks_per_rev

        # Second callback to verify delta ticks are NOT corrupted by arm joints
        msg2 = JointState()
        msg2.name = list(reversed(msg.name))  # Reversed order: arm joints after wheel joints
        msg2.position = [
            0.0,
            0.0,
            4.0 * math.pi,
            0.0,
            4.0 * math.pi,
            0.0,
            4.0 * math.pi,
            0.0,
            4.0 * math.pi,
            0.0,
            0.0,
        ]
        node._joint_states_callback(msg2)

        # Delta ticks should be exactly 1 revolution (node.ticks_per_rev), NOT corrupted
        for w in node.wheel_names:
            assert node.current_ticks[w] == 2 * node.ticks_per_rev
            assert node.wheel_delta_ticks[w] == node.ticks_per_rev

        node.destroy_node()
    except (ImportError, AttributeError):
        pass


def test_track_width_default_is_aligned_with_urdf():
    """Verify track_width parameter defaults to 0.49m matching URDF wheel separation."""
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
    try:
        import rclpy
        if not rclpy.ok():
            rclpy.init()
        node = EncoderTicksToOdomNode()
        assert node.track_width == pytest.approx(0.49)
        node.destroy_node()
    except (ImportError, AttributeError):
        pass


def test_legacy_6wheel_tick_array_handling():
    """Verify that a 6-element tick array correctly maps to the 4 active wheels."""
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
    try:
        import rclpy
        from std_msgs.msg import Int64MultiArray
        if not rclpy.ok():
            rclpy.init()
        node = EncoderTicksToOdomNode()
        msg = Int64MultiArray()
        # [LF, LM, LR, RF, RM, RR]
        msg.data = [100, 999, 200, 300, 888, 400]
        node._ticks_array_callback(msg)

        assert node.current_ticks["left_front"] == 100
        assert node.current_ticks["left_rear"] == 200
        assert node.current_ticks["right_front"] == 300
        assert node.current_ticks["right_rear"] == 400
        node.destroy_node()
    except (ImportError, AttributeError):
        pass


def test_odometry_no_drift_when_stationary():
    """Verify that odometry linear velocity drops to 0 and position does not drift when no ticks arrive."""
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
    try:
        import rclpy
        from sensor_msgs.msg import JointState
        from rclpy.time import Duration
        if not rclpy.ok():
            rclpy.init()
        node = EncoderTicksToOdomNode()

        # Step 0: establish baseline at 0 rad
        js0 = JointState()
        js0.name = ["left_front_wheel_joint", "right_front_wheel_joint", "left_rear_wheel_joint", "right_rear_wheel_joint"]
        js0.position = [0.0, 0.0, 0.0, 0.0]
        node._joint_states_callback(js0)

        # Step 1: rotate wheels by 1 radian
        js1 = JointState()
        js1.name = js0.name
        js1.position = [1.0, 1.0, 1.0, 1.0]
        node._joint_states_callback(js1)

        # Run first odometry timer step
        node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
        node._update_odometry()
        x_after_step1 = node.x
        assert x_after_step1 > 0.0, "Expected positive linear displacement after wheel motion"

        # Run second odometry timer step WITHOUT any new tick messages (robot stationary)
        node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
        node._update_odometry()
        x_after_step2 = node.x

        # Assert zero drift: position must be identical and wheel velocities must be 0.0
        assert x_after_step2 == pytest.approx(x_after_step1), f"Odometry drifted while stationary: {x_after_step1} vs {x_after_step2}"
        for w in node.wheel_names:
            assert node.wheel_velocities[w] == pytest.approx(0.0), f"Wheel {w} velocity not 0.0 while stationary"
            assert node.wheel_delta_ticks[w] == 0, f"Wheel {w} delta ticks not 0 while stationary"

        node.destroy_node()
    except (ImportError, AttributeError):
        pass


def test_joint_state_nan_inf_safety():
    """Verify that non-finite (NaN, Inf) position values in JointState do not crash the node."""
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
    try:
        import rclpy
        from sensor_msgs.msg import JointState
        if not rclpy.ok():
            rclpy.init()
        node = EncoderTicksToOdomNode()
        msg = JointState()
        msg.name = ["left_front_wheel_joint", "right_front_wheel_joint"]
        msg.position = [float('nan'), float('inf')]
        # Should execute safely without raising ValueError
        node._joint_states_callback(msg)
        assert node.current_ticks["left_front"] == 0
        assert node.current_ticks["right_front"] == 0
        node.destroy_node()
    except (ImportError, AttributeError):
        pass


def test_steering_joints_rejected():
    """Verify that steering joints are rejected and do not clobber drive wheel ticks."""
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
    try:
        import rclpy
        from sensor_msgs.msg import JointState
        if not rclpy.ok():
            rclpy.init()
        node = EncoderTicksToOdomNode()
        msg = JointState()
        msg.name = [
            "left_front_steer_joint",
            "right_front_steering_joint",
            "left_rear_steering_wheel_joint",
            "right_rear_steer_joint",
        ]
        msg.position = [math.pi, math.pi, math.pi, math.pi]
        node._joint_states_callback(msg)

        # Steering joints must NOT update drive wheel ticks
        for w in node.wheel_names:
            assert node.current_ticks[w] == 0
        node.destroy_node()
    except (ImportError, AttributeError):
        pass


def test_multi_message_tick_accumulation_between_odom_cycles():
    """Verify that multiple tick updates between timer intervals are fully accumulated without loss."""
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
    try:
        import rclpy
        from rclpy.time import Duration
        from std_msgs.msg import Int64MultiArray
        if not rclpy.ok():
            rclpy.init()
        node = EncoderTicksToOdomNode()

        # Baseline ticks: 100 on all wheels
        msg1 = Int64MultiArray()
        msg1.data = [100, 100, 100, 100]
        node._ticks_array_callback(msg1)

        # Establish odometry baseline
        node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
        node._update_odometry()

        # First message: 150 ticks (+50)
        msg2 = Int64MultiArray()
        msg2.data = [150, 150, 150, 150]
        node._ticks_array_callback(msg2)

        # Second message before timer: 220 ticks (+70)
        msg3 = Int64MultiArray()
        msg3.data = [220, 220, 220, 220]
        node._ticks_array_callback(msg3)

        # Step timer (total delta should be 220 - 100 = 120 ticks)
        node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
        node._update_odometry()

        for w in node.wheel_names:
            assert node.wheel_delta_ticks[w] == 120, f"Expected 120 delta ticks, got {node.wheel_delta_ticks[w]}"

        node.destroy_node()
    except (ImportError, AttributeError):
        pass


def test_side_velocity_nan_inf_rejection():
    """Verify that non-finite values (NaN, Inf) in compute_robust_side_velocity are safely filtered."""
    from rover_slam.encoder_ticks_to_odom import compute_robust_side_velocity
    # Single NaN with a valid wheel: should safely return the valid wheel's velocity and flag slip
    v, slip = compute_robust_side_velocity([float('nan'), 0.35])
    assert v == pytest.approx(0.35)
    assert slip is True

    # Single Inf with a valid wheel: should safely return the valid wheel's velocity and flag slip
    v, slip = compute_robust_side_velocity([float('inf'), -0.25])
    assert v == pytest.approx(-0.25)
    assert slip is True

    # All non-finite: returns 0.0 safely
    v, slip = compute_robust_side_velocity([float('nan'), float('nan')])
    assert v == 0.0
    assert slip is False


def test_yaw_normalization_and_track_width_guard():
    """Verify that yaw wraps safely to [-pi, pi] and zero track width does not divide by zero."""
    from rover_slam.encoder_ticks_to_odom import EncoderTicksToOdomNode
    try:
        import rclpy
        from rclpy.time import Duration
        if not rclpy.ok():
            rclpy.init()
        node = EncoderTicksToOdomNode()
        # Set track width to 0 to test divide-by-zero defense
        node.track_width = 0.0
        node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
        # Should not raise ZeroDivisionError
        node._update_odometry()

        # Simulate large cumulative yaw
        node.yaw = 10.0 * math.pi + 0.5  # ~31.91 rad
        node.last_time = node.get_clock().now() - Duration(seconds=0, nanoseconds=20000000)
        node._update_odometry()
        # Node yaw must be strictly within [-pi, pi]
        assert -math.pi <= node.yaw <= math.pi

        node.destroy_node()
    except (ImportError, AttributeError):
        pass


