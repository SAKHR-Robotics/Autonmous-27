"""Known-answer checks for mission measurements; no ROS or mock nodes needed.

These checks guard measurement contracts. Do not interpret them as a live
navigation validation or a mission-completion result.
"""
from types import SimpleNamespace

import pytest

from global_path_benchmarking.report_generator import (
    calculate_control_metrics,
    calculate_path_metrics,
)


def pose(x, y):
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))


def test_known_polyline_length():
    result = calculate_path_metrics([(0.0, 0.0), (3.0, 4.0), (6.0, 8.0)])
    assert result['length'] == pytest.approx(10.0)
    assert result['ratio'] == pytest.approx(1.0)


def test_known_tracking_offset():
    commands = [(0.0, 1.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)]
    odometry = [(0.0, 1.0, 1.0, 0.0, 1.0, 0.0),
                (1.0, 2.0, 1.0, 0.0, 1.0, 0.0)]
    result = calculate_control_metrics(commands, odometry, [(0.0, 0.0), (10.0, 0.0)])
    assert result['mean_cte'] == pytest.approx(1.0)
    assert result['max_cte'] == pytest.approx(1.0)
    assert result['rms_cte'] == pytest.approx(1.0)


def test_known_final_goal_distance():
    result = calculate_control_metrics(
        [(0.0, 1.0, 0.0, 0.0)],
        [(0.0, 2.0, 0.0, 0.0, 1.0, 0.0)],
        [(0.0, 0.0), (3.0, 0.0)], pose(3.0, 0.0))
    assert result['goal_accuracy_m'] == pytest.approx(1.0)


def test_no_streams_cannot_produce_controller_score():
    result = calculate_control_metrics([], [], [])
    assert result.get('controller_score') is None, 'No execution evidence: score must be unavailable'


def test_commands_without_odometry_cannot_measure_goal_error():
    result = calculate_control_metrics(
        [(0.0, 1.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0)],
        [], [(0.0, 0.0), (3.0, 0.0)], pose(3.0, 0.0))
    assert result.get('goal_accuracy_m') is None, 'No pose observations: goal error is unavailable'


def test_missing_map_cannot_measure_clearance():
    result = calculate_path_metrics([(0.0, 0.0), (1.0, 0.0)])
    assert result.get('nearest_obstacle') is None, 'No map: clearance is unavailable'


@pytest.mark.parametrize('bad_value', [float('nan'), float('inf')])
def test_nonfinite_path_is_rejected(bad_value):
    try:
        result = calculate_path_metrics([(0.0, 0.0), (bad_value, 1.0)])
    except ValueError:
        return
    assert result['success'] is False, 'Nonfinite geometry must not be a successful path'


def test_no_streams_have_no_measured_metrics():
    result = calculate_control_metrics([], [], [])
    assert all(value is None for value in result.values())


def test_one_command_does_not_establish_cadence_or_smoothness():
    result = calculate_control_metrics([(1.0, 0.5, 0.0, 0.0)], [], [])
    assert result['mean_linear_vel'] == pytest.approx(0.5)
    assert result['cmd_freq_hz'] is None
    assert result['linear_jerk_std'] is None
    assert result['controller_score'] is None


def test_known_command_cadence_counts_intervals():
    commands = [(0.0, 1.0, 0.0, 0.0), (0.5, 1.0, 0.0, 0.0),
                (1.0, 1.0, 0.0, 0.0)]
    result = calculate_control_metrics(commands, [], [])
    assert result['cmd_freq_hz'] == pytest.approx(2.0)
    assert result['mean_cte'] is None
    assert result['controller_score'] is None


@pytest.mark.parametrize('times', [(1.0, 1.0), (2.0, 1.0)])
def test_invalid_command_time_does_not_establish_score(times):
    result = calculate_control_metrics(
        [(times[0], 1.0, 0.0, 0.0), (times[1], 1.0, 0.0, 0.0)], [], [])
    assert result['cmd_freq_hz'] is None
    assert result['control_effort'] is None
    assert result['controller_score'] is None


def test_missing_map_cannot_measure_cost():
    result = calculate_path_metrics([(0.0, 0.0), (1.0, 0.0)])
    assert result['avg_cost'] is None
    assert result['total_cost'] is None


def test_nonfinite_goal_is_rejected():
    result = calculate_path_metrics([(0.0, 0.0), (1.0, 0.0)], goal_pose=pose(float('nan'), 0.0))
    assert result['success'] is False


def test_available_metrics_do_not_prove_execution_completion():
    commands = [(t, 1.0, 0.0, 0.0) for t in (0.0, 0.5, 1.0)]
    odometry = [(t, t, 0.0, 0.0, 1.0, 0.0) for t in (0.0, 0.5, 1.0)]
    args = (commands, odometry, [(0.0, 0.0), (1.0, 0.0)], pose(1.0, 0.0))
    assert calculate_control_metrics(*args)['controller_score'] is None
    assert calculate_control_metrics(*args, execution_complete=True)['controller_score'] > 0


def test_completion_without_required_observations_has_no_score():
    result = calculate_control_metrics([], [], [], execution_complete=True)
    assert result['controller_score'] is None
