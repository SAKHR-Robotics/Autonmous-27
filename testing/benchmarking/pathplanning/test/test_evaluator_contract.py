"""Known-answer checks for mission measurements; no ROS or mock nodes needed.

Failures expose current evaluator defects. Do not mark them xfail or interpret
these checks as a live navigation validation.
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
