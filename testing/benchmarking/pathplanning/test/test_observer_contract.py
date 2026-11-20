"""Exercise observer/runner methods with ROS message types, without starting nodes."""
import json
import math
from pathlib import Path as FilePath
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml

pytest.importorskip('rclpy', reason='Source ROS 2 Jazzy for observer contracts')
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from global_path_benchmarking.benchmarking_node import BenchmarkingNode
from global_path_benchmarking.report_generator import costmap_array
from global_path_benchmarking.testing_node import TestingNode as RunnerNode


def path_message(coords):
    path = Path()
    path.header.frame_id = 'map'
    for x, y in coords:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        path.poses.append(pose)
    return path


def observer(grid=None):
    return SimpleNamespace(start_pose=None, goal_pose=None, costmap=grid,
                           cost_arr=costmap_array(grid))


def grid_message():
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 5
    grid.info.resolution = 1.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 25
    return grid


@pytest.mark.parametrize('coords', [[], [(0, 0)], [(0, 0), (float('nan'), 1)]])
def test_observer_does_not_override_invalid_geometry(coords):
    result = BenchmarkingNode.evaluate_path(observer(), path_message(coords), None)
    assert result['success'] is False


def test_valid_geometry_without_map_is_incomplete():
    result = BenchmarkingNode.evaluate_path(observer(), path_message([(0, 0), (1, 0)]), None)
    assert result['success'] is None
    assert result['blocked_cells'] is None


def test_detected_sample_collision_is_not_success():
    grid = grid_message()
    grid.data[0] = 100
    result = BenchmarkingNode.evaluate_path(observer(grid), path_message([(0, 0), (1, 0)]), None)
    assert result['blocked_cells'] > 0
    assert result['success'] is False


@pytest.mark.parametrize('unsupported', ['unknown', 'rotated', 'bad_size', 'zero_resolution', 'frame_mismatch'])
def test_unsupported_map_does_not_look_safe(unsupported):
    grid = grid_message()
    if unsupported == 'unknown':
        grid.data[0] = -1
    elif unsupported == 'rotated':
        grid.info.origin.orientation.z = math.sin(0.5)
        grid.info.origin.orientation.w = math.cos(0.5)
    elif unsupported == 'bad_size':
        grid.data = [0]
    elif unsupported == 'zero_resolution':
        grid.info.resolution = 0.0
    else:
        grid.header.frame_id = 'other_frame'
    result = BenchmarkingNode.evaluate_path(observer(grid), path_message([(0, 0), (1, 0)]), None)
    assert result['success'] is None
    assert result['safety_margin'] is None


def test_path_arrival_writes_incomplete_score_and_can_compile(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    config = yaml.safe_load((FilePath(__file__).parents[1] / 'config/benchmark_config.yaml').read_text())['benchmark']
    state = observer(grid_message())
    state.static_eval = BenchmarkingNode.evaluate_path(state, path_message([(0, 0), (1, 0)]), None)
    state.static_time = 0.2
    state.replan_eval = None
    state.replan_time = None
    state.cmd_history = []
    state.odom_history = []
    state.limits, state.weights = config['limits'], config['weights']
    state.pass_threshold = config['pass_threshold']
    state.get_logger = lambda: Mock()
    state.generate_comparison_plot = Mock()
    BenchmarkingNode.process_scenario_results(state, {'id': 'trial'})
    result = json.loads((tmp_path / 'reports/temp_trial.json').read_text())
    assert result['outcome'] == 'INCOMPLETE'
    assert result['controller_score'] is None
    assert result['final_score'] is None
    assert result['mean_cte'] is None
    assert state.generate_comparison_plot.call_args.args[3] is None

    from global_path_benchmarking import testing_node
    # PDF availability is checked separately; verify a failed exporter is not called a success.
    monkeypatch.setattr(testing_node, 'generate_pdf_report', lambda *args: False)
    state.scenarios = [{'id': 'trial'}]
    assert RunnerNode.compile_reports(state) is False
    markdown = (tmp_path / 'reports/numerical_report.md').read_text()
    assert 'INCOMPLETE' in markdown and 'N/A' in markdown
    assert '95.0' not in markdown
    assert (tmp_path / 'reports/numerical_report.html').is_file()


def test_observer_reads_actual_odometry_heading():
    state = SimpleNamespace(odom_history=[], get_clock=lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1000000000)))
    message = Odometry()
    message.pose.pose.orientation.z = math.sin(math.pi / 4)
    message.pose.pose.orientation.w = math.cos(math.pi / 4)
    BenchmarkingNode.odom_callback(state, message)
    assert state.odom_history[-1][3] == pytest.approx(math.pi / 2)
