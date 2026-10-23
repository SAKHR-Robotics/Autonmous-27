"""Exercise real ROS launch expansion without starting perception processes.

Only the process boundary (Node) is replaced. Includes, conditions, scopes,
launch substitutions, and ROS parameter normalization use the installed ROS
implementation. These tests require a sourced Jazzy installation.
"""
from pathlib import Path

import pytest


@pytest.fixture
def expand(monkeypatch):
    launch = pytest.importorskip("launch")
    actions = pytest.importorskip("launch.actions")
    ros_actions = pytest.importorskip("launch_ros.actions")
    ros_utilities = pytest.importorskip("launch_ros.utilities")
    packages = pytest.importorskip("ament_index_python.packages")
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    perception = Path(__file__).resolve().parents[2]
    shares = {
        "marker_detection": perception / "marker_detection",
        "terrain_geometry": perception / "terrain_geometry_improved" / "terrain_geometry",
    }
    monkeypatch.setattr(packages, "get_package_share_directory", lambda name: str(shares[name]))
    captured = []

    class CaptureNode(launch.Action):
        def __init__(self, *, package, executable, parameters, **kwargs):
            super().__init__()
            self.package = package
            self.executable = executable
            self.parameters = parameters

        def execute(self, context):
            parameters = ros_utilities.evaluate_parameters(
                context, ros_utilities.normalize_parameters(self.parameters))
            merged = {}
            files = []
            for parameter in parameters:
                if isinstance(parameter, dict):
                    merged.update(parameter)
                else:
                    files.append(Path(parameter))
            captured.append((self.package, self.executable, merged, files))

    monkeypatch.setattr(ros_actions, "Node", CaptureNode)

    def run(overrides=None, *, branch=False):
        captured.clear()
        service = launch.LaunchService()
        if branch:
            path = shares["marker_detection"] / "launch" / "marker_branch.launch.py"
        else:
            path = perception / "perception_bringup" / "launch" / "perception_core.launch.py"
        service.include_launch_description(launch.LaunchDescription([
            actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(path)),
                launch_arguments=(overrides or {}).items(),
            ),
        ]))
        assert service.run() == 0
        return {executable: (package, parameters, files)
                for package, executable, parameters, files in captured}

    return run


def test_default_nodes_are_real_and_mapping_is_opt_in(expand):
    nodes = expand()
    assert set(nodes) == {
        "terrain_node", "marker_detection_node", "marker_tf_broadcaster",
        "marker_action_interface_node",
    }
    assert {package for package, _, _ in nodes.values()} == {"terrain_geometry", "marker_detection"}
    assert all(parameters["use_sim_time"] is False for _, parameters, _ in nodes.values())
    assert nodes["marker_detection_node"][2][0].name == "marker_detection.yaml"
    assert nodes["marker_tf_broadcaster"][2][0].name == "marker_tf.yaml"
    assert nodes["marker_action_interface_node"][2][0].name == "marker_action_interface.yaml"


@pytest.mark.parametrize("switch,executable", [
    ("terrain", "terrain_node"),
    ("marker_detection", "marker_detection_node"),
    ("marker_tf", "marker_tf_broadcaster"),
    ("marker_mapping", "marker_map_node"),
    ("marker_action", "marker_action_interface_node"),
])
def test_each_node_can_run_independently(expand, switch, executable):
    switches = {f"enable_{stage}": "false" for stage in (
        "terrain", "marker_detection", "marker_tf", "marker_mapping", "marker_action")}
    switches[f"enable_{switch}"] = "true"
    assert set(expand(switches)) == {executable}


def test_all_nodes_can_be_disabled(expand):
    assert expand({f"enable_{stage}": "false" for stage in (
        "terrain", "marker_detection", "marker_tf", "marker_mapping", "marker_action")}) == {}


def test_simulation_clock_topics_and_frame_reach_children(expand):
    nodes = expand({
        "use_sim_time": "true", "enable_marker_mapping": "true",
        "rgb_topic": "/test/rgb", "depth_topic": "/test/aligned_depth",
        "camera_info_topic": "/test/calibration",
        "input_pointcloud_topic": "/test/points", "target_frame": "test_base",
    })
    assert len(nodes) == 5
    assert all(parameters["use_sim_time"] is True for _, parameters, _ in nodes.values())
    terrain = nodes["terrain_node"][1]
    assert terrain["input_pointcloud_topic"] == "/test/points"
    assert terrain["target_frame"] == "test_base"
    detector = nodes["marker_detection_node"][1]
    assert detector["rgb_topic"] == "/test/rgb"
    assert detector["depth_topic"] == "/test/aligned_depth"
    assert detector["camera_info_topic"] == "/test/calibration"


@pytest.mark.parametrize("branch", [False, True])
def test_parameter_files_are_isolated(expand, tmp_path, branch):
    names = ("detection", "tf", "mapping", "action")
    executables = ("marker_detection_node", "marker_tf_broadcaster", "marker_map_node",
                   "marker_action_interface_node")
    overrides = {"use_sim_time": "true"}
    expected = {}
    for name, executable in zip(names, executables):
        path = tmp_path / f"{name}.yaml"
        path.write_text("/**:\n  ros__parameters: {}\n")
        argument = f"{name}_params_file" if branch else f"marker_{name}_params_file"
        overrides[argument] = str(path)
        expected[executable] = path
    if not branch:
        overrides["enable_marker_mapping"] = "true"
    nodes = expand(overrides, branch=branch)
    for executable, path in expected.items():
        assert nodes[executable][2] == [path]
        assert nodes[executable][1]["use_sim_time"] is True


def test_legacy_branch_still_includes_mapping(expand):
    assert set(expand(branch=True)) == {
        "marker_detection_node", "marker_tf_broadcaster", "marker_map_node",
        "marker_action_interface_node",
    }
