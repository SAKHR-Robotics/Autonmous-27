# PathPlanner dependency mocks

Everything here is test-only. The supported `mocked_planner.launch.py` starts
**real `erc_path_planner`/Nav2**, with a synthetic map, localization/motion and
Perception inputs. Production does not depend on this package.

| Asset | Purpose |
| --- | --- |
| `launch/mocked_planner.launch.py` | Wall-time mock dependencies + real production master |
| `launch/test_planner_standalone.launch.py` | Real production master with test map/optional identity `map → odom`; requires external rover odometry/sensors |
| `custom_maps/` | Synthetic dummy, dead-end, corridor and rock-field maps |
| `rviz/mocked_planner.rviz` | Real planning outputs plus mock rover/obstacle markers |
| `mock_rover_sim` | Integrates actual `/cmd_vel`, publishes `/odometry/filtered` and `map → odom → base_link` |
| `mock_perception` | Optional real bridge-compatible `Detection3DArray` plus legacy feature/scan/cloud outputs |

The supported mock profile selects `publish_detections:=true`, keeps
`publish_bridge_cloud:=false`, and starts one real bridge. Shared mock defaults
remain unchanged. The motion fixture uses zero noise/slip and 0.9 × 0.7 m visual
geometry matching the configured Nav2 footprint. This is a synthetic consistency
fixture; its odometry and TF come from one model, not independent sensor truth.
The fixture has no camera clearing source: its costmap observations are explicitly
marking-only. Removing an obstacle does not establish clearing behavior.

Legacy executables `drop_stone`, `algo_node`, `mock_base` and
`generate_benchmark_maps` remain installed. `algo_node` is a toy planner for
benchmarker development; it is never started by the supported PathPlanner launch.
Do not use it as evidence against or in favor of production Smac. Legacy scan
geometry is not validated for a moving sensor; the supported launch does not use it.
Old `global_path_benchmarking` mock executables/imports delegate here; do not
start both names simultaneously.

## Build inside the existing Jazzy image

Use the container preparation in the
[production guide](../../../../PathPlanning/erc_path_planner/README.md#docker-build-and-real-launch).
Inside that container:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /validation/log build --symlink-install \
  --base-paths /workspace/PathPlanning/erc_path_planner \
    /workspace/Perception/terrain_geometry_msgs \
    /workspace/testing/mocks/pathplanning/pathplanner_mock_tools \
    /workspace/testing/benchmarking/pathplanning \
  --build-base /validation/build --install-base /validation/install
source /validation/install/setup.bash
ros2 launch pathplanner_mock_tools mocked_planner.launch.py
```

Expected: map server, planner, controller, behaviors, smoother and navigator all
become active. No Gazebo, real SLAM or real Perception should run in this same
mock profile. It supplies both localization TF edges itself and uses wall time.

Optional arguments: `use_rviz`, `map`, `initial_x`, `initial_y`, `initial_yaw`,
`use_mock_rover`, `use_mock_perception`, `use_costmap_bridge`, `scenario_id`,
`scenarios_file`, `cmd_vel_topic`, `control_mode`, `target_package`, `target_launch`,
`use_external_module`. External targets must accept the documented production
launch arguments. Changing command topics requires the target's remaps to match.
Disable the corresponding mock before attaching a real dependency.
RViz requires an authorized display/Xauthority mount into the container.

The former `global_path_benchmarking live_test.launch.py` delegates to this launch.
The former **production** `test_planner_standalone.launch.py` has moved here;
no production wrapper launches testing code.

## Observe actual action results

In another host terminal:

```bash
docker exec -it pathplanner-run bash
source /opt/ros/jazzy/setup.bash
source /validation/install/setup.bash
ros2 run global_path_benchmarking navigation_probe \
  --mode plan --goal 2 -1 0 --output /validation/plan.json
ros2 run global_path_benchmarking navigation_probe \
  --mode navigate --goal 2 -1 0 --timeout 45 --output /validation/navigation.json
```

The probe records a unique action goal ID, terminal status, error code and
monotonic request-to-result latency. It waits for navigation completion; receiving
`/plan` is not completion. `--cancel-after 0.5` exercises cancellation. A timeout
requests cancellation and records `TIMEOUT`; absent servers/data remain unavailable.
Non-success returns exit code 1. JSON contains raw measurements, not a mission score.
Estimated map-TF goal error is explicitly not independent localization accuracy.

Use `--truth-map /absolute/scenario.yaml` for conservative swept rectangular
footprint checks against independent scenario geometry. Unknown/outside cells
are blocked; do not substitute a Nav2 output costmap as ground truth. Fixture
maps that omit spawned mock obstacles cannot establish obstacle clearance.

Reproduce the known boundary failure only in a **disposable** mocked run:

```bash
ros2 run global_path_benchmarking planner_map_probe \
  --output-dir /validation/map-probe
```

This replaces the map-server map. It requests planning only, never movement.
Expected in the recorded image: blocked-wall and too-narrow requests return
paths whose footprints leave map bounds; the independent gate fails and the
command returns exit code 1. The wide passage passes. These are unresolved
planner validation failures, not successful navigation trials.
Restart the mocked launch before other scenarios.

Run the numerical/observer/bridge checks:

```bash
PATHPLANNER_RUNTIME_TESTS=1 python3 -m pytest \
  /workspace/testing/benchmarking/pathplanning/test \
  -q -rs -p no:cacheprovider
```

The runtime bridge checks use distinct test topics and a separately named
production C++ bridge. Positive PDF-export tests skip when ReportLab is absent.
See [PathPlanning validation](../../../../PathPlanning/VALIDATION.md) for evidence,
known stale-input stopping failure and the Gazebo procedure.

Stop active navigation first, then Ctrl+C this run's launches. Stop only the
container you created; broad process-name cleanup is disabled.
