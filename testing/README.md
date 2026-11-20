# Staged stack validation

Start here before validating a module. This folder owns test harnesses, runtime
mocks, synthetic datasets and mocked launch entry points. Each executable harness
is a ROS 2 package; grouping directories and documentation need no package of
their own. Production modules remain separate ROS 2 packages.

## Working rules

1. **Keep dependencies one-way.** Test packages may depend on production packages.
   Production packages must not require test packages for normal operation.
   Keep runtime mocks and synthetic fixtures under `testing/`; production-local
   unit tests may remain in their package's `test/` directory. Move existing
   exceptions with compatibility and installation changes in the same step.
2. **Declare the boundary.** For each module, record real nodes, mocked upstream
   dependencies, observed outputs, topic types, QoS, rates, timestamps, frames and
   the owner of every TF edge. A mock must satisfy the consumer's actual contract.
   Replacing the module's own output cannot validate that module.
3. **Make launches explicit.** Before creating or changing a launch, ask Saif to
   settle unknown runtime choices using the launch contract below. Expose the
   agreed choices as documented arguments or named profiles. Reject incompatible
   settings and missing inputs; do not silently substitute data or skip nodes.
   Keep production launches and mocked test launches distinct.
4. **Use repeatable scenarios and scoped cleanup.** Give fixtures named scenarios,
   fixed seeds and known expected results. Use an agreed isolated ROS domain with
   no rover hardware. Allow only one publisher per authoritative output/TF edge.
   Stop processes owned by the run; do not kill unrelated ROS processes by name.
   Keep generated results separate from input fixtures and identify their run.
5. **Close one stage with evidence.** Provide exact commands, expected observations,
   failure conditions and shutdown steps. Record passed, failed, blocked and
   untested separately, with environment and revision. Address each failure with
   file, cause, proposed fix and a focused recheck. RViz inspection and automated
   checks are separate gates; synthetic success does not establish sensor accuracy.

## Launch contract to settle before implementation

Confirm unknown values once for a profile and reuse the recorded decision.
Current recorded validation uses the existing `autonomous27:jazzy` Docker image.
Historical Humble instructions are not the current validation environment.
Module-specific clock/TF/input choices remain explicit in their running guides;
this index does not approve additional launch profiles.

| Decision | Required record |
| --- | --- |
| Runtime | ROS distribution, host/container, workspace path, build/source commands, ROS domain |
| Test boundary | Module under test, real nodes, exact mocked dependencies, external prerequisites |
| Clock and geometry | Wall time or `/clock` owner, `use_sim_time`, frame IDs, TF owners, sensor extrinsics |
| Interfaces and scenario | Topic/action/service names and types, QoS, rates, units, seed, duration, expected outputs and tolerances |
| Operation and view | Nodes started by launch, RViz on/off and config, displayed topics/fixed frame, output directory, shutdown behavior |

Launch files should not prompt interactively. Resolve choices in conversation,
then represent them through arguments/configuration so runs remain repeatable.

## Current layout — 2026-11-20

```text
testing/
├── mocks/
│   ├── pathplanning/pathplanner_mock_tools/
│   ├── perception/perception_test_tools/
│   └── slam/slam_mock_tools/
├── benchmarking/
│   ├── pathplanning/                 # global_path_benchmarking package
│   ├── slam/slam_test_tools/
│   └── perception/perception_benchmark_tools/  # offline helper timing only
└── validation_bringup/               # documented future shared orchestration
```

[Mock ownership](mocks/README.md), [benchmark status](benchmarking/README.md),
[bringup boundaries](validation_bringup/README.md), and the
[phased completion plan & criteria](TESTING_PLAN.md) describe the layout and evaluation model.
The mission is the evaluation goal: module observers explain route quality,
tracking, perception and localization across the same mission. Mocked and real
runs must identify their dependencies; mission success with mocked inputs does
not establish full-stack readiness.

## Real module + completing mock quick-start

Each module pair runs the **real production module** alongside test-only inputs
and/or plant kinematics so you can evaluate real outputs end-to-end:

| Module under test | Real nodes | Completing mock / launch command | What it evaluates |
| :--- | :--- | :--- | :--- |
| **PathPlanner + Controller** | `erc_path_planner` (Smac, MPPI, smoother, C++ bridge) | `ros2 launch pathplanner_mock_tools mocked_planner.launch.py use_rviz:=true`<br>`ros2 run global_path_benchmarking scenario_benchmark_runner --output-dir /validation/pp` | Plan validity, `0.90 × 0.70 m` swept clearance, `NavigateToPose` / `NavigateThroughPoses`, dynamic replanning, cancellation & stale-input stopping (`PP10`/`PP11` gates) |
| **Perception** | `terrain_geometry`, `marker_detection` | `ros2 launch perception_test_tools test_stack.launch.py pipeline:=all scenario:=clear_aruco_nav` | Real terrain boxes/costmap (`ground_z_offset` configurable), non-ambiguous tilted ArUco 6-DoF pose, dropout/missing-depth/invalid-cloud degradation |
| **SLAM (Local EKF)** | `rover_slam` (encoder, slip filter, EKF) | `ros2 launch slam_test_tools local_mocked.launch.py`<br>`ros2 run slam_test_tools local_estimator_probe --output /validation/local-run.json` | 4-wheel odometry, slip diagnostics, IMU/wheel dropout & recovery, rotated mount / gravity sign, tick vs joint input modes |
| **SLAM (Global RTAB-Map)** | `rover_slam` (`ekf` + `rtabmap`) | `./testing/launch_world_and_rover.sh` + `ros2 launch slam_test_tools test_slam_standalone.launch.py` | Requires Gazebo (`final_world_R&A.world`) or `rosbag2` for coherent moving RGB-D imagery (no Python 3D RGB-D renderer) |

Original ROS package names and launch defaults are preserved. Mock implementations
were extracted into `pathplanner_mock_tools` and `slam_mock_tools`; old Python
imports and executable names delegate to those packages. Rebuild in a separate
build/install directory after changes; an old symlink overlay can still point
to deleted source paths. `launch_world_and_rover.sh` now uses scoped process-group
cleanup instead of global `pkill`; historical `reports/` remain archived and new
runs require an external `--output-dir`.

## Open issues and implementation status

Phases 0, 1, and 2 of [TESTING_PLAN.md](TESTING_PLAN.md) are **implemented in source**
and awaiting clean Jazzy Docker build/runtime verification:

| ID | Location / finding | Resolution status |
| --- | --- | --- |
| T01 | `benchmarking/pathplanning/package.xml` | **Implemented (static):** Added `launch`, `launch_ros`, `ament_index_python`, and `python3-matplotlib`; documented optional `python3-reportlab`. Clean `rosdep`/colcon verification in Jazzy pending. |
| T02 | `live_test.launch.py` / `mocked_planner.launch.py` target handling | **Implemented (static):** `live_test.launch.py` forwards all documented args; `mocked_planner.launch.py` fails explicitly if target launch file is missing. Runtime check pending. |
| T03 | Duplicate costmap bridge risk in `live_test.launch.py` | **Implemented (static):** Compatibility wrapper delegates solely to `mocked_planner.launch.py` (`use_costmap_bridge` forwarded once to `erc_path_planner`). |
| T04 | Global `pkill -9` in `testing_node.py` / `launch_world_and_rover.sh` | **Implemented (static):** `--clean` remains rejected in `testing_node.py` and `benchmark.launch.py`; `launch_world_and_rover.sh` uses `setsid` + process-group `cleanup()` without global `pkill`. |
| T05 | Standalone SLAM sensors & nominal local-EKF adapter | **Extended (static):** `rover_sensor_adapter` now supports slip scaling, rotated IMU mounts, gravity sign, and `joint_states` / `ticks_array` / `individual_ticks`; global SLAM evaluator remains Phase 3. |
| T06 | Standalone synthetic map/TF launch ownership | **Resolved:** Lives in `pathplanner_mock_tools/launch/test_planner_standalone.launch.py` with `launch_bridge` arg; production has zero test dependencies. |
| T07 | CWD-relative `maps/` and `reports/` generation | **Implemented (static):** `scenario_benchmark_runner` resolves installed shares via `ament_index_python` and requires `--output-dir` outside source; `scene_assets.py` accepts `output_dir`. |

## Current guides and evidence

| Area | Active guide | What exists / remaining limit |
| --- | --- | --- |
| Perception mocks | [Package README](mocks/perception/perception_test_tools/README.md), [scenarios and commands](mocks/perception/Readme.md) | Live synthetic sensor scenes and offline terrain generator. Historical synthetic checks do not establish real perception or mission accuracy. |
| SLAM mocks | [Package README](mocks/slam/slam_mock_tools/README.md) | Shared-rover sensor adapter, RGB dropout gate and legacy smoke stubs. Adapter consistency is checked; fixed landmark stubs are not positive landmark acceptance. |
| PathPlanner benchmark | [README](benchmarking/pathplanning/README.md), [evidence](benchmarking/pathplanning/VALIDATION.md) | Evaluator/report corrections checked; complete mission, trial accounting and swept-footprint validation remain pending. |
| SLAM validation | [Testing guide](benchmarking/slam/slam_test_tools/README.md), [production evidence](../SLAM/VALIDATION.md) | Local mocked and Gazebo stages recorded; drift, reset recovery and other accuracy/visual gates remain open. |
| Perception timing | [Package README](benchmarking/perception/perception_benchmark_tools/README.md) | Offline helper timings only; not a mission accuracy observer or production-node rate. |

Shared rules remain: document a boundary, check inputs/mocks, evaluate one real
module, then replace mocks with real dependencies during mission integration.
Each stage provides its own commands, expected results and remaining limits.
Passing one stage does not establish whole-mission readiness.

## Commands to validate the layout

From the repository root on the host (or `/workspace` in the agreed container):

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
colcon list --base-paths testing
```

Expected: six `ros.ament_python` packages: `global_path_benchmarking`,
`pathplanner_mock_tools`, `perception_benchmark_tools`, `perception_test_tools`,
`slam_mock_tools`, and `slam_test_tools`, without duplicate package names.
Discovery is not a build/runtime pass. Use the area-specific commands linked
above for validation; the old broad-cleanup shell scripts are not recommended.

Production documentation outside `testing/` is owned by the corresponding module
agent. New commands must use the current paths above; old paths in historical
validation logs describe the layout used when that evidence was recorded.
