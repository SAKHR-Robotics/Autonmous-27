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

[Mock ownership](mocks/README.md), [benchmark status](benchmarking/README.md) and
[bringup boundaries](validation_bringup/README.md) describe what actually exists.
The mission is the evaluation goal: module observers explain route quality,
tracking, perception and localization across the same mission. Mocked and real
runs must identify their dependencies; mission success with mocked inputs does
not establish full-stack readiness.

Original ROS package names and launch defaults are preserved. Mock implementations
were extracted into `pathplanner_mock_tools` and `slam_mock_tools`; old Python
imports and executable names delegate to those packages. Rebuild in a separate
build/install directory after relocation; an old symlink overlay can still point
to deleted source paths. SLAM now has an isolated `local_mocked.launch.py` profile
in `slam_test_tools`; shared bringup and its future GUI remain unimplemented.

Historical benchmark reports remain in `benchmarking/pathplanning/reports/` and
historical design documents remain beside their modules. Neither establishes
current readiness. The legacy `launch_world_and_rover.sh` remains at its existing
path while its unsafe cleanup awaits a separate fix.

## Open issues and proposed fixes

These are findings to resolve in bounded implementation steps. The layout step
preserves launch defaults and mock algorithms.
Known behavior defects remain open; folder organization does not fix scoring.

| ID | Location / finding | Proposed resolution and acceptance check |
| --- | --- | --- |
| T01 | `benchmarking/pathplanning/package.xml` now declares `ament_python`; remaining launch/report dependencies still need reconciliation, including launch libraries, NumPy, YAML, Matplotlib and Pillow. | Reconcile imports against declared dependencies, distinguish optional report tools, then build/discover the package in the chosen clean environment. |
| T02 | `benchmarking/pathplanning/launch/live_test.launch.py` declares `target_package` and `target_launch` but includes the hardcoded ERC launch. Missing ERC can silently produce an empty group. | Confirm supported target modes, honor their arguments, and fail clearly for a missing selected target. Check both internal and external modes. |
| T03 | The same live launch adds a costmap bridge while `PathPlanning/erc_path_planner/launch/path_planning.launch.py` already starts one. | Agree one bridge owner per profile. Confirm one authoritative bridge-cloud publisher and the expected message contract. |
| T04 | `benchmarking/pathplanning/global_path_benchmarking/testing_node.py` uses global `pkill -9` with `clean`; `launch_world_and_rover.sh` also kills processes by name. | Replace cleanup with tracking/stopping run-owned processes. Verify a separate unrelated process survives stopping the test run. Keep `clean` disabled meanwhile. |
| T05 | The standalone SLAM wrapper still requires external RGB-D sensors. A separate nominal local-EKF profile now reuses the shared rover and a sensor adapter. | See the [SLAM evidence](../SLAM/VALIDATION.md) for tested cases, lateral-fusion drift and clock-reset recovery failures. Legacy landmark/obstacle stubs remain unvalidated smoke fixtures. |
| T06 | `PathPlanning/erc_path_planner/launch/test_planner_standalone.launch.py` publishes a synthetic map anchor from a production package; its default map is `config/dummy_map.yaml`. | Move test orchestration and synthetic assets under the appropriate testing package, updating install rules and references; agree whether a compatibility wrapper is needed. |
| T07 | `mocks/pathplanning/pathplanner_mock_tools/custom_maps/` now has package installation. PathPlanner still generates `maps/` and `reports/` relative to the caller's working directory. | Assign fixture ownership to a test package, resolve installed resources through package share, and expose a separate run output directory. Check launching from outside the source tree. |

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
