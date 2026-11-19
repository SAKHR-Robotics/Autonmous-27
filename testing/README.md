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
The environment is currently awaiting confirmation; the repository contains both
Jazzy and Humble instructions. No runtime profile is approved by this document.

| Decision | Required record |
| --- | --- |
| Runtime | ROS distribution, host/container, workspace path, build/source commands, ROS domain |
| Test boundary | Module under test, real nodes, exact mocked dependencies, external prerequisites |
| Clock and geometry | Wall time or `/clock` owner, `use_sim_time`, frame IDs, TF owners, sensor extrinsics |
| Interfaces and scenario | Topic/action/service names and types, QoS, rates, units, seed, duration, expected outputs and tolerances |
| Operation and view | Nodes started by launch, RViz on/off and config, displayed topics/fixed frame, output directory, shutdown behavior |

Launch files should not prompt interactively. Resolve choices in conversation,
then represent them through arguments/configuration so runs remain repeatable.

## Current layout — 2026-11-19

```text
testing/
├── mocks/
│   ├── pathplanning/pathplanner_mock_tools/
│   ├── perception/perception_test_tools/
│   └── slam/slam_mock_tools/
├── benchmarking/
│   ├── pathplanning/                 # global_path_benchmarking package
│   ├── slam/slam_test_tools/
│   └── perception/                   # documentation; observer not implemented
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
to deleted source paths. No new launch profile or GUI has been implemented.

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
| T05 | `benchmarking/slam/slam_test_tools/launch/test_slam_standalone.launch.py` does not supply complete SLAM sensor inputs. Existing stubs are fixed smoke fixtures. | Agree local-EKF versus RGB-D SLAM stages, then supply coherent missing inputs and separate ground truth. Do not claim trajectory accuracy from the current stubs. |
| T06 | `PathPlanning/erc_path_planner/launch/test_planner_standalone.launch.py` publishes a synthetic map anchor from a production package; its default map is `config/dummy_map.yaml`. | Move test orchestration and synthetic assets under the appropriate testing package, updating install rules and references; agree whether a compatibility wrapper is needed. |
| T07 | `mocks/pathplanning/pathplanner_mock_tools/custom_maps/` now has package installation. PathPlanner still generates `maps/` and `reports/` relative to the caller's working directory. | Assign fixture ownership to a test package, resolve installed resources through package share, and expose a separate run output directory. Check launching from outside the source tree. |

## Validation sequence

| Stage | Bounded result | Completion gate |
| --- | --- | --- |
| 0 — Foundation (documented) | Inventory, rules, issue list and runtime decision | Review this document and confirm the runtime; no runtime pass claimed. |
| 1 — Package discovery and installation | Layout reorganized; discovery passed; installation pending | Selected packages build and their executables, launch files and fixtures resolve from the overlay. |
| 2 — Mock feeders alone | Validate one feeder at a time before consuming its data | Expected message types, QoS, rates, timestamps, frames, deterministic truth and dropout behavior. |
| 3 — One real module with mocked dependencies | Dedicated agreed launch and RViz profile for that module | Expected real outputs, failure/recovery behavior, no duplicate owners, visual checks recorded separately. |
| 4 — Replace mocks with real dependencies | Integrate only modules that passed their isolated stage | Repeat boundary checks and compare against agreed tolerances; document remaining gaps. |

Run Stage 3 separately for each module, rather than treating a combined stack as
proof that each module works. Each implementation step must supply its own
commands and observations before proceeding.

## Commands to validate the layout

From the repository root on the host (or `/workspace` in the agreed container):

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
colcon list --base-paths testing
git show --stat HEAD
```

Expected: five `ros.ament_python` packages: `global_path_benchmarking`,
`pathplanner_mock_tools`, `perception_test_tools`, `slam_mock_tools`, and
`slam_test_tools`. No duplicate package names. Discovery is not a build/runtime
pass. See the [PathPlanner validation record](benchmarking/pathplanning/VALIDATION.md)
for offline benchmark checks and known failures.

Production documentation outside `testing/` has intentionally not been edited.
Commands there using `testing/PathPlanner`, `testing/SLAM` or `testing/Perception`
need the new paths above; the other agent owns those documentation updates.
