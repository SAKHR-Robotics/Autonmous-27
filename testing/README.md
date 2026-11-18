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

## Existing inventory

The following is a source review on 2026-11-18, not a new runtime validation.

| Location / package | Existing capability | Current limit |
| --- | --- | --- |
| `Perception/perception_test_tools` / `perception_test_tools` | Deterministic RGB, depth, CameraInfo, PointCloud2 and truth markers; feeder-only and real-pipeline launches; installed RViz view | Combined launch hardcodes wall time. Optional motion owns `odom -> base_link`. Confirm these choices before changes. Historical evidence: [Perception validation](Perception/VALIDATION.md). |
| `SLAM/slam_test_tools` / `slam_test_tools` | EKF monitor, landmark and bridge-cloud stubs, wrapper for real EKF/RTAB-Map | Wrapper requires external sensor streams, sensor TF and clock; it is not a complete mocked SLAM run. Uses production SLAM RViz views. See [actual capabilities](SLAM/slam_test_tools/README.md). |
| `PathPlanner` / `global_path_benchmarking` | Mock rover, obstacle feeder, scenarios, benchmark runner and installed RViz view | Packaging, launch selection, interface ownership and process cleanup need repair before accepting a run. Its [README](PathPlanner/README.md) also describes intended capabilities. |
| `custom_maps/`, `launch_world_and_rover.sh` | Loose map fixtures and Gazebo launcher | Maps are outside an installed test package. Script cleanup targets process names globally. These are not accepted isolated test entry points. |

## Open issues and proposed fixes

These are findings to resolve in bounded implementation steps. No launch or node
behavior has been changed during the foundation stage.

| ID | Location / finding | Proposed resolution and acceptance check |
| --- | --- | --- |
| T01 | `PathPlanner/package.xml` lacks `ament_python` buildtool and dependencies used by launch/report code, including launch libraries, NumPy, YAML, Matplotlib and Pillow. | Reconcile imports against declared dependencies, distinguish optional report tools, then build/discover the package in the chosen clean environment. |
| T02 | `PathPlanner/launch/live_test.launch.py` declares `target_package` and `target_launch` but includes the hardcoded ERC launch. Missing ERC can silently produce an empty group. | Confirm supported target modes, honor their arguments, and fail clearly for a missing selected target. Check both internal and external modes. |
| T03 | The same live launch adds a costmap bridge while `PathPlanning/erc_path_planner/launch/path_planning.launch.py` already starts one. | Agree one bridge owner per profile. Confirm one authoritative bridge-cloud publisher and the expected message contract. |
| T04 | `PathPlanner/global_path_benchmarking/testing_node.py` uses global `pkill -9` with `clean`; `launch_world_and_rover.sh` also kills processes by name. | Replace cleanup with tracking/stopping run-owned processes. Verify a separate unrelated process survives stopping the test run. Keep `clean` disabled meanwhile. |
| T05 | `SLAM/slam_test_tools/launch/test_slam_standalone.launch.py` does not supply complete SLAM sensor inputs. Existing stubs are fixed smoke fixtures. | Agree local-EKF versus RGB-D SLAM stages, then supply coherent missing inputs and separate ground truth. Do not claim trajectory accuracy from the current stubs. |
| T06 | `PathPlanning/erc_path_planner/launch/test_planner_standalone.launch.py` publishes a synthetic map anchor from a production package; its default map is `config/dummy_map.yaml`. | Move test orchestration and synthetic assets under the appropriate testing package, updating install rules and references; agree whether a compatibility wrapper is needed. |
| T07 | `testing/custom_maps/` has no package installation. PathPlanner generates `maps/` and `reports/` relative to the caller's working directory. | Assign fixture ownership to a test package, resolve installed resources through package share, and expose a separate run output directory. Check launching from outside the source tree. |

## Validation sequence

| Stage | Bounded result | Completion gate |
| --- | --- | --- |
| 0 — Foundation (current) | Inventory, rules, issue list and runtime decision | Review this document and confirm the runtime; no runtime pass claimed. |
| 1 — Package discovery and installation | Repair harness manifests/resource installation | Selected packages build and their executables, launch files and fixtures resolve from the overlay. |
| 2 — Mock feeders alone | Validate one feeder at a time before consuming its data | Expected message types, QoS, rates, timestamps, frames, deterministic truth and dropout behavior. |
| 3 — One real module with mocked dependencies | Dedicated agreed launch and RViz profile for that module | Expected real outputs, failure/recovery behavior, no duplicate owners, visual checks recorded separately. |
| 4 — Replace mocks with real dependencies | Integrate only modules that passed their isolated stage | Repeat boundary checks and compare against agreed tolerances; document remaining gaps. |

Run Stage 3 separately for each module, rather than treating a combined stack as
proof that each module works. Each implementation step must supply its own
commands and observations before proceeding.

## Commands to review Stage 0

From the host checkout:

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
git show --stat HEAD
git diff HEAD^ HEAD -- testing/README.md
```

Expected: a documentation-only foundation change. No mock or production process
starts. Read the issue list and launch contract before choosing the first runtime
profile.

After entering the agreed ROS environment, this read-only command can inspect
testing package discovery without launching anything:

```bash
colcon list --base-paths testing
```

Expected package names: `perception_test_tools`, `slam_test_tools`, and
`global_path_benchmarking`. Discovery alone does not establish successful builds
or dependency completeness. Build, launch and RViz validation commands will be
provided with Stage 1 after the environment decision.
