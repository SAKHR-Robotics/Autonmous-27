# PathPlanner validation — 20 November 2026

## Result

Production/testing separation, package builds, modular launch composition,
interface repairs and synthetic closed-loop operation are implemented.
**Full navigation correctness and safe stopping do not pass.** Do not interpret
active nodes, a successful action or old benchmark scores as release approval.

Work was performed on `TestingValidation` using the existing `autonomous27:jazzy`
image, ID `sha256:d952a9381d1f0b15c8212df0a78e023f3058235c659c44267bb782500df235ad`.
Host ROS was not used. Source was mounted read-only in the runtime container.
The initial run's builds, logs, maps, JSON traces, screenshots and databases were
stored at `/tmp/pathplanner-validation-20261120/`. That temporary runtime was
cleared between chat turns. The initial evidence below was inspected during the
run and is recorded here, but those original files are no longer available.
The final build/test/action recheck is retained outside source packages at
`/home/saif/.codex/artifacts/pathplanner-validation-20261120/`.
Only this run's processes/containers were controlled. No changes were pushed.

## Changes and findings

| ID | Location, evidence and cause | Result |
| --- | --- | --- |
| PP01 | Production package contained dummy/scenario maps and an artificial map-anchor test launch. | Moved into `testing/mocks/pathplanning/pathplanner_mock_tools`; production has no testing dependency or compatibility wrapper. Existing duplicate scenario maps were byte-compared before removing production copies. |
| PP02 | `live_test.launch.py` duplicated the production bridge, ignored target selection and passed a map while leaving map-server startup disabled. | Compatibility launch delegates to mock-owned composition, which includes the real production master once, explicitly selecting its saved-map mode. Target arguments are used. |
| PP03 | Mock Perception supplied legacy features but not the bridge's `Detection3DArray` input. | Added opt-in `publish_detections`; supported mocked profile enables it and disables direct bridge-cloud publishing. Shared defaults remain unchanged. |
| PP04 | Recovery behavior commands bypassed the smoother; controller odometry was implicit. | Controller and behaviors feed `/cmd_vel_nav`; smoother alone publishes `/cmd_vel`. Controller/navigator/smoother explicitly use filtered odometry. |
| PP05 | Independent lifecycle managers introduced an observed navigator-before-Smac startup race during this implementation. | Master now uses one ordered manager; independent component launches retain their own optional managers. Final mocked navigation succeeds. |
| PP06 | Rover's `/cmd_vel` Gazebo bridge was bidirectional, creating a second ROS command publisher. | Both Rover launch entry points now bridge commands ROS→Gazebo only. Final real integration observes only the smoother publishing `/cmd_vel`. |
| PP07 | MPPI rectangle footprint checking was implicit/off by default; mock visual geometry differed from costmaps. | Enabled `CostCritic.consider_footprint`; mocked profile selects the existing 0.9 × 0.7 m footprint geometry. Production footprint, unknown-space policy, speeds and tuning were not shrunk/relaxed to make tests pass. |
| PP08 | Legacy scenario runner published artificial input onto Nav2's `/global_costmap/costmap`, and cleanup killed processes by name. | Truth moved to `/testing/pathplanner/scenario_truth`; `/map` input is transient-local. Broad cleanup and the destructive old suite wrapper were removed. Supported probes use actual action handles/results. Legacy combined scoring remains unvalidated. |
| PP09 | README claimed GPU acceleration/collision monitor and an automatically opened RViz; view used `/local_plan`. | Current guides describe actual components/inputs; production RViz uses `/transformed_global_plan`, adds robot/footprint and optional MPPI markers. Test view shows mock markers separately. |
| PP10 | Smac returned successful paths for blocked-wall and 0.6 m passage fixtures; independent 0.9 × 0.7 m swept-footprint check detects out-of-bounds geometry. Centers approach Y=5.9869 m in a map ending at Y=6 m. | **FAIL, unresolved.** Reproduced with unknown traversal both true and false; changing that flag is not a fix. Do not execute those paths. Root cause needs investigation in installed Smac/collision checking/smoothing; no unsupported upstream-cause claim is made. |
| PP11 | Pausing localization and perception publishers for 2 s, then requesting navigation, still produced recovery rotation up to 1 rad/s; probe timed out and canceled after 5 s. | **FAIL, unresolved.** Cached TF/observations plus the absence of an explicit input-freshness stopping policy permit commands. A stopped output publisher alone is not a fail-closed safety mechanism. Stop/cancel versus stop/wait behavior and age limits require a decision before adding enforcement. |
| PP12 | Fresh-database real Gazebo run: native entity XY remained within centimeters of its origin, but EKF ended near (-1.39, 86.96) m and RTAB-Map's grid grew to 79 × 1772 cells. | **Integrated motion validation blocked.** No navigation motion was commanded. This is a stationary diagnostic mismatch, not a timestamp-aligned accuracy score. Preserve the existing SLAM fusion decision; do not hide the error with synthetic localization or different unapproved settings. |

## Executed evidence

The named files in this table refer to the initial, now-cleared temporary run.
The final retained recheck is listed separately below.

| Check | Evidence/result |
| --- | --- |
| Production-only build | `production-build.log`: `erc_path_planner` builds alone with no testing packages. |
| Combined dependency build | `final-build.log`: 11 packages build, including planner, testing packages, Rover, worlds, production SLAM and Perception. |
| Evaluator, observer, independent geometry and installed real bridge tests | `final-tests.log` / `final-tests.xml`: **47 passed, 2 skipped**; positive PDF tests require missing ReportLab. |
| Bridge geometry | Six runtime cases: yaw rotation, tilted vertical projection, empty/NaN/zero-quaternion observations and point-count cap. Headers and request identity are checked. An initial test harness reused identical timestamps and consumed queued prior replies; unique request stamps corrected that harness defect before the final passing run. |
| Independent components | Planner, controller and navigation sublaunches each reached active state with real Nav2 and external synthetic dependencies in isolated domain 98. Lifecycle state queries waited for configuration to finish; querying a busy single-threaded lifecycle node with a shorter timeout caused an initial harness timeout. |
| Lifecycle/clock/interfaces | Real nodes activate; nested costmaps use wall time in mock mode. Filtered odometry selected explicitly. One bridge and one velocity publisher. Component/standalone argument expansion checked. Final action success also establishes active planner/controller/navigator. |
| Mock navigation | `final-navigation.json`: terminal `SUCCEEDED`, 4.65 s request-to-result, estimated map-TF goal error 0.301 m, within existing configured 0.35 m goal tolerance. The mock and its estimate share one model; no physical/localization accuracy claim. |
| Negative planning | Out-of-map goal aborts (204); goal inside mocked obstacle aborts (207). These are observed installed error codes. |
| Cancellation | Action terminal status 5 (canceled) after an explicit cancellation at 0.5 s. Probe records cancellation separately from deadline timeout. |
| Reverse motion | A goal behind the rover succeeded in 17.36 s; 136 negative-velocity commands, minimum -0.210 m/s. |
| Dynamic injection | Distinct action goal IDs before/after adding an obstacle; both produce paths. This establishes request correlation/response, not full dynamic collision avoidance. |
| Independent map gates | `final-map-probe/map-scenarios.json`: blocked-wall and too-narrow FAIL; wide-passage PASS with a conservative swept rectangle check. No motion is executed by this probe. |
| Stale dependencies | `stale-dependencies.json`: timeout/canceled; 99 commands recorded, peak angular 1 rad/s despite missing fresh input. This is a failed stopping test. |
| Real integration | `gazebo-integration.json`: real clock, IMU, odometry, camera/depth cloud, EKF, map, terrain boxes and production bridge all observed. Single owners: RTAB-Map map, EKF filtered odometry, terrain boxes, bridge cloud, smoother velocity. Final plan aborts (203) while localization drifts; no completed Gazebo mission claimed. |
| RViz | `rviz-production.png` inspected with real integration data and `Global Status: Ok`; it visibly exposes the distorted map. `rviz-mock.png` inspected with mock data. Initial cross-container capture attempts lacked data/one crashed; final captures run in the same authorized GUI-enabled runtime and exit cleanly. Rendering does not establish map correctness. |

The map probe uses independently generated occupancy geometry, not Nav2 output
costmaps. It checks segments between path vertices and a conservative rectangular
sweep; unknown/outside cells are blocked. Synthetic box-cloud observations are
marking-only. Empty clouds do not prove Nav2 clearing. No quality thresholds were
lowered to obtain a pass.

## Final retained recheck

After the temporary runtime was cleared, the same image was used to rebuild the
four packages needed for mocked validation. Retained `build.log` reports four
successful builds; `tests.log` / `tests.xml` report **47 passed, 2 PDF skips**.
`interfaces.json` records all six real lifecycle nodes active, with one map,
bridge-cloud and velocity publisher. `navigation.json` records completed real
Nav2 navigation; `out-of-map.json` records rejection; `cancellation.json` records
actual cancellation. `map-probe/map-scenarios.json` reproduces the two failed
boundary gates and passing wide passage. `exit-codes.json` preserves the expected
nonzero failure/cancellation exits. Gazebo and RViz were not repeated after the
runtime reset; their earlier observed results remain historical evidence above.

## Reproduce the supported mocked stage

Prepare/build the disposable container using the
[production guide](info.md#8-see-real-pathplanner-in-action-with-mocked-dependencies) and
[mock build instructions](../testing/mocks/pathplanning/pathplanner_mock_tools/README.md#build-inside-the-existing-jazzy-image).
Run only one dependency profile in each ROS domain/container.

```bash
ros2 launch pathplanner_mock_tools mocked_planner.launch.py
```

In another sourced terminal in that container:

```bash
ros2 run global_path_benchmarking navigation_probe \
  --mode navigate --goal 2 -1 0 --timeout 45 --output /validation/navigation.json
PATHPLANNER_RUNTIME_TESTS=1 python3 -m pytest \
  /workspace/testing/benchmarking/pathplanning/test -q -rs -p no:cacheprovider
ros2 run global_path_benchmarking planner_map_probe --output-dir /validation/map-probe
```

Expected: navigation reports its actual terminal action; 47 tests pass with two
PDF skips in the recorded image; the map probe **returns 1** because the two
negative geometry gates fail. The map probe replaces the map server's map.
Restart the mocked launch before another profile; do not move the rover along
its invalid returned paths. Probe JSON remains raw evidence, not a mission score.

## Gazebo and real-module integration

Build additional real packages inside the same Jazzy image, with artifacts
outside source:

```bash
colcon --log-base /validation/log build --symlink-install \
  --base-paths /workspace/PathPlanning/erc_path_planner \
    /workspace/Rover/my_robot_description /workspace/worlds /workspace/SLAM/rover_slam \
    /workspace/Perception/terrain_geometry_msgs /workspace/Perception/marker_detection_msgs \
    /workspace/Perception/terrain_geometry /workspace/Perception/marker_detection \
    /workspace/Perception/perception_bringup \
  --build-base /validation/build --install-base /validation/install
source /validation/install/setup.bash
```

Use a GUI-enabled container for the interactive world and RViz. Every terminal
must source Jazzy and the same overlay and share the same ROS domain.
Stop mocked launches first. Run each command in its own terminal:

```bash
ros2 launch my_robot_description gazebo.launch.py \
  'world:=final_world_R&A.world' \
  bridge_sim_tf:=false publish_map_tf:=false publish_camera_tf:=false
```

```bash
ros2 launch rover_slam slam_bringup.launch.py use_sim_time:=true \
  launch_costmap:=false database_path:=/validation/pathplanner-fresh-run.db
```

Choose a fresh database filename for each independent run; do not silently reuse
an old mapping experiment.

```bash
ros2 launch perception_bringup perception.launch.py \
  use_sim_time:=true launch_marker_branch:=false
```

```bash
ros2 launch erc_path_planner path_planning.launch.py \
  use_sim_time:=true launch_rviz:=true
```

No sensor/SLAM/Perception mocks or identity map anchors belong in this profile.
The tested headless equivalent starts
`gz sim -s --headless-rendering -r '/workspace/worlds/worlds/final_world_R&A.world'`
then `spawn_rover.launch.py` with the same three TF switches, instead of the
combined Gazebo launch. Do not run both spawn paths.

Expected: one clock, fresh sensor streams, one complete TF chain, active Nav2
nodes and a stable localization estimate. Stop the validation if localization
moves while simulator truth does not, TF is ambiguous, or freshness is absent.
Those conditions were not all satisfied in this checkpoint. Do not continue to
claim full navigation accuracy from this run.

For independent planning with external Gazebo odometry, the test-owned
`test_planner_standalone.launch.py` exposes `odom_topic:=/odom` and an optional
`publish_map_anchor`. It supplies synthetic global context, so it cannot validate
real SLAM. If another publisher already owns `map → odom`, disable that anchor.

## Remaining work and decisions

Full Gazebo controller completion is blocked by the localization diagnostic and
failed boundary/stopping gates. Resolve these before advancing motion validation.
Required safety decision: cancel on stale inputs or stop/wait for recovery, with
explicit maximum ages and a validated enforcement point for every command source.
The Smac map-boundary failure needs an upstream investigation or a justified
production validity guard; it must not be hidden by padded test maps or relaxed gates.

Real camera clearing/free-space recovery, autonomous clock-reset recovery,
missing-TF recovery, full-duration dynamic avoidance, navigation through poses,
terrain/landmark accuracy, hardware stopping and ERC mission criteria remain
untested or blocked here. Existing SLAM reset/accuracy and Perception ground/marker
limitations still apply. Legacy benchmark scores and historical reports are not
accepted release criteria. No hardware safety or ERC compliance claim is made.
