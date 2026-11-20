# PathPlanner — module information and operating guide

**Read this file first.** It is the module's consolidated handoff: specification,
interfaces, publishers/subscribers, package ownership, launch arguments, required
mocks, testing commands and outstanding work. Last reconciled with implementation
and validation commit **`5cde253`**, 20 November 2026, on `TestingValidation`.

Use this file for orientation and routine operation without rediscovering the
codebase. Before changing behavior, inspect the affected implementation and update
this file in the same change. The [validation record](VALIDATION.md) preserves
experiment details; older `docs/` and `Prop&Solu.md` are historical references.

## 1. Current status and next work

| State | What is established |
| --- | --- |
| Implemented | One production package; real-only master/component launches; test-owned maps, mocks and synthetic TF; corrected bridge input and velocity routing; production and mock RViz views |
| Passed | Production-only build; real Nav2 activation; isolated bridge geometry; mocked closed-loop goal completion, cancellation and reverse motion; map/bridge/velocity publisher ownership |
| Last test checkpoint | **47 passed, 2 skipped** in `autonomous27:jazzy`; skipped positive PDF-export tests require ReportLab |
| Failed: PathPlanner boundary gate | Real Smac returns paths for blocked-wall/too-narrow scenarios whose rectangular footprints leave map bounds. Independent geometry rejects them. Changing unknown traversal alone did not fix it. |
| Failed: stale-input stopping | Paused localization/obstacle publishers still allowed recovery rotation up to 1 rad/s. A timed-out probe canceled the request; the stack did not independently enforce the required stopping policy. |
| Blocked: full Gazebo navigation | In a fresh-database stationary diagnostic, native rover XY stayed near the origin while EKF drifted about 87 m. Real-module interfaces worked; integrated motion accuracy did not pass. |
| Missing implementation/decision | Explicit freshness enforcement covering every motion-command source; choose stop-and-cancel versus stop-and-wait, maximum input ages and recovery conditions |
| Pending investigation | Map-boundary collision failure; existing SLAM fusion/localization problem; select a justified fix and reproduce the failing scenarios before claiming success |
| Pending validation | Real clearing/removal of obstacles, clock-reset/missing-TF recovery, full dynamic avoidance, navigation through poses, hardware stopping and agreed mission criteria |
| External limits | Perception ground classification and positive marker/landmark validation remain open. Existing SLAM reset/accuracy limits still apply. |

A successful ROS action is **not** proof of collision-free geometry, localization
accuracy or safe stopping. The numerical test suite passes while the two scenario
safety gates above fail. Do not relax thresholds or substitute fake SLAM to hide
those failures. Legacy report scores are not accepted mission requirements.

## 2. Scope, packages and files

The module converts external maps/localization/obstacle observations and navigation
goals into body velocity commands. It does not implement a physical motor driver,
SLAM, camera driver or terrain classifier. MPPI is CPU-based. No collision monitor
is currently launched.

| Package/location | Responsibility |
| --- | --- |
| `PathPlanning/erc_path_planner` / ROS package `erc_path_planner` | Production `ament_cmake` package: own C++ obstacle bridge; Nav2 launch/configuration, behavior trees and production RViz |
| `testing/mocks/pathplanning/pathplanner_mock_tools` / `pathplanner_mock_tools` | Python dependency mocks, synthetic maps, artificial TF, standalone test launches and mock RViz |
| `testing/benchmarking/pathplanning` / `global_path_benchmarking` | Action probes, independent geometry checks, observer/export tests and legacy diagnostic reports |
| `Rover/my_robot_description` | Real Gazebo rover description, sensor/command bridges and simulation launches |
| `SLAM/rover_slam` | Real EKF/RTAB-Map inputs and localization TF |
| `Perception/perception_bringup` | Real Perception orchestration; terrain observations feed the production bridge |

There is **no new singular** `testing/mock/pathplanner` directory. Reuse the
existing plural `testing/mocks/pathplanning/` hierarchy. Production neither
imports nor depends on testing packages; test launches include real production.

```text
PathPlanning/
├── info.md                         # This contract/runbook
├── README.md                       # Entry point
├── VALIDATION.md                   # Detailed evidence and failures
└── erc_path_planner/
    ├── package.xml, CMakeLists.txt
    ├── src/costmap_bridge_node.cpp
    ├── config/nav2_params.yaml
    ├── launch/                     # Real components only
    ├── behavior_trees/              # Real Nav2 action orchestration
    └── rviz/nav2_default_view.rviz
```

Runtime: existing **`autonomous27:jazzy` Docker image**, ROS 2 Jazzy and its installed
Nav2/ROS/Python dependencies. Do not source host Humble for this module. C++ builds
use `rclcpp`, `sensor_msgs`, `vision_msgs`; runtime launch dependencies include
`launch`, `launch_ros`, `ament_index_python`, `nav2_common`, Nav2 servers/plugins,
lifecycle manager and `rviz2`. Tests/mocks additionally use their declared Python,
message and ROS dependencies; the build commands below include generated messages.

## 3. Specification and processing

```text
External Perception boxes → REAL bridge → Nav2 global/local obstacle marking
External camera cloud ─────────────────→ Nav2 sensor-origin clearing
External /map + localization TF → REAL Smac path → REAL MPPI control
Mission goal → REAL BT navigation/recovery ─────→ /cmd_vel_nav
                                      REAL velocity smoother → /cmd_vel → rover
```

| Component | Current configuration/behavior |
| --- | --- |
| Obstacle bridge | `Detection3DArray` → filled XY box-footprint `PointCloud2`, retaining input frame/stamp; yaw rotation and conservative tilted-box projection |
| Bridge limits | `sample_spacing=0.05` m, `max_points=200000`; requires positive finite spacing and positive cap; rejects nonfinite/invalid geometry, zero quaternion and nonpositive dimensions; X/Y box sizes above 100 m are skipped |
| Bridge freshness/clearing | Does not transform frames or impose an input-age deadline. Empty input produces empty output, which does **not** erase previously marked Nav2 cells. Reaching the sample cap can omit remaining samples/boxes. |
| Global planner | `GridBased`: `nav2_smac_planner::SmacPlannerHybrid`, Reeds–Shepp, minimum turning radius 0.8 m, 72 angular bins, tolerance 0.5 m, max planning time 5 s; unknown traversal enabled |
| Global costmap | `map` frame, 0.05 m cells, static + obstacle + inflation layers; update/publish 1 Hz; tracks unknown space |
| Local costmap | `odom` frame, rolling 10 × 10 m, 0.05 m cells; obstacle + inflation layers; update 5 Hz, publish 2 Hz |
| Footprint/inflation | Rectangle `[[0.45,0.35],[0.45,-0.35],[-0.45,-0.35],[-0.45,0.35]]` m; inflation radius 0.85 m, cost scaling 3.0 |
| Obstacle observations | Box cloud marking-only; actual depth cloud clearing-only. Production layers accept configured obstacle heights -0.5 to 2.5 m. Correct acquisition stamps and sensor TF are required. |
| Controller | `FollowPath`: `nav2_mppi_controller::MPPIController`, DiffDrive, 20 Hz, 2000 rollouts × 56 steps at 0.05 s (2.8 s horizon); footprint-aware CostCritic enabled |
| Speed envelope | Linear -0.5 to 1.0 m/s, angular ±1.0 rad/s; configured limits are not measured hardware limits |
| Goal/progress | Goal tolerance 0.35 m and 0.4 rad; progress requires 0.15 m movement within 15 s |
| Smoother | 20 Hz, `OPEN_LOOP`, acceleration limits `[2.5,0,3.2]`, deceleration `[-2.5,0,-3.2]`, command timeout 1 s; odometry topic configured but odometry feedback is used only when closed-loop feedback is selected |
| Behavior tree | Replan at 1 Hz; planner/controller contextual clear-costmap recovery; outer recovery allows 6 retries and cycles clear, spin 1.57 rad, wait 5 s, backup 0.30 m at 0.15 m/s |
| Lifecycle | Master activates selected nodes in order: optional map server, planner, controller, behavior server, smoother, navigator. Bond timeout 4 s. Standalone component launches have their own optional managers. |

Rates are configuration targets, not guaranteed measured throughput. The planner
frequency setting is 20 Hz, while these behavior trees request periodic replanning
at 1 Hz. Do not confuse those values. Current footprint is conservative relative
to the Gazebo model; do not shrink it just to pass narrow-passage tests.

## 4. Publishers, subscribers and actions

### Production contract topics

Names below are root-namespace defaults. Direction is explicit for each edge.
`/tf`, parameters and optional debug endpoints also depend on the installed Nav2
version; this table defines the supported module contract, not every plugin's
private ROS implementation detail.

| Topic | Message type | Publisher | Subscribers/consumers |
| --- | --- | --- | --- |
| `/perception/obstacles_only` | `vision_msgs/msg/Detection3DArray` | Real terrain node **or** obstacle memory, one selected owner; mock perception in isolated profile | `costmap_bridge_node` |
| `/bridge/pointcloud` | `sensor_msgs/msg/PointCloud2` | Production `costmap_bridge_node`, exactly one | Global/local obstacle layers; RViz |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | Actual camera/Gazebo sensor bridge | Global/local clearing sources; Perception externally |
| `/map` | `nav_msgs/msg/OccupancyGrid` | RTAB-Map **or** selected map server, never both | Global static layer; RViz |
| `/map_updates` | `map_msgs/msg/OccupancyGridUpdate` | Optional selected map producer; may be absent | Global static layer when updates are supplied; RViz |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | Real EKF, or mock rover in isolated profile | Controller and navigator; smoother if closed-loop feedback is selected; probe |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz `2D Goal Pose` or external goal client | BT navigator's goal-topic interface |
| `/plan` | `nav_msgs/msg/Path` | Real planner server | RViz/observers; navigation/controller exchange the requested path through action data |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Controller or active recovery behavior | Velocity smoother |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity smoother, sole intended command publisher | Real motor interface/Gazebo ROS→Gazebo bridge, or mock rover; probe |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Global costmap | RViz/diagnostic observers; never a mock input topic |
| `/local_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Local costmap | RViz/diagnostic observers |
| `/global_costmap/costmap_raw`, `/local_costmap/costmap_raw` | `nav2_msgs/msg/Costmap` | Corresponding real costmap | Behavior server collision checking |
| `/global_costmap/published_footprint`, `/local_costmap/published_footprint` | `geometry_msgs/msg/PolygonStamped` | Corresponding real costmap | Behavior server; RViz footprint display |
| `/global_costmap/costmap_updates`, `/local_costmap/costmap_updates` | `map_msgs/msg/OccupancyGridUpdate` | Corresponding costmap when incremental updates are emitted | RViz; full maps are also enabled in this configuration |
| `/transformed_global_plan` | `nav_msgs/msg/Path` | MPPI, subject to debug publication settings | RViz MPPI path display |
| `/trajectories` | `visualization_msgs/msg/MarkerArray` | MPPI when visualization is enabled | Optional RViz debug display |
| `/robot_description` | `std_msgs/msg/String` | External robot-state publisher | RViz production robot model; mock view uses markers instead |
| `/clock` | `rosgraph_msgs/msg/Clock` | One Gazebo/bag clock when using simulation time | Every node with `use_sim_time=true`; absent in the wall-time mock profile |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | External localization/mount owners described below | Nav2, bridge-cloud consumers, RViz and probe |

Bridge input/output are reliable, volatile, depth 10. Map consumption requests
transient-local durability. Camera clouds use sensor-oriented QoS; consumers must
match the actual publisher. RViz profiles include explicit QoS choices. Inspect
live endpoints with `ros2 topic info TOPIC --verbose` if a producer or image changes.
`geometry_msgs/Twist` is selected consistently; enabling stamped velocity commands
requires coordinated changes to every producer/consumer.

### Action/service interfaces

| Interface | Server | Caller and meaning |
| --- | --- | --- |
| `/compute_path_to_pose` — `nav2_msgs/action/ComputePathToPose` | Planner server | BT navigator or `navigation_probe --mode plan`; path response only |
| `/compute_path_through_poses` — `nav2_msgs/action/ComputePathThroughPoses` | Planner server | Multi-pose navigation BT |
| `/follow_path` — `nav2_msgs/action/FollowPath` | Controller server | BT navigator; executes requested path |
| `/navigate_to_pose` — `nav2_msgs/action/NavigateToPose` | BT navigator | Probe/mission/RViz goal interface; wait for terminal result |
| `/navigate_through_poses` — `nav2_msgs/action/NavigateThroughPoses` | BT navigator | Mission client; current full-duration acceptance remains pending |
| `/spin`, `/backup`, `/drive_on_heading`, `/wait` — corresponding `nav2_msgs/action/Spin`, `BackUp`, `DriveOnHeading`, `Wait` types | Behavior server | Recovery/behavior clients |
| `/global_costmap/clear_entirely_global_costmap`, `/local_costmap/clear_entirely_local_costmap` — `nav2_msgs/srv/ClearEntireCostmap` | Real costmaps | Behavior trees; map probe clears global costmap after changing its map |
| `/map_server/load_map` — `nav2_msgs/srv/LoadMap` | Map server, only when selected | Disposable map probe; changes the input map |
| `/<lifecycle-node>/get_state`, `change_state` | Each lifecycle node | Lifecycle manager/diagnostic clients |

ROS actions also create their generated goal/result/cancel services and feedback/
status topics. Nodes expose standard parameter services, `/parameter_events`,
`/rosout`, lifecycle transition events and lifecycle bonds as applicable. Optional
Nav2 outputs include unsmoothed/optimal paths, per-layer costmap debugging and raw
updates; they are not required navigation inputs. Use `ros2 node info /NODE` for
an installed-version endpoint inventory rather than treating a debug topic as a
stable integration contract.

### Test-only interfaces

| Topic/interface | Publisher → subscriber | Type/purpose |
| --- | --- | --- |
| `/odometry/filtered`, `/tf` | Mock rover → real Nav2 | Synthetic localization, 50 Hz physics update; not independent localization truth |
| `/rover_marker` | Mock rover → test RViz | `MarkerArray`, visible simulated rover |
| `/rover/joint_states` | Mock rover → optional observers | `JointState`, test visualization/telemetry |
| `/rover/motor_rpm` | Mock rover → optional observers | `Float32MultiArray`, simulated RPM output |
| `/cmd_vel/stamped` | Optional external client → mock rover | `TwistStamped`, legacy auxiliary input; not used by default PathPlanner |
| `/motor_driver/rpm_feedback` | External test driver → mock rover | `Float32MultiArray`, only drives physics in `motor_rpm` mode |
| `/initialpose`, `/start_pose` | Test client → mock rover | `PoseWithCovarianceStamped` / `PoseStamped`, resets synthetic pose; does not initialize real SLAM |
| `/perception/obstacles_only` | Mock perception → real bridge | `Detection3DArray`, opted in by supported mocked launch |
| `/perception/obstacle_markers` | Mock perception → test RViz | `MarkerArray`, obstacle cylinders |
| `/mock_perception/spawn_obstacle`, `/clicked_point` | Test client/RViz → mock perception | `Point` / `PointStamped`, injects a synthetic obstacle |
| `/terrain/obstacle_features`, `/mock_perception/pointcloud`, `/scan` | Mock perception → optional legacy tools | `ObstacleFeatureArray`, `PointCloud2`, `LaserScan`; default real PathPlanner does not consume these outputs |
| `/bridge/pointcloud` direct mock output | Mock perception, only with `publish_bridge_cloud=true` | Disabled in supported profile to preserve the real bridge as sole owner |
| `/testing/pathplanner/scenario_truth` | Legacy test runner → observers | `OccupancyGrid`; separate from real Nav2 output costmaps |

`navigation_probe` subscribes to selected odometry, `/cmd_vel` and TF, and sends
real actions. It records one goal ID/terminal result with monotonic end-to-end
latency, estimated goal error and raw command/odometry measurements. It does not
publish synthetic Nav2 results or a mission score. `planner_map_probe` changes
the selected map-server input and checks real paths against independently created
maps, treating unknown/outside cells as blocked. It never executes motion.

## 5. Clock, TF and dependency ownership

| Profile | Clock | `map → odom` | `odom → base_link` | Sensors/obstacles |
| --- | --- | --- | --- | --- |
| Real production/Gazebo | Real clock or `/clock`, selected consistently | RTAB-Map/localizer | EKF/localizer | Real camera, Perception and mounts |
| Isolated mocked planner | Wall time, `use_sim_time=false` | Mock rover identity transform | Mock rover motion | Synthetic boxes; no camera clearing source |
| Standalone external-rover test | Usually Gazebo `/clock` | Optional test-owned static anchor | External rover/EKF; select correct odometry topic | External sensors/Perception; synthetic map only |

The mock rover also owns `base_link → base_footprint`; it supplies no camera
mounts. Its marker-based view does not require a robot-description publisher.

Production expects a continuous transform chain and correctly stamped metric
observations. Real Rover robot description owns mount TF. With real SLAM set
`bridge_sim_tf=false`, `publish_map_tf=false`, `publish_camera_tf=false` on the
Rover launch, and leave SLAM's optional costmap disabled. Never run two owners
of one TF edge or both a mock rover and real EKF in the same profile.

## 6. Production and testing launch reference

| Launch command suffix | What it starts / requirements |
| --- | --- |
| `erc_path_planner path_planning.launch.py` | Real master; external map/TF/sensors by default; optional saved-real-map server and RViz |
| `erc_path_planner planner.launch.py` | Real global planner/global costmap + its lifecycle manager; needs external map, TF and observations |
| `erc_path_planner controller.launch.py` | Real controller/local costmap + manager; needs external localization/observations and FollowPath requests; publishes intermediate commands, needs smoother for rover motion |
| `erc_path_planner navigation.launch.py` | Real behavior server, smoother, navigator + manager; needs running real planner/controller and localization |
| `erc_path_planner obstacle_bridge.launch.py` | Real bridge only, no mocked input |
| `erc_path_planner rviz.launch.py` | Production view only |
| `pathplanner_mock_tools mocked_planner.launch.py` | Real master + synthetic map/localization/motion/Perception, optional test RViz; self-contained isolated wall-time profile |
| `pathplanner_mock_tools test_planner_standalone.launch.py` | Real master + synthetic map/optional map anchor; **not self-contained**, requires external rover/localization/sensors |
| `global_path_benchmarking live_test.launch.py` | Compatibility include of supported mocked launch |

Master arguments and defaults:

| Argument | Default | Meaning |
| --- | --- | --- |
| `use_slam` | `true` | Consume external `/map`; this flag does **not** launch SLAM |
| `map` | empty | Required absolute map YAML path with `use_slam=false`; no production dummy fallback |
| `use_sim_time` | `false` | Select shared clock; rewrites nested costmap parameters too |
| `autostart` | `true` | Activate selected lifecycle nodes |
| `params_file` | installed `config/nav2_params.yaml` | Nav2 configuration |
| `odom_topic` | `/odometry/filtered` | Shared configured odometry topic |
| `obstacles_topic` | `/perception/obstacles_only` | Bridge detection input |
| `launch_planner`, `launch_controller`, `launch_navigation`, `launch_bridge` | `true` | Select real components; external equivalents are required when needed components are disabled |
| `launch_rviz` | `false` | Start production view |

Planner/controller/navigation sublaunches expose `use_sim_time`, `autostart`,
`params_file`, `odom_topic`, `manage_lifecycle` (default true). The master sets
`manage_lifecycle=false` and uses one manager. Do not start duplicate components.
Bridge launch exposes `use_sim_time`, `input_topic`, `output_topic`; sampling
parameters are node parameters, not additional launch arguments. RViz exposes
`use_sim_time`. Saved real maps can use the production map-server option; no
synthetic map or artificial localization is installed by the production package.

Mock launch defaults: `use_rviz=false`, `use_mock_rover=true`,
`use_mock_perception=true`, `use_costmap_bridge=true`, `use_external_module=false`,
`target_package=erc_path_planner`, `target_launch=path_planning.launch.py`,
`map=<mock package>/custom_maps/dummy_map.yaml`, `cmd_vel_topic=/cmd_vel`,
`control_mode=cmd_vel`, `initial_x=0.0`, `initial_y=0.0`, `initial_yaw=0.0`,
`scenario_id=''`, `scenarios_file=''`.
It forces wall time, zero noise/slip, 0.9 × 0.7 m mock visual geometry,
`publish_detections=true` and `publish_bridge_cloud=false`. It removes the absent
camera-clearing source from that profile's costmap observations. It does not change
shared mock defaults or production configuration files. An external target must
accept the forwarded production arguments; topic overrides must match its remaps.

Standalone external-rover defaults: `use_sim_time=true`, `odom_topic=/odom`,
`publish_map_anchor=true`, `launch_rviz=false`, production `params_file`,
mock `custom_maps/dummy_map.yaml`. Disable its anchor if another map→odom owner exists.

## 7. Do we have enough mocks to test PathPlanner on its own?

**Yes, for the current isolated marking-only navigation fixture. No new mock is
needed for the demonstration below.** Build the existing packages and use the
supported mocked master, which starts the necessary dependencies automatically.

| Dependency replaced | Existing implementation | What remains real |
| --- | --- | --- |
| SLAM/global map | Synthetic `custom_maps/dummy_map.yaml` through real Nav2 map server | Global static layer, Smac planner |
| EKF/localization, rover/motor response | `mock_rover_sim`: odometry, TF, body-command integration, rover markers | MPPI, recovery behaviors, velocity smoother and navigation actions |
| Perception obstacle boxes | `mock_perception` opted into production `Detection3DArray` contract | C++ bridge and both costmap marking layers |
| Camera clearing | No mock in supported profile; explicitly disabled | Clearing needs separate real/suitable coherent sensor inputs and later validation |

The synthetic estimate and motion share one model. This supports isolated
functional checks; it does not validate physical SLAM/sensors, real actuator
response or stopping safety. Legacy `algo_node` is a toy planner for benchmarker
development; **never substitute it for real Smac in this validation**. Do not run
`mock_base` and `mock_rover_sim` together or start compatibility executable aliases
alongside their real implementations.

## 8. See real PathPlanner in action with mocked dependencies

### A. Host: create one GUI-enabled container

Run from your logged-in Linux graphical desktop. The read-only X socket/cookie
mounts allow RViz without changing X-server permissions. If DISPLAY or the cookie
is absent, use a graphical terminal or run headless with `use_rviz=false`.
Use a fresh container name if `pathplanner-demo` already exists; do not delete an
unrelated existing container. The following commands create one new run:

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
mkdir -p "$HOME/.codex/artifacts"
export PP_ARTIFACTS=$(mktemp -d "$HOME/.codex/artifacts/pathplanner-demo.XXXXXX")
export PP_XAUTH="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"
test -n "${DISPLAY:-}" && test -f "$PP_XAUTH"

docker run -d --name pathplanner-demo --network none --shm-size=1g \
  -e DISPLAY="$DISPLAY" -e XAUTHORITY=/tmp/pathplanner.xauthority \
  -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
  -e ROS_DOMAIN_ID=96 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e PYTHONDONTWRITEBYTECODE=1 -e ROS_LOG_DIR=/validation/ros-log \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  -v "$PP_XAUTH":/tmp/pathplanner.xauthority:ro \
  -v "$PWD":/workspace:ro -v "$PP_ARTIFACTS":/validation \
  -w /workspace autonomous27:jazzy sleep infinity

echo "Artifacts: $PP_ARTIFACTS"
docker exec -it pathplanner-demo bash
```

For headless use omit the DISPLAY/XAUTHORITY/X socket/cookie arguments and mounts;
the ROS build/launch steps are identical. Keep all terminals in this same container.

### B. Inside Docker: build once

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /validation/log build --symlink-install \
  --base-paths /workspace/PathPlanning/erc_path_planner \
    /workspace/Perception/terrain_geometry_msgs \
    /workspace/testing/mocks/pathplanning/pathplanner_mock_tools \
    /workspace/testing/benchmarking/pathplanning \
  --build-base /validation/build --install-base /validation/install
source /validation/install/setup.bash
```

Expected: all four packages finish. Stop on a build failure; do not source an old
unrelated overlay as a substitute. Generated output stays in `/validation`, not source.

### C. Terminal A, inside that container: launch

```bash
ros2 launch pathplanner_mock_tools mocked_planner.launch.py use_rviz:=true
```

Headless: use `use_rviz:=false`. This one command starts the real production
bridge/Nav2, map server and necessary dependency mocks. Do not start Gazebo,
SLAM, real Perception, a separate bridge or another map anchor for this profile.
Wait for the lifecycle manager to report that managed nodes are active. Initial
Smac configuration can take several seconds; busy lifecycle services may not
answer immediately.

### D. Terminal B: inspect and send a short goal

Open another **host terminal**, then enter the same container:

```bash
docker exec -it pathplanner-demo bash
source /opt/ros/jazzy/setup.bash
source /validation/install/setup.bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 topic info /cmd_vel --verbose
ros2 topic info /bridge/pointcloud --verbose
```

Expected: lifecycle state `active [3]`; smoother is the sole `/cmd_vel` publisher;
production bridge is the sole `/bridge/pointcloud` publisher. If either is duplicated,
stop the extra profile before moving on.

Request planning without motion, then navigation:

```bash
ros2 run global_path_benchmarking navigation_probe \
  --mode plan --goal 2 -1 0 --output /validation/plan.json
ros2 run global_path_benchmarking navigation_probe \
  --mode navigate --goal 2 -1 0 --timeout 45 --output /validation/navigation.json
```

In test RViz, expect a mock rover, obstacle markers, costmaps and real Smac path.
The rover should move toward (2, -1) and stop. The last validated fixture completed
in about 4.65 s with estimated final position error about 0.30 m, within the existing
0.35 m configured tolerance. Timing is not a fixed performance guarantee.
The probe must report `SUCCEEDED`/status 4; a visible path alone is not completion.
`/validation/navigation.json` records actual terminal status, goal ID and samples.
RViz's `2D Goal Pose` also works for exploration inside the synthetic map; use the
probe when you need a recorded result. Its `2D Pose Estimate` resets the mock only.

### E. Terminal B: cancellation and known negative checks

Cancel a new navigation request after half a second:

```bash
ros2 run global_path_benchmarking navigation_probe \
  --mode navigate --goal 5 -1 0 --cancel-after 0.5 \
  --output /validation/cancel.json
```

Expected: `CANCELED`, action status 5, exit code 1 because the action did not
succeed. This is an intentional cancellation result, not a failed build.

An out-of-map goal must fail planning without moving the rover:

```bash
ros2 run global_path_benchmarking navigation_probe \
  --mode plan --goal 100 100 0 --output /validation/out-of-map.json
```

Expected: `ABORTED`, nonzero error code/exit code 1. To inspect live box-to-cloud
response, add a synthetic obstacle from the same sourced terminal:

```bash
ros2 topic pub --once /mock_perception/spawn_obstacle geometry_msgs/msg/Point \
  '{x: 4.0, y: -1.0, z: 0.2}'
```

Expect an additional obstacle marker and updated bridge/costmap observations.
Publication/response alone does not establish safe avoidance. Restart the profile
to restore its default obstacle layout before repeatable trials. RViz's Publish
Point tool is an alternative only if you manually add it to the toolbar; it is
not included in the default view.

### F. Terminal B: numerical checks and unresolved boundary reproduction

```bash
PATHPLANNER_RUNTIME_TESTS=1 python3 -m pytest \
  /workspace/testing/benchmarking/pathplanning/test -q -rs -p no:cacheprovider
ros2 run global_path_benchmarking planner_map_probe --output-dir /validation/map-probe
```

Expected in the recorded image: **47 passed, 2 PDF skips**. The map probe changes
the selected map server's map and requests paths only. Expected current results:
blocked-wall FAIL, too-narrow FAIL, wide-passage PASS, overall exit code 1. Those
failures are genuine outstanding gates, not expected successful navigation.
Do not execute the invalid returned paths. Restart the mocked launch before
another motion trial or profile.

### G. Stop this run and preserve results

Wait for the probe's navigation/cancellation to finish and confirm the rover has
stopped. In Terminal A use Ctrl+C. Stop streaming diagnostic commands with Ctrl+C.
From the host:

```bash
docker stop --timeout 10 pathplanner-demo
```

Files remain in the printed host `PP_ARTIFACTS` directory. Do not use `killall`,
process-name `pkill`, or delete other builds/containers. Later you can
`docker start pathplanner-demo` and re-enter/source the same overlay.

## 9. Real production and Gazebo profile

For production alone, build only `/workspace/PathPlanning/erc_path_planner` with
the same external build/install directories; no testing packages are required.
Then run `ros2 launch erc_path_planner path_planning.launch.py` with the correct
shared clock and actual external map/localization/Perception. Production supplies
no missing dependencies automatically. For a saved real map, explicitly select
`use_slam:=false map:=/absolute/path/map.yaml`; localization is still required.

For Gazebo integration, first stop the mocked launch. In the same Jazzy image,
build the additional real packages (the GUI mounts above are already suitable):

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

Use a separate sourced container terminal for each of these four launches:

```bash
# Real Gazebo world, rover and sensor bridges; no artificial map/localization TF.
ros2 launch my_robot_description gazebo.launch.py \
  'world:=final_world_R&A.world' \
  bridge_sim_tf:=false publish_map_tf:=false publish_camera_tf:=false
```

```bash
# Real EKF and RTAB-Map; choose a fresh database filename for every independent run.
ros2 launch rover_slam slam_bringup.launch.py use_sim_time:=true \
  launch_costmap:=false database_path:=/validation/pathplanner-gazebo-new.db
```

```bash
# Real terrain Perception; markers are not required for this baseline.
ros2 launch perception_bringup perception.launch.py \
  use_sim_time:=true launch_marker_branch:=false
```

```bash
ros2 launch erc_path_planner path_planning.launch.py \
  use_sim_time:=true launch_rviz:=true
```

All terminals source `/opt/ros/jazzy/setup.bash` and
`/validation/install/setup.bash`. Do not also launch `spawn_rover` when the
combined Gazebo launch already spawns it. This full profile needs no dependency
mocks. Verify actual clock, camera, `/odom`, EKF, map, Perception and bridge
streams; compare stationary localization against simulator truth before moving.

**Current gate:** the last real integration showed severe EKF drift and a
consequently distorted map. Treat this procedure as a reproducible integration
check; full autonomous motion is blocked pending the issues in section 1.
A short successful planning response in one run does not override later drift.
Do not replace SLAM with an identity transform and call the result real-SLAM validation.

## 10. Agent handoff and evidence maintenance

The current task state is: packaging/interfaces/runbooks implemented; core mocked
operation passes; the boundary and stale-input gates fail; full Gazebo motion is
blocked by localization. No freshness policy choice has been recorded. Next work
is the specific boundary investigation, explicit stale-input policy/enforcement,
and coordination with the existing SLAM localization investigation.

Do not duplicate mocks, fork a second testing hierarchy, move artificial TF back
into production, change shared SLAM mock defaults, relax collision thresholds,
or turn legacy scores into acceptance requirements. Preserve other work on the
shared branch, stage explicit scoped files, and never push without authorization.

Detailed historical and final retained recheck evidence is in `VALIDATION.md`.
The initial `/tmp/pathplanner-validation-20261120/` runtime was cleared between
turns; its observed Gazebo/RViz results are historical, not available raw artifacts.
The later recheck was saved under
`/home/saif/.codex/artifacts/pathplanner-validation-20261120/` (availability depends
on workspace retention). This documentation update does not rerun those tests.
Future changes must update this file's contract, status and run commands together
so the next agent can start here instead of repeating the inventory.
