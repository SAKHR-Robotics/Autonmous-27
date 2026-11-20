# ERC PathPlanner — production only

`erc_path_planner` is an `ament_cmake` ROS 2 Jazzy package. Its own C++ node
converts Perception boxes to obstacle-marking points; upstream Nav2 packages
provide planning, control, costmaps and navigation. MPPI uses CPU computation.
This package does not include a collision monitor or a motor hardware driver.

## Launch ownership

| Launch | Starts | Required external inputs |
| --- | --- | --- |
| `path_planning.launch.py` | Master: selected components, one ordered lifecycle manager, optional map server/RViz | Map, localization TF/odometry, obstacle detections, clearing sensor cloud |
| `planner.launch.py` | Real Smac server/global costmap and its lifecycle manager | `/map`, `map → base_link`, obstacle/sensor inputs |
| `controller.launch.py` | Real MPPI server/local costmap and its lifecycle manager | `/odometry/filtered`, `odom → base_link`, sensor inputs; `FollowPath` requests |
| `navigation.launch.py` | Behavior server, velocity smoother, BT navigator and lifecycle manager | Running real planner/controller servers, localization, navigation goals |
| `obstacle_bridge.launch.py` | Production C++ box-to-cloud bridge | `Detection3DArray` observations |
| `rviz.launch.py` | Production navigation view | The selected clock and published visualization data |

Component launches default to `manage_lifecycle:=true`. The master passes
`manage_lifecycle:=false` and activates selected nodes in dependency order:
map server when selected, planner, controller, behaviors, smoother, navigator.
Do not start a component independently while the master already owns it.
Controller-only output is `/cmd_vel_nav`; actual rover motion also requires the
smoother in `navigation.launch.py` or another explicit command consumer.

The master defaults to `use_slam:=true`: it consumes an external `/map` and
publishes no localization TF. For an existing saved real map, use
`use_slam:=false map:=/absolute/path/map.yaml`. A missing map fails at launch;
there is no fallback dummy map. Localization and TF remain external in either mode.

Master arguments: `use_sim_time`, `autostart`, `params_file`, `odom_topic`,
`obstacles_topic`, `use_slam`, `map`, `launch_planner`, `launch_controller`,
`launch_navigation`, `launch_bridge`, `launch_rviz`. Component launches expose
clock, autostart, parameter file and odometry topic. Bridge input/output topics
are configurable through its dedicated launch. Topic overrides within costmap
YAML must agree with the bridge and actual sensor publishers.

## Interfaces

| Interface | Type | Owner/consumer |
| --- | --- | --- |
| `/perception/obstacles_only` | `vision_msgs/msg/Detection3DArray` | External Perception → bridge, reliable/volatile |
| `/bridge/pointcloud` | `sensor_msgs/msg/PointCloud2` | Bridge only → global/local marking layers |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | Actual sensor → clearing-only sources |
| `/map` | `nav_msgs/msg/OccupancyGrid` | RTAB-Map or explicitly selected map server → static layer, transient-local |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF → controller, navigator and smoother; configurable |
| `/plan` | `nav_msgs/msg/Path` | Real Smac output; publishing it does not prove goal arrival |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | Controller/recovery behaviors → smoother |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Smoother → external rover/motor consumer |
| `compute_path_to_pose` | `nav2_msgs/action/ComputePathToPose` | Planner-only request/result |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Navigation until success/abort/cancel/timeout |
| `navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | Ordered navigation; not numerically validated in this checkpoint |

Required TF: `map → odom → base_link → sensor frames`. RTAB-Map owns
`map → odom`, EKF owns `odom → base_link`, and robot description owns mounts.
Keep Gazebo's map/odometry TF bridges disabled with real SLAM. Do not launch
SLAM's optional costmap alongside PathPlanner's costmaps with the same names.
All nodes/costmaps must use the same clock; the component launches rewrite the
nested costmap clock parameters too. Wall-time mocks do not publish `/clock`.

Box samples provide marking only. An empty box array produces an empty cloud;
it does **not** erase old Nav2 obstacle cells. Clearing requires actual rays,
correct sensor extrinsics and a separately validated freshness/stopping policy.
Do not use the synthetic mock cloud's `map` origin as a clearing sensor.

The current costmap footprint is a conservative 0.9 × 0.7 m rectangle.
The mocked fixture explicitly uses that geometry. The Gazebo rover's physical
geometry differs; the footprint was not shrunk to make passages pass.
Hybrid A* uses Reeds–Shepp with 0.8 m turning radius; MPPI uses DiffDrive.
These constraints are compatible but can exclude some skid-steer maneuvers.
Unknown traversal remains enabled pending a deployment policy decision.

## Docker build and real launch

On the host, from the workspace:

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
export PATHPLANNER_ARTIFACTS=$(mktemp -d /tmp/pathplanner-run.XXXXXX)
docker run -d --name pathplanner-run --network none --shm-size=1g \
  -e ROS_DOMAIN_ID=96 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e PYTHONDONTWRITEBYTECODE=1 -e ROS_LOG_DIR=/validation/ros-log \
  -v "$PWD":/workspace:ro -v "$PATHPLANNER_ARTIFACTS":/validation \
  -w /workspace autonomous27:jazzy sleep infinity
docker exec -it pathplanner-run bash
```

Inside Docker, a production-only build needs no testing packages:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /validation/log build --symlink-install \
  --base-paths /workspace/PathPlanning/erc_path_planner \
  --build-base /validation/build --install-base /validation/install
source /validation/install/setup.bash
ros2 launch erc_path_planner path_planning.launch.py use_sim_time:=true
```

Expected: nodes wait for actual sensors, TF, clock and map. Missing inputs are
not supplied automatically. Lifecycle activation must succeed before sending goals.
For hardware/wall time, choose `use_sim_time:=false` with the appropriate real
sensor and localization bringup. This is an interface procedure, not hardware approval.

Inspect one command at a time in another terminal sourced into the same container:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 topic info /bridge/pointcloud --verbose
ros2 topic info /cmd_vel --verbose
```

Expected: active navigation nodes, one production bridge publisher, and the
smoother as the sole ROS velocity-command publisher. The Gazebo command bridge
is ROS→Gazebo only. A bidirectional command bridge is incorrect here.

## RViz and testing

In a GUI-enabled container on the same ROS graph:

```bash
ros2 launch erc_path_planner rviz.launch.py use_sim_time:=true
```

The production view shows map/global/local costmaps, Smac path, MPPI's
`/transformed_global_plan`, bridge points, robot model, TF and local footprint.
The `2D Goal Pose` tool publishes `/goal_pose`; the navigator handles it.
`2D Pose Estimate` does not initialize RTAB-Map/EKF by itself. MPPI trajectory
markers are disabled in the view by default; enabling the display also requires
MPPI's `visualize` parameter and adds computation overhead.

For **real PathPlanner with mocked dependencies**, use the
[mock package guide](../../testing/mocks/pathplanning/pathplanner_mock_tools/README.md).
For real Gazebo sensors/SLAM/Perception, use the
[validation runbook](../VALIDATION.md#gazebo-and-real-module-integration).

Cancel active navigation and wait for zero velocity before stopping launches.
Stop this run's launches with Ctrl+C, then stop only its container on the host:

```bash
docker stop --timeout 10 pathplanner-run
```

Artifacts remain in the host directory printed by `echo "$PATHPLANNER_ARTIFACTS"`.
See [VALIDATION.md](../VALIDATION.md) for failures and untested behavior.
