# Perception: Linux / ROS 2 Jazzy runbook

Use Ubuntu 24.04 with ROS 2 Jazzy, or the project's Linux Docker environment.
The package layout and marker improvements are preserved from `jazzy-port`.
All synthetic inputs, mock outputs and artificial localization fixtures belong
under top-level `testing/`; production launches start real processing only.

## 1. Build and source

From the repository root in a Linux terminal:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths Perception PathPlanning/erc_path_planner \
  testing/Perception/perception_test_tools --ignore-src --rosdistro jazzy -r -y
colcon build --base-paths Perception PathPlanning/erc_path_planner \
  testing/Perception/perception_test_tools --symlink-install
source install/setup.bash
```

Source Jazzy and the workspace overlay in every terminal. Use a fresh build/install
directory if an old symlink build references relocated launch files. Git enforces
LF for executable scripts so Linux shebangs work. No Windows runtime is required.

| Package | Responsibility |
| --- | --- |
| `terrain_geometry` | Terrain processing, optional costmap, calibrated surface risk, obstacle memory and master launch |
| `terrain_geometry_msgs` | Existing obstacle geometry interfaces |
| `marker_detection` | ArUco RGB-D/PnP, tracking, TF, mapping, action targets and optional RTAB-Map adapter |
| `marker_detection_msgs` | Existing marker interfaces |
| `perception_test_tools` under `testing/Perception/` | Explicit synthetic sensors, truth overlays and test orchestration |

Internal filtering helpers remain library functions to avoid unnecessary cloud
serialization. Independent launches correspond to real ROS processing stages.

## 2. Independent stages and the master

```bash
ros2 launch terrain_geometry terrain.launch.py use_sim_time:=true
ros2 launch marker_detection marker_detection.launch.py use_sim_time:=true
ros2 launch marker_detection marker_tf.launch.py use_sim_time:=true
ros2 launch marker_detection marker_mapping.launch.py use_sim_time:=true
ros2 launch marker_detection marker_action_interface.launch.py use_sim_time:=true
ros2 launch marker_detection landmark_adapter.launch.py use_sim_time:=true
ros2 launch terrain_geometry obstacle_memory.launch.py use_sim_time:=true
```

Use `false` for real cameras/wall-clock inputs. Do not start individual nodes again
when the master owns them. Marker consumers need tracked observations; mapping also
needs camera-to-map TF. Memory needs odometry TF. When launching memory beside an
independent terrain node, set `publish_to_obstacles_only:=false` on terrain.

```bash
# Terrain + marker detection/tracking, marker TF and action targets:
ros2 launch terrain_geometry perception_system.launch.py use_sim_time:=true
# Terrain only:
ros2 launch terrain_geometry perception_system.launch.py launch_marker_branch:=false
# Marker branch only:
ros2 launch terrain_geometry perception_system.launch.py launch_terrain:=false
# Combined processing and visualization:
ros2 launch terrain_geometry perception_system.launch.py launch_rviz:=true enable_costmap:=true
```

| Master argument | Default / behavior |
| --- | --- |
| `launch_terrain`, `launch_marker_branch` | `true`; independently select pipelines |
| `launch_marker_detection`, `launch_marker_tf`, `launch_marker_action` | `true`; apply inside the selected marker branch |
| `launch_marker_mapping` | `false`; requires localization |
| `launch_landmark_adapter` | `false`; enable for RTAB-Map landmarks |
| `launch_memory` | `false`; stable-frame memory, exclusive obstacle-output ownership |
| `enable_terrain_risk` | `false`; requires positive measured limits and gravity reference |
| `enable_costmap`, `launch_rviz`, `publish_debug_topics` | `false`; optional processing/visualization |
| `use_sim_time` | `true`, preserving the Jazzy master default; explicitly use `false` for hardware/synthetic inputs |
| `target_frame`, `rviz_fixed_frame` | `base_link` |
| Camera topic arguments | Simulation-style `/camera/...` defaults; configurable for hardware/tests |
| `detection_params_file`, `tf_params_file`, `mapping_params_file`, `action_params_file`, `landmark_params_file`, `memory_params_file` | Separate, scoped configuration files |
| `rviz_config` | Installed combined view |

Inspect all options with `--show-args`. Individual marker launches retain
`params_file`. Topic/clock launch arguments override matching YAML values.
The legacy marker branch retains mapping enabled by default; the master explicitly
disables it unless requested. Use the terrain-only launch for its full tuning surface.

## 3. Perception-only tests: what to launch

### Without Gazebo, a physical camera, or SLAM

After the scoped build, this starts test sensors and the real perception nodes:

```bash
ros2 launch perception_test_tools test_stack.launch.py \
  pipeline:=all scenario:=rocks_and_marker launch_rviz:=true
```

Select `pipeline:=terrain` or `pipeline:=markers` for independent tests. The test
stack uses wall-clock ROS time and needs no `/clock`. All sensor topics are under
`/testing/perception/camera/`. No camera driver, rover control or fake map frame starts.

For separate feeder/processing terminals:

```bash
# Terminal A: only the testing input feeder.
ros2 launch perception_test_tools sensor_inputs.launch.py scenario:=rocks_and_marker
# Terminal B: only real processing, explicitly connected to test inputs.
ros2 launch terrain_geometry perception_system.launch.py \
  use_sim_time:=false enable_costmap:=true launch_rviz:=true \
  input_pointcloud_topic:=/testing/perception/camera/points \
  rgb_topic:=/testing/perception/camera/image_raw \
  depth_topic:=/testing/perception/camera/depth \
  camera_info_topic:=/testing/perception/camera/image_info
```

`test_stack.launch.py` automatically uses the test view with green truth geometry.
For the explicit master command, optionally add:

```bash
rviz_config:=$(ros2 pkg prefix perception_test_tools)/share/perception_test_tools/rviz/perception_test_view.rviz
```

Expected evidence: rock observations near base (2.0, 0.6, 0.25) m, marker ID 0 near
optical depth 2 m, occupied/inflated cells around the rock, and healthy diagnostics.
Compare the same frames/timestamps. These fixtures are not photorealistic scenes.

Scenarios: `rocks_and_marker`, `ground_only`, `missing_depth`, `invalid_cloud`,
`dropout`, `turn_occlusion`, `terrain_hazard`. Missing depth preserves RGB/PnP but
must report unavailable depth. Invalid/dropout input must not imply clear terrain.

Memory/turn fixture:

```bash
ros2 launch perception_test_tools test_stack.launch.py \
  pipeline:=terrain launch_memory:=true scenario:=turn_occlusion launch_rviz:=true
```

This explicitly supplies synthetic odometry and rotates cloud geometry. Synthetic
motion is restricted to terrain-only tests; RGB-D fixtures are fixed-camera tests.
Never run this TF producer beside real odometry. No map localization is fabricated.

### Existing Gazebo camera

Build/source the rover and worlds packages separately, then:

```bash
# Terminal A:
ros2 launch my_robot_description gazebo.launch.py world:=world1.world publish_map_tf:=false
# Terminal B:
ros2 launch terrain_geometry perception_system.launch.py \
  use_sim_time:=true enable_costmap:=true launch_rviz:=true
```

This Jazzy rover launch already bridges `/clock`; do not add a second publisher.
Check `ros2 topic echo /clock --once`, camera TF and sensor topics. A world without
a visible matching ArUco marker is not a positive marker test. Global mapping
remains off until genuine localization supplies map-to-camera TF.

### Physical RGB-D camera

```bash
# Terminal A: driver external to perception.
ros2 launch realsense2_camera rs_launch.py enable_color:=true enable_depth:=true \
  enable_sync:=true align_depth.enable:=true pointcloud.enable:=true
# Terminal B: use actual topic names from ros2 topic list -t.
ros2 launch terrain_geometry perception_system.launch.py use_sim_time:=false \
  launch_rviz:=true enable_costmap:=true \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  input_pointcloud_topic:=/camera/camera/depth/color/points
```

Supply measured camera-to-base TF via the calibrated URDF or one explicit mount
publisher. Do not invent mounting values or duplicate existing transforms. Match
RGB-aligned depth and RGB CameraInfo resolutions. Measure printed marker size and
match the configured dictionary (`DICT_4X4_50` by default) and per-ID sizes.

For bags, use recorded TF and `ros2 bag play /path/to/bag --clock` with
`use_sim_time:=true`; otherwise use explicitly wall-clock-compatible playback.

## 4. Visualization and interfaces

```bash
ros2 launch terrain_geometry rviz.launch.py use_sim_time:=true
ros2 launch marker_detection rviz.launch.py use_sim_time:=true
```

These start only RViz and accept `fixed_frame`, `rviz_config`, `rgb_topic` and
`input_pointcloud_topic`. Prefer master `launch_rviz:=true` for the combined view.
Sensor displays use Best Effort. Optional global map/marker/debug displays stay
disabled until their producers and TF exist. Debug clouds need `publish_debug_topics`.

| Output | Contract |
| --- | --- |
| `/perception/local_bboxes` | Current `vision_msgs/Detection3DArray` |
| `/perception/obstacles_only` | `Detection3DArray`; owned by terrain normally, exclusively by enabled memory |
| `/terrain/obstacle_features`, `/terrain/obstacle_markers` | Detailed geometry and current-frame visualization |
| `/terrain/costmap` | Optional grid; unknown/unobserved cells stay unknown; failed/stale processing does not become free space |
| `/terrain/diagnostics` | Processing health/freshness; an empty detection list is not a health signal |
| `/marker_poses`, `/marker_detection/targets`, `/marker_map` | Tracked poses, validity-gated action targets, optional global map |
| `/perception/landmark_detections` | Native RTAB-Map landmarks; positive ID offset, position covariance, configured angular variance |
| `/bridge/pointcloud` | Existing planner bridge output for cloud consumers |

The planner already starts its bridge. For SLAM without the planner, use
`ros2 launch erc_path_planner obstacle_bridge.launch.py use_sim_time:=true`.
Do not launch a duplicate bridge. Bounding-box points are marking-only;
clearing uses actual camera observations. Adapt consumer clearing-cloud topics
to the active hardware/test source.

**Memory:** `memory_params_file` configures TTL (initially 5 s), association and
optional observed-ground clearing. Defaults are tunable, not ERC requirements.
Ground clearing is off until validated and needs debug ground clouds.

**Surface risk:** requires positive measured `max_slope_deg`, `max_step_m`,
`max_roughness_m`, plus TF from a calibrated gravity-aligned `gravity_frame`
(default `odom`). A tilted odom frame is not a gravity reference. Hazardous
observed patches enter obstacle boxes. Sparse/missing depth remains unknown.
This is not validated negative-obstacle detection or guaranteed crater coverage.

## 5. Validate and diagnose

```bash
python3 -m pytest Perception/terrain_geometry/test Perception/marker_detection/test \
  testing/Perception/perception_test_tools/test -q
ros2 topic echo /terrain/diagnostics --once
ros2 topic echo /perception/local_bboxes --once
ros2 topic echo /marker_detection/targets --once
ros2 run tf2_ros tf2_echo base_link <actual_camera_header_frame>
```

| Symptom | Check/action |
| --- | --- |
| No simulated updates | Unpause Gazebo; verify clock/time settings |
| Missing RViz data | Topic names, Best Effort QoS, fixed frame, camera TF |
| Wrong marker depth/scale | Aligned depth, calibration, printed size/dictionary |
| Unknown terrain map | Diagnostics, input age, observed ground and TF; do not force free space to hide failure |
| No global marker map | Genuine localization TF and enabled mapping are required |
| Memory/TF conflict | Stop synthetic motion before starting real odometry; one owner per transform/output |

See [integration limits](INTEGRATION_REVIEW.md) and the
[testing README](../testing/Perception/Readme.md). Physical accuracy, sun/dust/blur,
target-computer latency, safe stopping on perception loss and full autonomous
navigation still require rover validation. The official ERC competitor zone
linked 2026 documents at review time; no unverified 2027 compliance is claimed.
