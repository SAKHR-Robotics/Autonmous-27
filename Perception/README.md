# Perception — ERC 2027 rover

ROS 2 **Jazzy / Ubuntu 24.04**. This folder contains real terrain and ArUco
processing. Gazebo or the real camera supplies sensor data; neither camera
drivers nor synthetic publishers are started by the core launch.

## 1. Packages and separation

| Package | Implemented responsibility |
| --- | --- |
| `terrain_geometry` | Point-cloud filtering, ground removal, clustering, obstacle features and local costmap |
| `terrain_geometry_msgs` | Terrain obstacle message definitions |
| `marker_detection` | ArUco detection, RGB-D/PnP pose estimation, tracking, marker TF, global marker mapping and action targets |
| `marker_detection_msgs` | Marker detection, pose, map and target message definitions |
| `perception_bringup` | Selectable launch orchestration; no perception algorithms |

Terrain computation lives in helper modules; `terrain_node.py` owns ROS
subscriptions, parameters, publication and pipeline sequencing. Marker stages
1–4 share `marker_detection_node.py`; TF, global mapping and action conversion
are separate real nodes. These adapters are implemented logic, not stubs.
Some terrain helpers still import ROS messages or TF; they are not all
ROS-independent libraries.

Synthetic sensor inputs and fake perception outputs belong in top-level
[`testing/`](../testing/), outside this folder. Existing SLAM test publishers
are recorded in the [integration review](INTEGRATION_REVIEW.md); relocating
those and creating tester/benchmarker tools is deferred. There is deliberately
no stub launch inside the production perception package.

## 2. Build perception only

Run from the repository root on Ubuntu 24.04:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-sklearn
rosdep install --from-paths Perception --ignore-src -r -y --rosdistro jazzy
colcon build --base-paths Perception --symlink-install
source install/setup.bash
```

`rosdep` must already be initialized and updated. Patchwork++ is optional;
`ground_removal_backend:=auto` uses the implemented NumPy fallback when it is
unavailable. This fallback is a real algorithm, not synthetic data.

**Do not use an unrestricted root build yet:** `PathPlaning/src/` contains a
second, incompatible `terrain_geometry_msgs` package. `--base-paths Perception`
excludes that duplicate for the perception build; it does not resolve the
repository-wide schema conflict.

## 3. Launch real processing

Default: terrain, marker detection/tracking, marker TF and action targets;
global marker mapping is off until localization is available.

```bash
ros2 launch perception_bringup perception_core.launch.py
```

Terrain only:

```bash
ros2 launch perception_bringup perception_core.launch.py \
  enable_marker_detection:=false enable_marker_tf:=false enable_marker_action:=false
```

Markers only, without global mapping:

```bash
ros2 launch perception_bringup perception_core.launch.py enable_terrain:=false
```

With an existing Gazebo rover/camera and `/clock`:

```bash
ros2 launch perception_bringup perception_core.launch.py use_sim_time:=true
```

With a separately started real camera, select its actual RGB, RGB-aligned depth,
RGB calibration and point-cloud topics. The following names are examples; inspect
`ros2 topic list -t` first:

```bash
ros2 launch perception_bringup perception_core.launch.py \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  input_pointcloud_topic:=/camera/camera/depth/color/points
```

For global marker mapping, provide `map -> odom -> base_link -> camera` TF and
add `enable_marker_mapping:=true`. No localization is fabricated. The existing
`ros2 launch marker_detection marker_branch.launch.py` entry point retains its
previous behavior of starting all four marker nodes, including global mapping.

Use [bringup options](perception_bringup/README.md) for independent node switches
and per-node marker YAML files. Terrain keeps its existing tuning arguments in
[`terrain.launch.py`](terrain_geometry_improved/terrain_geometry/launch/terrain.launch.py).
Use that launch directly for the full terrain tuning surface. Sensor topics and
`use_sim_time` passed as launch arguments override the corresponding marker YAML
values. Unspecified processing defaults remain unchanged.

## 4. Interfaces and TF ownership

| Input | Type | Requirement |
| --- | --- | --- |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | Finite XYZ points; transform its header frame to `base_link` |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | RGB image used for ArUco detection |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Must be RGB-aligned for RGB-D association; not just any raw depth image |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB calibration matching the image resolution |
| `/tf`, `/tf_static`, optional `/clock` | Standard TF/clock messages | Robot/localization owns base/camera/map frames; Gazebo or bag playback owns simulation time |

Camera topic names above are launch defaults, not a guarantee about a driver.
RGB-only detection/PnP can continue without matched depth, with the existing
quality flags indicating unavailable depth. Incorrect alignment can produce
incorrect depth associations even when frames arrive successfully.

| Output | Type | Actual integration status |
| --- | --- | --- |
| `/terrain/costmap` | `nav_msgs/msg/OccupancyGrid` | Local, inflated grid in `target_frame`; not directly consumed by the current planner/SLAM launches |
| `/terrain/obstacle_features` | `terrain_geometry_msgs/msg/ObstacleFeatureArray` | Planner bridge subscribes, but duplicate schemas prevent a reliable workspace-wide build |
| `/terrain/obstacle_markers` | `visualization_msgs/msg/MarkerArray` | RViz obstacle geometry |
| `/marker_poses` | `marker_detection_msgs/msg/MarkerTrackedPoseArray` | Tracked marker observations in camera frame |
| `/marker_map` | `marker_detection_msgs/msg/MarkerMap` | Optional map-frame marker estimates; requires localization and marker TF |
| `/marker_detection/targets` | `marker_detection_msgs/msg/MarkerActionTargetArray` | Base-frame targets with validity/quality gates; no controller consumer implemented under `Control/` |

The core launch creates no base/camera static transforms. Use the rover URDF or
the calibrated real robot TF tree, with one owner per transform. Marker TF
broadcasts `aruco_marker_<id>` frames; it does not localize the rover. All
participating stacks must agree on `use_sim_time`.

## 5. Verification and known limitations

```bash
ros2 launch perception_bringup perception_core.launch.py --show-args
colcon test --base-paths Perception --packages-select perception_bringup
colcon test-result --verbose
ros2 node list
ros2 topic hz /terrain/costmap
ros2 topic echo /marker_detection/targets --once
ros2 run tf2_ros tf2_echo base_link <actual_camera_optical_frame>
```

With the default launch, expect `terrain_node`, `marker_detection`,
`marker_tf_broadcaster`, and `marker_action_interface`; `marker_map` appears
only when enabled. Rate/echo commands need live sensor data. For clocked bag
playback, run `ros2 bag play /path/to/bag --clock` and launch perception with
`use_sim_time:=true`; the bag or external robot stack must also supply valid TF.

Launch tests expand real ROS launch conditions/includes/scopes but intercept
process creation, so they do not test algorithms or DDS sensor flow. Runtime
checks require sourced Jazzy and were not executed in the Windows editing
shell. See the [integration review](INTEGRATION_REVIEW.md) before treating empty
costmaps as evidence of traversable terrain. This implementation does not yet
establish reliable slope/crater or negative-obstacle detection.

Detailed processing references: [terrain](terrain_geometry_improved/terrain_geometry/README.md)
and [markers](marker_detection/README.md).
