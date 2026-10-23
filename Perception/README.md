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
[`terrain.launch.py`](terrain_geometry/terrain_geometry/launch/terrain.launch.py).
Use that launch directly for the full terrain tuning surface. Sensor topics and
`use_sim_time` passed as launch arguments override the corresponding marker YAML
values. Unspecified processing defaults remain unchanged.

## 4. Complete Gazebo and real-camera workflows

### Gazebo: start the camera, processing and combined RViz

1. **Build the simulation and perception packages.** From the repository root:

   ```bash
   source /opt/ros/jazzy/setup.bash
   sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-rviz2
   rosdep install --from-paths Perception Rover/my_robot_description worlds --ignore-src -r -y --rosdistro jazzy
   colcon build --base-paths Perception Rover/my_robot_description worlds --symlink-install
   source install/setup.bash
   ```

   This excludes the duplicate planner message package. After the folder rename,
   rebuild before launching; an old symlink installation may point to the old
   source path. If colcon reports a stale CMake source directory, remove only
   `build/terrain_geometry_msgs` and rebuild. Do not delete the whole workspace.

2. **Terminal A: start the existing world and rover.** Source Jazzy and the
   workspace in every new terminal:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash
   ros2 launch my_robot_description gazebo.launch.py world:=world1.world
   ```

   The existing launch bridges RGB, depth, CameraInfo and point clouds to the
   default perception topics. It does not launch perception. Inspect startup
   errors and wait for the rover to spawn; simulation assets outside perception
   have not been runtime-validated by this change.

3. **Terminal B: supply the missing simulation clock bridge.** The current
   rover launch does not bridge `/clock`:

   ```bash
   ros2 run ros_gz_bridge parameter_bridge '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
   ```

   Run `ros2 topic echo /clock --once` in another terminal. Expect increasing
   simulation time while Gazebo is unpaused. If Gazebo exposes only a world
   clock, inspect `gz topic -l` and bridge that clock with a remap to `/clock`.
   Use only one clock publisher.

4. **Terminal C: start the core and master RViz view.**

   ```bash
   ros2 launch perception_bringup perception_core.launch.py use_sim_time:=true use_rviz:=true
   ```

   Expect a raw point cloud, terrain costmap/obstacle boxes and annotated marker
   image. The fixed frame defaults to `base_link`, so SLAM is not required.
   Global marker-map display is disabled until mapping/localization are enabled.
   Marker results require a visible marker with the configured dictionary and
   physical size; a world with no marker is not a positive marker-detection test.

5. **Validate before moving.** Use the checks below. If driving is needed, run
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard` in another sourced
   terminal after installing `ros-jazzy-teleop-twist-keyboard`. Observe changes
   in obstacle geometry as the camera approaches or turns away from a rock;
   do not interpret an empty map alone as a successful test.

### Real camera: start the driver and provide calibrated robot TF

1. **Prepare the device.** Build perception using section 2, install a Jazzy
   RealSense driver compatible with the connected device, connect the camera,
   and confirm that its firmware/USB connection is healthy. The current
   [upstream launch parameters](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_camera/launch/rs_launch.py)
   expose alignment and point-cloud generation explicitly; inspect your installed
   version with `ros2 launch realsense2_camera rs_launch.py --show-args`.

2. **Terminal A: start the camera with aligned depth and point clouds.**

   ```bash
   ros2 launch realsense2_camera rs_launch.py \
     enable_color:=true enable_depth:=true enable_sync:=true \
     align_depth.enable:=true pointcloud.enable:=true
   ```

   `ros2 topic list -t` must show RGB, RGB-aligned depth, color CameraInfo and
   point clouds. The example below assumes the usual `/camera/camera/...`
   namespace; use the actual names if different.

3. **Provide the measured camera mount transform.** Prefer the robot's calibrated
   URDF/robot_state_publisher. The camera driver publishes internal optical TF;
   it cannot know where the camera is mounted on `base_link`. If that mount TF
   has no other publisher, set `MOUNT_X`, `MOUNT_Y`, `MOUNT_Z` in metres and
   `MOUNT_ROLL`, `MOUNT_PITCH`, `MOUNT_YAW` in radians from your measured
   calibration, then run in a separate terminal:

   ```bash
   ros2 run tf2_ros static_transform_publisher \
     --x "${MOUNT_X:?Set measured MOUNT_X}" --y "${MOUNT_Y:?Set measured MOUNT_Y}" \
     --z "${MOUNT_Z:?Set measured MOUNT_Z}" \
     --roll "${MOUNT_ROLL:?Set calibrated MOUNT_ROLL}" \
     --pitch "${MOUNT_PITCH:?Set calibrated MOUNT_PITCH}" \
     --yaw "${MOUNT_YAW:?Set calibrated MOUNT_YAW}" \
     --frame-id base_link --child-frame-id camera_link
   ```

   Verify the driver's root frame is `camera_link`; change that child frame if
   the driver uses a different one. Do not reuse simulated mounting values or
   publish a second transform for an already-owned child frame.

4. **Terminal B: launch perception and the master RViz view.**

   ```bash
   ros2 launch perception_bringup perception_core.launch.py use_rviz:=true \
     rgb_topic:=/camera/camera/color/image_raw \
     depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
     camera_info_topic:=/camera/camera/color/camera_info \
     input_pointcloud_topic:=/camera/camera/depth/color/points
   ```

   Keep `use_sim_time:=false` (the default). The master RViz launch follows these
   RGB/cloud topic overrides automatically.

5. **Test with measured targets.** Place a rock/box and a printed marker in view.
   Generate a marker matching the current default dictionary:

   ```bash
   ros2 run marker_detection generate_aruco_marker.py \
     --dictionary DICT_4X4_50 --id 0 --output /tmp/erc-marker-0.png
   ```

   Print with a white margin around the marker. Match the dictionary and measured
   outer-black-border side length to the detector YAML. Compare obstacle and
   marker positions to tape measurements in the same coordinate frame. Record
   observed error and processing rate; this guide does not claim an unmeasured
   accuracy or FPS target.

### Separate module views and the master view

```bash
# Terrain processing and its separate viewer (two terminals):
ros2 launch terrain_geometry terrain.launch.py use_sim_time:=true
ros2 launch terrain_geometry rviz.launch.py use_sim_time:=true
# Marker processing without terrain, then its separate viewer:
ros2 launch perception_bringup perception_core.launch.py enable_terrain:=false use_sim_time:=true
ros2 launch marker_detection rviz.launch.py use_sim_time:=true
# View both existing pipelines without starting new processing nodes:
ros2 launch perception_bringup rviz.launch.py use_sim_time:=true
```

For real hardware omit `use_sim_time:=true`; pass `rgb_topic` and
`input_pointcloud_topic` to standalone viewers when the driver uses other names.
Every viewer accepts `fixed_frame` and `rviz_config`; the master processing
launch instead exposes `rviz_fixed_frame` alongside `use_rviz` and `rviz_config`.
Run one chosen viewer to avoid duplicate windows. Terrain debug clouds are
optional displays: enable `publish_debug_topics:=true` on the terrain launch
before enabling those displays. Global-map visualization needs localization,
`enable_marker_mapping:=true`, and its RViz display enabled manually. Custom
marker target messages are inspected with `ros2 topic echo`, not a built-in
RViz pose display.

### Observable checks and troubleshooting

| Check | Command / action | Expected evidence |
| --- | --- | --- |
| Camera feeds | `ros2 topic hz /camera/depth/color/points` (substitute real topic) | Nonzero incoming rate; raw cloud has finite geometry in RViz |
| Camera frame | `ros2 topic echo /camera/depth/color/points --once --field header` | Frame and timestamp correspond to the active camera/clock |
| Mount TF | `ros2 run tf2_ros tf2_echo base_link <frame_from_header>` | Connected transform matching the physical mount |
| Terrain | `ros2 topic echo /terrain/obstacle_features --once` | Obstacle centroid/dimensions correspond to the visible test object |
| Costmap | Display `/terrain/costmap` and `/terrain/obstacle_markers` | Rock footprint and inflation align with point-cloud geometry |
| Marker tracking | `ros2 topic echo /marker_poses --once` | Correct visible ID and plausible pose/quality flags after repeated observations |
| Action interface | `ros2 topic echo /marker_detection/targets --once` | Inspect `pose_valid`, `stable`, `usable_for_action`, confidence and age; validity is not guaranteed merely because messages arrive |
| Diagnostics | `ros2 topic echo /marker_detection/diagnostics --once` | Inspect sync/depth availability, latency and warnings |

| Symptom | Location / likely cause | Action |
| --- | --- | --- |
| No data in simulated nodes | `/clock` missing or Gazebo paused | Start the clock bridge; unpause and verify clock updates |
| Cloud/TF errors | RViz fixed frame or missing camera mount TF | Use `base_link` and inspect the exact sensor header frame |
| Images/clouds missing in RViz | Topic or QoS mismatch | Select the live topic; sensor displays use Best Effort |
| 2D markers but poor/no depth association | Raw depth selected instead of RGB-aligned depth, bad calibration or sync | Inspect aligned-depth topic, dimensions, stamps and diagnostics |
| Marker distance is scaled incorrectly | Wrong printed marker size or dictionary | Match detector YAML to the measured marker |
| Empty or stale costmap after invalid data | Existing invalid-input behavior | Read the integration review; do not count this as a clear-terrain result |

## 5. Interfaces and TF ownership

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

## 6. Verification and known limitations

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

RViz configuration checks validate view topics/frames; they do not render RViz.
Launch tests expand real ROS launch conditions/includes/scopes but intercept
process creation, so they do not test algorithms or DDS sensor flow. Runtime
checks require sourced Jazzy and were not executed in the Windows editing
shell. See the [integration review](INTEGRATION_REVIEW.md) before treating empty
costmaps as evidence of traversable terrain. This implementation does not yet
establish reliable slope/crater or negative-obstacle detection.

Detailed processing references: [terrain](terrain_geometry/terrain_geometry/README.md)
and [markers](marker_detection/README.md).
