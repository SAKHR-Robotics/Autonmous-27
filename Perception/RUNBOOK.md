# Perception runbook — Docker / ROS 2 Jazzy

Run builds, Python tests and ROS commands inside `autonomous27:jazzy`. The host
workspace only supplies source files. `Perception/` contains five packages;
combined orchestration belongs to `perception_bringup`.

## 1. Open an isolated validation container

On the host, use a fresh artifact directory so another agent's build is untouched:

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
PERCEPTION_CHECK_DIR="$(mktemp -d /tmp/perception-check.XXXXXX)"
docker run --rm -it --name perception-check --network none \
  --mount type=bind,source="$PWD",target=/workspace,readonly \
  --mount type=bind,source="$PERCEPTION_CHECK_DIR",target=/validation \
  --env ROS_DOMAIN_ID=91 --env PYTHONDONTWRITEBYTECODE=1 \
  --entrypoint bash autonomous27:jazzy
```

The container has a private network and read-only source. It cannot see ROS nodes
on the host or in another container. Run any chosen test feeder in this same
container. No packages are installed into the image and no host Python is used.
This setup is headless; GUI/hardware workflows use the project's normally
configured GUI/hardware container with explicit device/display access instead.

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd /validation
colcon --log-base /validation/log build --base-paths /workspace/Perception \
  --build-base /validation/build --install-base /validation/install --symlink-install
source /validation/install/setup.bash
```

Five packages should finish: two message packages, terrain, markers and bringup.
Use `docker exec -it perception-check bash` for another terminal and source Jazzy
and `/validation/install/setup.bash` there too. In the regular project container,
the repository is also mounted at `/workspace`; use a dedicated fresh build/install
base rather than a shared overlay. Do not remove another agent's build directory.

## 2. Inspect or start production processing

```bash
ros2 launch perception_bringup perception.launch.py --show-args
# Combined processing; requires actual external inputs:
ros2 launch perception_bringup perception.launch.py use_sim_time:=false
# Terrain only:
ros2 launch terrain_geometry terrain.launch.py use_sim_time:=false
# Marker branch without global localization:
ros2 launch marker_detection marker_branch.launch.py use_sim_time:=false launch_marker_mapping:=false
```

Choose ONE of these processing commands; do not duplicate its nodes by also
starting their individual launches. Without sensors, nodes may start correctly
but must report unavailable/degraded processing. That is a startup check, not
successful perception validation. Stop a launch with Ctrl+C; production nodes
should exit without shutdown tracebacks.

| Package / launch | Starts | Must be supplied externally |
| --- | --- | --- |
| `perception_bringup perception.launch.py` | Terrain, marker detector/tracker, marker TF and action interface by default | Camera streams and calibrated camera-to-base TF; clock when selected |
| `perception_bringup rviz.launch.py` | Combined RViz only | GUI, data and TF |
| `terrain_geometry terrain.launch.py` | Terrain processing node | PointCloud2 and sensor-to-target TF |
| `terrain_geometry obstacle_memory.launch.py` | Retained obstacle memory | Local box observations and memory-frame TF |
| `terrain_geometry rviz.launch.py` | Terrain RViz only | GUI, data and TF |
| `terrain_geometry perception_system.launch.py` | Deprecated forwarding entry to combined bringup | Installed `perception_bringup` and its requirements |
| `marker_detection marker_detection.launch.py` | Detector, depth association, PnP and tracking in one node | RGB; calibration for poses; aligned depth for fusion |
| `marker_detection marker_tf.launch.py` | Camera-to-marker TF broadcaster | Tracked marker topic |
| `marker_detection marker_action_interface.launch.py` | Rover-base targets and compatibility poses | Tracked topic and camera-to-base TF |
| `marker_detection marker_mapping.launch.py` | Global map and optional map markers | Tracked topic and genuine map-to-camera TF |
| `marker_detection landmark_adapter.launch.py` | Native RTAB-Map landmark messages | Tracked marker topic |
| `marker_detection marker_branch.launch.py` | Detector, TF, action interface and mapping by default | Selected stages' inputs; disable mapping explicitly without localization |
| `marker_detection rviz.launch.py` | Marker RViz only | GUI, data and TF |

The combined launch preserves existing feature switches: mapping, memory,
landmarks, RViz, costmap and risk checks default off. The marker branch retains
its legacy mapping-on default. We have not changed optional feature selection.
The combined/marker/terrain processing launches inherit `use_sim_time=true`;
standalone RViz, memory and landmark launches default false. Always pass time.
Individual parameter files remain scoped per node; launch clock/topic overrides
win over the corresponding YAML values. Inspect each launch with `--show-args`.

## 3. Select input providers and clock explicitly

For Gazebo or bag playback publishing `/clock`, use `use_sim_time:=true`.
For wall-clock physical cameras or the existing synthetic sensor feeder, use
`false`. All communicating nodes need the same time basis. Docker does not choose
this setting. A stopped simulation clock also stops ROS-time diagnostic timers;
consumers need their own observation-loss policy.

Workspace camera aliases are `/camera/image_raw`, `/camera/depth/image_raw`,
`/camera/camera_info` and `/camera/depth/color/points`. Check actual driver topics
and frame headers. Depth for markers must be aligned to RGB, with matching RGB
CameraInfo dimensions; a remapping does not perform alignment.

Example after starting a correctly configured physical camera driver:

```bash
ros2 launch perception_bringup perception.launch.py use_sim_time:=false \
  rgb_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  input_pointcloud_topic:=/camera/camera/depth/color/points
```

Camera-to-base mounting TF must come from measured URDF/calibration. Global
mapping needs genuine map-to-camera TF; memory needs odometry TF. Perception
does not fabricate map/odom anchors or launch camera drivers. Start any Gazebo,
bag, hardware or mocked provider separately in the intended ROS environment.
For bags, check the recorded TF and use `ros2 bag play /path/to/bag --clock`
when selecting simulation time. Do not start a second clock or TF authority.

## 4. RViz and observation checks

In the GUI-enabled Jazzy container, with the same overlay sourced:

```bash
ros2 launch perception_bringup perception.launch.py use_sim_time:=false launch_rviz:=true
# Or attach the combined view to already-running processing:
ros2 launch perception_bringup rviz.launch.py use_sim_time:=false
```

Choose processing-plus-RViz OR standalone RViz. Default fixed frame is
`base_link`. Camera remapping is forwarded by combined startup. Standalone RViz
accepts `rgb_topic`, `input_pointcloud_topic`, `fixed_frame` and `rviz_config`.
Sensor subscriptions use Best Effort. RobotModel, raw/pose-debug images, global map, costmap and debug
cloud displays start disabled; enable them after starting their producers.
`enable_costmap:=true` enables the grid producer; `publish_debug_topics:=true`
enables terrain debug clouds. Production views contain no testing truth display.

```bash
ros2 node list
ros2 topic echo /terrain/diagnostics --once
ros2 topic echo /marker_detection/diagnostics --once
ros2 topic echo /perception/local_bboxes --once
ros2 topic echo /marker_detection/targets --once
ros2 run tf2_ros tf2_echo base_link <actual_camera_header_frame>
```

A valid obstacle frame should have finite geometry and the configured target
frame; action targets must report valid base-frame geometry before use. Inspect
headers, flags, covariance and freshness, not just whether messages arrive.
Unavailable terrain must not become a valid empty observation or a clear map.
See [interface details](README.md#data-sent-to-other-modules) and
[validation commands and limits](TESTING_GUIDE.md).

## 5. Existing world: validate one branch at a time

This is a live GUI profile for the existing `autonomous27:jazzy` container,
not the headless isolated profile in section 1. It uses the world already
selected in the SLAM review, `final_world_R&A.world`, real Gazebo camera data,
simulation time and ROS domain 92. No mock feeder runs with these sensors.
Commands below are a validation procedure, not a claim that this world has
passed Perception accuracy checks.

### 5.1 Start Docker and build once

On the host, if `autonomous27_container` is not already running:

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
bash docker/run_docker.sh
```

Inside that container:

```bash
source /opt/ros/jazzy/setup.bash
cd /workspace
colcon --log-base /tmp/perception-live/log build --symlink-install \
  --paths worlds Rover/my_robot_description SLAM/rover_slam \
    Perception/terrain_geometry_msgs Perception/marker_detection_msgs \
    Perception/terrain_geometry Perception/marker_detection Perception/perception_bringup \
  --build-base /tmp/perception-live/build --install-base /tmp/perception-live/install \
  --cmake-args -DBUILD_TESTING=OFF
```

This builds eight explicitly located packages without recursively discovering
the world asset tree. SLAM is included for the later optional integration stage.
No existing shared build/install directory is removed or reused.
This builds runtime artifacts; the separate pytest command below runs production
regression checks without enabling unrelated CMake lint targets.

For every additional host terminal, enter the same container:

```bash
docker exec -it autonomous27_container bash
```

In **every container terminal**, including the first, run:

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/perception-live/install/setup.bash
export ROS_DOMAIN_ID=92
cd /workspace
```

Use terminal A for Gazebo, B for processing, C for RViz, and D for checks.
Use another sourced terminal for teleoperation or the optional SLAM process.
Keep A running throughout; stop B/C with Ctrl+C before changing branches.

### 5.2 Start the world and rover; validate sensors first

Terminal A:

```bash
ros2 launch my_robot_description gazebo.launch.py \
  'world:=final_world_R&A.world' \
  publish_map_tf:=false publish_camera_tf:=false bridge_sim_tf:=false
```

This one launch starts Gazebo, spawns the rover, runs robot/joint state
publishers and bridges camera, depth, calibration, IMU, odometry and clock.
Do not also run `spawn_rover.launch.py` or a second world launch. The current
robot_state_publisher already supplies the optical sensor child; enabling the
extra camera TF would duplicate it. The map anchor and simulated odometry TF
are disabled; local `base_link` Perception does not need global localization.

Wait for the rover to settle and check in D, one command at a time:

```bash
ros2 topic echo /clock --once
ros2 topic echo /camera/depth/color/points --field header --once --qos-reliability best_effort
ros2 topic echo /camera/image_raw --field header --once --qos-reliability best_effort
ros2 topic echo /camera/depth/image_raw --field encoding --once --qos-reliability best_effort
ros2 topic echo /camera/camera_info --field k --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo base_link my_robot/camera_link/camera --ros-args -p use_sim_time:=true
```

Pass this prerequisite only when time advances, images/clouds are updating,
calibration has valid positive focal lengths, and camera-to-base TF resolves.
The selected Gazebo depth stream was observed as `32FC1` metres in the SLAM
review; verify the actual encoding. Use the actual header frame in the TF command
if it differs. Check stream cadence with `ros2 topic hz TOPIC` and stop it with
Ctrl+C. A paused simulation will not deliver advancing simulated timestamps.

Optional manual driving in another sourced terminal:

```bash
ros2 run my_robot_description teleop_gui.py
```

Start stationary. The GUI sends velocity commands; do not run a second command
source such as a planner during this check.

### 5.3 Terrain alone

Terminal B:

```bash
ros2 launch terrain_geometry terrain.launch.py use_sim_time:=true \
  enable_costmap:=true publish_debug_topics:=true
```

Terminal C:

```bash
ros2 launch terrain_geometry rviz.launch.py use_sim_time:=true fixed_frame:=base_link
```

Enable `Robot Model`, `Debug Ground Cloud`, `Debug Clustered Cloud` and `Terrain
Costmap` in RViz alongside the raw camera points and obstacle markers. If needed,
add PointCloud2 `/terrain/debug/voxel_cloud`, using Best Effort subscriptions.
The latter is filtered non-ground data before clustering. Compare the same
visible patch across raw points, ground, non-ground and boxes.

The RViz Grid at base_link Z=0 is a drawing aid, **not the physical ground**.
Do not decide the floor is misplaced simply because it is below that grid.

Terminal D:

```bash
ros2 topic echo /terrain/diagnostics --once
ros2 topic echo /perception/local_bboxes --once
ros2 topic info /perception/obstacles_only --verbose
```

Inspect a clear patch of ground, a visible rock, and a gentle rise separately.
No box should cover the known clear patch; real rocks must remain detected with
plausible size and position. Other rocks elsewhere in the camera view can still
produce valid boxes. Move slowly and repeat from another distance and heading.
Record the location/viewpoint where a floor patch first enters the non-ground
cloud. Correct raw geometry with incorrect classification isolates segmentation;
incorrect raw geometry requires checking frames/calibration first. A misleading
box with correct classified points instead implicates clustering/box geometry.

The controlled native ground checks currently fail. Healthy diagnostics and a
rendered costmap are not an accuracy pass. Stop B and C before the marker stage;
leave A running.

### 5.4 Marker detection, tracking, TF and targets

Start with detector/tracker only in B:

```bash
ros2 launch marker_detection marker_detection.launch.py use_sim_time:=true
```

In C:

```bash
ros2 launch marker_detection rviz.launch.py use_sim_time:=true fixed_frame:=base_link
```

Point the camera at an actual visible world marker. Enable the 2D detection
debug image, then inspect in D:

```bash
ros2 topic echo /marker_detection/diagnostics --once
ros2 topic echo /marker_detections --once
ros2 topic echo /marker_poses --once
```

Check the known ID/corners, pose validity, track state, covariance, quality flags,
depth fusion status and observation age. The configured dictionary and physical
marker size must match the scene; current defaults are `DICT_4X4_50` and 0.21 m.
Do not treat an empty detection array as failure if no compatible marker is
visible. Positive acceptance needs a readable, geometrically suitable marker.

Stop only B. Replace the detector-only launch with the full local marker branch:

```bash
ros2 launch marker_detection marker_branch.launch.py use_sim_time:=true \
  launch_marker_mapping:=false launch_landmark_adapter:=false
```

It now starts the detector/tracker, marker TF and action interface, without global
mapping. C can remain open. Inspect targets in D:

```bash
ros2 topic echo /marker_detection/targets --once
```

For a usable observation, verify `pose_valid`, `stable`, `usable_for_action`,
finite base-frame distances/covariance and current timestamps. A confirmed track
alone is insufficient. For a confirmed marker ID N, inspect its configured TF
frame with `ros2 run tf2_ros tf2_echo base_link aruco_marker_N --ros-args -p use_sim_time:=true`,
replacing N with the actual ID. Hide the marker, wait beyond the action freshness
limit and check that it is not kept actionable; reveal it and check recovery.
If the camera stream itself stops, messages may stop rather than publishing a
new invalid target, so consumers must enforce header age.

An ambiguous observation being rejected is correct rejection behavior, not a
positive camera-to-action pass. Never lower gates merely to get a usable target.
Stop B/C before switching to combined startup.

### 5.5 Combined startup, recording and optional integration

With A still running, start in B:

```bash
ros2 launch perception_bringup perception.launch.py use_sim_time:=true \
  launch_rviz:=true enable_costmap:=true publish_debug_topics:=true
```

This owns processing and combined RViz; do not also start the separate branches
or C's RViz. Repeat the local checks while stationary, turning and moving slowly.
In D, record a failing view for later replay in a dedicated output directory:

```bash
mkdir -p /tmp/perception-live/results
ros2 bag record -o "/tmp/perception-live/results/perception_$(date +%Y%m%d_%H%M%S)" \
  /clock /tf /tf_static /camera/image_raw /camera/depth/image_raw \
  /camera/camera_info /camera/depth/color/points /imu/data /odom \
  /terrain/diagnostics /perception/local_bboxes /terrain/debug/ground_cloud \
  /terrain/debug/voxel_cloud /marker_detections /marker_poses \
  /marker_detection/targets /marker_detection/diagnostics
```

Ctrl+C stops the recorder. Inspect `ros2 bag info BAG_DIRECTORY` before using a
recording, particularly static TF. Bag playback belongs in a separate run with
the original simulator/input publishers stopped; replay the input topics only,
so recorded Perception outputs do not duplicate newly computed outputs. Images
and clouds can make large recordings; record just the bounded failing maneuver.

For optional memory/global mapping/SLAM-landmark interfaces, first run genuine
localization in another sourced terminal. Keep the existing fusion defaults;
SLAM's independent accuracy concerns remain documented in its own review:

```bash
ros2 launch rover_slam slam_bringup.launch.py use_sim_time:=true \
  odom_topic:=/odom launch_encoder:=false launch_rviz:=false \
  launch_static_tf:=false launch_camera:=false launch_costmap:=false \
  database_path:="/tmp/perception-live/results/rtabmap_$(date +%Y%m%d_%H%M%S).db"
```

Check `odom -> base_link` and `map -> base_link` using `tf2_echo` before enabling
the corresponding consumers. Stop B and restart combined Perception with **one
additional switch at a time**, checking each boundary before combining them:

| Additional switch | Check / acceptance evidence |
| --- | --- |
| `launch_landmark_adapter:=true` | Inspect `/perception/landmark_detections`; IDs, observation timestamps/frame and covariance must match accepted tracked observations. Positive marker ID offset is 1 by default. Rejection-only traffic does not prove positive SLAM ingestion or map improvement. |
| `launch_memory:=true` | `/perception/obstacles_only` has one publisher, memory; local boxes remain current observations. Check odom-frame retention, expiry and loss/recovery. Default ground clearing is disabled. |
| `launch_marker_mapping:=true` | Inspect `/marker_map` and `/marker_map/visualization`; a stationary marker's global pose should remain consistent while the rover moves. Numeric tolerances and localization quality need independent evidence. |
| `enable_terrain_risk:=true` | Deferred until gravity reference and measured positive `max_slope_deg`, `max_step_m`, `max_roughness_m` are agreed. Zero defaults intentionally do not define a working profile. |

For repeatable input-loss checks, use the separate
[controlled fixture profile](TESTING_GUIDE.md#controlled-production-message-flow)
and its `invalid_cloud`, `missing_depth` and `dropout` scenarios, after stopping
the live Perception profile. Keep its test-only publishers isolated from real
inputs. Pausing Gazebo also pauses `/clock`; it is not a sensor-dropout test and
ROS-time diagnostic timers can stop with it.

Run the production software regression suite separately in a sourced terminal:

```bash
python3 -m pytest /workspace/Perception/perception_bringup/test \
  /workspace/Perception/terrain_geometry/test \
  /workspace/Perception/marker_detection/test -q -p no:cacheprovider
```

The optional feeder-launch check can skip if `perception_test_tools` is not built
in this live profile. Test results do not replace the observed geometry checks.

Stop recording and send zero velocity before closing teleoperation. Stop
Perception/RViz, optional SLAM, and finally A with Ctrl+C. Stop only this run's
processes; never use broad `pkill` commands. No ground-truth trajectory/map
accuracy or rover traversability pass is implied by these startup/visual checks.
