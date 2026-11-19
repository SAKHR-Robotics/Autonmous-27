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
