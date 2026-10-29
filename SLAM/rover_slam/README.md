# rover_slam running guide

Target: **Linux, Ubuntu 24.04, ROS 2 Jazzy**. Run commands from the workspace
root (`Autonmous-27`), not from this package directory. Source ROS and the
workspace overlay in every terminal. See [the work record](../SLAMImprovments.md)
for verification evidence and pending checks.

## 1. Build and understand the inputs

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths SLAM/rover_slam testing/SLAM/slam_test_tools \
  --ignore-src -r -y
colcon build --symlink-install \
  --base-paths SLAM/rover_slam testing/SLAM/slam_test_tools \
  --packages-select rover_slam slam_test_tools
source install/setup.bash
```

Gazebo workflows also require the existing `my_robot_description`, world assets
and ros_gz installation. Perception/navigation workflows require their packages
already built and sourced. The minimal command above does not build them.

| Input | Type / requirement |
| --- | --- |
| `/odom` (Gazebo) or `/wheel/odom_raw` (encoder converter/driver) | `nav_msgs/msg/Odometry`; select exactly one source |
| `/imu/data` | `sensor_msgs/msg/Imu`; stamped, valid orientation/rates/acceleration, known sensor TF |
| `/camera/image_raw`, `/camera/depth/image_raw` | `sensor_msgs/msg/Image`; depth aligned to RGB; supported metric depth encoding |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo`; matches RGB/depth geometry |
| Optional `/perception/landmark_detections` | `rtabmap_msgs/msg/LandmarkDetections`; real Perception adapter, camera-relative observations |

Outputs: `/wheel/odom_filtered`, `/wheel/slip_detected`, `/odometry/filtered`,
`/map`, RTAB-Map graph/path/status, and estimated TF. Encoder mode additionally
publishes `/wheel/odom_raw`, `/wheel/per_wheel_speeds`, `/wheel/single_wheel_slip`.
Health topics are `/slam/encoder_diagnostics` (when converter runs) and
`/slam/slip_diagnostics`; robot_localization also emits diagnostics.

**TF ownership:** EKF owns `odom -> base_link`; RTAB-Map owns `map -> odom`.
Robot description or physical sensor driver owns calibrated sensor frames.
The current Gazebo launch adds optical frames and already supplies
`my_robot/camera_link/camera` through robot_state_publisher. Leave all SLAM
fallback static TF switches off for that launch. Never add a static map->odom
while RTAB-Map runs. One `/clock` bridge serves every simulated module.

## 2. Run SLAM alone with Gazebo and testing helpers

These terminals run real SLAM without running real Perception or navigation.
Gazebo supplies its sensors and wheel odometry; a landmark is optional.
A textured RGB-D scene is necessary for meaningful mapping. A constant fake
marker or the stationary Perception image fixture cannot validate moving SLAM.

**Terminal 1 — existing simulation and real simulated sensors:**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_robot_description gazebo.launch.py \
  bridge_sim_tf:=false publish_map_tf:=false publish_camera_tf:=false
```

**Terminal 2 — real SLAM through the top-level testing package:**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Use a new path for a fresh experiment; no database is deleted.
ros2 launch slam_test_tools test_slam_standalone.launch.py \
  use_sim_time:=true odom_topic:=/odom launch_encoder:=false \
  database_path:="$HOME/.ros/slam_run_$(date +%Y%m%d_%H%M%S).db" \
  launch_rviz:=true
```

This wrapper starts the production master and global RViz. Both mocks are
**off**. It requires the external simulator; it is not a complete synthetic
SLAM tester. For EKF-only observation, add `launch_rtabmap:=false rviz_view:=local`.
Without testing orchestration, run the production equivalent directly:

```bash
ros2 launch rover_slam slam_bringup.launch.py use_sim_time:=true \
  odom_topic:=/odom launch_encoder:=false launch_rviz:=true
```

**Terminal 3 — optional teleoperation:**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run my_robot_description teleop_gui.py
```

**Terminal 4 — attach the existing test-only health monitor:**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run slam_test_tools ekf_validator --ros-args \
  -p use_sim_time:=true -p expected_frequency_hz:=30.0
```

The monitor observes the already-running EKF. Do not start a second EKF using
`ekf_validation.launch.py` in this same run. Its status checks freshness,
finite values, cadence and speed bounds; it is not an accuracy referee.

What to observe: filtered odometry at approximately **30 Hz**, continuous local
TF, global TF and a growing map after valid images arrive. Map publication is
subscriber/update dependent; it need not run at the EKF frequency.

```bash
ros2 topic hz /odometry/filtered
ros2 topic echo /slam/slip_diagnostics --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link my_robot/camera_link/camera
ros2 run tf2_ros tf2_echo map odom
ros2 topic info /map --verbose
```

For initial input diagnosis, inspect `/imu/data` and camera headers, their
publisher QoS, and TF at those timestamps. A published map/odom message alone
does not prove accurate localization. Detailed testers/referees/benchmarks are
owned by the separate testing work; see [testing package guide](../../testing/SLAM/slam_test_tools/README.md).

## 3. Run independent stages or physical sensors

Run each row in a separate terminal only when you need that stage. A standalone
stage does not manufacture its upstream dependencies.

| Stage | Command and external inputs |
| --- | --- |
| Encoder converter | `ros2 launch rover_slam encoder.launch.py input_mode:=joint_states`; needs wheel JointState positions. Use `input_mode:=ticks_array` or `individual_ticks` for hardware counts. |
| Slip covariance filter | `ros2 launch rover_slam slip.launch.py odom_topic:=/wheel/odom_raw`; needs stamped wheel odometry, `/imu/data` and sensor TF. |
| Local EKF with preprocessing | `ros2 launch rover_slam ekf.launch.py launch_encoder:=true encoder_input_mode:=ticks_array odom_topic:=/wheel/odom_raw`; needs ticks, IMU and TF. With an external odometry driver, leave `launch_encoder:=false`. |
| RTAB-Map only | `ros2 launch rover_slam rtabmap.launch.py`; needs `/odometry/filtered`, RGB-D, calibration and TF. |
| Visualization | `ros2 launch rover_slam rviz.launch.py rviz_view:=local` or `rviz_view:=global`; local uses `odom`, global uses `map`. |

For EKF without the slip filter, add `launch_slip:=false`; `odom_topic` then
feeds EKF directly. `ekf_validation.launch.py` in **slam_test_tools** deliberately
starts an EKF plus its monitor; use it only when no other EKF runs.

**Hardware preparation:** measure wheel radius, effective track width, encoder
counts per revolution, wheel order/signs, IMU mounting, gravity convention and
camera extrinsics. Edit an external copy of `config/encoder.yaml` and
`config/slip.yaml`, and supply their absolute paths. Defaults match simulation,
not a calibrated physical rover. Counter overflow/reset appears as a rejected
speed discontinuity and a new baseline; no particular counter width is assumed.

```bash
# Driver/URDF already provide sensors, mounts and any physical camera.
ros2 launch rover_slam slam_bringup.launch.py use_sim_time:=false \
  launch_encoder:=true encoder_input_mode:=ticks_array odom_topic:=/wheel/odom_raw \
  encoder_params_file:=/absolute/path/encoder.yaml \
  slip_params_file:=/absolute/path/slip.yaml launch_rviz:=true
```

One physical RealSense can serve Perception and SLAM. To start it independently:

```bash
ros2 launch rover_slam vision_helper.launch.py
```

Or add `launch_camera:=true` to the master, **not both**. The helper normalizes
color, aligned depth and point-cloud topics to the interfaces above. Check the
actual topic list and supported stream profiles on the installed driver/device.
D435 has no IMU: this helper does not supply `/imu/data`; an external calibrated
IMU is required. Its raw camera TF still needs a measured base-to-camera mount.
Do not enable generic SLAM mount/optical fallbacks alongside the driver's TF.

In `slip.yaml`, `imu_includes_gravity: true` assumes standard specific force.
Set false only for a verified gravity-free stream. Keep EKF's
`imu0_remove_gravitational_acceleration` consistent in your EKF profile.
A missing/stale IMU inflates wheel covariance and reports degraded operation;
it cannot establish slip or stationary ground truth.

## 4. Run with real Perception and navigation

Use one sourced terminal per command. Start Gazebo as in section 2, or start
physical drivers/robot description. Use the same clock setting everywhere.
For Gazebo, the production module sequence is:

```bash
# Terminal A: SLAM; Nav2 will own navigation costmaps.
ros2 launch rover_slam slam_bringup.launch.py use_sim_time:=true \
  odom_topic:=/odom launch_encoder:=false launch_costmap:=false launch_rviz:=true

# Terminal B: real Perception; enable its native RTAB-Map adapter explicitly.
ros2 launch terrain_geometry perception_system.launch.py use_sim_time:=true \
  launch_landmark_adapter:=true launch_marker_mapping:=true

# Terminal C: existing navigation entry point; starts the obstacle bridge too.
ros2 launch erc_path_planner path_planning.launch.py use_sim_time:=true use_slam:=true
```

These are the existing module entry points, not a claim of a validated full
rover mission. Global marker mapping is optional and needs SLAM TF; leave it
false if you only need native landmark observations. The adapter retains its
quality gates and positive ID offset. RTAB-Map must not be configured to feed
estimated global marker positions back as independent observations.

If running **SLAM + Perception without navigation**, inspect obstacles by adding
`launch_costmap:=true` to SLAM and running the actual bridge separately:

```bash
ros2 launch erc_path_planner obstacle_bridge.launch.py use_sim_time:=true
```

Do not start this extra bridge when the navigation launch already owns it.
The optional SLAM costmap is a fixed 40 m obstacle inspection view in `map`,
not a replacement for Nav2's global/local navigation maps. It has no static map
layer. Box-derived points mark obstacles only; measured camera rays clear.
Empty box arrays or memory expiry alone do not guarantee old marks disappear.

Optional `launch_memory` in Perception needs fresh local `odom` TF. Keep
`enable_terrain_risk:=false`: planar EKF does not supply measured physical
roll/pitch for gravity-based slope assessment. Calibrate/validate that separately.
Do not run testing mocks alongside their real output publishers.

## 5. Arguments, troubleshooting and checks

```bash
ros2 launch rover_slam slam_bringup.launch.py --show-args
bash scripts/launch_slam.sh
bash scripts/launch_slam.sh --mode test --use-sim-time true -- launch_rviz:=false
bash scripts/launch_slam.sh --node ticks -- input_mode:=ticks_array
```

Master defaults: EKF and RTAB-Map on; slip filter on; encoder, camera, fallback
static TF, costmap and RViz off. Master `odom_topic` defaults to `/odom` for the
existing simulation; standalone `ekf.launch.py` defaults to `/wheel/odom_raw`.
When enabling the converter, select `/wheel/odom_raw` to prevent competing input.

Profiles: `encoder_params_file`, `slip_params_file`, `ekf_params_file`,
`rtabmap_params_file`, `camera_params_file`, `costmap_params_file`. Independent
encoder/slip/RTAB-Map/camera/costmap launches use `params_file`. Explicit launch
clock/topic/frame selections override profiles. Common frames are `base_frame`,
`odom_frame`, `map_frame`; override all dependent external modules consistently.
RViz accepts `rviz_view`, optional `rviz_config` and master `rviz_fixed_frame`.

Database: `database_path` accepts an absolute path or `~/...`; its directory
must exist and be writable. `delete_db_on_start:=false` preserves it. Use a new
filename per experiment. Only explicitly set true when that database is
expendable. The master forwards this flag to RTAB-Map.

| Symptom | Check / correction |
| --- | --- |
| No raw wheel odometry | Select one input mode; verify every configured wheel, freshness, units and realistic speed bounds. Output is measurement-driven, not a fabricated 50 Hz heartbeat. |
| Degraded slip diagnostics | Verify IMU timestamps, orientation/gravity setting and transform into the odometry child frame. Missing data is not a healthy zero-speed reading. |
| No map or global TF | First verify RGB-D synchronization, calibration, EKF output, sensor TF and RTAB-Map logs. An empty textureless scene or constant marker mock is insufficient. |
| Duplicate/jumping TF | Disable simulator odom TF bridging and static map anchors. Current Gazebo URDF already supplies optical sensor frames. SLAM fallback switches are only for externally supplied incomplete descriptions. |
| RViz blank or wrong frame | Use local view before global mapping initializes; enable optional displays only when their publishers exist. GUI rendering remains a separate runtime check. |

Pure/static checks and ROS-dependent regressions:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest SLAM/rover_slam/test testing/SLAM/slam_test_tools/test -q
bash -n scripts/launch_slam.sh
```

Run in isolated ROS_DOMAIN_ID with no rover hardware attached. Full moving
RGB-D, GUI, RTAB-Map optimization, Nav2 clearing, outdoor timing and physical
calibration checks are pending unless explicitly recorded in the work record.
A boolean monitor or passing mathematical tests do not certify autonomous
operation; the controller still needs its own freshness/stop policy.
