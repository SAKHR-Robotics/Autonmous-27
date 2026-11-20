> Mock implementations now live in `testing/mocks/slam/slam_mock_tools`.
> The executable names below remain compatibility adapters to that package.

# slam_test_tools — existing SLAM helpers

These are **testing-only helpers**, not a completed tester/referee/benchmark.
The production module guide contains the full terminal sequence:
[SLAM running guide](../../../../SLAM/rover_slam/README.md).

## Real local estimator with selected mocked dependencies

The coordinated `local_mocked.launch.py` runs the real encoder, slip filter and
EKF against the shared mock rover plus a SLAM-owned missing-sensor adapter.
It uses wall time, no artificial localization TF, no Perception and no RTAB-Map.
Mock rover odometry is isolated on `/testing/slam/truth`; only the real EKF
publishes `/odometry/filtered`. Geometry is explicitly 0.06 m radius / 0.49 m
track with zero noise/slip. Production launch defaults remain unchanged.

Use the [isolated Docker commands and current results](../../../../SLAM/VALIDATION.md).
Build the shared mock package and `terrain_geometry_msgs` as well as the SLAM
packages before using this profile. In separate sourced container terminals:

```bash
# A: real local estimation plus explicit upstream dependencies.
ros2 launch slam_test_tools local_mocked.launch.py
# Optional tick/mount/gravity checks:
ros2 launch slam_test_tools local_mocked.launch.py wheel_output_mode:=ticks_array imu_mount_yaw_rad:=1.5708
```

```bash
# B: bounded stationary, straight, turn, IMU loss/recovery and wheel loss/recovery.
ros2 run slam_test_tools local_estimator_probe --output /validation/local-run.json
```

The observer checks eight truth/input-loss/recovery contracts and records raw
samples plus timestamp-matched position/yaw errors. It exits nonzero for a
contract failure. Its accuracy verdict remains explicitly unestablished;
successful exit is **not** localization accuracy approval. With the unchanged
EKF profile, the controlled turn still shows approximately 0.326 m maximum
error. Scratch forward-acceleration-only/no-acceleration profiles reduce this
to approximately 0.0028 m, but no fusion default has been changed automatically.
The optional `ekf_params_file` argument permits that controlled comparison.
All settled recovery diagnostics must be healthy; a single healthy sample is
insufficient. Missing wheels must stop both raw and filtered wheel output.

The existing production local RViz can observe this real estimator:

```bash
ros2 launch rover_slam rviz.launch.py use_sim_time:=false rviz_view:=local \
  raw_odom_topic:=/wheel/odom_raw
```

Stop the observer first if interrupted, then stop launch A with one Ctrl+C.
The observer sends zero command on exit; the shared rover also has a command
timeout. Stop only run-owned processes. This fixture shares a kinematic model
between its sensors and truth, so it is not independent physical validation.

## Build and run with Gazebo

Build `rover_slam` and `slam_test_tools` and source the overlay as described in
that guide. Start the existing Gazebo rover with its odom TF bridge disabled,
then run:

```bash
ros2 launch slam_test_tools test_slam_standalone.launch.py \
  use_sim_time:=true odom_topic:=/odom launch_encoder:=false launch_rviz:=true
```

This starts **real EKF + RTAB-Map**. Gazebo must supply RGB, aligned depth,
CameraInfo, wheel odometry, IMU, sensor TF and `/clock`. It does not launch a
world, simulate sensors or publish estimated dynamic TF. Both mocks are off.
Use `launch_rtabmap:=false rviz_view:=local` for local-estimator-only checks.
Use a fresh absolute `database_path` for each experiment; deletion defaults off.

## Observe or start a local EKF validation run

Attach to an already-running EKF:

```bash
ros2 run slam_test_tools ekf_validator --ros-args \
  -p use_sim_time:=true -p expected_frequency_hz:=30.0
```

Or deliberately start EKF plus its monitor, without another EKF running:

```bash
ros2 launch slam_test_tools ekf_validation.launch.py \
  use_sim_time:=true odom_topic:=/odom launch_encoder:=false
```

`/ekf/validation_status` is false while waiting for cadence, after a dropout,
or for invalid/stale/unrealistic data. `/ekf/validation_frequency_hz` reports
measured cadence. This monitor has no ground truth and cannot judge accuracy.
The old `ros2 launch rover_slam ekf_validator.launch.py` forwards explicitly to
this testing launch. The old production-package executable was relocated;
use `ros2 run slam_test_tools ekf_validator`.

## Explicit existing mocks

```bash
ros2 run slam_test_tools mock_aruco_publisher --ros-args -p use_sim_time:=true
ros2 run slam_test_tools costmap_test_stub --ros-args -p use_sim_time:=true
```

The first publishes native `rtabmap_msgs/msg/LandmarkDetection` on
`/perception/landmark_detection`, ID 1. The second publishes PointCloud2 on
`/bridge/pointcloud` and markers on `/terrain/obstacle_markers`, never on the
Detection3DArray interface `/perception/obstacles_only`. Wrapper switches are
`launch_aruco_stub:=true` and `launch_costmap_stub:=true` respectively.

These are fixed sensor-relative smoke fixtures, **not coherent moving-scene
truth**. Do not use them to claim trajectory accuracy, loop closure or map
quality. Never run beside real Perception/the actual bridge producing those
outputs, or on attached rover hardware. They supply neither complete RGB-D
SLAM inputs nor artificial `map -> odom` / `odom -> base_link` localization.
The legacy PoseStamped fixture in `testing/mocks/slam/slam_mock_tools/reference/` is not installed.

## Regression checks and future work

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest SLAM/rover_slam/test testing/benchmarking/slam/slam_test_tools/test \
  testing/mocks/slam/slam_mock_tools/test -q -p no:cacheprovider
```

Use an isolated `ROS_DOMAIN_ID`. Pure tests cover wheel timing, discontinuities,
slip thresholds, covariance and mount rotation. ROS contract checks require
an installed/sourced overlay; missing dependencies are explicit skips.

Moving RGB-D feeders, full accuracy referees, mission benchmark runners and
broader scene truth tooling remain later testing work. The local nominal
adapter/observer above does not replace them.
SLAM requirements and the pending TestingPlan.md reconciliation are recorded in
[SLAMImprovments.md](../../../../SLAM/SLAMImprovments.md). Historical test design
files elsewhere in this folder are proposals, not runnable implementation.
