# slam_test_tools — existing SLAM helpers

These are **testing-only helpers**, not a completed tester/referee/benchmark.
The production module guide contains the full terminal sequence:
[SLAM running guide](../../../SLAM/rover_slam/README.md).

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
The legacy PoseStamped fixture in `reference/` is not installed.

## Regression checks and future work

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest SLAM/rover_slam/test testing/SLAM/slam_test_tools/test -q
```

Use an isolated `ROS_DOMAIN_ID`. Pure tests cover wheel timing, discontinuities,
slip thresholds, covariance and mount rotation. ROS contract checks require
an installed/sourced overlay; missing dependencies are explicit skips.

New coherent sensor feeders, dedicated accuracy referees, benchmark runners
and scene truth adapters remain the separate testing project's responsibility.
SLAM requirements and the pending TestingPlan.md reconciliation are recorded in
[SLAMImprovments.md](../../../SLAM/SLAMImprovments.md). Historical test design
files elsewhere in this folder are proposals, not runnable implementation.
