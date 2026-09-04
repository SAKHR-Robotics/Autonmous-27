# GitHub Issues / Task Breakdown: SLAM Subsystem (`rover_slam`)

This document contains GitHub-ready issues for the SLAM subsystem. Each section is formatted to be copied and pasted directly into a new GitHub Issue or GitHub Projects card.

---

## Issue 1: Fix Divergent IMU Accelerometer Integration in Heuristic Slip Checker

### Description
In `heuristic_slip_checker.py` (line 117), linear velocity is estimated by open-loop integration of raw IMU linear acceleration (`self.imu_integrated_vx += ax * dt`). Because uncalibrated MEMS accelerometer readings contain sensor bias and tilt-induced gravity leakage, this integral drifts rapidly (exceeding +/- 100 m/s in seconds). This causes `slip_detected = True` to permanently trigger, forcing the node to inflate wheel covariance by 100x indefinitely and causing EKF to completely ignore valid wheel odometry.

### Approach Suggested to Solve
1. Remove open-loop accelerometer integration (`self.imu_integrated_vx += ax * dt`).
2. Implement kinematic slip detection based on **Yaw-Rate (Gyroscope) Consistency**:
   - Compute wheel-derived angular velocity:
     ```text
     ω_wheel = (v_right - v_left) / track_width
     ```
   - Compare `ω_wheel` with calibrated IMU angular velocity `ω_imu,z` (from `/imu/data`).
   - Flag slip when:
     ```text
     |ω_wheel - ω_imu,z| > slip_angular_threshold (e.g., > 0.20 rad/s)
     ```
3. Implement transient linear acceleration comparison:
   - Compute wheel acceleration `a_wheel = (v_wheel_t - v_wheel_prev) / dt`.
   - Compare with instantaneous IMU `a_x` (without integrating over time).

### Acceptance Criteria
- [ ] No unbounded numerical accumulation or integral state in `heuristic_slip_checker.py`.
- [ ] `/wheel/slip_detected` stays `false` during nominal driving on flat/firm ground.
- [ ] `/wheel/slip_detected` switches to `true` when wheels spin without matching IMU gyro/acceleration response.
- [ ] Covariance inflation scales back down to nominal values once slip ceases.

### How to Validate & Commands Needed
1. Launch the slip checker node:
   ```bash
   ros2 run rover_slam heuristic_slip_checker
   ```
2. In a second terminal, monitor the slip flag and filtered odometry:
   ```bash
   ros2 topic echo /wheel/slip_detected
   ros2 topic echo /wheel/odom_filtered
   ```
3. Publish stationary IMU and odometry data and verify that `/wheel/slip_detected` remains `false` over several minutes without drifting:
   ```bash
   ros2 topic pub -r 50 /imu/data sensor_msgs/msg/Imu "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'imu_link'}, angular_velocity: {x: 0.0, y: 0.0, z: 0.0}, linear_acceleration: {x: 0.0, y: 0.0, z: 9.81}}"
   ros2 topic pub -r 50 /wheel/odom_raw nav_msgs/msg/Odometry "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'odom'}, child_frame_id: 'base_link', twist: {twist: {linear: {x: 0.0}, angular: {z: 0.0}}}}"
   ```

---

## Issue 2: Route Filtered Wheel Odometry to EKF (Resolve Bypass Bug)

### Description
`heuristic_slip_checker.py` publishes filtered odometry with dynamically adjusted covariance to `/wheel/odom_filtered`. However, `config/ekf.yaml` and `launch/ekf.launch.py` currently configure `robot_localization` (`ekf_filter_node`) to subscribe directly to `/wheel/odom_raw`. As a result, the slip checker's output is completely bypassed and EKF never receives the inflated covariance during wheel slippage.

### Approach Suggested to Solve
1. Update `config/ekf.yaml`:
   - Change `odom0: /wheel/odom_raw` to `odom0: /wheel/odom_filtered`.
2. Update `launch/ekf.launch.py` and `launch/slam_bringup.launch.py`:
   - Ensure `encoder_ticks_to_odom` outputs `/wheel/odom_raw`.
   - Ensure `heuristic_slip_checker` subscribes to `/wheel/odom_raw` and outputs `/wheel/odom_filtered`.
   - Ensure `ekf_node` consumes `/wheel/odom_filtered`.
   - Ensure launch arguments (`odom_topic`) correctly remap inputs when playing back rosbag or Gazebo data.

### Acceptance Criteria
- [ ] `ros2 node info /ekf_filter_node` shows an active subscription to `/wheel/odom_filtered`.
- [ ] When slip occurs, the inflated covariance in `/wheel/odom_filtered` is directly consumed by `ekf_filter_node`.
- [ ] No unmapped topics or broken publisher-subscriber links in `rqt_graph`.

### How to Validate & Commands Needed
1. Launch EKF and pre-processors:
   ```bash
   ros2 launch rover_slam ekf.launch.py
   ```
2. Verify node subscriptions and graph:
   ```bash
   ros2 node info /ekf_filter_node
   rqt_graph
   ```
3. Echo the filtered state output to confirm fusion:
   ```bash
   ros2 topic echo /odometry/filtered
   ```

---

## Issue 3: Resolve Duplicate / Colliding Static TF Broadcasters in SLAM Launch

### Description
`launch/static_transforms.launch.py` contains static transform publishers for `base_link -> camera_link` and `base_link -> imu_link`. However, the robot description (`my_robot.urdf.xacro`) and `robot_state_publisher` already publish these exact same fixed joint transforms on `/tf_static`. Running both concurrently produces duplicate TF authority on `/tf_static`, resulting in frame jitter, transform lookup errors, and navigation instability.

### Approach Suggested to Solve
1. Make `robot_state_publisher` (URDF) the single source of truth for robot sensor geometry.
2. Remove redundant `base_link -> camera_link` and `base_link -> imu_link` broadcasters from `launch/static_transforms.launch.py`.
3. Add a condition or standalone flag to `static_transforms.launch.py` so static transforms are only published if `robot_state_publisher` is NOT running (e.g., during isolated sensor testing).

### Acceptance Criteria
- [ ] `ros2 run tf2_tools view_frames` produces a clean tree with no duplicate frames or conflicting publisher warnings.
- [ ] Zero TF warnings in RViz2 when `slam_bringup.launch.py` and the robot description are launched together.

### How to Validate & Commands Needed
1. Launch the robot description and SLAM stack:
   ```bash
   ros2 launch rover_slam slam_bringup.launch.py
   ```
2. Generate and inspect the TF frame tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```
3. Echo TF transform to ensure stable publishing rate:
   ```bash
   ros2 run tf2_ros tf2_echo base_link camera_link
   ros2 run tf2_ros tf2_echo base_link imu_link
   ```

---

## Issue 4: Standardize Camera Optical Frame Conventions (ROS REP-103)

### Description
According to ROS REP-103 and RealSense / OpenCV standards, optical frames must follow:
- **+X**: Right
- **+Y**: Down
- **+Z**: Forward (along optical axis)

Currently, the robot URDF names the camera child frame `my_robot/camera_link/camera`, while RTAB-Map, `vision_helper`, and `marker_detection` expect `camera_depth_optical_frame` / `camera_color_optical_frame`. This discrepancy leads to inverted or rotated point clouds in RTAB-Map and incorrect marker coordinates.

### Approach Suggested to Solve
1. Update `Rover/my_robot_description/urdf/my_robot.urdf.xacro`:
   - Rename the camera optical frame to `camera_depth_optical_frame` and/or `camera_color_optical_frame`.
   - Apply the standard optical rotation: `rpy="-1.570796 0 -1.570796"`.
2. Ensure `rtabmap.yaml` and launch files consistently reference the updated frame ID.

### Acceptance Criteria
- [ ] Camera optical link matches standard frame naming `camera_depth_optical_frame`.
- [ ] Depth point clouds in RViz2 align perfectly with physical rover orientation (+X forward, +Y left, +Z up in `base_link`).

### How to Validate & Commands Needed
1. Launch display or Gazebo:
   ```bash
   ros2 launch my_robot_description display.launch.py
   ```
2. Verify transform rotation between `camera_link` and `camera_depth_optical_frame`:
   ```bash
   ros2 run tf2_ros tf2_echo camera_link camera_depth_optical_frame
   ```
   *Expected Translation: [0, 0, 0], Rotation (RPY): [-1.571, 0, -1.571]*

---

## Issue 5: Connect RealSense Filtered Depth Stream to RTAB-Map

### Description
`launch/vision_helper.launch.py` configures spatial, temporal, decimation, and hole-filling filters for the RealSense D435 to eliminate sunlight and IR noise, publishing to `/camera/depth/filtered`. However, in `launch/slam_bringup.launch.py` and `launch/rtabmap.launch.py`, RTAB-Map is passed `/camera/depth/image_raw` directly by default. The depth post-processing filter is therefore bypassed.

### Approach Suggested to Solve
1. In `launch/slam_bringup.launch.py`:
   - Configure RTAB-Map's `depth_topic` parameter to default to `/camera/depth/filtered` when `launch_camera` is true.
   - Set fallback to `/camera/depth/image_raw` only in simulation mode.
2. In `launch/rtabmap.launch.py`:
   - Ensure the remapping aligns with `/camera/depth/filtered`.

### Acceptance Criteria
- [ ] When launching `slam_bringup.launch.py launch_camera:=true`, RTAB-Map subscribes to `/camera/depth/filtered`.
- [ ] RealSense filter parameters in `config/realsense_filters.yaml` take effect on the depth map consumed by SLAM.

### How to Validate & Commands Needed
1. Launch the complete SLAM stack with camera enabled:
   ```bash
   ros2 launch rover_slam slam_bringup.launch.py launch_camera:=true
   ```
2. Verify RTAB-Map subscriptions:
   ```bash
   ros2 node info /rtabmap
   ```
   *Verify `depth/image` is mapped to `/camera/depth/filtered`.*

---

## Issue 6: Configure EKF for 3D State Estimation (Disable 2D Mode for Terrain Slopes)

### Description
In `config/ekf.yaml`, `two_d_mode: true` is currently enabled. On Mars / ERC competition grounds, the rover navigates ramps, craters, and 3D terrain slopes. 2D mode forces Z = 0, Roll = 0, and Pitch = 0, which causes state estimation to diverge when the rover drives over inclined surfaces.

### Approach Suggested to Solve
1. In `config/ekf.yaml`:
   - Change `two_d_mode: false`.
2. Update `imu0_config` to fuse:
   - Roll & Pitch orientation: `[false, false, false, true, true, true, ...]`
   - Angular velocity (X, Y, Z): `[..., true, true, true, ...]`
   - Linear acceleration (X, Y): `[..., true, true, false]`
3. Ensure `imu0_remove_gravitational_acceleration: true` is active.

### Acceptance Criteria
- [ ] EKF publishes full 3D poses (Roll, Pitch, Yaw, X, Y, Z) in `/odometry/filtered`.
- [ ] Tilting the IMU / rover reflects accurate pitch and roll changes in `/odometry/filtered` without being clamped to zero.

### How to Validate & Commands Needed
1. Launch EKF node:
   ```bash
   ros2 launch rover_slam ekf.launch.py
   ```
2. Monitor `/odometry/filtered` orientation while simulating non-zero pitch/roll on the IMU:
   ```bash
   ros2 topic echo /odometry/filtered --field pose.pose.orientation
   ```

---

## Issue 7: Unify Nav2 Costmap Ownership Between SLAM and PathPlanning

### Description
`rover_slam/launch/costmap.launch.py` launches a standalone `nav2_costmap_2d` and `nav2_lifecycle_manager` node. At the same time, `PathPlanning/erc_path_planner/launch/path_planning.launch.py` launches the complete Nav2 bringup stack containing its own global and local costmap servers. Running both packages simultaneously leads to duplicate costmaps, lifecycle conflicts, and namespace collisions.

### Approach Suggested to Solve
1. Establish clear responsibility boundaries:
   - `rover_slam` produces the global static map (`/map`) and localized odometry (`/odometry/filtered` + TF `map -> odom -> base_link`).
   - `PathPlanning/erc_path_planner` manages the active Nav2 global and local costmaps.
2. In `rover_slam/launch/slam_bringup.launch.py`:
   - Add an `enable_standalone_costmap` flag (default: `false`).
   - Only launch `costmap.launch.py` if running isolated SLAM tests without PathPlanning.

### Acceptance Criteria
- [ ] No lifecycle node collisions when launching both `rover_slam` and `erc_path_planner`.
- [ ] `erc_path_planner` receives `/map` from RTAB-Map and successfully populates the static layer in `global_costmap`.

### How to Validate & Commands Needed
1. Launch SLAM bringup and Path Planning simultaneously:
   ```bash
   ros2 launch rover_slam slam_bringup.launch.py
   ros2 launch erc_path_planner path_planning.launch.py
   ```
2. Verify that Nav2 lifecycle nodes transition smoothly to ACTIVE state:
   ```bash
   ros2 lifecycle get /global_costmap/global_costmap
   ros2 lifecycle get /local_costmap/local_costmap
   ```

---

## Issue 8: Bridge ArUco Landmark Perception to RTAB-Map Graph Optimization

### Description
RTAB-Map supports global graph optimization and drift elimination using landmark constraints published as `geometry_msgs/msg/PoseStamped` on topic `/perception/aruco_pose`. The `Perception/marker_detection` package detects and tracks ArUco markers and publishes `/marker_detection/targets` (`MarkerActionTargetArray`). An adapter/remapping is needed to feed these detections as standard `PoseStamped` landmarks to RTAB-Map.

### Approach Suggested to Solve
1. In `Perception/marker_detection` (or `rover_slam`):
   - Publish confirmed marker poses as `geometry_msgs/msg/PoseStamped` on `/perception/aruco_pose`.
   - Gate distance at <= 3.5m and require >= 3 consecutive confirmed frames to prevent noisy jumps.
2. Configure RTAB-Map in `rover_slam/launch/rtabmap.launch.py` to ingest `/perception/aruco_pose` for landmark graph optimization.

### Acceptance Criteria
- [ ] Valid `PoseStamped` messages are published on `/perception/aruco_pose` whenever an ArUco marker is confirmed.
- [ ] RTAB-Map logs loop-closure / landmark constraints when observing known markers.

### How to Validate & Commands Needed
1. Launch marker detection:
   ```bash
   ros2 launch marker_detection marker_detection.launch.py
   ```
2. Echo the landmark topic to verify output:
   ```bash
   ros2 topic echo /perception/aruco_pose
   ros2 topic hz /perception/aruco_pose
   ```
