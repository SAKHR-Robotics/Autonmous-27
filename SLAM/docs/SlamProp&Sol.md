# SLAM Subsystem: Problems, Root Causes & Actionable Solutions

This document outlines every identified issue in the current SLAM subsystem (`rover_slam`), its technical root cause, and the step-by-step solution to ensure full functionality and seamless integration with the 4-wheel rover, Perception, and PathPlanning.

---

## Summary Matrix

| # | Problem | Severity | Impact | Primary File(s) |
| :--- | :--- | :--- | :--- | :--- |
| **1** | **Slip Checker Accel Integration Divergence** | Critical | Infinite covariance inflation / EKF ignores wheels | `heuristic_slip_checker.py` |
| **2** | **EKF Bypasses Filtered Odometry** | High | Slip detection output never used by EKF | `ekf.yaml`, `ekf.launch.py` |
| **3** | **Conflicting / Duplicate Static TF Broadcasters** | High | TF tree jitter & transform lookup failures | `static_transforms.launch.py`, URDF |
| **4** | **Camera Optical Frame Naming & Conventions** | Medium | RTAB-Map & perception pointcloud alignment errors | URDF, `slam_bringup.launch.py` |
| **5** | **RealSense Filtered Depth Stream Disconnect** | Medium | RTAB-Map uses raw noisy depth instead of filtered stream | `slam_bringup.launch.py`, `rtabmap.launch.py` |
| **6** | **EKF 2D Mode Clamping on 3D Terrain** | Medium | State estimation fails on slopes, ramps, and bumps | `config/ekf.yaml` |
| **7** | **Redundant Standalone Costmap Lifecycle Collision** | Medium | Conflicts with PathPlanning Nav2 bringup stack | `costmap.launch.py`, `nav2_params.yaml` |
| **8** | **Perception Topics & ArUco Landmark Interface Gap** | Medium | RTAB-Map cannot integrate ArUco loop closures | `rtabmap.launch.py`, `marker_detection` |

---

## Detailed Problems & Solutions

### 1. Slip Checker Accel Integration Divergence
* **Problem**: The `heuristic_slip_checker` rapidly enters a permanent `slip_detected = True` state within seconds of launching, inflating covariance by 100x and forcing EKF to permanently disregard wheel odometry.
* **Root Cause**: In `heuristic_slip_checker.py` (line 117), linear velocity is estimated by integrating raw IMU accelerometer readings:
  $$\hat{v}_{imu} = \int a_x \, dt$$
  Raw MEMS accelerometer bias, tilt gravity leakage, and noise accumulate linearly over time without bound.
* **Solution**:
  1. Remove open-loop accelerometer double integration.
  2. Detect slip using robust physical indicators:
     - **Angular Velocity / Gyro Consistency**: Compare wheel-derived yaw rate $\omega_{wheel} = \frac{v_r - v_l}{L}$ against IMU Gyro $\omega_{imu, z}$.
     - **Kinematic Consistency**: Flag slip if $|\omega_{wheel} - \omega_{imu, z}| > \text{threshold}$.
     - **Acceleration Derivative / Saturation**: Compare wheel acceleration against IMU linear acceleration magnitude (without accumulating integral drift).

---

### 2. EKF Bypasses Filtered Odometry
* **Problem**: The slip-checker outputs `/wheel/odom_filtered`, but EKF does not subscribe to it.
* **Root Cause**: In `config/ekf.yaml`, `odom0` is set to `/wheel/odom_raw`, completely bypassing `heuristic_slip_checker.py`.
* **Solution**:
  1. Set `odom0: /wheel/odom_filtered` in `config/ekf.yaml`.
  2. Ensure `slam_bringup.launch.py` and `ekf.launch.py` route:
     $$\text{Encoders} \xrightarrow{} \text{encoder\_ticks\_to\_odom} \xrightarrow{/\text{wheel/odom\_raw}} \text{heuristic\_slip\_checker} \xrightarrow{/\text{wheel/odom\_filtered}} \text{ekf\_node}$$

---

### 3. Duplicate & Colliding Static TF Broadcasters
* **Problem**: TF tree errors, jumping frames in RViz2, and `ExtrapolationException` during transform lookups.
* **Root Cause**: `static_transforms.launch.py` broadcasts `base_link -> camera_link` and `base_link -> imu_link` via `static_transform_publisher`. Simultaneously, `robot_state_publisher` reads the URDF and broadcasts the exact same fixed transforms on `/tf_static`.
* **Solution**:
  1. Make `robot_state_publisher` (URDF) the single source of truth for robot sensor geometry.
  2. Remove redundant static transforms from `static_transforms.launch.py` or disable the launch file when running the full rover description.

---

### 4. Camera Optical Frame Naming & Conventions
* **Problem**: Point clouds and depth images appear misaligned or inverted in RTAB-Map and Perception nodes.
* **Root Cause**: ROS REP-103 standard requires an optical frame (+Z forward, +X right, +Y down). The URDF uses `my_robot/camera_link/camera`, while SLAM and perception expect `camera_depth_optical_frame` / `camera_color_optical_frame`.
* **Solution**:
  1. Standardize the optical frame name in the URDF:
     ```xml
     <link name="camera_depth_optical_frame"/>
     <joint name="camera_depth_optical_joint" type="fixed">
       <parent link="camera_link"/>
       <child link="camera_depth_optical_frame"/>
       <origin xyz="0 0 0" rpy="-1.570796 0 -1.570796"/>
     </joint>
     ```
  2. Use `camera_depth_optical_frame` consistently across SLAM and Perception.

---

### 5. RealSense Filtered Depth Stream Disconnect
* **Problem**: Sunlight and outdoor IR noise corrupt RTAB-Map 3D grid maps.
* **Root Cause**: `vision_helper.launch.py` runs spatial, temporal, decimation, and hole-filling filters and publishes `/camera/depth/filtered`. However, `slam_bringup.launch.py` and `rtabmap.launch.py` default `depth_topic` to `/camera/depth/image_raw`.
* **Solution**:
  1. In `slam_bringup.launch.py`, dynamically route `depth_topic` to `/camera/depth/filtered` when `launch_camera:=true`.

---

### 6. EKF 2D Mode Clamping on 3D Terrain
* **Problem**: State estimation drifts or fails when the rover traverses slopes, craters, and ramps.
* **Root Cause**: `config/ekf.yaml` specifies `two_d_mode: true`, which forces $Z = 0$, $\text{Roll} = 0$, and $\text{Pitch} = 0$.
* **Solution**:
  1. In `config/ekf.yaml`, set `two_d_mode: false`.
  2. Configure `imu0_config` to fuse Roll and Pitch alongside Yaw and angular rates for full 3D terrain awareness.

---

### 7. Redundant Standalone Costmap Lifecycle Collision
* **Problem**: Running `slam_bringup.launch.py` alongside `PathPlanning/erc_path_planner` spawns two competing `nav2_costmap_2d` lifecycle nodes and managers.
* **Root Cause**: `rover_slam/launch/costmap.launch.py` starts a costmap server, while `erc_path_planner/launch/path_planning.launch.py` already includes Nav2 bringup with its own global and local costmap servers.
* **Solution**:
  1. Make `erc_path_planner` the sole owner of active Nav2 costmaps.
  2. Disable the standalone costmap in `slam_bringup.launch.py` when running full navigation, or make it an optional debug flag (`enable_costmap:=false` by default).

---

### 8. Perception Topics & ArUco Landmark Interface Gap
* **Problem**: RTAB-Map cannot perform global graph optimization using ArUco markers.
* **Root Cause**: RTAB-Map listens for landmark constraints (`geometry_msgs/msg/PoseStamped` or `rtabmap_msgs/msg/Landmarks`) on `/perception/aruco_pose`. The `Perception/marker_detection` package publishes custom `MarkerActionTargetArray` on `/marker_detection/targets` and dynamic TF `camera_optical_frame -> aruco_marker_<ID>`.
* **Solution**:
  1. Add an adapter or bridge node that converts confirmed `MarkerTrackedPose` / `MarkerActionTarget` from `marker_detection` into the `PoseStamped` topic expected by RTAB-Map.
  2. Feed terrain obstacles from `erc_path_planner`'s `costmap_bridge_node` (`/bridge/pointcloud`) into the Nav2 obstacle layer.
