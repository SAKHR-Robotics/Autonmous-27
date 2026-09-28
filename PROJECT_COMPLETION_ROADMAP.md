# Autonomous-27: Master Project Completion Roadmap & Engineering Task Matrix

> **Document Purpose**: Authoritative implementation backlog and execution roadmap detailing all required engineering tasks, technical specifications, affected files, topic contracts, and verification commands to bring the Autonomous-27 Mars Rover system to 100% completion.

---

## 📑 Table of Contents
1. [Executive Summary & System Architecture Alignment](#1-executive-summary--system-architecture-alignment)
2. [Task Progress Dashboard](#2-task-progress-dashboard)
3. [Phase 1: Rock Memory & Perception Hardening](#phase-1-rock-memory--perception-hardening)
4. [Phase 2: Navigation Stack (Nav2) & Costmap Bridge](#phase-2-navigation-stack-nav2--costmap-bridge)
5. [Phase 3: SLAM, State Estimation & ArUco Loop-Closure](#phase-3-slam-state-estimation--aruco-loop-closure)
6. [Phase 4: Full Integrated Autonomy Benchmarking (Mission Testing)](#phase-4-full-integrated-autonomy-benchmarking-mission-testing)
7. [Phase 5: Sim-to-Real Hardware Actuation & Sensor Drivers](#phase-5-sim-to-real-hardware-actuation--sensor-drivers)
8. [Master Playbook: Launch Order & Verification Suite](#8-master-playbook-launch-order--verification-suite)

---

## 1. Executive Summary & System Architecture Alignment

The system architecture defined in [`General_Docs/FlowWTopics.dot`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/General_Docs/FlowWTopics.dot) and visual graph [`General_Docs/FlowWTopics.png`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/General_Docs/FlowWTopics.png) defines the production dataflow:

```
[Sensors & Drivers]
   ├── Wheel Encoders ──► encoder_ticks_to_odom ──► heuristic_slip_checker ──► robot_localization EKF (50 Hz)
   ├── BNO055 IMU ─────────────────────────────────────────────────────────────┘   │
   └── RealSense D435 (RGB-D) ────────────────────────┐                            │ (odom -> base_link TF)
                                                      │                            ▼
[Perception & SLAM]                                   ├──► rtabmap Visual SLAM (1-5 Hz) ──► (map -> odom TF)
   ├── PointCloud2 ──► terrain_node (Patchwork++/DBSCAN)                           │             │
   │                      │ (local_bboxes)                                         │             │
   │                      ▼                                                        │             │
   │               persistent_memory_node (5s TTL)                                 │             │
   │                      │ (/perception/obstacles_only)                           │             │
   │                      ▼                                                        ▼             │
[Navigation (Nav2)] costmap_bridge_node ──► /bridge/pointcloud ──────────► nav2_costmap_server    │
                                                                           │ (global/local costmap)
                                                                           ▼
                                                                  nav2_planner (Smac Hybrid A*)
                                                                           │ (/plan)
                                                                           ▼
                                                                  nav2_controller (MPPI)
                                                                           │ (/cmd_vel)
                                                                           ▼
                                                                  motor_driver (Skid-Steer)
```

This document details the concrete work packages required to finalize every node and edge in this pipeline.

---

## 2. Task Progress Dashboard

| Task ID | Phase | Priority | Summary | Target Subsystem | Status |
| :--- | :--- | :---: | :--- | :--- | :---: |
| **TASK-1.1** | Phase 1 | 🔴 **CRITICAL** | Build `persistent_memory_node.py` (5.0s TTL Blind-Spot Buffer) | `Perception/terrain_geometry` | 🔲 To Do |
| **TASK-1.2** | Phase 1 | 🔴 **CRITICAL** | Integrate `persistent_memory_node` into `perception_system.launch.py` | `Perception/terrain_geometry` | 🔲 To Do |
| **TASK-1.3** | Phase 1 | 🟡 **HIGH** | Calibrate Patchwork++ ground removal height parameters | `Perception/terrain_geometry` | 🔲 To Do |
| **TASK-2.1** | Phase 2 | 🔴 **CRITICAL** | Verify `costmap_bridge_node.cpp` 3D bounding box volume sampling | `PathPlanning/erc_path_planner` | 🔲 To Do |
| **TASK-2.2** | Phase 2 | 🟡 **HIGH** | Tune MPPI controller parameters & cost critics for skid-steer | `PathPlanning/erc_path_planner` | 🔲 To Do |
| **TASK-2.3** | Phase 2 | 🟢 **MEDIUM** | Validate Nav2 recovery behaviors (Spin, Backup, Wait) | `PathPlanning/erc_path_planner` | 🔲 To Do |
| **TASK-3.1** | Phase 3 | 🟡 **HIGH** | Standardize ArUco camera topics & calibrate metric `marker_size_m` | `Perception/marker_detection` | 🔲 To Do |
| **TASK-3.2** | Phase 3 | 🟡 **HIGH** | Verify RTAB-Map global drift reset via `/perception/aruco_pose` | `SLAM/rover_slam` | 🔲 To Do |
| **TASK-3.3** | Phase 3 | 🟢 **MEDIUM** | Validate `heuristic_slip_checker` dynamic covariance in sand | `SLAM/rover_slam` | 🔲 To Do |
| **TASK-4.1** | Phase 4 | 🟡 **HIGH** | Master one-command autonomy bringup script verification | `scripts/` | 🔲 To Do |
| **TASK-4.2** | Phase 4 | 🔴 **CRITICAL** | Execute 10-minute Mars Yard autonomous exploration benchmark | `testing/` | 🔲 To Do |
| **TASK-4.3** | Phase 4 | 🟢 **MEDIUM** | Autonomous ArUco landmark target approach & standoff stop | `testing/` | 🔲 To Do |
| **TASK-5.1** | Phase 5 | 🟢 **MEDIUM** | Deploy physical skid-steer motor controller bridge | `Control/` | 🔲 To Do |
| **TASK-5.2** | Phase 5 | 🟢 **MEDIUM** | Physical sensor driver bringup checklist (RealSense & BNO055) | Hardware Bringup | 🔲 To Do |

---

## Phase 1: Rock Memory & Perception Hardening

### TASK-1.1: Build `persistent_memory_node.py` (5.0s TTL Blind-Spot Retention)
* **Goal**: Prevent obstacle memory loss when the rover swerves or turns around a rock and the rock leaves the camera's $69^\circ$ field of view.
* **Target File**: `Perception/terrain_geometry/terrain_geometry/persistent_memory_node.py`
* **Package Registration**: Add entry point in [`Perception/terrain_geometry/setup.py`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/Perception/terrain_geometry/setup.py):
  ```python
  'persistent_memory_node = terrain_geometry.persistent_memory_node:main'
  ```
* **Topic Contract**:
  * **Input**: `/perception/local_bboxes` (`vision_msgs/msg/Detection3DArray`) from `terrain_node.py`.
  * **Output**: `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`) and `/terrain/obstacle_markers` (`visualization_msgs/msg/MarkerArray`).
* **Implementation Details**:
  1. Use `tf2_ros` to transform obstacle bounding box centroids from camera/sensor frame into `odom` (or `base_link` at observation time).
  2. Implement an obstacle track database that associates new detections with existing tracks using Euclidean distance threshold ($d \le 0.5\text{ m}$).
  3. Apply Exponential Moving Average (EMA, $\alpha = 0.5$) for smooth position updates.
  4. Track `last_seen_timestamp` per obstacle. If an obstacle is not seen in the current frame, retain it for `ttl_seconds: 5.0`.
  5. Publish all currently active tracks (both current-frame detections and unexpired memory tracks) on `/perception/obstacles_only`.
* **Passing Criteria**:
  * Rotate the rover $90^\circ$ away from a rock in simulation; the 3D bounding box remains visible in RViz on `/terrain/obstacle_markers` for exactly 5 seconds before being pruned.

---

### TASK-1.2: Integrate `persistent_memory_node` into `perception_system.launch.py`
* **Goal**: Create a seamless 1-command perception stack that launches `terrain_node`, `persistent_memory_node`, and `marker_branch`.
* **Target Files**:
  * [`Perception/terrain_geometry/launch/perception_system.launch.py`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/Perception/terrain_geometry/launch/perception_system.launch.py)
  * [`Perception/terrain_geometry/terrain_geometry/terrain_node.py`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/Perception/terrain_geometry/terrain_geometry/terrain_node.py)
* **Implementation Details**:
  1. In `terrain_node.py`, ensure single-frame detections are published to `/perception/local_bboxes`. Remove or deprecate direct publishing to `/perception/obstacles_only` from `terrain_node.py` so that only `persistent_memory_node` owns `/perception/obstacles_only`.
  2. In `perception_system.launch.py`, instantiate `Node(package='terrain_geometry', executable='persistent_memory_node', name='persistent_memory_node')`.
* **Verification Command**:
  ```bash
  ros2 launch terrain_geometry perception_system.launch.py
  ros2 topic hz /perception/local_bboxes
  ros2 topic hz /perception/obstacles_only
  ```

---

### TASK-1.3: Calibrate Patchwork++ Ground Removal Parameters
* **Goal**: Eliminate false ground rock detections on slopes, ramps, and small terrain undulations.
* **Target File**: [`Perception/terrain_geometry/launch/terrain.launch.py`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/Perception/terrain_geometry/launch/terrain.launch.py)
* **Implementation Details**:
  1. Calibrate `ground_sensor_height`: Check URDF camera joint offset ($Z = 0.45\text{ m}$) and set `ground_sensor_height: 0.45`.
  2. Calibrate `ground_height_threshold: 0.20`: Increase threshold from `0.15m` to `0.20m` so small dirt waves and wheel ruts are categorized as drivable ground.
  3. Verify `ground_removal_backend: "patchwork"` is active with `pypatchworkpp`.
* **Passing Criteria**:
  * On `/terrain/debug/ground_cloud`, flat and gently sloped ground is 100% green with zero false-positive obstacle boxes on flat driving corridors.

---

## Phase 2: Navigation Stack (Nav2) & Costmap Bridge

### TASK-2.1: Verify `costmap_bridge_node.cpp` Footprint Volume Sampling
* **Goal**: Ensure Nav2 receives full physical 3D obstacle volumes instead of single-point centroids.
* **Target Files**:
  * [`PathPlanning/erc_path_planner/src/costmap_bridge_node.cpp`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/PathPlanning/erc_path_planner/src/costmap_bridge_node.cpp)
  * [`PathPlanning/erc_path_planner/config/nav2_params.yaml`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/PathPlanning/erc_path_planner/config/nav2_params.yaml)
* **Implementation Details**:
  1. Confirm `costmap_bridge_node` subscribes to `/perception/obstacles_only` (`vision_msgs/Detection3DArray`).
  2. Confirm the sampling loop uses a $0.05\text{ m}$ (5cm) grid step matching costmap cell resolution:
     $$\Delta x = 0.05\text{m}, \quad \Delta y = 0.05\text{m}$$
  3. Verify point cloud is published to `/bridge/pointcloud` (`sensor_msgs/PointCloud2`).
  4. Ensure `nav2_params.yaml` `local_costmap` and `global_costmap` have `obstacle_layer` listening to `/bridge/pointcloud`.
* **Passing Criteria**:
  * In RViz, costmap cells cover the entire rectangular boundary of detected boulders, leaving no unmapped hollow interiors.

---

### TASK-2.2: Tune MPPI Controller & Smac Planner for Rover Skid-Steer
* **Goal**: Achieve smooth, collision-free trajectory tracking without sudden wheel oscillations.
* **Target File**: [`PathPlanning/erc_path_planner/config/nav2_params.yaml`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/PathPlanning/erc_path_planner/config/nav2_params.yaml)
* **Parameters to Fine-Tune**:
  * `controller_server.FollowPath.vx_max: 0.8` (m/s)
  * `controller_server.FollowPath.wz_max: 0.8` (rad/s)
  * `controller_server.FollowPath.CostCritic.cost_weight: 4.5`
  * `controller_server.FollowPath.CostCritic.critical_cost: 300.0`
  * `controller_server.FollowPath.CostCritic.collision_cost: 1000000.0`
  * `planner_server.GridBased.minimum_turning_radius: 0.6` (m)
* **Passing Criteria**:
  * Rover navigates an S-curve path between rocks without colliding or stalling on tight turns.

---

### TASK-2.3: Validate Nav2 Recovery Behaviors
* **Goal**: Enable the rover to automatically unstick itself if trapped in dead-ends or rock clusters.
* **Target File**: [`PathPlanning/erc_path_planner/launch/path_planning.launch.py`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/PathPlanning/erc_path_planner/launch/path_planning.launch.py)
* **Implementation Details**:
  1. Verify `behavior_server` is running and included in `lifecycle_manager_navigation`.
  2. Verify recovery plugins: `Spin`, `Backup`, and `Wait`.
  3. Test scenario: Spawn rover facing a dead-end wall, issue a goal behind the wall, verify rover automatically backs up and replans.

---

## Phase 3: SLAM, State Estimation & ArUco Loop-Closure

### TASK-3.1: Standardize ArUco Camera Topics & Calibrate Metric Size
* **Goal**: Ensure marker detector receives valid RGB images and calculates exact 3D coordinates.
* **Target File**: [`Perception/marker_detection/config/marker_detection.yaml`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/Perception/marker_detection/config/marker_detection.yaml)
* **Implementation Details**:
  1. Set image topics matching Gazebo bridge:
     * `rgb_topic: "/camera/image_raw"`
     * `depth_topic: "/camera/depth/image_raw"`
     * `camera_info_topic: "/camera/camera_info"`
  2. Calibrate `marker_size_m`: Match exact marker size in Mars Yard world model (e.g. `0.20m` or `0.15m`).
  3. Verify `marker_action_interface_node.py` publishes confirmed marker poses to `/perception/aruco_pose` (`geometry_msgs/PoseStamped`).
* **Verification Command**:
  ```bash
  ros2 launch marker_detection marker_detection.launch.py
  ros2 topic echo /perception/aruco_pose
  ```

---

### TASK-3.2: Verify RTAB-Map Global Drift Reset via ArUco
* **Goal**: Eliminate long-term visual odometry drift using ArUco landmarks.
* **Target Files**:
  * [`SLAM/rover_slam/launch/rtabmap.launch.py`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/SLAM/rover_slam/launch/rtabmap.launch.py)
  * [`SLAM/rover_slam/config/rtabmap.yaml`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/SLAM/rover_slam/config/rtabmap.yaml)
* **Implementation Details**:
  1. Confirm remapping in `rtabmap.launch.py`: `('landmark', '/perception/aruco_pose')`.
  2. Drive rover in a 50m loop with simulated slip until odometry drifts by > 0.5m.
  3. Bring marker into camera view; observe RTAB-Map console log confirming landmark loop closure and `map -> odom` transform adjustment.

---

### TASK-3.3: Validate Wheel Slip Checker in Sand
* **Goal**: Ensure EKF does not blow up when wheels lose traction in loose Mars sand.
* **Target File**: [`SLAM/rover_slam/rover_slam/heuristic_slip_checker.py`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/SLAM/rover_slam/rover_slam/heuristic_slip_checker.py)
* **Implementation Details**:
  1. Drive rover into a sandy dune or obstacle where wheels spin.
  2. Monitor `/wheel/odom_filtered` covariance matrix: verify variance inflates from $10^{-4}$ to $10^{3}$.
  3. Confirm EKF suppresses wheel velocity and relies on IMU acceleration and visual odometry.

---

## Phase 4: Full Integrated Autonomy Benchmarking (Mission Testing)

### TASK-4.1: Master Autonomy Launch Orchestration
* **Goal**: One-command interactive launch of all subsystems with zero port conflicts.
* **Target File**: `scripts/launch_system.sh`
* **Sequence Checklist**:
  1. Clean old processes: `pkill -9 -f gazebo; pkill -9 -f gz; pkill -9 -f ros; pkill -9 -f rviz`
  2. Launch World & Rover: `ros2 launch my_robot_description gazebo.launch.py` (Wait 5s)
  3. Launch State Estimation & SLAM: `ros2 launch rover_slam slam_bringup.launch.py` (Wait 3s)
  4. Launch Perception: `ros2 launch terrain_geometry perception_system.launch.py` (Wait 2s)
  5. Launch Path Planning: `ros2 launch erc_path_planner path_planning.launch.py`

---

### TASK-4.2: 10-Minute Mars Yard Navigation Benchmark Mission
* **Goal**: Demonstrate continuous autonomous exploration in the Mars Yard without human intervention.
* **Test Procedure**:
  1. Start the full autonomy stack.
  2. Send 5 sequential waypoints across the Mars Yard via Nav2 Action Client or RViz `2D Goal Pose`.
  3. Log total travel distance, time to goal, clearance distance to rocks, and CPU/memory utilization.
* **Benchmark Acceptance Criteria**:
  * 0 collisions with rock models.
  * 0 TF timeout or lookup errors.
  * Goal reached accuracy within $\le 0.35\text{ m}$.

---

### TASK-4.3: Autonomous ArUco Target Approach & Precision Standoff
* **Goal**: Navigate to an ArUco marker and come to a controlled halt exactly 1.5m in front of it.
* **Target File**: [`Perception/marker_detection/marker_detection/marker_action_interface_node.py`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/Perception/marker_detection/marker_detection/marker_action_interface_node.py)
* **Procedure**:
  1. Spot marker from 6m away.
  2. Send navigation goal to Nav2 at offset $(X - 1.5\text{m}, Y=0)$ relative to marker frame.
  3. Verify rover halts cleanly aligned with the marker face.

---

## Phase 5: Sim-to-Real Hardware Actuation & Sensor Drivers

### TASK-5.1: Deploy Physical Skid-Steer Motor Controller Bridge
* **Goal**: Convert ROS 2 `/cmd_vel` into real motor drill actuator RPMs.
* **Target File**: [`Control/README.md`](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/Control/README.md)
* **Kinematic Equations**:
  $$v_L = v_x - \frac{\omega_z \cdot B}{2}, \quad v_R = v_x + \frac{\omega_z \cdot B}{2}$$
  $$\text{RPM}_L = \frac{v_L}{2\pi R} \times 60, \quad \text{RPM}_R = \frac{v_R}{2\pi R} \times 60$$
  where $B = 0.65\text{ m}$ (track width), $R = 0.15\text{ m}$ (wheel radius).
* **Firmware Contract**: Send target RPMs over UART/CAN to STM32 motor driver.

---

### TASK-5.2: Physical Sensor Driver Bringup Checklist
* **RealSense D435 USB Driver**:
  * Set `launch_camera:=true` in `slam_bringup.launch.py`.
  * Verify `/camera/color/image_raw` at 30 Hz and `/camera/depth/image_rect_raw` at 30 Hz.
* **BNO055 IMU Driver**:
  * Connect over I2C/UART; verify `/imu/data` at 50–100 Hz with gravity vector aligned to $-Z$.
* **Wheel Encoders**:
  * Verify hardware encoder node streams `/wheel/ticks` to [encoder_ticks_to_odom.py](file:///home/pavly-saad/mars_rover_ws/src/Autonmous-27/SLAM/rover_slam/rover_slam/encoder_ticks_to_odom.py).

---

## 8. Master Playbook: Launch Order & Verification Suite

### Clean Restart / Kill Zombie Processes
```bash
pkill -9 -f gazebo; pkill -9 -f ign; pkill -9 -f gz; pkill -9 -f ros; pkill -9 -f rviz; killall -9 -q ruby gz server rviz2 parameter_bridge ros2 robot_state_publisher
```

### Build Everything with Symlinks
```bash
colcon build --symlink-install
source install/setup.bash
```

### Full Autonomy Launch Playbook (4 Terminals)

* **Terminal 1: Simulation & Rover Spawn**
  ```bash
  source install/setup.bash
  ros2 launch my_robot_description gazebo.launch.py
  ```

* **Terminal 2: State Estimation & SLAM**
  ```bash
  source install/setup.bash
  ros2 launch rover_slam slam_bringup.launch.py
  ```

* **Terminal 3: Perception (Ground Filter, Memory, ArUco)**
  ```bash
  source install/setup.bash
  ros2 launch terrain_geometry perception_system.launch.py
  ```

* **Terminal 4: Nav2 Path Planning & Controller**
  ```bash
  source install/setup.bash
  ros2 launch erc_path_planner path_planning.launch.py
  ```

### Real-Time Diagnostics
```bash
# Check TF Tree connectivity
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link

# Check Dataflow Topic Rates
ros2 topic hz /odometry/filtered
ros2 topic hz /perception/obstacles_only
ros2 topic hz /bridge/pointcloud
ros2 topic hz /cmd_vel
```
