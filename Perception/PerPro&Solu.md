# Perception Subsystem: Problems, Status Audit & Comprehensive Solutions (`PerPro&Solu.md`)

This document provides a complete technical audit of the **Perception Subsystem**, detailing:
1. **Problems that have been SOLVED in the current codebase.**
2. **Current ACTIVE PROBLEMS & GAPS that remain to be implemented.**
3. **Exact Engineering Solutions for each active issue.**
4. **Summary of Task Changes (What was removed, added, or modified).**
5. **Testing Calibration Tasks (Added based on simulation testing observations).**

---

## 📑 Table of Contents
- [1. Technical Audit: What is Already SOLVED in Code](#1-technical-audit-what-is-already-solved-in-code)
- [2. Active Problems & Required Solutions](#2-active-problems--required-solutions)
  - [Problem 1: Blind-Spot Memory Loss During Rover Turns](#problem-1-blind-spot-memory-loss-during-rover-turns)
  - [Problem 2: Topic & Message Inconsistency with Path Planning](#problem-2-topic--message-inconsistency-with-path-planning)
  - [Problem 3: SLAM Landmark Interface Mismatch (`/perception/aruco_pose`)](#problem-3-slam-landmark-interface-mismatch-perceptionaruco_pose)
  - [Problem 4: Fragmented System Launch & Configuration](#problem-4-fragmented-system-launch--configuration)
- [3. What Happened to the Tasks? (Added, Removed & Consolidated)](#3-what-happened-to-the-tasks-added-removed--consolidated)
- [4. Master Task Assignment Backlog](#4-master-task-assignment-backlog)
- [5. Testing Feedback & Calibration Tasks (Tasks 8, 9, 10)](#5-testing-feedback--calibration-tasks-tasks-8-9-10)

---

## 1. Technical Audit: What is Already SOLVED in Code

A rigorous audit of `Perception/terrain_geometry` and `Perception/marker_detection` confirms that several foundational problems are already solved:

| Component | Status in Code | Why It Is Solved |
| :--- | :---: | :--- |
| **QoS Sensor Data Handling** | ✅ **SOLVED** | Both `terrain_node.py` and `marker_detection_node.py` explicitly subscribe using `qos_profile_sensor_data` (`BEST_EFFORT`), preventing silent frame dropping from RealSense cameras and Gazebo plugins. |
| **Point Cloud Vectorization** | ✅ **SOLVED** | `terrain_node.py` implements a zero-IPC, in-memory pipeline using vectorized NumPy, SciPy, and cKDTree operations (ROI filtering, Patchwork++ ground segmentation, 5cm voxel downsampling, ROR outlier removal, and DBSCAN clustering). No per-point Python loops exist. |
| **TF Lookup & Coordinate Caching** | ✅ **SOLVED** | `CloudFrameTransformer` in `tf_transform.py` caches the 4x4 homogeneous transformation matrix from `camera_depth_optical_frame` to `base_link`, preventing node crashes during boot latency. |
| **Advanced ArUco Vision & Tracking** | ✅ **SOLVED** | `marker_detection` implements subpixel corner refinement, image quality gating (blur/contrast), MAD depth filtering, OpenCV SolvePnP, time-aware Kalman filtering, and quaternion orientation smoothing. |
| **2D Costmap Generation** | ✅ **SOLVED** | `occupancy_grid.py` and `costmap_inflation.py` compute direct 2D obstacle rasterization with exponential decay cost inflation. |

---

## 2. Active Problems & Required Solutions

---

### Problem 1: Blind-Spot Memory Loss During Rover Turns

#### 🔴 The Problem
When the rover turns or maneuvers around an obstacle, the obstacle leaves the camera's active field of view (69° x 42° FOV) and instantly disappears from perception. As a result, Nav2's local planner loses awareness of obstacles in blind spots, risking collisions during sharp swerves.

#### 🔍 Root Cause
`terrain_node.py` and its internal `ObstacleTracker` (`obstacle_tracking.py`, lines 22–29) deliberately only emit obstacles that are matched to a detection *in the current frame*. Unmatched tracks are aged internally but never emitted on `/terrain/obstacle_features` or `/terrain/costmap` to prevent ghost obstacles.

#### 🟢 The Solution
Implement a dedicated **`persistent_memory_node.py`** in `terrain_geometry`:
1. Subscribes to single-frame 3D bounding boxes on `/perception/local_bboxes` (`vision_msgs/msg/Detection3DArray`).
2. Implements a Time-To-Live (TTL) memory buffer ($TTL = 5.0\text{ seconds}$).
3. Publishes persistent obstacle bounding boxes on **`/perception/obstacles_only`** (`vision_msgs/msg/Detection3DArray`) and visual markers on `/terrain/obstacle_markers` for 5 seconds after an object leaves camera FOV.

---

### Problem 2: Topic & Message Inconsistency with Path Planning

#### 🔴 The Problem
`PathPlanning/erc_path_planner/src/costmap_bridge_node.cpp` was subscribing to `/terrain/obstacle_features` (`terrain_geometry_msgs/ObstacleFeatureArray`) and converting only 1 centroid point per rock, causing Nav2 to ignore physical obstacle volume and bypass persistent memory.

#### 🔍 Root Cause
Path planning and perception used different message types and topic contracts.

#### 🟢 The Solution (Standardized on Option 1)
1. **In `terrain_node.py`:** Add publisher for standard `vision_msgs/msg/Detection3DArray` on `/perception/local_bboxes`.
2. **In `persistent_memory_node.py`:** Publish persistent obstacles on `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`).
3. **In `erc_path_planner/costmap_bridge_node.cpp`:** Update subscriber to `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`), sample the full 3D bounding box footprint at 5cm resolution, and publish `sensor_msgs/msg/PointCloud2` on `/bridge/pointcloud` for Nav2's `ObstacleLayer`.

---

### Problem 3: SLAM Landmark Interface Mismatch (`/perception/aruco_pose`)

#### 🔴 The Problem
Saif SLAM (RTAB-Map) expects a standard `geometry_msgs/msg/PoseStamped` on topic **`/perception/aruco_pose`** to trigger landmark-based global drift hard resets. The `marker_detection` package outputs `/marker_poses` (`MarkerTrackedPoseArray`) and `/marker_detection/targets` (`MarkerActionTargetArray`).

#### 🔍 Root Cause
`marker_detection` provides high-level action targets for manipulation, but did not expose the exact `PoseStamped` topic contract expected by RTAB-Map SLAM.

#### 🟢 The Solution
In `marker_detection` (`marker_detection_node.py` or `marker_action_interface_node.py`):
1. Add a publisher for `geometry_msgs/msg/PoseStamped` on `/perception/aruco_pose`.
2. Implement distance gating ($d \le 3.5\text{m}$) and temporal confidence gating (>= 3 consecutive confirmed frames).
3. Publish the 6-DOF pose in `base_link` frame whenever a verified marker is tracked.

---

### Problem 4: Fragmented System Launch & Configuration

#### 🔴 The Problem
Launching the full perception stack requires manually running separate terminal commands for `terrain_node`, `marker_detection`, and RViz, with parameters scattered across different directories.

#### 🔍 Root Cause
Lack of a top-level unified launch file and consolidated parameter file.

#### 🟢 The Solution
1. Create `Perception/terrain_geometry/config/perception_params.yaml` containing centralized parameters for `terrain_node`, `persistent_memory_node`, and `marker_detection_node`.
2. Create `Perception/terrain_geometry/launch/perception_system.launch.py` to concurrently start all 3 nodes with one command.

---

## 3. What Happened to the Tasks? (Added, Removed & Consolidated)

### ❌ What Was Removed / Marked Obsolete
1. **Writing ArUco SolvePnP Solver from Scratch:** Removed. The `marker_detection` package already provides production-ready SolvePnP, MAD depth rejection, and Kalman tracking.
2. **Rewriting Point Cloud Filtering / Ground Removal:** Removed. `terrain_geometry` already contains high-performance vectorized implementations of Patchwork++, RANSAC, voxel downsampling, and DBSCAN clustering.
3. **Writing Custom TF Broadcasters for Markers:** Removed. `marker_tf_broadcaster.py` and `frame_transform.py` already exist in `marker_detection`.

### ➕ What Was Added & Refined
1. **`vision_msgs` Output Standardization in `terrain_node.py`:** Added to eliminate custom message dependencies.
2. **Dedicated `persistent_memory_node.py` with 5s TTL Buffer:** Added to solve the camera blind-spot problem for Nav2.
3. **SLAM Pose Bridge in `marker_detection`:** Added to connect `marker_detection` directly to RTAB-Map on `/perception/aruco_pose`.
4. **Costmap Bridge Update in `PathPlanning`:** Added to ingest `vision_msgs/msg/Detection3DArray` and sample bounding box volumes into `PointCloud2`.
5. **Centralized `perception_system.launch.py` & `perception_params.yaml`:** Added for unified 1-command startup.

---

## 4. Master Task Assignment Backlog

| Task ID | Task Summary | Component | Status |
| :--- | :--- | :--- | :---: |
| **Task 1** | Standardize `terrain_node` to publish `vision_msgs/Detection3DArray` on `/perception/local_bboxes`. | `terrain_geometry` | 🔲 To Do |
| **Task 2** | Implement `persistent_database.py` (3D Spatial Association & EMA Smoothing). | `terrain_geometry` | 🔲 To Do |
| **Task 3** | Implement `persistent_memory_node.py` with 5.0s TTL blind-spot retention on `/perception/obstacles_only`. | `terrain_geometry` | 🔲 To Do |
| **Task 4** | Bridge `marker_detection` to publish standard `PoseStamped` on `/perception/aruco_pose` for Saif SLAM. | `marker_detection` | 🔲 To Do |
| **Task 5** | Update `costmap_bridge_node.cpp` in `PathPlanning` to subscribe to `/perception/obstacles_only`. | `erc_path_planner` | 🔲 To Do |
| **Task 6** | Create `perception_params.yaml` and `perception_system.launch.py` for unified startup. | `terrain_geometry` | 🔲 To Do |
| **Task 7** | Closed-loop Gazebo simulation validation and 10-minute Nav2 benchmark run. | `Testing` | 🔲 To Do |
| **Task 8** | Resolve ArUco camera topics and calibrate physical `marker_size_m` in `marker_detection.yaml`. | `marker_detection` | 🔲 To Do |
| **Task 9** | Tune Patchwork++ ground removal height parameters to eliminate false ground obstacles. | `terrain_geometry` | 🔲 To Do |
| **Task 10** | Calibrate terrain costmap inflation radius to eliminate double inflation and oversized obstacles. | `terrain_geometry` | 🔲 To Do |

---

## 5. Testing Feedback & Calibration Tasks (Tasks 8, 9, 10)

The following 3 tasks directly resolve the issues observed during testing:

---

### Task 8: Resolve ArUco Camera Topics & Calibrate Physical `marker_size_m`

#### 📌 Problem Description
1. In `config/marker_detection.yaml`, camera topics default to `/camera/image_raw` instead of `/camera/color/image_raw`, causing missing frame inputs and empty topic data.
2. The distance to the ArUco marker is calculated incorrectly because `marker_size_m` is hardcoded to `0.21m` instead of matching the true physical marker size used in the simulation/world.
3. `validate_against_depth` rejects valid detections when depth frame sync fails.

#### 💡 What Needs to Be Done
- In `config/marker_detection.yaml`, update topics:
  - `rgb_topic: "/camera/color/image_raw"`
  - `depth_topic: "/camera/depth/image_rect_raw"`
  - `camera_info_topic: "/camera/camera_info"`
- Measure the exact physical marker side length in the Gazebo model (e.g., `0.20m` or `0.15m`) and set `marker_size_m: <exact_size_in_meters>`.
- Relax depth difference threshold: `max_depth_position_difference_m: 0.40`.

#### 🧪 Verification & Commands
```bash
ros2 launch worlds world_Rotated_Aruco.launch.py &
ros2 launch marker_detection marker_detection.launch.py &
ros2 topic echo /marker_poses
```
* **Passing Criteria:** `/marker_poses` streams continuous data when marker is in view, and calculated distance $Z$ matches Gazebo ground truth to within <= 2cm.

---

### Task 9: Tune Patchwork++ Ground Removal Height Parameters to Eliminate False Ground Obstacles

#### 📌 Problem Description
Flat ground terrain is occasionally classified as rock clusters, and ground slopes generate false obstacle boxes. This happens because `ground_sensor_height` and `ground_height_threshold` in `terrain.launch.py` are mismatched with the actual camera mounting height on the rover chassis.

#### 💡 What Needs to Be Done
- Check the TF height of `camera_link` relative to `base_link` in the rover URDF (e.g., $Z = 0.45\text{m}$) and set `ground_sensor_height: <actual_mounting_height>`.
- Increase `ground_height_threshold: 0.20` (or `0.22m`) so small ground bumps and dirt waves are classified as traversable ground.
- Ensure `ground_removal_backend: "patchwork"` is active to handle uneven concentric terrain rings.

#### 🧪 Verification & Commands
```bash
ros2 launch my_robot_description gazebo.launch.py world:=world1.world &
ros2 launch terrain_geometry terrain.launch.py publish_debug_topics:=true &
```
* **Passing Criteria:** Flat and gently sloping ground is 100% colored in solid green on `/terrain/debug/ground_cloud`, with zero false-positive bounding boxes on flat driving paths.

---

### Task 10: Calibrate Terrain Costmap Inflation Radius to Prevent Oversized Obstacle Footprints

#### 📌 Problem Description
The obstacle costmap generated by `terrain_node` is significantly larger than the real physical rocks. This happens because `terrain_geometry` inflates obstacles with `robot_radius: 0.3` + `costmap_inflation_radius: 0.6` (0.9m buffer), and then Nav2's `InflationLayer` inflates them a second time, turning a 20cm stone into an impassable 2.5m obstacle.

#### 💡 What Needs to Be Done
- Let Nav2 own the dynamic inflation buffer (`nav2_params.yaml`).
- In `terrain.launch.py`, reduce `costmap_inflation_radius: 0.20` and `robot_radius: 0.0` for raw perception grids.
- Set `cost_scaling_factor: 5.0` in `terrain.launch.py` for a sharper cost gradient around rock perimeters.
- Ensure `grid_resolution: 0.05` (5cm) matches Nav2 local costmap resolution for 1:1 cell alignment.

#### 🧪 Verification & Commands
```bash
ros2 launch terrain_geometry terrain.launch.py &
ros2 launch erc_path_planner path_planning.launch.py &
```
* **Passing Criteria:** High-cost red footprints tightly hug rock boundaries with a realistic safety margin (<= 0.4m), allowing the rover to navigate narrow 1.5m canyon gaps without false blockages.
