# Perception Subsystem: Problems, Root Causes & Comprehensive Solutions (`PerPro&Solu.md`)

This document provides a complete breakdown of all architectural, algorithmic, interface, and hardware synchronization problems identified in the **Perception Subsystem**, alongside their concrete, production-ready engineering solutions.

---

## 📑 Table of Contents
1. [Problem 1: QoS Profile Mismatch (Silent PointCloud & Image Dropping)](#problem-1-qos-profile-mismatch-silent-pointcloud--image-dropping)
2. [Problem 2: Blind-Spot Memory Loss & Obstacle Flickering](#problem-2-blind-spot-memory-loss--obstacle-flickering)
3. [Problem 3: Topic & Interface Inconsistencies Across Modules (Nav2 & SLAM)](#problem-3-topic--interface-inconsistencies-across-modules-nav2--slam)
4. [Problem 4: Dual / Fragmented ArUco Detection Implementations](#problem-4-dual--fragmented-aruco-detection-implementations)
5. [Problem 5: 3D Point Cloud Processing Latency on Jetson Orin Nano](#problem-5-3d-point-cloud-processing-latency-on-jetson-orin-nano)
6. [Problem 6: Coordinate Frame (TF) Lookup Failures at Startup](#problem-6-coordinate-frame-tf-lookup-failures-at-startup)
7. [Problem 7: Optical Jitter & False Positives in ArUco Pose Estimation](#problem-7-optical-jitter--false-positives-in-aruco-pose-estimation)
8. [Problem 8: Decentralized Configuration & Launch Management](#problem-8-decentralized-configuration--launch-management)

---

## Problem 1: QoS Profile Mismatch (Silent PointCloud & Image Dropping)

### 🔴 The Problem
When running the perception nodes alongside the Intel RealSense D435 camera driver or Gazebo simulation plugins, nodes frequently receive **0 messages** on `/camera/depth/color/points` and `/camera/color/image_raw`. No error or crash is logged in the terminal, giving the false impression that nodes are hanging or frozen.

### 🔍 Root Cause
ROS 2 Quality of Service (QoS) incompatibility. High-bandwidth camera drivers (RealSense ROS 2 wrapper and Gazebo sensor plugins) publish raw sensor streams using `BEST_EFFORT` reliability to avoid network buffering delays. If ROS 2 subscriber nodes default to `RELIABLE` QoS, the ROS 2 middleware strictly drops all mismatched communication packets without throwing an error.

### 🟢 The Solution
1. **Explicit SensorDataQoS Configuration in Node Subscriptions:**
   Configure all raw image and point cloud subscribers to use `qos_profile_sensor_data` or explicit `QoSReliabilityPolicy.BEST_EFFORT`.
   ```python
   from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

   sensor_qos = QoSProfile(
       reliability=QoSReliabilityPolicy.BEST_EFFORT,
       history=QoSHistoryPolicy.KEEP_LAST,
       depth=5
   )
   self.create_subscription(PointCloud2, '/camera/depth/color/points', self.pointcloud_callback, sensor_qos)
   ```
2. **RViz Display Settings:**
   Set the **Reliability Policy** dropdown to **Best Effort** in RViz for all raw camera displays.
3. **Downstream Reliable Publishing:**
   Keep processed output topics (`/perception/obstacles_only`, `/terrain/costmap`, `/perception/aruco_pose`) on `RELIABLE` QoS so Nav2 and SLAM never miss an obstacle or landmark update.

---

## Problem 2: Blind-Spot Memory Loss & Obstacle Flickering

### 🔴 The Problem
As soon as the rover turns or an obstacle leaves the camera’s active field of view ($69^\circ \times 42^\circ$ FOV), the obstacle instantly vanishes from the perception output. When the camera rotates back and forth during navigation, obstacles flicker in and out of existence, causing the Nav2 MPPI local controller to oscillate and plan erratic paths.

### 🔍 Root Cause
`terrain_node.py` operates strictly as a frame-by-frame instantaneous geometric extractor without temporal persistence, cross-frame track correlation, or a Time-to-Live (TTL) memory buffer.

### 🟢 The Solution
1. **Implement `persistent_database.py` (Spatial Association & Smoothing):**
   - Correlate incoming 3D bounding boxes with active tracks using 3D Euclidean distance (KD-Tree nearest-neighbor matching within a radius threshold $d_{thresh} = 0.4\text{m}$).
   - Smooth coordinate noise across frames using an Exponential Moving Average (EMA) filter:
     $$\mathbf{P}_{smooth}^{(t)} = \alpha \mathbf{P}_{meas}^{(t)} + (1 - \alpha) \mathbf{P}_{smooth}^{(t-1)} \quad (\text{with } \alpha = 0.3)$$
2. **Implement `persistent_memory_node.py` (Blind-Spot TTL Retention):**
   - Assign a Time-to-Live timer (e.g., $TTL = 5.0\text{ seconds}$) to each tracked obstacle.
   - When an obstacle leaves the field of view, keep publishing its last known smoothed coordinates in `base_link` or `odom` frame until the TTL timer expires.
   - Publish persistent obstacles to `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`).

---

## Problem 3: Topic & Interface Inconsistencies Across Modules (Nav2 & SLAM)

### 🔴 The Problem
Different modules in the rover workspace expect different topic names and message formats:
- Path planning’s `costmap_bridge_node.cpp` subscribes to `/terrain/obstacle_features` (`terrain_geometry_msgs/msg/ObstacleFeatureArray`).
- System specifications (`FlowWTopics.dot` and `PerceptionAiGuide.md`) expect `/perception/obstacles_only` and `/perception/local_bboxes` (`vision_msgs/msg/Detection3DArray`).
- Saif SLAM expects `/perception/aruco_pose` (`geometry_msgs/msg/PoseStamped`), but `marker_detection` published `/marker_detection/targets` and `/marker_poses`.

### 🔍 Root Cause
Decentralized development across sub-teams without strict end-to-end interface harmonization.

### 🟢 The Solution
1. **Standardize `terrain_node.py` with Dual-Topic Publishing:**
   - Primary Standard Topic: Publish `vision_msgs/msg/Detection3DArray` on `/perception/local_bboxes`.
   - Backward-Compatibility Topic: Continue publishing `terrain_geometry_msgs/msg/ObstacleFeatureArray` on `/terrain/obstacle_features`.
2. **Bridge Marker Pose to SLAM:**
   - Ensure the ArUco marker module publishes standard `geometry_msgs/msg/PoseStamped` on `/perception/aruco_pose` with frame `base_link` or `camera_depth_optical_frame`.
3. **Standardize Nav2 Ingestion:**
   - Nav2 costmap server directly ingests `/perception/obstacles_only` or converts it via `costmap_bridge_node.cpp`.

---

## Problem 4: Dual / Fragmented ArUco Detection Implementations

### 🔴 The Problem
The workspace contains a comprehensive standalone ROS 2 package `marker_detection` (with OpenCV SolvePnP, MAD depth rejection, and Kalman filters), while legacy documentation referenced a non-existent `aruco_detector_node.py` inside `terrain_geometry`.

### 🔍 Root Cause
Redundant architectural specifications created during roadmap drafts that diverged from the implemented `marker_detection` package.

### 🟢 The Solution
1. **Unify under `marker_detection`:**
   Designate `Perception/marker_detection` as the definitive, single-source ArUco vision engine for the rover.
2. **Expose Unified Aliases & Launchers:**
   - Add a ROS 2 topic remapping / publisher inside `marker_detection` to stream `/perception/aruco_pose`.
   - Include `marker_detection` within the unified system launcher (`perception_system.launch.py`).

---

## Problem 5: 3D Point Cloud Processing Latency on Jetson Orin Nano

### 🔴 The Problem
Processing raw RealSense point clouds ($640 \times 480 \approx 307,200$ points) at $30\text{ Hz}$ in Python can cause high CPU load ($>80\%$) and latency spikes ($>80\text{ms}$), dropping the perception pipeline rate below the required $15\text{ Hz}$.

### 🔍 Root Cause
Running expensive clustering algorithms (such as DBSCAN) and ground plane fitting over uncompressed point clouds with thousands of background/sky points.

### 🟢 The Solution
1. **Early Spatial ROI Cropping:**
   Before any ground removal or clustering, crop the raw cloud to the immediate driving bounding box ($X \in [0.2, 7.0]\text{m}$, $Y \in [-3.5, 3.5]\text{m}$, $Z \in [-0.5, 1.2]\text{m}$) using vectorized NumPy slicing. This strips $>60\%$ of irrelevant points immediately.
2. **Aggressive Voxel Downsampling:**
   Apply a 3D Voxel Grid filter with a leaf size of $5\text{cm} \times 5\text{cm} \times 5\text{cm}$ right after ground removal, reducing the remaining obstacle cloud from $\sim 50,000$ points down to $<2,000$ points.
3. **Vectorized Implementations Only:**
   Ensure zero per-point Python loops across all filtering stages (ROI, Ground Removal, Voxel Filter, DBSCAN, Costmap Inflation).
4. **Target Pipeline Latency:** $\le 35\text{ms}$ per frame on Jetson Orin Nano ($\sim 25\text{ Hz}$).

---

## Problem 6: Coordinate Frame (TF) Lookup Failures at Startup

### 🔴 The Problem
When perception nodes start before the TF tree or `robot_state_publisher` is fully initialized, `lookup_transform()` throws `TransformException` / `LookupException`, causing nodes to crash or drop initial point cloud frames.

### 🔍 Root Cause
Unprotected, blocking TF lookups in the PointCloud subscriber callback without cached transform matrices or timeout exception handlers.

### 🟢 The Solution
1. **Transform Matrix Caching (`tf_transform.py`):**
   - Query the static/dynamic transform `camera_depth_optical_frame` $\rightarrow$ `base_link` asynchronously with a timeout buffer ($100\text{ms}$).
   - Cache the resulting $4 \times 4$ homogeneous transformation matrix. If a lookup fails for a single frame, use the cached matrix until an updated transform is broadcast.
2. **Graceful Error Recovery:**
   - Wrap TF lookup in `try-except (tf2_ros.LookupException, tf2_ros.ExtrapolationException)` and drop the frame cleanly with a throttled debug warning instead of crashing the node.

---

## Problem 7: Optical Jitter & False Positives in ArUco Pose Estimation

### 🔴 The Problem
At long distances ($>3.0\text{m}$) or under bright solar illumination in Mars analog terrain, ArUco corner detection suffers from pixel noise, causing 3D distance and yaw estimates to jitter by $\pm 10\text{cm}$ and $\pm 15^\circ$. Single-frame false positive detections can trigger false SLAM loop closures.

### 🔍 Root Cause
SolvePnP sensitivity to subpixel corner jitter at shallow camera angles, combined with lack of temporal confidence gating.

### 🟢 The Solution
1. **Subpixel Corner Refinement:**
   Apply `cv2.cornerSubPix()` with a $5 \times 5$ window before passing corners to `cv2.solvePnP(flags=cv2.SOLVEPNP_IPPE_SQUARE)`.
2. **Distance & Aspect Ratio Gating:**
   - Discard marker detections with distance $d > 3.5\text{m}$.
   - Discard marker corner sets with extreme quadrilateral distortion (non-planar geometry).
3. **Temporal Multi-Frame Gating:**
   Only publish a marker pose to `/perception/aruco_pose` if the exact same Marker ID is verified across $\ge 3$ consecutive frames.
4. **Kalman Filter Smoothing:**
   Use the 6-DOF position and quaternion filter built into `marker_detection` (`position_kalman.py` & `quaternion_filter.py`).

---

## Problem 8: Decentralized Configuration & Launch Management

### 🔴 The Problem
Users had to manually launch multiple separate terminal commands to start `terrain_node`, `marker_detection`, and RViz, each loading disjoint YAML parameter files with duplicate topic names.

### 🔍 Root Cause
Absence of a top-level unified launch file and centralized configuration dictionary.

### 🟢 The Solution
1. **Centralized YAML (`terrain_geometry/config/perception_params.yaml`):**
   Group parameters cleanly under `terrain_node`, `persistent_memory_node`, and `marker_detection_node`.
2. **Unified System Launcher (`terrain_geometry/launch/perception_system.launch.py`):**
   A single command spins up all perception nodes with shared parameters:
   ```bash
   ros2 launch terrain_geometry perception_system.launch.py
   ```
3. **Master RViz Configuration (`Perception/rviz/perception_system_view.rviz`):**
   A centralized RViz config displays the entire system (PointClouds, Ground separation, Costmaps, Obstacles, ArUco markers, and TFs) in one synchronized window.

---

## 📊 Summary Problem-to-Solution Matrix

| # | Problem Area | Core Issue | Resolution |
|---|---|---|---|
| **1** | **QoS Compatibility** | Silent pointcloud dropping on `BEST_EFFORT` streams | Enforce `SensorDataQoS` on subscribers, `Reliable` on outputs. |
| **2** | **Obstacle Persistence** | Obstacles vanish when leaving camera FOV | Implement `persistent_database.py` (EMA) & `persistent_memory_node.py` (5s TTL). |
| **3** | **Interface Inconsistency** | Topic & message mismatches with SLAM & Nav2 | Publish standard `vision_msgs/Detection3DArray` & `/perception/aruco_pose`. |
| **4** | **ArUco Fragmentation** | Two separate marker pipelines referenced | Unify under `marker_detection` with standard `/perception/aruco_pose` bridge. |
| **5** | **Compute Latency** | High CPU load ($>80\text{ms}$) on Jetson Orin Nano | Early ROI crop + 5cm voxel downsampling to maintain $<35\text{ms}$ latency. |
| **6** | **TF Lookup Drops** | Transform exceptions crash nodes at boot | Implement matrix caching and graceful try-catch recovery in `tf_transform.py`. |
| **7** | **ArUco Vision Jitter** | False positives & corner jitter at distance | Subpixel refinement + 3-frame temporal gating + Kalman filtering. |
| **8** | **System Bringup** | Fragmented launchers and scattered configs | Create `perception_params.yaml` and `perception_system.launch.py`. |

---

## 🎯 9. Master Implementation Tasks (GitHub Tasks Format)

See the full task breakdown with issue descriptions, step-by-step instructions, and verification commands below for team task distribution.

