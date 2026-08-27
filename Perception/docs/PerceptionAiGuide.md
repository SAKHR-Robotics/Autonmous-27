# 🧠 Perception AI Implementation Guide & Roadmap (`PerceptionAiGuide.md`)

> 💡 **AI Maintenance Directive:** This document is the master roadmap for AI assistants and human engineers implementing the Perception Subsystem. When completing a task or updating code, **mark the task checkbox (`- [x]`) and update the Document Status Log.**

---

## 📋 Metadata & Document Control
* **Subsystem:** Autonomous Perception Module
* **Target Hardware:** NVIDIA Jetson Orin Nano (8GB CUDA GPU) + Intel RealSense D435 RGB-D Camera
* **ROS 2 Environment:** ROS 2 Humble / Jazzy
* **Workspace Path:** `/home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws/Perception`
* **GitHub Project Label:** `label:Perception`
* **Current Status:** Core Terrain Geometry algorithms (`terrain_node.py`) implemented; Standardization, Persistent Memory, and ArUco 3D Vision Issues pending.

---

## 📖 1. The Big Picture & System Architecture

The **Perception Subsystem** provides 3D spatial awareness and visual target recognition for an autonomous Mars-analog rover. To optimize compute performance on the Jetson Orin Nano, the architecture strictly separates **State Estimation/SLAM** (handled by the **Saif SLAM Module**) from **Obstacle Recognition & Target Pose Estimation** (handled by this **Perception Module**).

```
+---------------------------------------------------------------------------------------------------+
|                                          HARDWARE LAYER                                           |
|                  Intel RealSense D435 RGB-D Camera + Wheel Encoders + BNO IMU                     |
+-------------------------------------------------+-------------------------------------------------+
                                                  |
                 +--------------------------------+--------------------------------+
                 | /camera/depth/color/points                                      | /camera/color/image_raw
                 v                                                                 v
+------------------------------------------------+               +-----------------------------------+
|            TERRAIN GEOMETRY NODE               |               |        ARUCO DETECTOR NODE        |
| 1. ROI Crop & base_link TF                     |               | 1. 2D Marker Corner Extraction    |
| 2. Ground Removal (Patchwork++ / RANSAC)       |               | 2. SolvePnP Math (Intrinsics)     |
| 3. Voxel Grid Downsampling (5cm leaf size)     |               | 3. 3D Distance & 6-DOF Pose       |
| 4. Radius Outlier Removal (ROR)                |               +-----------------+-----------------+
| 5. DBSCAN Euclidean Clustering                 |                                 |
+------------------------+-----------------------+                                 | /perception/aruco_pose
                         |                                                         v
                         | /perception/local_bboxes              +-----------------------------------+
                         v                                       |          SAIF SLAM MODULE         |
+------------------------------------------------+               | (RTAB-Map + EKF robot_localiz.)   |
|            PERSISTENT MEMORY NODE              |               | 1. EKF odom -> base_link (100 Hz) |
| 1. Spatial Nearest-Neighbor Matching           |               | 2. RTAB-Map map -> odom (1-5 Hz)  |
| 2. Exponential Moving Average (EMA Smoothing)  |               | 3. Hard reset on ArUco pose       |
| 3. Memory Retention (Blind Spot TTL Buffer)    |               +-----------------------------------+
+------------------------+-----------------------+
                         |
                         | /perception/obstacles_only
                         v
+---------------------------------------------------------------------------------------------------+
|                                        NAV2 COSTMAP SERVER                                        |
| Fuses persistent rock bounding boxes onto 2D occupancy costmap for Smac A* Path Planner           |
+---------------------------------------------------------------------------------------------------+
```

---

## 🔗 1.5. Inter-Module Contracts: Interface with SLAM & Path Planning

To ensure seamless integration across all rover subsystems, the Perception module adheres to strict inter-module data contracts with **Saif SLAM** (`saif SLAM.html`) and **Path Planning** (`PathPlannerGuide.html`):

### A. Contract with Saif SLAM Module (`SLAM/Slam_Docu/saif SLAM.html`)
* **What SLAM Provides to Perception:**
  * **TF Transforms (`map &rarr; odom &rarr; base_link &rarr; camera_link`):** Published by `robot_localization` EKF ($100\text{ Hz}$) and RTAB-Map ($1-5\text{ Hz}$). Allows perception to transform raw camera point clouds into `base_link` coordinates.
  * **Fused Odometry (`/odometry/filtered`):** High-speed velocity feedback used by perception's obstacle tracker to compensate for rover motion during clustering.
* **What Perception Provides to SLAM:**
  * **ArUco 3D Target Pose (`/perception/aruco_pose` - `geometry_msgs/msg/PoseStamped`):** Measured via OpenCV SolvePnP. When RTAB-Map sees a known marker ID, it triggers an instant **hard drift reset** on the global `map &rarr; odom` transform.

### B. Contract with Path Planning Module (`PathPlaning/PathPlanner_Docu/PathPlannerGuide.html`)
* **What Path Planning Needs from Perception:**
  * **Persistent 3D Bounding Boxes (`/perception/obstacles_only` - `vision_msgs/msg/Detection3DArray`):** High-level 3D rock obstacles ingested by Nav2's `costmap_2d` Obstacle Layer for real-time MPPI local steering and Smac Hybrid A* global path planning.
  * **Local Costmap (`/terrain/costmap` - `nav_msgs/msg/OccupancyGrid`):** 2D occupancy grid rasterized at $5\text{cm}$ cell resolution, pre-inflated by the robot footprint radius ($0.3\text{m}$) plus safety buffer ($0.6\text{m}$).
* **Key Constraints for Path Planning:**
  * **Zero Latency Jumps:** Perception obstacle topics must be smoothed (via Persistent Memory Node's EMA filter) so MPPI steering controllers do not experience coordinate stutter.
  * **Frame Consistency:** All obstacle bounding boxes published to Nav2 must specify `header.frame_id = "base_link"` or `"odom"`.

---

## 📂 2. Full End-State Folder Structure

```
Autonmous_Ws/Perception/
├── PerceptionAiGuide.md                     # [THIS FILE] AI Roadmap & Master Reference
├── perception_Docu/
│   └── perception.html                      # Interactive Glassmorphism Architecture Report
├── ros dataset/                             # Recorded RealSense D435 bag files for testing
│
└── terrain_geometry_improved/               # Core Perception ROS 2 Package Root
    ├── terrain_geometry_msgs/               # Custom ROS 2 Interface Package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       ├── ObstacleFeature.msg          # Single obstacle telemetry (centroid, size, distance)
    │       └── ObstacleFeatureArray.msg     # Array of detected obstacle features
    │
    └── terrain_geometry/                    # Primary Perception Package
        ├── CMakeLists.txt / setup.py
        ├── package.xml
        ├── launch/
        │   ├── terrain.launch.py            # Standalone Terrain Geometry Launcher
        │   └── perception_system.launch.py  # Unified System Launcher (All 3 Nodes)
        ├── config/
        │   ├── terrain_params.yaml          # Terrain Node Parameters (Ground, Voxel, DBSCAN)
        │   └── perception_params.yaml       # Central System Parameters & QoS Profiles
        ├── test/
        │   ├── test_ground_removal.py       # Unit test for RANSAC / Patchwork++ backends
        │   ├── test_clustering.py           # Unit test for DBSCAN cluster extraction
        │   ├── test_persistent_memory.py    # Integration test for EMA rock memory retention
        │   └── test_aruco_pnp.py            # Unit test for OpenCV SolvePnP 3D pose math
        └── terrain_geometry/                # Python Source Code
            ├── __init__.py
            ├── terrain_node.py              # Node 1: Terrain Geometry & Clustering Executable
            ├── persistent_memory_node.py    # Node 2: Persistent Rock Memory Executable
            ├── aruco_detector_node.py       # Node 3: ArUco 3D Vision Executable
            ├── roi_filter.py                # ROI Spatial Filter (base_link Crop Box)
            ├── ground_removal.py            # Ground Plane Removal (Patchwork++ / RANSAC)
            ├── voxel_filter.py              # 3D Voxel Grid Downsampling (5cm Leaf Size)
            ├── outlier_filter.py            # Radius Outlier Removal (ROR)
            ├── clustering.py                # DBSCAN Euclidean Clustering Engine
            ├── persistent_database.py       # EMA Rock Memory & Spatial Association Engine
            ├── aruco_pnp_solver.py          # OpenCV Corner Detector & SolvePnP Math
            ├── obstacle_features.py         # 3D Bounding Box & Telemetry Generator
            ├── obstacle_tracking.py         # Single-frame Nearest-Neighbor Associate
            ├── occupancy_grid.py            # 2D Grid Rasterizer
            ├── costmap_inflation.py         # Exponential Cost Inflation Buffer
            ├── tf_transform.py              # Coordinate Transform Utilities (tf2)
            ├── visualization.py             # RViz2 3D MarkerArray Publisher
            ├── benchmark.py                 # Algorithmic Benchmark Suite
            └── performance_profiling.py     # Real-time Execution Profiler
```

---

## 📊 3. Implementation Audit: What is DONE vs. TO-DO

| Module / Component | Code File | Status | Description |
| :--- | :--- | :---: | :--- |
| **Main Terrain Node** | `terrain_node.py` | ✅ **DONE** | Orchestrates point cloud ingestion, processing pipeline, and publishing. |
| **Ground Removal** | `ground_removal.py` | ✅ **DONE** | Implements Patchwork++ and RANSAC ground plane segmentation. |
| **Voxel Downsampling** | `voxel_filter.py` | ✅ **DONE** | Reduces point cloud density by 90% using a 5cm voxel leaf size. |
| **Outlier Filter** | `outlier_filter.py` | ✅ **DONE** | Radius Outlier Removal (ROR) strips airborne noise and dust. |
| **DBSCAN Clustering** | `clustering.py` | ✅ **DONE** | Groups floating obstacle points into physical 3D rock clusters. |
| **Costmap Inflation** | `costmap_inflation.py` | ✅ **DONE** | Applies robot footprint radius inflation with exponential decay. |
| **Custom Messages** | `terrain_geometry_msgs` | ✅ **DONE** | Provides `ObstacleFeature.msg` and `ObstacleFeatureArray.msg`. |
| **Standardized Bounding Boxes** | `terrain_node.py` | 🔲 **Task 1A Series** | Update node to publish standard `vision_msgs/msg/Detection3DArray`. |
| **Persistent Memory Node** | `persistent_memory_node.py`| 🔲 **Task 2A Series** | Build database node using EMA smoothing for rock memory retention. |
| **ArUco Detector Node** | `aruco_detector_node.py` | 🔲 **Task 3A Series** | Build OpenCV SolvePnP node for 3D marker distance & pose estimation. |
| **Central System Launcher** | `perception_system.launch.py`| 🔲 **Task 4A Series** | Unified launch script to spin up all 3 perception nodes concurrently. |
| **System Validation & Test** | `test/` | 🔲 **Task 5A Series** | Automated unit tests and end-to-end simulation validation. |

---

## 🎯 4. Master Task Roadmap & GitHub Issue Guide

Below is the complete task roadmap. Each task entry contains its full GitHub issue body, implementation guidelines, git workflow, and verification steps.

---

### - [ ] 📦 [Perception] [Task 1A.1 & 1A.2] Standardize Interface Messages to vision_msgs

#### 📌 Issue Summary & Objective
Standardize the obstacle detection output messages of `terrain_geometry` by converting internal bounding box structures (`ObstacleFeatureArray`) into ROS 2 standard `vision_msgs/msg/Detection3DArray` messages. This prepares raw obstacle streams for ingestion by the Persistent Memory Node and Nav2.

---

#### 🌿 Git Branch & Workflow Instructions
```bash
# 1. Ensure main branch is up to date
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
git checkout main
git pull origin main

# 2. Create feature branch
git checkout -b perception/task-1a-vision-msgs

# 3. Commit changes (after implementing)
git add Perception/terrain_geometry_improved/
git commit -m "feat(perception): add vision_msgs Detection3DArray publisher to terrain_node"

# 4. Push branch and open Pull Request
git push -u origin perception/task-1a-vision-msgs
```

---

#### 🛠️ Implementation Overview & Guidelines
* **Objective:** Update `terrain_geometry` package dependencies to depend on `vision_msgs`.
* **Data Transformation:** Design a conversion mechanism (e.g. helper function or class method in `obstacle_features.py`) to transform detected 3D bounding box features into `vision_msgs/msg/Detection3DArray`.
* **Field Mapping Guidelines:** Ensure mapped 3D bounding boxes populate center position ($X, Y, Z$) and dimensions ($dx, dy, dz$). You are free to design the data conversion pipeline in whatever way is cleanest and most efficient.
* **Topic Publisher:** Update `terrain_node.py` to construct a publisher streaming these standardized bounding boxes on topic `/perception/local_bboxes` using reliable QoS.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Build workspace
cd /home/saif/Desktop/MESEKET/Autonmous-27
colcon build --packages-select terrain_geometry
source install/setup.bash

# Step 2: Run Terrain Node
ros2 launch terrain_geometry terrain.launch.py

# Step 3: Echo topic in new terminal
ros2 topic echo /perception/local_bboxes vision_msgs/msg/Detection3DArray
```
* **Expect to see:** Terminal prints valid `vision_msgs/msg/Detection3DArray` messages containing 3D bounding box centroids and sizes matching physical rocks in front of the rover.

---

### - [ ] 🎯 [Perception Checkpoint 1] Validate Raw Local Bounding Box Interface

#### 📌 Milestone Gate Description
Validate that `/perception/local_bboxes` streams clean `vision_msgs/msg/Detection3DArray` messages at $\ge 10\text{ Hz}$ in Gazebo simulation with correct frame headers (`base_link`) without memory leaks or dropped frames.

---

#### 🛠️ Verification Implementation Overview
Verify that raw local bounding boxes published on `/perception/local_bboxes` conform to ROS 2 standard `vision_msgs` specifications and maintain frame ID alignment with `base_link`.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Launch simulation
ros2 launch my_robot_description gazebo.launch.py world:=world1.world

# Step 2: Launch terrain node
ros2 launch terrain_geometry terrain.launch.py

# Step 3: Verify publishing frequency
ros2 topic hz /perception/local_bboxes

# Step 4: Confirm frame ID
ros2 topic echo /perception/local_bboxes --field header.frame_id
```
* **Expect to see:** Frequency $\ge 10\text{ Hz}$ and frame ID equals `base_link` with reliable 3D bounding boxes corresponding accurately to physical Gazebo rocks.

---

### - [ ] 📦 [Perception] [Task 2A.1 & 2A.2] Spatial Association & EMA Rock Position Smoothing

#### 📌 Issue Summary & Objective
Design and implement a spatial tracking database (`persistent_database.py`) that performs 3D spatial data association and position smoothing to eliminate camera jitter on detected rocks.

---

#### 🌿 Git Branch & Workflow Instructions
```bash
git checkout main
git checkout -b perception/task-2a-ema-database
git add Perception/terrain_geometry_improved/
git commit -m "feat(perception): implement persistent database spatial association and EMA smoothing"
git push -u origin perception/task-2a-ema-database
```

---

#### 🛠️ Implementation Overview & Guidelines
* **Objective:** Create a spatial database module in `terrain_geometry/persistent_database.py` capable of tracking detected rocks across consecutive frames.
* **Spatial Association:** Implement a 3D distance matching technique (such as Euclidean nearest-neighbor, Hungarian algorithm, or spatial KD-tree) to correlate incoming raw bounding boxes (`/perception/local_bboxes`) with previously registered tracks.
* **Position Smoothing:** Apply a smoothing algorithm (such as Exponential Moving Average or Kalman Filtering) to smooth position noise and camera jitter.
* **Flexibility Note:** Feel free to choose the optimal data structures, distance thresholds, or smoothing weights ($\alpha$) that yield the smoothest tracking results.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Run unit test script for persistent database
cd /home/saif/Desktop/MESEKET/Autonmous-27
colcon build --packages-select terrain_geometry
source install/setup.bash
python3 -m unittest terrain_geometry.test.test_persistent_memory
```
* **Expect to see:** All unit tests pass, confirming noisy detection inputs settle smoothly onto true rock coordinates without spawning duplicate tracks.

---

### - [ ] 📦 [Perception] [Task 2A.3 & 2A.4] Persistent Memory Node & Blind Spot Retention

#### 📌 Issue Summary & Objective
Build `persistent_memory_node.py` to ingest raw bounding boxes (`/perception/local_bboxes`) and output persistent rock memory (`/perception/obstacles_only`) with a memory retention buffer (Time-to-Live / TTL) to maintain rock obstacles when the camera rotates away.

---

#### 🌿 Git Branch & Workflow Instructions
```bash
git checkout main
git checkout -b perception/task-2a-memory-node
git add Perception/terrain_geometry_improved/
git commit -m "feat(perception): add persistent_memory_node with TTL blind spot retention"
git push -u origin perception/task-2a-memory-node
```

---

#### 🛠️ Implementation Overview & Guidelines
* **Objective:** Construct an executable ROS 2 node (`persistent_memory_node.py`) that maintains obstacle memory when objects leave the camera's active field of view.
* **Data Ingestion:** Subscribe to `/perception/local_bboxes` (`vision_msgs/msg/Detection3DArray`).
* **Memory Retention:** Implement a retention mechanism (such as time-based TTL counters or frame decay) so that obstacles remain active in memory for a configurable duration (e.g. 5 seconds) after being unobserved.
* **Topic Publishing:** Publish persistent obstacle arrays on `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`) and publish visual 3D bounding box markers on `/terrain/obstacle_markers`.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Launch nodes
ros2 run terrain_geometry terrain_node &
ros2 run terrain_geometry persistent_memory_node

# Step 2: Test retention in Gazebo
# Action: Drive rover towards a rock, then rotate camera 90 degrees away into a blind spot.
ros2 topic echo /perception/obstacles_only
```
* **Expect to see:** Topic `/perception/obstacles_only` continues publishing the rock's smoothed coordinates for 5 seconds after it leaves the camera view.

---

### - [ ] 🎯 [Perception Checkpoint 2] Validate Memory Retention & Spatial Association

#### 📌 Milestone Gate Description
Confirm that the Persistent Memory Node eliminates camera jitter, avoids duplicate tracks, and maintains blind-spot obstacles for Nav2 costmap overlays.

---

#### 🛠️ Verification Implementation Overview
Validate spatial association and memory retention by verifying that the Persistent Memory Node smooths position jitter and maintains unobserved obstacles for 5 seconds.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Verify topic publishing rate
ros2 topic hz /perception/obstacles_only

# Step 2: Display markers in RViz2
rviz2

# Step 3: Drive rover near rocks in Gazebo and rotate camera 90 degrees away
```
* **Expect to see:** Topic publishing rate is steady at $10\text{ Hz}$, and rock markers remain static and visible on RViz map display when camera rotates away into blind spots.

---

### - [ ] 📦 [Perception] [Task 3A.1 & 3A.2] OpenCV ArUco Corner Extraction & SolvePnP Solver

#### 📌 Issue Summary & Objective
Implement `aruco_pnp_solver.py` utilizing OpenCV ArUco detection and Perspective-n-Point (`SolvePnP`) algorithms to compute 3D translation ($X, Y, Z$) and 6-DOF orientation of mission target markers using camera intrinsics.

---

#### 🌿 Git Branch & Workflow Instructions
```bash
git checkout main
git checkout -b perception/task-3a-aruco-solver
git add Perception/terrain_geometry_improved/
git commit -m "feat(perception): implement ArUco corner extraction and SolvePnP 3D pose math"
git push -u origin perception/task-3a-aruco-solver
```

---

#### 🛠️ Implementation Overview & Guidelines
* **Objective:** Design an ArUco 3D pose estimation module in `terrain_geometry/aruco_pnp_solver.py`.
* **Marker Extraction:** Use OpenCV's ArUco module to locate marker 2D corner pixels in RGB image frames.
* **3D Pose Solver:** Solve the Perspective-n-Point (PnP) math using camera intrinsic parameters ($f_x, f_y, c_x, c_y$) and known physical marker dimensions to calculate 3D translation vector ($X, Y, Z$) and rotation.
* **Flexibility Note:** You can choose any suitable OpenCV PnP solver algorithm (e.g. `SOLVEPNP_IPPE_SQUARE`, `SOLVEPNP_ITERATIVE`) and implement subpixel corner refinement or noise filtering as you see fit.

---

#### 🧪 Verification & Testing Guide
```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27
colcon build --packages-select terrain_geometry
source install/setup.bash
python3 -m unittest terrain_geometry.test.test_aruco_pnp
```
* **Expect to see:** Unit test passes, verifying calculated 3D distance matches ground truth pixel projection to within $< 1\text{cm}$ error.

---

### - [ ] 📦 [Perception] [Task 3A.3 & 3A.4] ArUco Detector Node & Target Pose Publisher

#### 📌 Issue Summary & Objective
Construct `aruco_detector_node.py` subscribing to `/camera/color/image_raw` and `/camera/camera_info` to publish target marker 3D pose on `/perception/aruco_pose` (`geometry_msgs/msg/PoseStamped`) for Saif SLAM loop closures.

---

#### 🌿 Git Branch & Workflow Instructions
```bash
git checkout main
git checkout -b perception/task-3a-aruco-node
git add Perception/terrain_geometry_improved/
git commit -m "feat(perception): implement aruco_detector_node and pose publisher"
git push -u origin perception/task-3a-aruco-node
```

---

#### 🛠️ Implementation Overview & Guidelines
* **Objective:** Create an executable ROS 2 node (`aruco_detector_node.py`) that processes color camera streams to output 3D target poses.
* **Image Bridge:** Use `cv_bridge` to convert incoming ROS image streams (`/camera/color/image_raw`) into OpenCV-compatible image matrices.
* **Camera Calibration:** Extract camera intrinsic parameters from `/camera/camera_info`.
* **Output Topic:** Publish detected marker 3D target poses on `/perception/aruco_pose` (`geometry_msgs/msg/PoseStamped`) for Saif SLAM landmark reset events and state machine mission navigation.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Launch Gazebo world with ArUco marker
ros2 launch worlds world_Rotated_Aruco.launch.py &

# Step 2: Run Node
ros2 run terrain_geometry aruco_detector_node

# Step 3: Echo pose
ros2 topic echo /perception/aruco_pose geometry_msgs/msg/PoseStamped
```
* **Expect to see:** Terminal outputs a valid `PoseStamped` message detailing exact 3D distance ($X$ meters ahead, $Y$ offset) matching Gazebo marker location.

---

### - [ ] 🎯 [Perception Checkpoint 3] Validate ArUco 3D Pose Estimation Accuracy

#### 📌 Milestone Gate Description
Verify that the ArUco vision node accurately estimates 3D target distance and pose without relying on depth sensor IR noise.

---

#### 🛠️ Verification Implementation Overview
Validate ArUco 3D distance estimation accuracy by comparing published `PoseStamped` topics against ground truth Gazebo marker coordinates.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Echo topic
ros2 topic echo /perception/aruco_pose

# Step 2: Measure 3D translation (X, Y, Z) error against Gazebo ground truth marker coordinates
```
* **Expect to see:** Topic `/perception/aruco_pose` updates cleanly whenever an ArUco marker enters camera field-of-view, with 3D translation error $\le 2\text{cm}$ at $2.0\text{m}$ distance without coordinate jumps.

---

### - [ ] 📦 [Perception] [Task 4A.1 & 4A.2] Create Centralized YAML Parameters & System Launcher

#### 📌 Issue Summary & Objective
Create `config/perception_params.yaml` and `launch/perception_system.launch.py` to spin up `terrain_node`, `persistent_memory_node`, and `aruco_detector_node` concurrently with parameter management and QoS profiles.

---

#### 🌿 Git Branch & Workflow Instructions
```bash
git checkout main
git checkout -b perception/task-4a-launcher
git add Perception/terrain_geometry_improved/
git commit -m "feat(perception): add centralized parameters and perception_system.launch.py"
git push -u origin perception/task-4a-launcher
```

---

#### 🛠️ Implementation Overview & Guidelines
* **Objective:** Centralize parameters and create a unified launch script for the perception subsystem.
* **Parameter File:** Create `config/perception_params.yaml` consolidating configuration parameters for all three nodes (ROI spatial bounds, voxel leaf size, DBSCAN epsilon, EMA alpha weights, camera topics, QoS settings).
* **Launch Script:** Create `launch/perception_system.launch.py` using ROS 2 `LaunchDescription` and `Node` actions to start all 3 nodes concurrently with parameter loading.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Launch entire perception system
colcon build --packages-select terrain_geometry
source install/setup.bash
ros2 launch terrain_geometry perception_system.launch.py

# Step 2: Verify active nodes
ros2 node list
```
* **Expect to see:** Nodes `/terrain_node`, `/persistent_memory_node`, and `/aruco_detector_node` are all running concurrently without parameter parsing errors or QoS mismatch warnings.

---

### - [ ] 🎯 [Perception Checkpoint 4] Validate System Launcher & QoS Compatibility

#### 📌 Milestone Gate Description
Confirm all three nodes execute concurrently from a single launch file without camera Best Effort / Reliable QoS mismatch warnings or memory leaks.

---

#### 🛠️ Verification Implementation Overview
Validate system launch integrity, ROS 2 QoS compatibility, and compute resource utilization across all perception nodes executing together.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Run ROS 2 Doctor for QoS report
ros2 doctor --report

# Step 2: Monitor CPU and RAM overhead in htop
htop
```
* **Expect to see:** Zero QoS mismatch warnings on camera topics, and total CPU load remains below $40\%$ on the compute stack.

---

### - [ ] 📦 [Perception] [Task 5A.1 & 5A.2] Validate End-to-End Nav2 Integration & Performance Benchmarking

#### 📌 Issue Summary & Objective
Perform end-to-end integration testing in Gazebo Mars Yard simulation, verifying that persistent rock obstacles on `/perception/obstacles_only` overlay onto Nav2's `global_costmap`, and run `benchmark.py` to log real-time performance.

---

#### 🌿 Git Branch & Workflow Instructions
```bash
git checkout main
git checkout -b perception/task-5a-integration
git add Perception/terrain_geometry_improved/
git commit -m "test(perception): complete end-to-end Nav2 integration and performance benchmarking"
git push -u origin perception/task-5a-integration
```

---

#### 🛠️ Implementation Overview & Guidelines
* **Objective:** Perform end-to-end integration testing and profiling of the full perception pipeline.
* **Nav2 Costmap Integration:** Verify closed-loop compatibility with Gazebo (`world1.world`) and confirm Nav2 `costmap_2d` server subscribes to `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`) and paints inflated obstacle cost buffers.
* **Benchmarking:** Execute `benchmark.py` to record per-stage execution timing (Ground Removal, Voxel Grid, DBSCAN) and output log to `perception_Docu/benchmark_report.txt`.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Launch simulation + Nav2 + Perception
ros2 launch my_robot_description gazebo.launch.py world:=world1.world &
ros2 launch terrain_geometry perception_system.launch.py &

# Step 2: Open RViz2
rviz2
```
* **Expect to see:** Rocks in Gazebo appear as solid red inflated cost zones on the RViz costmap display. The rover navigates autonomously around rocks without collision while keeping total pipeline latency $\le 60\text{ms}$.

---

### - [ ] 🏆 [Perception Final Gateway] Validate Full Subsystem Integration & Performance Metrics

#### 📌 Milestone Gate Description
Final sign-off verifying zero collisions during a 15-minute autonomous navigation run in Gazebo Mars Yard world, confirming competition readiness.

---

#### 🛠️ Verification Implementation Overview
Perform final subsystem gateway verification confirming that all perception nodes, memory retention, ArUco 3D vision, and Nav2 costmap overlays operate flawlessly.

---

#### 🧪 Verification & Testing Guide
```bash
# Step 1: Execute 15-minute autonomous navigation run in Gazebo
ros2 launch my_robot_description gazebo.launch.py world:=world1.world &
ros2 launch terrain_geometry perception_system.launch.py

# Step 2: Inspect benchmark log report
cat /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws/Perception/perception_Docu/benchmark_report.txt
```
* **Expect to see:** Complete autonomous perception pipeline operates cleanly without crashes or memory leaks, achieving $100\%$ obstacle avoidance success rate over a 15-minute testing run.

---

## 🛠️ 5. AI Developer Maintenance Instructions
When implementing or editing code in this repository:
1. Open this file (`PerceptionAiGuide.md`).
2. Mark completed sub-tasks using `- [x]`.
3. Update Section 3 (**Implementation Audit**) table.
4. Keep all issue titles and test commands aligned with the workspace.
