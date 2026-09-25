# General Architecture & Engineering Documentation (`General_Docs/`)

This directory houses system-level diagrams, sub-system architectural pipelines, granular block deep-dives, hardware component specifications, and interactive blueprints for the Autonomous-27 rover.

---

## 📂 Subsystem Diagram Directory Structure

```text
General_Docs/
├── FlowWTopics.png                           # Master ROS 2 Inter-Node Topic & Message Routing
├── rover_detailed_pipeline.png               # End-to-End Algorithmic System Pipeline
├── rover_blackbox_pipeline.png               # High-Level Subsystem Boundary Contracts
├── tf_tree_architecture.png                  # System REP-105 Coordinate Frame Hierarchy
├── rover_architecture.html                   # Interactive 3D/Visual Rover Blueprint
├── Docker_Beginners_Guide.pdf                # Containerized Environment Guide
│
├── 1-Perception/                             # 👁️ Perception Subsystem Diagrams
│   ├── 00_perception_master_pipeline.png
│   ├── 01_terrain_geometry_6stage_pipeline.png
│   ├── 02_roi_and_patchwork_ground_removal.png
│   ├── 03_voxel_downsampling_and_outlier_removal.png
│   ├── 04_dbscan_clustering_and_bounding_boxes.png
│   ├── 05_persistent_memory_and_ema_tracker.png
│   ├── 06_aruco_detection_and_pnp_block.png
│   └── 07_terrain_costmap_generation.png
│
├── 2-SLAM/                                   # 🧭 SLAM & State Estimation Diagrams
│   ├── 00_slam_master_pipeline.png
│   ├── 01_encoder_ticks_to_odom_kinematics.png
│   ├── 02_heuristic_slip_checker_and_covariance.png
│   ├── 03_realsense_depth_postprocessing_filters.png
│   ├── 04_robot_localization_ekf_100hz_fusion.png
│   ├── 05_rtabmap_visual_slam_and_loop_closure.png
│   └── 06_nav2_costmap_2d_layering.png
│
├── 3-Nav2/                                   # 🗺️ Nav2 Navigation & Path Planning Diagrams
│   ├── 00_nav2_master_pipeline.png
│   ├── 01_costmap_bridge_ingestion_block.png
│   ├── 02_global_and_local_costmap_servers.png
│   ├── 03_smac_hybrid_a_star_planner_block.png
│   ├── 04_mppi_controller_rollouts_block.png
│   ├── 05_behavior_tree_replanning_block.png
│   └── 06_velocity_smoother_and_skid_steer_bridge.png
│
└── 4-Hardware/                               # ⚙️ Hardware, Electrical & Power Architecture
    ├── 00_hardware_master_architecture.png
    ├── 01_power_distribution_block.png
    ├── 02_master_compute_and_microcontroller_bridge.png
    ├── 03_drivetrain_and_actuators_block.png
    ├── 04_sensor_suite_and_buses_block.png
    ├── 05_rep105_coordinate_tf_tree.png
    ├── hardware_specs.md
    └── index.html
```

---

## 📌 Complete Architecture Diagram Catalog

### 🌐 1. System-Wide Overviews (Root)
| Diagram / Asset | Description | Scope |
| :--- | :--- | :--- |
| [`FlowWTopics.png`](FlowWTopics.png) | Master inter-node ROS 2 message routing, topics, and message types | Full System Routing |
| [`rover_detailed_pipeline.png`](rover_detailed_pipeline.png) | End-to-end algorithmic pipeline connecting sensors, filters, SLAM, and planning | System Data Flow |
| [`rover_blackbox_pipeline.png`](rover_blackbox_pipeline.png) | High-level module boundaries and external sensor/actuator contracts | System Blackbox |
| [`tf_tree_architecture.png`](tf_tree_architecture.png) | Complete REP-105 coordinate transform hierarchy (`map` ➔ `odom` ➔ `base_link`) | TF Coordinate Tree |
| [`rover_architecture.html`](rover_architecture.html) | Interactive 3D Rover Blueprint, subsystem breakdown & live parameters | Master UI Blueprint |

---

### 👁️ 2. Perception Subsystem (`1-Perception/`)
| Diagram | Stage / Focus | Description |
| :--- | :--- | :--- |
| [`00_perception_master_pipeline.png`](1-Perception/00_perception_master_pipeline.png) | **Master Pipeline** | Dual-stream architecture: 3D point cloud terrain geometry & ArUco solvePnP |
| [`01_terrain_geometry_6stage_pipeline.png`](1-Perception/01_terrain_geometry_6stage_pipeline.png) | **PCL 6-Stage** | Raw point cloud filtering, ground segmentation, downsampling, clustering & memory |
| [`02_roi_and_patchwork_ground_removal.png`](1-Perception/02_roi_and_patchwork_ground_removal.png) | **Blocks 1 & 2** | Camera-to-base spatial crop & Patchwork++ Concentric Zone ground plane separation |
| [`03_voxel_downsampling_and_outlier_removal.png`](1-Perception/03_voxel_downsampling_and_outlier_removal.png) | **Blocks 3 & 4** | 5cm uniform voxel downsampling and kd-Tree radius outlier noise removal |
| [`04_dbscan_clustering_and_bounding_boxes.png`](1-Perception/04_dbscan_clustering_and_bounding_boxes.png) | **Block 5** | Euclidean DBSCAN clustering (&epsilon;=0.12m, min=15), centroid and 3D bounding boxes |
| [`05_persistent_memory_and_ema_tracker.png`](1-Perception/05_persistent_memory_and_ema_tracker.png) | **Block 6** | EMA obstacle position filtering (&alpha;=0.7), track association, and blind-spot memory |
| [`06_aruco_detection_and_pnp_block.png`](1-Perception/06_aruco_detection_and_pnp_block.png) | **ArUco Vision** | Sub-pixel corner detection, OpenCV solvePnP 6-DoF solver, and covariance gate |
| [`07_terrain_costmap_generation.png`](1-Perception/07_terrain_costmap_generation.png) | **Costmap Out** | Direct 2D grid rasterization (`/terrain/costmap`) with safety radius inflation |

---

### 🧭 3. SLAM & State Estimation Subsystem (`2-SLAM/`)
| Diagram | Stage / Focus | Description |
| :--- | :--- | :--- |
| [`00_slam_master_pipeline.png`](2-SLAM/00_slam_master_pipeline.png) | **Master Pipeline** | Complete 6-block architecture: kinematics, slip rejection, 100Hz EKF, RTAB-Map |
| [`01_encoder_ticks_to_odom_kinematics.png`](2-SLAM/01_encoder_ticks_to_odom_kinematics.png) | **Block 1** | Differential drive kinematics, 4-wheel tick deltas, and single-wheel slip isolation |
| [`02_heuristic_slip_checker_and_covariance.png`](2-SLAM/02_heuristic_slip_checker_and_covariance.png) | **Block 2** | Wheel yaw vs IMU gyro comparison, linear velocity clamping, covariance inflation ($10^3$) |
| [`03_realsense_depth_postprocessing_filters.png`](2-SLAM/03_realsense_depth_postprocessing_filters.png) | **Block 3** | Decimation, spatial smoothing, temporal persistence, hole-filling, and 4m clipping |
| [`04_robot_localization_ekf_100hz_fusion.png`](2-SLAM/04_robot_localization_ekf_100hz_fusion.png) | **Block 4** | 15-state EKF sensor fusion @ 100Hz, publishing continuous `odom ➔ base_link` TF |
| [`05_rtabmap_visual_slam_and_loop_closure.png`](2-SLAM/05_rtabmap_visual_slam_and_loop_closure.png) | **Block 5** | FAST/GFTT visual memory, loop closure graph optimization, ArUco landmark constraints |
| [`06_nav2_costmap_2d_layering.png`](2-SLAM/06_nav2_costmap_2d_layering.png) | **Block 6** | Static SLAM map ingestion, obstacle cloud raytracing, and exponential safety inflation |

---

### 🗺️ 4. Nav2 Navigation & Path Planning Subsystem (`3-Nav2/`)
| Diagram | Stage / Focus | Description |
| :--- | :--- | :--- |
| [`00_nav2_master_pipeline.png`](3-Nav2/00_nav2_master_pipeline.png) | **Master Pipeline** | Smac Hybrid A* planner, MPPI controller, costmap bridge, and Behavior Trees |
| [`01_costmap_bridge_ingestion_block.png`](3-Nav2/01_costmap_bridge_ingestion_block.png) | **Block 1** | Sampling 3D bounding box perimeters into dense synthetic PointCloud2 for costmaps |
| [`02_global_and_local_costmap_servers.png`](3-Nav2/02_global_and_local_costmap_servers.png) | **Block 2** | 50x50m global costmap vs 10x10m rolling local costmap in `odom` frame |
| [`03_smac_hybrid_a_star_planner_block.png`](3-Nav2/03_smac_hybrid_a_star_planner_block.png) | **Block 3** | Non-holonomic 3D search space (X, Y, &theta;) generating feasible Reeds-Shepp routes |
| [`04_mppi_controller_rollouts_block.png`](3-Nav2/04_mppi_controller_rollouts_block.png) | **Block 4** | 2,000 parallel rollouts @ 20Hz, cost critics (Goal, PathAlign, Obstacle, Constraints) |
| [`05_behavior_tree_replanning_block.png`](3-Nav2/05_behavior_tree_replanning_block.png) | **Block 5** | Navigation orchestrator: dynamic hazard detection, detours, recovery sequences |
| [`06_velocity_smoother_and_skid_steer_bridge.png`](3-Nav2/06_velocity_smoother_and_skid_steer_bridge.png) | **Block 6** | Acceleration ramping, safety envelopes, and differential/skid-steer motor PWM |

---

### ⚙️ 5. Hardware, Electrical & Power Architecture (`4-Hardware/`)
| Diagram / Spec | Stage / Focus | Description |
| :--- | :--- | :--- |
| [`00_hardware_master_architecture.png`](4-Hardware/00_hardware_master_architecture.png) | **Master Hardware** | 20V battery, PDB, Jetson Orin Nano, STM32 Blackpill, 4WD DC motors, sensors |
| [`01_power_distribution_block.png`](4-Hardware/01_power_distribution_block.png) | **Power & PDB** | High-current battery distribution, 19V/12V buck-boost, 5V/3.3V logic regulation, E-Stop |
| [`02_master_compute_and_microcontroller_bridge.png`](4-Hardware/02_master_compute_and_microcontroller_bridge.png) | **Compute Bridge** | Jetson (ROS 2 master) <-> STM32 (real-time motor PWM & encoders) via USB serial |
| [`03_drivetrain_and_actuators_block.png`](4-Hardware/03_drivetrain_and_actuators_block.png) | **Drivetrain** | 4x planetary DC motors, 4x H-Bridge drivers, 4x quadrature optical encoders |
| [`04_sensor_suite_and_buses_block.png`](4-Hardware/04_sensor_suite_and_buses_block.png) | **Sensors & Buses** | RealSense D435i (USB 3.0), BNO055 IMU (I2C/UART), Wheel Encoders (GPIO Interrupts) |
| [`05_rep105_coordinate_tf_tree.png`](4-Hardware/05_rep105_coordinate_tf_tree.png) | **TF Tree** | Strict separation: `map` (SLAM) ➔ `odom` (continuous EKF) ➔ `base_link` ➔ sensors |
| [`hardware_specs.md`](4-Hardware/hardware_specs.md) | **Hardware Specs** | Pinouts, motor specifications, battery sizing, and electrical constraints |
