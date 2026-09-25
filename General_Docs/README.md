# General Architecture & Engineering Documentation (`General_Docs/`)

This directory houses system-level diagrams, hardware component specifications, pipeline graphs, and architectural guides for the Autonomous-27 rover.

---

## 📌 Document & Architecture Diagram Catalog

### 🌐 1. System-Wide Architectural Overviews
| Document / Asset | Description | Scope |
| :--- | :--- | :--- |
| [`rover_architecture.html`](rover_architecture.html) | Interactive 3D/Visual Rover System Blueprint and sub-system walkthrough | Master Blueprint |
| [`FlowWTopics.png`](FlowWTopics.png) | Master inter-node ROS 2 message routing & topic connections | Full System Routing |
| [`rover_detailed_pipeline.png`](rover_detailed_pipeline.png) | End-to-end node architecture and internal algorithmic data flow | System Pipeline |
| [`rover_blackbox_pipeline.png`](rover_blackbox_pipeline.png) | High-level module boundary and external sensor/actuator contracts | System Blackbox |
| [`tf_tree_architecture.png`](tf_tree_architecture.png) | REP-105 coordinate transform frame hierarchy (`map` ➔ `odom` ➔ `base_link`) | TF Coordinate Tree |

### 🧩 2. Subsystem Master Pipelines
| Diagram | Subsystem | Description |
| :--- | :--- | :--- |
| [`slam_pipeline.png`](slam_pipeline.png) | **SLAM** | Wheel kinematics, slip checker, 100Hz EKF fusion, and RTAB-Map SLAM |
| [`perception_pipeline.png`](perception_pipeline.png) | **Perception** | 3D point cloud terrain geometry & ArUco solvePnP landmark tracking |
| [`path_planning_pipeline.png`](path_planning_pipeline.png) | **Nav2** | Costmap bridge, Smac Hybrid A* planner, MPPI controller, and Behavior Trees |
| [`hardware_control_pipeline.png`](hardware_control_pipeline.png) | **Hardware** | Jetson Orin Nano, STM32 Blackpill ECU, PDB, 4WD motors, and sensor buses |

### 🔬 3. Deep-Dive Subsystem Block Diagrams
| Diagram | Focus Area | Description |
| :--- | :--- | :--- |
| [`slam_slip_and_ekf_block.png`](slam_slip_and_ekf_block.png) | **SLAM Blocks 1, 2, 4** | Differential kinematics, heuristic slip covariance inflation, and 100Hz local EKF |
| [`slam_rtabmap_costmap_block.png`](slam_rtabmap_costmap_block.png) | **SLAM Blocks 3, 5, 6** | RealSense depth filtering, RTAB-Map visual graph SLAM, and Nav2 Costmap 2D |
| [`perception_terrain_geometry_block.png`](perception_terrain_geometry_block.png) | **Perception PCL** | 6-stage spatial crop, Patchwork++ ground removal, DBSCAN, and EMA memory |
| [`perception_aruco_vision_block.png`](perception_aruco_vision_block.png) | **Perception ArUco** | Sub-pixel corner detection, OpenCV solvePnP 6-DoF solver, and Kalman gating |
| [`nav2_mppi_dynamic_avoidance_block.png`](nav2_mppi_dynamic_avoidance_block.png) | **Nav2 MPPI & BT** | 2,000 trajectory rollout evaluations @ 20Hz, dynamic avoidance, and recovery loops |

### 📚 4. Engineering Specifications & Guides
| Document | Description | Format |
| :--- | :--- | :--- |
| [`hardware_specs/`](hardware_specs/) | Electrical schematics, motor specs, battery distribution, and mechanical constraints | Markdown / Specs |
| [`humble_to_jazzy.md`](humble_to_jazzy.md) | ROS 2 Jazzy (Ubuntu 24.04) migration changes and API compatibility notes | Markdown |
| [`Docker_Beginners_Guide.pdf`](Docker_Beginners_Guide.pdf) | Containerized development environment tutorial | PDF |

---

## 🌐 Coordinate Systems & TF Tree (REP-105)

```
map (Global World / RTAB-Map SLAM)
 └── odom (Continuous Smooth Local Odometry / robot_localization EKF)
      └── base_link (Rover Kinematic Center)
           ├── chassis
           ├── camera_link
           │    ├── camera_depth_frame -> camera_depth_optical_frame
           │    └── my_robot/camera_link/camera (Gazebo sensor bridge)
           └── imu_link
```
