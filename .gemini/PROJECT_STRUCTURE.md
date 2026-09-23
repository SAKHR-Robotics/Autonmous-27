# Autonomous-27: Project Structure & Architecture Map

## Overview
Autonomous Mars Rover platform built on ROS 2 (Jazzy / Humble) for terrain navigation, multi-sensor SLAM, obstacle perception, and path planning.

---

## Subsystems & Packages

### 1. Rover Model & Simulation (`Rover/my_robot_description`)
- **Package**: `my_robot_description`
- **Entry Points**:
  - `launch/gazebo.launch.py`: Spawns rover in Gazebo Mars Yard world
  - `launch/spawn_rover.launch.py`: Spawns rover model into existing Gazebo instance
  - `launch/display.launch.py`: RViz robot state visualization
  - `scripts/teleop_gui.py`: Manual driving GUI interface
- **Core Files**:
  - `urdf/my_robot.urdf.xacro`: 4-wheeled rover kinematics, links, and sensor frames
  - `urdf/gazebo.xacro`: DiffDrive plugin and sensor simulation properties
  - `urdf/macros.xacro`: Suspension and wheel geometric macros
  - `rviz/robot_view.rviz`: Rover model RViz layout

### 2. SLAM & State Estimation (`SLAM/rover_slam`)
- **Package**: `rover_slam`
- **Entry Points**:
  - `launch/slam_bringup.launch.py`: Master state estimation bringup
  - `launch/ekf.launch.py`: EKF sensor fusion (`robot_localization`)
  - `launch/rtabmap.launch.py`: RGB-D RTAB-Map visual SLAM
- **Core Nodes & Configs**:
  - `rover_slam/encoder_ticks_to_odom.py`: 4-wheel tick to differential odom converter
  - `rover_slam/heuristic_slip_checker.py`: Compares wheel vs IMU yaw, inflates covariance on slip
  - `config/ekf.yaml`: EKF fusion parameters (fusing `/wheel/odom_filtered` + `/imu/data`)
  - `config/rtabmap.yaml`: Visual odometry and 3D occupancy grid parameters
  - `config/slam_visualization.rviz`: SLAM monitoring RViz configuration

### 3. Perception (`Perception/`)
- **Packages**:
  - `marker_detection`: ArUco marker detection, 3D pose estimation, and target tracking
  - `terrain_geometry`: Ground plane fitting and negative obstacle detection
- **Entry Points**:
  - `Perception/marker_detection/launch/marker_detection.launch.py`
  - `Perception/terrain_geometry/launch/terrain_geometry.launch.py`
- **Core Nodes**:
  - `marker_detection/aruco_detector.py`: OpenCV ArUco detector
  - `marker_detection/depth_processor.py`: PointCloud2 cluster extraction
  - `marker_detection/marker_action_interface_node.py`: ROS 2 Action Server for marker search

### 4. Path Planning & Navigation (`PathPlanning/erc_path_planner`)
- **Package**: `erc_path_planner`
- **Entry Points**:
  - `launch/path_planning.launch.py`: Nav2 stack bringup
- **Core Nodes & Configs**:
  - `config/nav2_params.yaml`: Smac Hybrid-A* planner & MPPI controller tuning
  - `src/costmap_bridge_node.cpp`: Bridges perception 3D detections into 2D costmaps
  - `behavior_trees/`: Nav2 navigate_to_pose XML behavior trees

### 5. Control & Actuation (`Control/`)
- **Docs & Specifications**:
  - Skid-steer kinematics conversion from `/cmd_vel` to per-wheel RPM
  - Hardware microcontroller communication protocols (CAN / Serial)

### 6. Simulation Worlds & Testing (`worlds/`, `testing/`)
- **`worlds/`**: Mars yard heightmap models and SDF simulation environments (*DO NOT SCAN RECURSIVELY*)
- **`testing/`**: Launch orchestrators, test harnesses, and validation scripts (`launch_world_and_rover.sh`)

### 7. Containerized Environment (`docker/`)
- `docker-compose.yml`: Standard container with ROS 2, Nav2, RTAB-Map, and Gazebo
- `run_docker.sh`: Launch container mounting repo at `/workspace`
- `enter_docker.sh`: Attach shell to active container

---

## TF Coordinate Tree (REP-105)
```
map (Global World / RTAB-Map SLAM)
 └── odom (Smooth Local Odometry / robot_localization EKF)
      └── base_link (Rover Kinematic Center)
           ├── base_footprint (Ground plane projection, z=0)
           ├── camera_link -> camera_depth_optical_frame
           ├── imu_link
           ├── left_front_arm_link -> left_front_wheel_link
           ├── left_rear_arm_link -> left_rear_wheel_link
           ├── right_front_arm_link -> right_front_wheel_link
           └── right_rear_arm_link -> right_rear_wheel_link
```

---

## Guardrails & Fast Navigation
- **Never scan or index `worlds/`**: High-polygon 3D rock meshes and texture heightmaps.
- **Never scan build artifacts**: Exclude `build/`, `install/`, `log/`, `.git/`.
- **Target queries by subsystem**: Look directly in `Rover/`, `SLAM/`, `Perception/`, or `PathPlanning/`.
