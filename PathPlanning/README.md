# ERC 2026 Path Planning & Control Architecture

This package provides the complete autonomous navigation, obstacle avoidance, and motion control stack for the **ERC 2026 Rover**, integrating **ROS 2 Nav2**, the **Smac Hybrid A\* Global Planner**, the **MPPI Unified Local Planner & Controller**, and downstream **Motor Control Hardware Interfaces**.

---

## 🧭 1. System Overview: Perception, SLAM, Path Planning & Control

The rover's autonomous pipeline strictly separates **Visual SLAM & State Estimation**, **3D Obstacle Perception**, **Macro & Micro Path Planning**, and **Motor Actuation**:

```
                      [ RealSense D435 Hardware / Simulation ]
                                   │               │
     /camera/depth/color/points    │               │ /camera/color/image_raw + CameraInfo
     (3D Point Cloud)              │               │ /camera/depth/image_rect_raw (Depth Image)
                                   ▼               ▼
                 +─────────────────────+       +──────────────────────+
                 │  PERCEPTION MODULE  │       │   VISUAL SLAM MODULE │
                 │ (Rocks & Costmaps)  │       │      (RTAB-Map)      │
                 +──────────┬──────────+       +───────────▲──────────+
                            │                              │
                            ├────► /perception/aruco_pose ─┘
                            │      (Landmark Loop-Closure Reset)
                            ▼
           /perception/obstacles_only (vision_msgs/Detection3DArray)
                            │
                            ▼
                 +─────────────────────+
                 │ COSTMAP BRIDGE NODE │
                 │ (3D BBox -> Points) │
                 +──────────┬──────────+
                            │ /bridge/pointcloud (sensor_msgs/PointCloud2)
                            ▼
        ┌─────────────────────────────────────────────────────────────┐
        │                 NAV2 PATH PLANNING STACK                    │
        │                                                             │
        │  [ Global Costmap ] ──► [ Smac Hybrid A* Global Planner ]   │
        │                                  │ /plan                    │
        │                                  ▼                          │
        │  [ Local Costmap  ] ──► [ MPPI Local Planner & Controller ] │
        └──────────────────────────────┬──────────────────────────────┘
                                       │ /cmd_vel (geometry_msgs/Twist)
                                       ▼
        ┌─────────────────────────────────────────────────────────────┐
        │             MOTOR DRIVER (CONTROL SUBSYSTEM)                │
        │  • Skid-Steer / Differential Drive Kinematic Decomposition │
        │  • Output: Left & Right Wheel RPM / CAN Bus Motor Commands │
        └──────────────────────────────┬──────────────────────────────┘
                                       ▼
                             [ Physical Motors ]
```

---

## 🏛️ 2. Detailed Path Planning + Control Architecture

In modern autonomous robotics, rather than using separate, decoupled modules for local planning and control, our architecture unifies **Local Path Adjustment (Real-time Swerving)** and **Kinematic Control** directly inside the **MPPI Controller** running at **20 Hz**.

```
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│ 1. GLOBAL PLANNER (Smac Hybrid A*)                                                          │
│ • Computes the initial macro-highway from Start to Goal (across 50+ meters).                │
│ • Uses Reeds-Shepp vehicle kinematics (respects 0.8m minimum turning radius).               │
│ • Outputs: /plan (nav_msgs/Path)                                                            │
└──────────────────────────────────────────────┬──────────────────────────────────────────────┘
                                               │
                                               ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│ 2 & 3. UNIFIED LOCAL PLANNER + CONTROLLER (MPPI Controller)                                 │
│                                                                                             │
│ [ Local Planning Function ]                                                                 │
│ • Scans immediate obstacles in a 10m x 10m rolling window from /local_costmap/costmap.      │
│ • Samples 2,000 candidate trajectory rollouts every 50ms (20 Hz).                           │
│ • Dynamically adjusts the path to swerve smoothly around newly appeared obstacles.         │
│                                                                                             │
│ [ Control Function ]                                                                        │
│ • Evaluates vehicle velocity and acceleration limits.                                       │
│ • Optimizes cost critics (ConstraintCritic, PathAlignCritic, GoalCritic, CostCritic).       │
│ • Outputs: /cmd_vel_nav (linear velocity vx, angular velocity wz)                          │
└──────────────────────────────────────────────┬──────────────────────────────────────────────┘
                                               │
                                               ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│ 4. SAFETY & SMOOTHING LAYER (Collision Monitor & Velocity Smoother)                         │
│ • Validates robot footprint safety zones and limits jerk.                                   │
│ • Outputs: /cmd_vel (geometry_msgs/Twist)                                                   │
└──────────────────────────────────────────────┬──────────────────────────────────────────────┘
                                               │
                                               ▼
┌─────────────────────────────────────────────────────────────────────────────────────────────┐
│ 5. MOTOR HARDWARE DRIVER (Control Subsystem)                                                │
│ • Converts /cmd_vel (m/s and rad/s) into left/right wheel RPMs or PWM for motor ESCs.       │
│ • Differential / Skid-steer kinematics: v_left = vx - (wz * L / 2), v_right = vx + (wz*L/2)│
└──────────────────────────────────────────────┬──────────────────────────────────────────────┘
                                               ▼
                                  [ Hardware Actuators / ESCs ]
```

---

## 🔄 3. End-to-End Pipeline & Data Flowchart

![Path Planning Pipeline](../General_Docs/path_planning_pipeline.png)

```mermaid
flowchart TD
    subgraph INPUTS["🌐 External Subsystem Inputs"]
        MAP["<b>SLAM / RTAB-Map</b><br/>/map (OccupancyGrid)"]
        ODOM["<b>Localization / EKF</b><br/>/odometry/filtered (Odometry)<br/>TF: map ➔ odom ➔ base_link"]
        PERC["<b>Perception (Persistent Memory)</b><br/>/perception/obstacles_only<br/>(vision_msgs/Detection3DArray)"]
        GOAL["<b>Mission Command / UI</b><br/>/goal_pose (PoseStamped)"]
    end

    subgraph BRIDGES["🌉 Ingestion & Bridge Layer"]
        BRIDGE_NODE["<b>costmap_bridge_node</b><br/>Samples 3D Bounding Boxes<br/>into dense boundary PointCloud"]
    end

    subgraph NAV2_CORE["🧭 Path Planning & Navigation Core"]
        GC["<b>Global Costmap</b><br/>• Static Layer (/map)<br/>• Obstacle Layer (/bridge/pointcloud)<br/>• Inflation Layer"]
        LC["<b>Local Costmap (10x10m)</b><br/>• Rolling Window (odom frame)<br/>• Obstacle Layer (/bridge/pointcloud)<br/>• Inflation Layer"]
        
        SMAC["<b>Global Planner Server</b><br/>nav2_smac_planner::SmacPlannerHybrid<br/>(Reeds-Shepp search)"]
        MPPI["<b>Controller Server</b><br/>nav2_mppi_controller::MPPIController<br/>(2,000 rollouts @ 20Hz)"]
        BT["<b>Behavior Tree Navigator</b><br/>Orchestrates Replanning & Recoveries"]
        SAFETY["<b>Collision Monitor & Velocity Smoother</b><br/>Clamps acceleration & safety envelopes"]
    end

    subgraph OUTPUTS["⚙️ Control & Actuation Outputs"]
        MOTOR_DRIVER["<b>motor_driver Node</b><br/>(Control Subsystem)<br/>Differential / Skid-Steer Kinematics"]
        MOTORS["<b>Hardware Actuators</b><br/>Left/Right Wheel ESCs / CAN Motors"]
    end

    %% Data Connections
    PERC -->|"/perception/obstacles_only"| BRIDGE_NODE
    BRIDGE_NODE -->|"/bridge/pointcloud"| GC
    BRIDGE_NODE -->|"/bridge/pointcloud"| LC
    BRIDGE_NODE -->|"/bridge/pointcloud"| SAFETY
    MAP -->|"/map"| GC
    ODOM -->|"/odometry/filtered & TFs"| LC
    ODOM -->|"/odometry/filtered & TFs"| MPPI

    GOAL -->|"/goal_pose"| BT
    BT -->|"ComputePathToPose"| SMAC
    GC -->|"Global Cost Grid"| SMAC
    SMAC -->|"/plan (Global Route)"| BT
    BT -->|"FollowPath (/plan)"| MPPI
    LC -->|"Local Cost Window"| MPPI
    
    MPPI -->|"/cmd_vel_nav"| SAFETY
    SAFETY -->|"/cmd_vel (Twist)"| MOTOR_DRIVER
    MOTOR_DRIVER -->|"Wheel RPM / PWM"| MOTORS
    
    %% Dynamic Replanning Feedback
    MPPI -.->|"NO_VALID_TRAJECTORY (Trapped)"| BT
    BT -.->|"Trigger Detour Replan"| SMAC
```

---

## 📊 4. Summary of Block Interfaces & Topics

| Subsystem / Block | Role | Inputs Consumed | Outputs Produced | Message Type |
| :--- | :--- | :--- | :--- | :--- |
| **`costmap_bridge_node`** | Translates persistent 3D obstacles into dense point clouds | `/perception/obstacles_only` | `/bridge/pointcloud` | `vision_msgs/Detection3DArray` ➔ `sensor_msgs/PointCloud2` |
| **`global_costmap`** | Global obstacle map across the full terrain | `/map`, `/bridge/pointcloud`, TF | `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` |
| **`local_costmap`** | High-frequency rolling window (10x10m) centered on rover | `/bridge/pointcloud`, `/odometry/filtered`, TF | `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` |
| **`planner_server` (Smac)** | Calculates kinematically feasible global macro path | `/goal_pose`, `/global_costmap/costmap` | `/plan` | `nav_msgs/Path` |
| **`controller_server` (MPPI)** | Real-time obstacle avoidance + velocity control | `/plan`, `/local_costmap/costmap`, `/odometry/filtered` | `/cmd_vel_nav`, `/local_plan` | `geometry_msgs/Twist`, `nav_msgs/Path` |
| **`collision_monitor`** | Emergency safety envelope & velocity clamping | `/cmd_vel_nav`, `/bridge/pointcloud` | `/cmd_vel` | `geometry_msgs/Twist` |
| **`bt_navigator`** | Master state machine (Replanning & Recovery) | Feedback from Planner & Controller | Lifecycle & recovery triggers | Behavior Tree Action XML |
| **`motor_driver` (Control)** | Actuation interface to physical wheels | `/cmd_vel` | Left & Right wheel RPM / CAN packets | Custom Motor / CAN Frames |

---

## 🛠️ 5. System Prerequisites & Dependencies

Before building this module, install ROS 2 Navigation2 and its bringup packages:

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-vision-msgs -y
```

### ROS 2 Package Dependencies
- `rclcpp`
- `sensor_msgs`
- `vision_msgs` (Standard ROS 2 Vision Messages)
- `geometry_msgs`
- `nav_msgs`
- `nav2_bringup`
- `nav2_smac_planner`
- `nav2_mppi_controller`
- `nav2_costmap_2d`
- `nav2_bt_navigator`

---

## 🏗️ 6. Building the Package

```bash
# 1. Navigate to workspace root
cd /path/to/your/cloned/repository

# 2. Build the path planner package
colcon build --packages-select erc_path_planner
source install/setup.bash
```

---

## 🚀 7. Execution & Quickstart

### Standard Execution (With SLAM / Live Rover)
By default, `use_slam:=true`. Nav2's `map_server` is disabled and RTAB-Map SLAM owns the live `/map` topic:
```bash
# Terminal 1: Launch Nav2 Path Planning Stack (RTAB-Map supplies /map)
ros2 launch erc_path_planner path_planning.launch.py use_sim_time:=true

# Terminal 2: Launch RViz Visualization Dashboard
ros2 launch erc_path_planner rviz.launch.py
```

### Standalone Testing in Simulation (World + Rover + Teleop + Planner)
To test path planning in isolation with the simulated rover without running SLAM:

#### Method A: Interactive Launcher (Fastest)
```bash
bash scripts/launch_planning.sh
# Select Option 2 (Standalone Testing with dummy_map)
```

#### Method B: Manual Step-by-Step Playbook
1. **Terminal 1 (Simulation & Rover Spawn)**:
   ```bash
   source install/setup.bash
   ros2 launch my_robot_description gazebo.launch.py publish_map_tf:=true
   ```
2. **Terminal 2 (Teleoperation GUI)**:
   ```bash
   source install/setup.bash
   ros2 run my_robot_description teleop_gui.py
   ```
3. **Terminal 3 (Standalone Path Planner)**:
   ```bash
   source install/setup.bash
   ros2 launch erc_path_planner test_planner_standalone.launch.py
   ```
4. **Terminal 4 (Set Goal / Verification)**:
   Use the **2D Goal Pose** tool in the opened RViz2 window to set navigation targets, or echo `/cmd_vel` to verify motor commands:
   ```bash
   ros2 topic echo /cmd_vel
   ```

---

## 🗺️ 8. Benchmark & Operational Maps

All custom 2D occupancy grid maps are located in [`PathPlanning/erc_path_planner/maps/`](file:///e:/SHAKR/Autonmous-27/PathPlanning/erc_path_planner/maps/):

| Map Name | Image / YAML | Dimensions & Resolution | Terrain Description | Key Test Objective |
| :--- | :--- | :---: | :--- | :--- |
| **Rock Field** | `rock_field.yaml` (`.pgm`) | $10\text{m} \times 10\text{m}$ ($0.05\text{m/px}$) | Dense field of randomly scattered boulders | Smac Reeds-Shepp global swerving & MPPI avoidance |
| **Narrow Corridor** | `narrow_corridor.yaml` (`.pgm`) | $10\text{m} \times 10\text{m}$ ($0.05\text{m/px}$) | $1.2\text{m}$ canyon gate between walls | High-precision tracking through tight clearances |
| **Dead End** | `dead_end.yaml` (`.pgm`) | $10\text{m} \times 10\text{m}$ ($0.05\text{m/px}$) | U-shaped obstacle trap | Smac heuristic escape & reverse turnarounds |

### How to Launch Standalone Benchmarking with Custom Maps

```bash
# Launch standalone planner with the rock field map
ros2 launch erc_path_planner path_planning.launch.py use_slam:=false map:=$(ros2 pkg prefix erc_path_planner)/share/erc_path_planner/maps/rock_field.yaml

# Launch standalone planner with the narrow corridor map
ros2 launch erc_path_planner path_planning.launch.py use_slam:=false map:=$(ros2 pkg prefix erc_path_planner)/share/erc_path_planner/maps/narrow_corridor.yaml

# Launch standalone planner with the dead end trap map
ros2 launch erc_path_planner path_planning.launch.py use_slam:=false map:=$(ros2 pkg prefix erc_path_planner)/share/erc_path_planner/maps/dead_end.yaml
```

#### Option C: Dynamic Map Swapping at Runtime (Without restarting Nav2)
If Nav2 is already running and you want to swap the active map on-the-fly:
```bash
# 1. Run the map server with the new map YAML
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$(ros2 pkg prefix erc_path_planner)/share/erc_path_planner/maps/dead_end.yaml

# 2. Activate the map server lifecycle node
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

---

## 🧪 9. Standalone Testing & Benchmarking Suite

For standalone closed-loop testing, mock simulation, and automated multi-scenario benchmark evaluation without Gazebo or hardware:
* See the testing package at [`testing/PathPlanner/`](file:///e:/meseket/Autonmous-27/testing/PathPlanner/README.md).
* See the detailed tester tasks and developer guide at [`Path&controlTestingDoc.md`](file:///e:/meseket/Autonmous-27/testing/PathPlanner/Path&controlTestingDoc.md).


