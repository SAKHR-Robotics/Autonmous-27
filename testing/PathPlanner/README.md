# 🧭 Path Planner & Control Testing Suite: Architecture, Requirements & Integration Guide

A comprehensive, interactive black-box testing and benchmarking framework for the ERC Rover **Smac Hybrid A\*** Global Planner, **MPPI** Local Trajectory Controller (`erc_path_planner`), and downstream **Motor Control Kinematics**.

This document specifies the architecture, dependencies, data flow, and required enhancements for the **Path Planner & Control Testing & Benchmarking Suite**. It details the exact code and configuration edits needed to test the complete, integrated **Path Planning (Smac Hybrid A* + MPPI) + Control (Motor Driver / Kinematics)** subsystem.

---

## 🎯 1. End Goal & Core Purpose: What Does This Tester Do?

The overarching end goal of the **Path Planner & Control Testing Suite** is to serve as an **autonomous, hardware-independent simulation and validation gate** that verifies the entire navigation and motion control pipeline end-to-end:

> **"Can our rover plan kinematically feasible global paths (Reeds-Shepp) and execute smooth, collision-free trajectory tracking (MPPI) across rough Martian terrain—without needing physical hardware, Gazebo, or manual goal setting?"**

### The 5 Core Objectives:
1. **Autonomous Data Feeder on Demand:**
   - If input data (2D Occupancy Grid Maps, Start Position, 4 Intermediate Waypoints, Dynamic Obstacles) is missing, the tester **automatically synthesizes and injects it**.
   - No need to launch Gazebo, build custom simulation worlds, or manually click 2D goals in RViz.
2. **Deterministic Multi-Scenario Benchmarking:**
   - Evaluates the global planner (Smac Hybrid A*) and local trajectory controller (MPPI) across **9 realistic Mars Yard environments** (open plains, rock fields, narrow canyon gates, slopes, labyrinths, dead-end traps).
3. **Closed-Loop Kinematics & Control Simulation:**
   - Ingests `/cmd_vel` output from MPPI at 50 Hz, integrates differential/skid-steer kinematics with wheel slip modeling, and broadcasts continuous dynamic TF (`map -> odom -> base_link`) and `/odometry/filtered`.
   - Validates how motor acceleration limits, current limits, and wheel velocities behave under actual motion commands.
4. **Live Visual Telemetry & RViz2 Tracking:**
   - Real-time visual inspection in RViz2: watch the 3D Rover model drive through the terrain, observe the green Smac global path, inspect the cyan cloud of 2,000 MPPI trajectory rollouts, and view real-time costmap updates.
5. **Comprehensive Numerical Analysis & Automated Reports:**
   - Quantifies performance across **13 rigorous navigation and control metrics** (Cross-Track Error, Jerk, Turn Radii, Obstacle Clearance, Replanning Time, Control Effort).
   - Automatically exports interactive **HTML dashboards**, printable **PDF reports**, **CSV tables**, and **PNG trajectory plots**.

---

## 🏗️ 2. Tester Architecture & Working Mechanism

The benchmarking and testing suite (`testing/PathPlanner/`) is an autonomous, standalone verification harness designed to validate global navigation and local trajectory tracking **with or without physical hardware or Gazebo simulation**.

```
+---------------------------------------------------------------------------------------------------+
|                                     TESTER ORCHESTRATION LAYER                                    |
|                                    (testing_node / benchmark.py)                                  |
|                                                                                                   |
|  • Injects Map PNG/NPY -> /map (nav_msgs/OccupancyGrid)                                           |
|  • Injects Start Pose  -> /initialpose (geometry_msgs/PoseWithCovarianceStamped)                   |
|  • Injects Goal/Waypts -> /goal_pose / /waypoints (nav_msgs/Path)                                 |
|  • Dynamically Spawns  -> /perception/obstacles_only (vision_msgs/Detection3DArray)              |
+-----------------------------------+---------------------------------------------------------------+
                                    |
                                    v
+---------------------------------------------------------------------------------------------------+
|                                  NAVIGATION & PLANNING PIPELINE                                   |
|                                      (erc_path_planner)                                           |
|                                                                                                   |
|  1. Global Planner: Smac Hybrid A* (Reeds-Shepp model, 0.8m min turning radius)                   |
|     --> Computes Macro Highway: /plan (nav_msgs/Path)                                             |
|                                                                                                   |
|  2. Costmap Bridge: Converts 3D Perception Obstacles -> /bridge/pointcloud                        |
|     --> Ingested by /local_costmap/costmap (Rolling window 10m x 10m)                             |
|                                                                                                   |
|  3. Local Planner & Controller: Nav2 MPPI Controller (2000 rollouts, 20 Hz)                       |
|     --> Evaluates Critics (Cost, PathAlign, PathFollow, Goal, Constraint)                         |
|     --> Computes Optimal Control: /cmd_vel (geometry_msgs/Twist)                                  |
+-----------------------------------+---------------------------------------------------------------+
                                    |
                                    v
+---------------------------------------------------------------------------------------------------+
|                                  CONTROL & FEEDBACK SIMULATION LAYER                              |
|                                                                                                   |
|  • Mock Kinematics Node (mock_rover_sim.py):                                                      |
|    - Ingests /cmd_vel at 50 Hz                                                                    |
|    - Integrates differential/skid-steer 2D kinematics (x, y, theta)                               |
|    - Broadcasts dynamic TF tree: map -> odom -> base_link                                         |
|    - Publishes /odometry/filtered and 3D Rover Visual Markers (/rover_marker)                     |
|                                                                                                   |
|  • OR Real Control Subsystem (motor_driver_node):                                                 |
|    - Converts /cmd_vel into Left/Right wheel RPMs and current limits                              |
|    - Sends CAN / Serial packets to Motor ESCs                                                     |
+-----------------------------------+---------------------------------------------------------------+
                                    |
                                    v
+---------------------------------------------------------------------------------------------------+
|                                  EVALUATION & REPORTING ENGINE                                    |
|                                     (report_generator.py)                                         |
|                                                                                                   |
|  1. Numerical Analysis:                                                                           |
|     - Global: Search Time, Path Length, Curvature, Turning Radius, Obstacle Clearance, Cost.     |
|     - Local: Mean/Max/RMS Cross-Track Error (CTE), Linear/Angular Jerk, Goal Accuracy, Latency.  |
|     - System: Overall Score calculation based on weighted multi-objective criteria.               |
|                                                                                                   |
|  2. Visual Verification:                                                                          |
|     - Live RViz2 visual tracking with real-time trajectory clouds and rover footprint.            |
|     - Matplotlib trajectory comparisons (Reference vs Actual vs Global Plan).                    |
|                                                                                                   |
|  3. Automated Reports:                                                                            |
|     - Generates CSV, interactive HTML dashboards, and publication-ready PDF reports.              |
+---------------------------------------------------------------------------------------------------+
```

### 📂 Standalone Tester Directory Structure
```text
testing/PathPlanner/
├── Path&controlTestingDoc.md      # Detailed developer task specifications & verification matrix
├── README.md                      # Complete system guide, mathematical formulas & operational manual
├── run_testing_suite.sh           # Interactive & CI automated launcher script
├── package.xml                    # ROS 2 package definition (global_path_benchmarking)
├── mock/                          # Standalone Mock Simulation Layer
│   ├── __init__.py
│   ├── mock_rover_sim.py          # 50Hz Kinematic & physical actuator simulator with TF broadcaster
│   └── mock_perception.py         # Dynamic 3D obstacle injector (ObstacleFeatureArray)
├── global_path_benchmarking/      # Core Benchmarking & Evaluation Logic
│   ├── __init__.py
│   ├── testing_node.py            # Live interactive test runner with waypoint action client
│   ├── benchmarking_node.py       # Headless automated batch test runner
│   ├── algo_node.py               # Standalone algorithmic path solver benchmark
│   └── report_generator.py        # Compiles HTML dashboards, PDF reports, CSVs & Matplotlib plots
├── config/                        # Test Scenarios & Metric Configurations
│   ├── scenarios.yaml             # Definitions of 9 maps, start/goal poses & intermediate waypoints
│   └── benchmark_config.yaml      # Metric thresholds, slip ratios & scoring weights
├── launch/                        # ROS 2 Launch Scripts
│   ├── live_test.launch.py        # Interactive test launcher with RViz2 visualization
│   └── benchmark.launch.py        # Headless batch benchmarking launcher
├── maps/                          # Synthetic & recorded Mars Yard occupancy maps (.png/.pgm)
├── rviz/                          # Visualization configurations
│   └── benchmark_view.rviz        # Pre-configured RViz layout with rollout clouds & HUD overlay
└── reports/                       # Generated HTML/PDF/CSV benchmark output artifacts
```

---

## 📦 3. Prerequisites & What is Needed for the Tester to Work

### 3.1 Dependencies & ROS 2 Environment
* **ROS 2 Distribution:** Humble or Jazzy.
* **Core Nav2 Packages:**
  * `nav2_bringup`, `nav2_smac_planner`, `nav2_mppi_controller`, `nav2_costmap_2d`, `nav2_bt_navigator`, `nav2_lifecycle_manager`, `nav2_msgs`.
* **Message Packages:**
  * `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`, `vision_msgs`, `terrain_geometry_msgs`.
* **Python Libraries (for benchmarking & reporting):**
  * `numpy`, `matplotlib`, `Pillow (PIL)`, `PyYAML`, `reportlab` (for PDF generation).

### 3.2 Input Data & Scenario Feeding
To test any scenario without manual setup, the tester must provide:
1. **Map Data:** Synthetic or recorded occupancy grids (e.g. `empty_world.png`, `scattered_rocks.png`, `canyon_gate.png`, `marsyard_labyrinth.png`) loaded and published to `/map`.
2. **Start & Goal Coordinates:** Automated `/initialpose` broadcast and `/goal_pose` (or multi-waypoint `/navigate_through_poses` action goal).
3. **Dynamic Perception Obstacles:** Mock or recorded 3D obstacle boxes injected during runtime via `/perception/obstacles_only` to trigger dynamic replanning.
4. **TF Transforms:** Continuous `map -> odom -> base_link` tree published at $\ge 50\text{ Hz}$.

---

## 🔧 4. Edits & Enhancements Needed to Test the Whole PathPlanner + Control System

While the standalone tester validates planning and kinematic trajectory following, testing the **entire integrated PathPlanner + Control subsystem** requires the following modifications:

### A. Multi-Waypoint & Sequence Navigation Support
* **Current State:** The tester triggers single-goal navigation via `NavigateToPose`.
* **Required Edit:** Implement support for **4-waypoint navigation routes** (`NavigateThroughPoses` Action or Behavior Tree waypoint follower) to validate mission plans across complex Mars Yard terrains.

### B. Control Subsystem Integration (Motor Driver & Actuation)
* **Current State:** `mock_rover_sim.py` assumes ideal differential kinematics and instantaneous velocity tracking.
* **Required Edit:**
  * Connect the output of MPPI (`/cmd_vel`) to the actual `motor_driver_node` / micro-ROS CAN bridge.
  * Implement wheel velocity decomposition:
    $$v_{\text{left}} = v_x - \frac{\omega_z \cdot L}{2}, \quad v_{\text{right}} = v_x + \frac{\omega_z \cdot L}{2}$$
  * Model physical actuator constraints: motor acceleration ramps, current limits, torque limits, and motor deadband.

### C. Wheel Slip, Terrain Interaction & Sensor Feedback
* **Current State:** TF and Odometry are perfectly integrated from `/cmd_vel` without noise or slip.
* **Required Edit:**
  * Inject realistic wheel slip models (e.g. $10\text{--}25\%$ longitudinal slip on loose sand/slopes).
  * Subscribe to real or simulated wheel encoders + IMU fused through `robot_localization` (`/odometry/filtered`).
  * Validate how the MPPI controller reacts when actual velocity lags behind commanded velocity.

### D. Perception-Costmap Bridge Standardization
* **Current State:** Dual message formats exist (`ObstacleFeatureArray` vs `Detection3DArray`).
* **Required Edit:** Ensure `costmap_bridge_node.cpp` uniformly consumes `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`) and projects 3D obstacle bounds into `/bridge/pointcloud` for Nav2 `obstacle_layer`.

### E. Controller-Specific Performance Metrics
* **Current State:** Evaluates CTE, jerk, and velocity.
* **Required Edit:** Add control effort metric $J_{\text{control}} = \int (v_x^2 + \omega_z^2) dt$, motor saturation duration, heading overshoot, and waypoint deceleration profile accuracy.

---

## 🗺️ 5. The 9 Mars Yard Benchmark Scenarios

The suite evaluates the planner and controller across 9 distinct operational environments defined in `config/scenarios.yaml`:

| # | Scenario ID | Description | Difficulty | Key Test Objective |
|---|:---|:---|:---:|:---|
| 1 | `empty_straight` | Open terrain with zero obstacles | Baseline | Max velocity & straight-line path efficiency |
| 2 | `scattered_rocks_detour` | Scattered rock field with dynamic obstacle | Medium | Reeds-Shepp curve generation & MPPI replanning |
| 3 | `canyon_gate_passage` | Narrow 1.2m gate between two rock walls | Hard | Narrow passage clearance & precision steering |
| 4 | `marsyard_rough_slopes` | Sloped craters and rough elevation zones | Medium | Costmap slope cost minimization |
| 5 | `marsyard_labyrinth` | Multi-turn maze corridor | Extreme | Global search completeness & turnaround handling |
| 6 | `canyon_gate_blocked` | Gate dynamically blocked mid-run | Hard | Dynamic obstacle avoidance & detour replanning |
| 7 | `crater_field` | Dense field of overlapping craters | Hard | Navigating between low-cost saddles |
| 8 | `dead_end_trap` | U-shaped obstacle trap | Extreme | Smac heuristic escape without getting stuck |
| 9 | `snake_passage` | S-curved winding corridor | Hard | Continuous heading changes & curvature limits |

---

## 🎮 6. Live RViz2 Visualization Modes

### 1. Live Interactive Single Test (Mode 1)
- Launches the Path Planning bringup, Mock Kinematics simulator, Mock Perception, and opens RViz2.
- The map, start position, and 4 waypoints load automatically.
- Smac computes the **green global path** (`/plan`).
- MPPI evaluates thousands of **cyan candidate trajectories** (`/local_plan`).
- The **3D Rover Body & Heading Arrow** (`base_link`) physically drives along the route in real-time.
- You can also click **"2D Goal Pose"** in RViz to test arbitrary destinations on the fly!

### 2. Live Batch Benchmark (Mode 3 / Mode 2 with RViz enabled)
- Watch all 9 maps evaluated live in RViz sequentially.
- The map transitions automatically, the rover resets to the scenario start point, plans the path, drives through the terrain, avoids dynamic obstacles, records performance metrics, and progresses to the next scenario!

---

## 📊 7. Evaluation Metrics & Detailed Metric Reference

The testing suite benchmarks both the **Global Planner (Smac Hybrid A*)** and the **Local Controller (MPPI)** across 13 quantitative metrics:

### 🗺️ A. Global Path Planning Metrics

| Metric | Symbol | Units | Physical / Algorithmic Meaning | Target / Best Value |
| :--- | :---: | :---: | :--- | :--- |
| **Planning Time** | $T_{plan}$ | seconds (s) | Computation time from `/goal_pose` arrival to `/plan` publication. | $\le 2.0\text{ s}$ |
| **Path Length** | $L$ | meters (m) | Total cumulative length along the planned path polyline: $L = \sum \|P_{i+1} - P_i\|$. | Minimizes detour distance |
| **Length Ratio** | $L / D_{euc}$ | unitless | Ratio of path length to straight-line distance $D_{euc} = \|P_{goal} - P_{start}\|$. | $\le 1.35$ |
| **Turn Angles** | $\theta_{avg}, \theta_{max}$ | degrees ($^\circ$) | Heading deflection angle between consecutive segments. | $\text{Avg} \le 15^\circ, \text{Max} \le 45^\circ$ |
| **Turn Radii** | $R_{min}, R_{avg}$ | meters (m) | Circumcircle radius of waypoints: $R = \frac{a \cdot b \cdot c}{4 \cdot \text{Area}_\Delta}$. | $R_{min} \ge 0.8\text{ m}$ |
| **Blocked Cells** | $N_{blocked}$ | count | Number of costmap cells under rover footprint with obstacle cost $\ge 100$. | **0 (Zero collisions)** |
| **Average Cost** | $C_{avg}$ | $[0, 100]$ | Mean costmap cost sampled across all path points. | $C_{avg} \le 20.0$ |
| **Safety Margin** | $D_{clear}$ | meters (m) | Minimum distance from any waypoint to the closest obstacle. | $\ge 0.35\text{ m}$ |
| **Replanning Score** | $S_{replan}$ | $[0, 100]$ | Detects dynamic obstacles injected mid-run and synthesizes detour. | $100 / 100$ |

$$\text{Planner Score} = 0.25 S_{success} + 0.15 S_{time} + 0.25 S_{obstacle} + 0.15 S_{cost} + 0.10 S_{length} + 0.10 S_{replan}$$

---

### 🎮 B. Local MPPI Controller & Tracking Metrics

| Metric | Symbol | Units | Physical / Control Meaning | Target / Best Value |
| :--- | :---: | :---: | :--- | :--- |
| **Mean CTE** | $\overline{\text{CTE}}$ | meters (m) | **Cross-Track Error:** Average perpendicular distance from rover to reference path. | $\le 0.08\text{ m}$ |
| **Max CTE** | $\text{CTE}_{max}$ | meters (m) | Maximum deviation away from reference path during the traversal. | $\le 0.20\text{ m}$ |
| **RMS CTE** | $\text{CTE}_{rms}$ | meters (m) | Root Mean Square tracking error: $\sqrt{\frac{1}{N}\sum \text{CTE}_k^2}$. | $\le 0.10\text{ m}$ |
| **Linear Velocity** | $\bar{v}_x, v_{max}$ | m/s | Mean and peak forward driving speed commanded on `/cmd_vel`. | $\bar{v}_x \ge 0.4\text{ m/s}, v_{max} = 1.0\text{ m/s}$ |
| **Angular Velocity** | $\bar{\omega}_z, \omega_{max}$ | rad/s | Mean and peak rotational steering velocity commanded by MPPI. | $\omega_{max} \le 1.0\text{ rad/s}$ |
| **Linear Jerk Std** | $\sigma_{\Delta v}$ | m/s² | Standard deviation of linear acceleration $\frac{\Delta v}{\Delta t}$. | $\le 0.25\text{ m/s}^2$ |
| **Angular Jerk Std** | $\sigma_{\Delta \omega}$ | rad/s² | Standard deviation of angular acceleration $\frac{\Delta \omega}{\Delta t}$. | $\le 0.40\text{ rad/s}^2$ |
| **Command Rate** | $f_{cmd}$ | Hertz (Hz) | Publication frequency of `/cmd_vel` output by MPPI. | $20.0 \pm 1.0\text{ Hz}$ |
| **Goal Accuracy** | $E_{goal}$ | meters (m) | Final distance error between stopping position and goal coordinates. | $\le 0.10\text{ m}$ |

$$\text{Controller Score} = 0.40 S_{cte} + 0.30 S_{smoothness} + 0.30 S_{accuracy}$$

---

### 🏆 C. Combined System Score & Passing Criteria

$$\text{Overall System Score} = 0.50 \times \text{Planner Score} + 0.50 \times \text{Controller Score}$$

* **Passing Condition:** Overall Score $\ge 85.0 / 100$ and Blocked Cells $= 0$.

---

## 📄 8. Generated Output Reports & Dashboards

All benchmark results and visualization charts are compiled automatically into `reports/`:
- **Interactive Web Dashboard:** `reports/numerical_report.html` (Interactive tables, score cards, and comparison plots)
- **Printable PDF Summary:** `reports/numerical_report.pdf`
- **Markdown Summary:** `reports/numerical_report.md`
- **Visual Path Comparison PNGs:** `reports/result_scenario_<id>.png`
- **Raw Data Files:** `reports/results.json` and `reports/*.csv`

---

## 📋 9. Task Execution Checklist & GitHub Roadmap

For the actionable task breakdown, implementation checklists, and step-by-step acceptance criteria for the standalone tester, see:
👉 **[`Path&controlTestingDoc.md`](Path&controlTestingDoc.md)**

---

## 🛠️ 10. Execution Commands

```bash
# 1. Interactive Master Launcher
./testing/PathPlanner/run_testing_suite.sh

# 2. Live Interactive Mode with RViz2
ros2 launch global_path_benchmarking live_test.launch.py use_mock_rover:=true use_mock_perception:=true use_rviz:=true

# 3. Batch Benchmark (Headless or with RViz)
ros2 launch global_path_benchmarking benchmark.launch.py clean:=true use_rviz:=true

# 4. Run Single Scenario
ros2 launch global_path_benchmarking benchmark.launch.py scenario_id:=canyon_gate_passage use_rviz:=true
```
