# 🧭 Path Planner & Control Testing Suite: Architecture, Requirements & Integration Guide

This document specifies the architecture, dependencies, data flow, and required enhancements for the **Path Planner & Control Testing & Benchmarking Suite**. It also details the exact code and configuration edits needed to test the complete, integrated **Path Planning (Smac Hybrid A* + MPPI) + Control (Motor Driver / Kinematics)** subsystem, followed by a **GitHub Projects-style task breakdown**.

---

## 🏗️ 1. Tester Architecture & Working Mechanism

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

---

## 📦 2. Prerequisites & What is Needed for the Tester to Work

### 2.1 Dependencies & ROS 2 Environment
* **ROS 2 Distribution:** Humble or Jazzy.
* **Core Nav2 Packages:**
  * `nav2_bringup`, `nav2_smac_planner`, `nav2_mppi_controller`, `nav2_costmap_2d`, `nav2_bt_navigator`, `nav2_lifecycle_manager`, `nav2_msgs`.
* **Message Packages:**
  * `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`, `vision_msgs`, `terrain_geometry_msgs`.
* **Python Libraries (for benchmarking & reporting):**
  * `numpy`, `matplotlib`, `Pillow (PIL)`, `PyYAML`, `reportlab` (for PDF generation).

### 2.2 Input Data & Scenario Feeding
To test any scenario without manual setup, the tester must provide:
1. **Map Data:** Synthetic or recorded occupancy grids (e.g. `empty_world.png`, `scattered_rocks.png`, `canyon_gate.png`, `marsyard_labyrinth.png`) loaded and published to `/map`.
2. **Start & Goal Coordinates:** Automated `/initialpose` broadcast and `/goal_pose` (or multi-waypoint `/navigate_through_poses` action goal).
3. **Dynamic Perception Obstacles:** Mock or recorded 3D obstacle boxes injected during runtime via `/perception/obstacles_only` to trigger dynamic replanning.
4. **TF Transforms:** Continuous `map -> odom -> base_link` tree published at $\ge 50\text{ Hz}$.

---

## 🔧 3. Edits & Enhancements Needed to Test the Whole PathPlanner + Control System

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

## 📋 4. GitHub Projects Task Decomposition

Below is the structured breakdown of tasks ready for import into **GitHub Projects / Issues**:

---

### 🔹 Task 1: [Testing-Core] Multi-Waypoint & Action Client Support in Benchmarking Node
* **Description:** Extend the benchmarking test runner (`testing_node.py`) from single-goal `NavigateToPose` to multi-waypoint navigation (`NavigateThroughPoses` or custom 4-waypoint sequence).
* **Suggested What to Do in Steps:**
  1. Add an action client in `testing_node.py` targeting `nav2_msgs/action/NavigateThroughPoses`.
  2. Update `config/scenarios.yaml` to include an array of 4 intermediate waypoints $[W_1, W_2, W_3, W_4]$ for each benchmark map.
  3. Implement waypoint progression tracking and individual waypoint arrival tolerance checks ($\le 0.15\text{ m}$).
  4. Record segment-by-segment navigation time and metrics between consecutive waypoints.
* **Acceptance Criteria:**
  - `testing_node.py` successfully sends a 4-waypoint array to Nav2.
  - The rover visits all 4 waypoints in sequence and records timestamped completion for each.
  - Generates individual segment and cumulative path statistics in the summary report.

---

### 🔹 Task 2: [Control-Bridge] Skid-Steer Motor Driver Kinematics & Actuator Model
* **Description:** Implement a closed-loop control bridge that converts MPPI `/cmd_vel` into left/right wheel RPM commands and models physical motor constraints.
* **Suggested What to Do in Steps:**
  1. Create `control_bridge_node.py` (or integrate into `Rover/control/`).
  2. Implement differential/skid-steer kinematic splitting given rover track width $L = 0.65\text{m}$ and wheel radius $R = 0.15\text{m}$.
  3. Add realistic acceleration limits ($\max a = 1.2\text{ m/s}^2$), angular acceleration limits ($\max \alpha = 2.0\text{ rad/s}^2$), and velocity deadband ($|v| < 0.03\text{ m/s} \to 0$).
  4. Publish actuator commands on `/rover/motor_commands` and wheel velocities on `/rover/joint_states`.
* **Acceptance Criteria:**
  - Input `/cmd_vel` commands are smoothly converted to wheel RPMs without unbounded jumps.
  - Actuator limits prevent motor saturation and excessive jerk.
  - Unit test verifies RPM calculations across pure forward, reverse, in-place spin, and combined arc motions.

---

### 🔹 Task 3: [Kinematics-Sim] Slip Injection & Realistic Odometry Feedback
* **Description:** Upgrade `mock_rover_sim.py` to simulate terrain friction, longitudinal wheel slip, and realistic noisy odometry feedback.
* **Suggested What to Do in Steps:**
  1. Add configurable slip coefficient parameters `slip_ratio_linear` (e.g., $0.15$) and `slip_ratio_angular` (e.g., $0.10$) in `benchmark_config.yaml`.
  2. Modify kinematic Euler integration to apply slip degradation on commanded velocities when driving over sloped/rough terrain.
  3. Add Gaussian noise to simulated wheel encoder odometry and IMU angular velocity.
  4. Publish realistic `/odometry/filtered` and broadcast `odom -> base_link` TF accordingly.
* **Acceptance Criteria:**
  - Mock rover exhibits realistic drift and slip when executing high-acceleration maneuvers.
  - MPPI controller dynamically compensates for wheel slip to keep the rover centered on the reference path.
  - CTE under slip remains within acceptable tolerance ($\overline{\text{CTE}} \le 0.12\text{ m}$).

---

### 🔹 Task 4: [Perception-Costmap] Standardized 3D Obstacle Bridge Integration
* **Description:** Unify the perception obstacle pipeline with the Nav2 costmap via `costmap_bridge_node.cpp`.
* **Suggested What to Do in Steps:**
  1. Standardize message subscription in `costmap_bridge_node.cpp` to `vision_msgs/msg/Detection3DArray` on `/perception/obstacles_only`.
  2. Implement bounding-box voxelization that generates 3D point cloud clusters representing the physical obstacle extents.
  3. Publish point cloud to `/bridge/pointcloud` with frame ID `base_link` or `odom`.
  4. Configure Nav2 `nav2_params.yaml` `local_costmap` obstacle layer to mark and clear cells from `/bridge/pointcloud`.
* **Acceptance Criteria:**
  - Dynamic obstacles injected by `mock_perception.py` appear immediately in the Nav2 local costmap within $\le 50\text{ ms}$.
  - MPPI local planner detects obstacle cost and generates collision-free avoidance trajectories.
  - Zero false-positive obstacle clearances when obstacles are removed.

---

### 🔹 Task 5: [Evaluation-Metrics] Advanced Control & Energy Efficiency Evaluator
* **Description:** Add control-theoretic and energy efficiency metrics to `global_path_benchmarking/report_generator.py`.
* **Suggested What to Do in Steps:**
  1. Implement calculation of Control Effort:
     $$J_u = \int_{0}^{T} (v_x(t)^2 + \omega_z(t)^2) \, dt$$
  2. Calculate Steering Oscillations / Reversal Count: count number of sign changes in angular acceleration $\frac{d\omega_z}{dt}$.
  3. Implement Heading Tracking Error: $\Delta \theta = |\theta_{\text{rover}} - \theta_{\text{path\_tangent}}|$.
  4. Integrate these metrics into the final HTML/PDF report scorecards.
* **Acceptance Criteria:**
  - Benchmark output JSON/CSV includes `control_effort`, `steering_reversals`, and `heading_rmse`.
  - PDF/HTML reports render comparative bar charts for control smoothness across all 9 benchmark maps.

---

### 🔹 Task 6: [Visualization] Unified RViz2 Visual Dashboard & Live Telemetry Overlay
* **Description:** Build a comprehensive RViz2 profile and visual overlay for simultaneous monitoring of Path Planning and Control actuation.
* **Suggested What to Do in Steps:**
  1. Update `erc_path_planner/rviz/nav2_default_view.rviz` and `testing/PathPlanner/rviz/benchmark_view.rviz`.
  2. Add visual displays for:
     - Global Reference Path (Green polyline)
     - MPPI Trajectory Rollouts (Cyan multi-path cloud)
     - Real-time Rover Mesh / Footprint Marker
     - Commanded vs Actual Velocity Vector Arrows
     - Local and Global Costmap Heatmaps
     - 4 Waypoint Destination Pins
  3. Create launch configuration in `launch/live_test.launch.py` to auto-load all displays upon start.
* **Acceptance Criteria:**
  - Launching `live_test.launch.py` brings up RViz2 with zero missing topic warnings.
  - All 4 waypoints, global path, MPPI rollout cloud, and rover kinematic motion are visible and render at $\ge 30\text{ FPS}$.

---

### 🔹 Task 7: [CI/Automation] End-to-End Test Suite Automation Script
* **Description:** Ensure the master testing script `run_testing_suite.sh` runs all scenarios headless or visually with automatic exit codes for CI/CD pipelines.
* **Suggested What to Do in Steps:**
  1. Add non-interactive CLI flags to `run_testing_suite.sh` (e.g., `--batch`, `--rviz`, `--scenario <id>`, `--output-dir <path>`).
  2. Add exit status verification: return code `0` if all tests pass (Overall Score $\ge 85\%$, 0 collisions), return `1` on failure.
  3. Ensure clean teardown of all background ROS 2 nodes upon script completion or `SIGINT`.
* **Acceptance Criteria:**
  - `./run_testing_suite.sh --batch` executes all 9 maps headlessly and generates PDF/HTML reports in `reports/`.
  - Stale processes (`planner_server`, `controller_server`, `mock_rover_sim`) are cleanly killed on exit.
