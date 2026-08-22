# 🚀 Path Planner & MPPI Controller Standalone Testing Suite

A complete, interactive black-box testing and benchmarking framework for the ERC Rover **Smac Hybrid A\*** Global Planner and **MPPI** Local Controller (`erc_path_planner`).

It includes standalone **Mock Kinematics (50Hz Odom & dynamic TF)**, **Mock Perception (Obstacle Geometry)**, **Live RViz Path Tracking**, and automated **Evaluation Reports (HTML & PDF)** across 9 realistic Mars Yard scenarios.

---

## 📋 Table of Contents
1. [⚡ Quick Start: Interactive Test Runner](#-quick-start-interactive-test-runner)
2. [🗺️ The 9 Mars Yard Benchmark Scenarios](#️-the-9-mars-yard-benchmark-scenarios)
3. [🎮 Live RViz Visualization Modes](#-live-rviz-visualization-modes)
4. [🤖 The Mock Architecture](#-the-mock-architecture)
5. [📊 Evaluation Metrics & Reports](#-evaluation-metrics--reports)
6. [🛠️ Manual Execution & Launch Commands](#️-manual-execution--launch-commands)
7. [🌐 ROS 2 Humble & Jazzy Compatibility](#-ros-2-humble--jazzy-compatibility)

---

## ⚡ Quick Start: Interactive Test Runner

From the workspace root directory, simply run the master interactive launcher:

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27
./Autonmous_Ws/testing/PathPlanner/run_testing_suite.sh
```

### Interactive Menu Walkthrough
The script will automatically build all required packages, source your ROS 2 environment, and present an interactive menu:

```text
========================================================================
🧭 PATH PLANNER & MPPI CONTROLLER MASTER TESTING SUITE
========================================================================
Select Testing Mode:
  1) 🎮 Live Interactive Closed-Loop Test (Nav2 + Mock Rover + Live RViz)
  2) ⚡ Headless Automated Batch Benchmark (Fast batch run across all 9 maps with full reports)
  3) 📊 Visual Automated Batch Benchmark (Watch all 9 maps evaluated live in RViz)
  4) 🎯 Single Scenario Benchmark (Headless or with RViz)
  5) 🗺️  Verify Scenario Reference Maps & Paths (Generate PNG plots)
Enter choice [1-5] (default: 1):
```

---

## 🗺️ The 9 Mars Yard Benchmark Scenarios

The suite evaluates the planner and controller across 9 distinct operational environments:

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

## 🎮 Live RViz Visualization Modes

### 1. Live Interactive Single Test (Mode 1)
- Launches the Path Planning bringup, Mock Rover kinematics simulator, Mock Perception, and opens RViz2.
- The map and start position load automatically.
- Smac computes the **green global path** (`/plan`).
- MPPI evaluates thousands of **cyan candidate trajectories** (`/local_plan`).
- The **3D Rover Body & Heading Arrow** (`base_link`) physically drives along the route in real-time.
- You can also click **"2D Goal Pose"** in RViz to test arbitrary destinations on the fly!

### 2. Live Batch Benchmark (Mode 2 + RViz enabled)
- Can you watch all 9 maps live in RViz? **YES!**
- When selecting Mode 2, the script asks:
  ```text
  Open RViz2 live visualizer during batch benchmark? [Y/n]: Y
  ```
- RViz will open and run each of the 9 scenarios sequentially. You will see the map change, the rover reset to the start point, generate the path, drive through the terrain, avoid dynamic obstacles, record performance metrics, and automatically transition to the next map!

---

## 🤖 The Mock Architecture

The `mock/` package allows running full closed-loop tests **without Gazebo or physical hardware**:

```text
testing/PathPlanner/
├── mock/
│   ├── mock_rover_sim.py          # Real-time kinematic robot simulator (50Hz Odom + dynamic TF)
│   └── mock_perception.py         # Mock terrain obstacle generator (ObstacleFeatureArray)
```

1. **[`mock_rover_sim.py`](mock/mock_rover_sim.py)**:
   - **Subscribes to:** `/cmd_vel` (`geometry_msgs/msg/Twist`).
   - **Integrates:** Differential/skid-steer 2D kinematics ($x, y, \theta$) at 50 Hz.
   - **Broadcasts:** Continuous TF transforms (`map -> odom -> base_link`).
   - **Publishes:** `/odometry/filtered` (`nav_msgs/msg/Odometry`) and `/rover_marker` (3D Rover chassis, wheels, and heading arrow).

2. **[`mock_perception.py`](mock/mock_perception.py)**:
   - **Publishes:** `terrain_geometry_msgs/msg/ObstacleFeatureArray` on `/terrain/obstacle_features`.
   - **Tests:** `costmap_bridge_node` to verify obstacle pointcloud generation on `/bridge/pointcloud` and local costmap inflation.
   - **Dynamic Spawning:** Can dynamically drop new obstacles at any $(x, y)$ coordinate during runtime.

---

## 📊 Evaluation Metrics & Detailed Metric Reference

The testing suite benchmarks both the **Global Planner (Smac Hybrid A*)** and the **Local Controller (MPPI)** across 13 quantitative metrics:

---

### 🗺️ A. Global Path Planning Metrics Explained

| Metric | Symbol | Units | Physical / Algorithmic Meaning | Target / Best Value |
| :--- | :---: | :---: | :--- | :--- |
| **Planning Time** | $T_{plan}$ | seconds (s) | Total elapsed wall-clock computation time from `/goal_pose` arrival to `/plan` publication. Measures onboard search algorithm efficiency. | $\le 2.0\text{ s}$ |
| **Path Length** | $L$ | meters (m) | Total cumulative length along the planned path polyline: $L = \sum_{i=0}^{n-1} \|P_{i+1} - P_i\|$. | Minimizes detour distance |
| **Length Ratio** | $L / D_{euc}$ | unitless | Ratio of total path length to straight-line Euclidean distance $D_{euc} = \|P_{goal} - P_{start}\|$. Value of $1.0$ is a straight line. | $\le 1.35$ |
| **Turn Angles** | $\theta_{min}, \theta_{max}, \theta_{avg}$ | degrees ($^\circ$) | Heading deflection angle between consecutive segments: $\arccos\left(\frac{\vec{v}_1 \cdot \vec{v}_2}{\|\vec{v}_1\| \|\vec{v}_2\|}\right)$. High angles cause rover wheel slip on sand. | $\text{Avg} \le 15^\circ, \text{Max} \le 45^\circ$ |
| **Turn Radii** | $R_{min}, R_{max}, R_{avg}$ | meters (m) | Circumcircle radius of triangle formed by 3 waypoints: $R = \frac{a \cdot b \cdot c}{4 \cdot \text{Area}_\Delta}$. Validates that the path respects rover's turning radius limits. | $R_{min} \ge 0.8\text{ m}$ |
| **Blocked Cells** | $N_{blocked}$ | count | Number of costmap cells under the rover's footprint where obstacle cost $\ge 100$. | **0 (Zero collisions)** |
| **Average Cost** | $C_{avg}$ | cost $[0, 100]$ | Mean costmap cost sampled across all waypoints. Low cost means the planner avoids steep slopes, loose soil, and hazardous craters. | $C_{avg} \le 20.0$ |
| **Safety Margin** | $D_{clear}$ | meters (m) | Minimum Euclidean distance from any path waypoint to the closest obstacle cell. Provides safety buffer against drift. | $\ge 0.35\text{ m}$ |
| **Replanning Score** | $S_{replan}$ | $[0, 100]$ | Evaluates whether the planner successfully detects dynamic obstacles injected mid-run and synthesizes a valid detour within $\le 2.0$s. | $100 / 100$ |

$$\text{Planner Score} = 0.25 S_{success} + 0.15 S_{time} + 0.25 S_{obstacle} + 0.15 S_{cost} + 0.10 S_{length} + 0.10 S_{replan}$$

---

### 🎮 B. Local MPPI Controller & Tracking Metrics Explained

| Metric | Symbol | Units | Physical / Control Meaning | Target / Best Value |
| :--- | :---: | :---: | :--- | :--- |
| **Mean CTE** | $\overline{\text{CTE}}$ | meters (m) | **Cross-Track Error:** Average perpendicular distance from the rover's live position (from odometry) to the nearest segment on the global path. | $\le 0.08\text{ m}$ |
| **Max CTE** | $\text{CTE}_{max}$ | meters (m) | Maximum deviation away from the global reference path during the entire traversal. | $\le 0.20\text{ m}$ |
| **RMS CTE** | $\text{CTE}_{rms}$ | meters (m) | Root Mean Square tracking error: $\sqrt{\frac{1}{N}\sum \text{CTE}_k^2}$. Standard metric for path following accuracy. | $\le 0.10\text{ m}$ |
| **Linear Velocity** | $\bar{v}_x, v_{max}$ | m/s | Mean and peak forward driving speed commanded by MPPI on `/cmd_vel`. Verifies full acceleration utilization. | $\bar{v}_x \ge 0.4\text{ m/s}, v_{max} = 1.0\text{ m/s}$ |
| **Angular Velocity** | $\bar{\omega}_z, \omega_{max}$ | rad/s | Mean and peak rotational steering velocity commanded by MPPI. Ensures stable turning without excessive spin. | $\omega_{max} \le 1.0\text{ rad/s}$ |
| **Linear Jerk Std** | $\sigma_{\Delta v}$ | m/s² | Standard deviation of linear acceleration $\frac{\Delta v}{\Delta t}$. Lower values indicate smooth throttle control without motor vibrations. | $\le 0.25\text{ m/s}^2$ |
| **Angular Jerk Std** | $\sigma_{\Delta \omega}$ | rad/s² | Standard deviation of angular acceleration $\frac{\Delta \omega}{\Delta t}$. Lower values indicate smooth steering without oscillatory hunting. | $\le 0.40\text{ rad/s}^2$ |
| **Command Rate** | $f_{cmd}$ | Hertz (Hz) | Publication frequency of `/cmd_vel` output by MPPI. Verifies real-time control loop stability. | $20.0 \pm 1.0\text{ Hz}$ |
| **Goal Accuracy** | $E_{goal}$ | meters (m) | Final Euclidean distance error between rover stopping position and the destination coordinates: $\|P_{final} - P_{goal}\|$. | $\le 0.10\text{ m}$ |

$$\text{Controller Score} = 0.40 S_{cte} + 0.30 S_{smoothness} + 0.30 S_{accuracy}$$

---

### 🏆 C. Combined System Score & Passing Criteria

$$\text{Overall System Score} = 0.50 \times \text{Planner Score} + 0.50 \times \text{Controller Score}$$

* **Passing Condition:** Overall Score $\ge 85.0 / 100$ and Blocked Cells $= 0$.

---

### 📄 Generated Output Reports
All reports are compiled automatically into `reports/`:
- **Interactive Web Dashboard:** `reports/numerical_report.html` (Interactive tables, score cards, and comparison plots)
- **Printable PDF Summary:** `reports/numerical_report.pdf`
- **Markdown Summary:** `reports/numerical_report.md`
- **Visual Path Comparison PNGs:** `reports/result_scenario_<id>.png`
- **Raw Data Files:** `reports/results.json` and `reports/*.csv`

---

## 🛠️ Manual Execution & Launch Commands

If you prefer launching nodes directly from the terminal:

```bash
# 1. Build and source
colcon build --packages-select terrain_geometry_msgs erc_path_planner global_path_benchmarking
source install/setup.bash

# 2. Launch Live Interactive Mode with RViz
ros2 launch global_path_benchmarking live_test.launch.py use_mock_rover:=true use_mock_perception:=true use_rviz:=true

# 3. Launch Batch Benchmark (with RViz)
ros2 launch global_path_benchmarking benchmark.launch.py clean:=true use_rviz:=true

# 4. Run a single targeted scenario
ros2 launch global_path_benchmarking benchmark.launch.py scenario_id:=canyon_gate_passage use_rviz:=true
```

---

## 🌐 ROS 2 Humble & Jazzy Compatibility

- **Humble Ready:** Fully configured and tested for ROS 2 Humble.
- **Jazzy Ready:** Package manifests and Python nodes use standard ROS 2 cross-distribution interfaces. Switching to Jazzy only requires setting `enable_stamped_cmd_vel: true` and changing XML paths in `erc_path_planner/config/nav2_params.yaml`.
