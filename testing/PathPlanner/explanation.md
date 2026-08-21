# GPP Benchmarking Monitor: Standalone Integration & Rich Diagnostics Report

This document provides a technical explanation of how the Global Path Planner (GPP) benchmarking monitor has been enhanced to function independently, calculate complex geometric and cost metrics, and output premium reports (HTML and PDF) styled according to `mostafa report .pdf`.

---

## 1. Node Architecture & Standalone Operation

The benchmarking monitor (`benchmarking_node.py`) is designed as a standalone observer. It operates independently of the path planning algorithm or test orchestrator by subscribing to standard ROS 2 topics:

*   `/test_name` (`std_msgs/msg/String`): Sets the current test context (e.g., scenario name). Resets dynamic tracking states when switched. Defaults to `"general"`.
*   `/global_costmap/costmap` (`nav_msgs/msg/OccupancyGrid`): Provides obstacle and terrain cost information.
*   `/start_pose` (`geometry_msgs/msg/PoseStamped`): The initial position of the robot.
*   `/goal_pose` (`geometry_msgs/msg/PoseStamped`): The target position of the robot.
*   `/planned_path` (`nav_msgs/msg/Path`): The path planned by the planner algorithm.
*   `/planner_internal_time` (`std_msgs/msg/Float32`): The planner's internal execution duration (optional, falls back to ROS 2 roundtrip time if not published).

When run independently (e.g. `ros2 run global_path_benchmarking benchmarking_node`), the node collects path statistics for every planning request. Upon termination (via `SIGTERM` or Ctrl+C / `SIGINT`), the node automatically compiles the accumulated history of all runs and generates:
1.  **HTML Report:** `reports/benchmark_report.html`
2.  **PDF Report:** `reports/benchmark_report.pdf`
3.  **CSV History:** `reports/benchmark_history.csv`

---

## 2. Mathematical Formulations for Path Metrics

The monitor calculates several path characteristics:

### A. Path Distance & Euclidean Ratio
Let the planned path consist of coordinates $P = [P_0, P_1, \dots, P_n]$, where each $P_i = (x_i, y_i)$.
*   **Distance Covered:** The total length $L$ of the path:
    $$L = \sum_{i=0}^{n-1} \sqrt{(x_{i+1} - x_i)^2 + (y_{i+1} - y_i)^2}$$
*   **Euclidean Distance:** The straight-line distance $D_{euc}$ between the start pose $P_{start}$ and goal pose $P_{goal}$:
    $$D_{euc} = \sqrt{(x_{goal} - x_{start})^2 + (y_{goal} - y_{start})^2}$$
    *(Note: If start/goal poses are not published, the first ($P_0$) and last ($P_n$) waypoints of the path are used as fallbacks).*
*   **Path Length Ratio:** 
    $$\text{Ratio} = \frac{L}{D_{euc}} \quad (\text{Ratio} = 1.0 \text{ if } D_{euc} = 0.0)$$

### B. Turn Angles (Degrees)
For each three consecutive waypoints $P_{i-1}$, $P_i$, and $P_{i+1}$, the deviation angle $\theta_i$ represents the change in heading direction:
1.  Form vectors $\vec{v}_1 = P_i - P_{i-1}$ and $\vec{v}_2 = P_{i+1} - P_i$.
2.  Calculate magnitudes $\|\vec{v}_1\|$ and $\|\vec{v}_2\|$.
3.  Compute the angle of deflection:
    $$\theta_i = \arccos\left(\text{clip}\left(\frac{\vec{v}_1 \cdot \vec{v}_2}{\|\vec{v}_1\| \|\vec{v}_2\|}, -1.0, 1.0\right)\right) \times \frac{180}{\pi}$$
4.  The system calculates and records the **Minimum**, **Maximum**, and **Average** values of all $\theta_i$ along the path.

### C. Turn Radii (Meters)
The turn radius $R_i$ at a waypoint $P_i$ is computed as the circumcircle radius of the triangle formed by $P_{i-1}$, $P_i$, and $P_{i+1}$:
1.  Let the side lengths of triangle $\Delta P_{i-1}P_iP_{i+1}$ be $a = \|P_{i+1} - P_i\|$, $b = \|P_{i+1} - P_{i-1}\|$, and $c = \|P_i - P_{i-1}\|$.
2.  Compute the area of the triangle using the determinant (cross-product):
    $$\text{Area}_{\Delta} = \frac{1}{2} \left| x_{i-1}(y_i - y_{i+1}) + x_i(y_{i+1} - y_{i-1}) + x_{i+1}(y_{i-1} - y_i) \right|$$
3.  Compute circumradius:
    $$R_i = \frac{a \cdot b \cdot c}{4 \cdot \text{Area}_{\Delta}}$$
4.  **Collinear Filtering:** If the three points are perfectly collinear, $\text{Area}_{\Delta} = 0$ resulting in an infinite radius. To ensure physical significance for robot kinematics:
    *   $R_i$ is only computed if $\theta_i > 1.0^\circ$.
    *   To prevent float overflow and skewing the average, any $R_i > 100.0$ meters is capped at $100.0$m.
5.  The system reports the **Minimum**, **Maximum**, and **Average** turn radii.

### D. Nearest Obstacle Distance (Clearance)
For a costmap with origin $(x_o, y_o)$, resolution $r$, and a 2D occupancy grid where cell values $\ge 100$ indicate obstacles:
1.  Identify all obstacle cell indices $(row, col)$ and convert them to metric space coordinates:
    $$O_j = (x_o + col \cdot r, \ y_o + row \cdot r)$$
2.  For each point $P_i$ along the path, compute the Euclidean distance to all obstacle points:
    $$d(P_i, O_j) = \sqrt{(x_i - x_{O_j})^2 + (y_i - y_{O_j})^2}$$
3.  The path clearance is the minimum recorded distance:
    $$\text{Clearance} = \min_{i, j} d(P_i, O_j)$$

### E. Path Cost
Path cost evaluates the traversability and steepness of the terrain covered by the path:
1.  For each path waypoint $P_i = (x_i, y_i)$, identify the corresponding costmap cell coordinate:
    $$row = \lfloor \frac{y_i - y_o}{r} \rfloor, \quad col = \lfloor \frac{x_i - x_o}{r} \rfloor$$
2.  The cost of the point is the value in the costmap grid: $C_i = \text{cost\_grid}[row, col]$ (defaulting to 100 if out of bounds).
3.  **Total Path Cost:** $C_{total} = \sum_{i=0}^n C_i$
4.  **Average Path Cost:** $C_{avg} = \frac{C_{total}}{n+1}$

---

## 3. Report Styling & Design System

The visual design is implemented in a separate module (`report_generator.py`) and is shared by both `benchmarking_node` and `testing_node` to ensure identical styling across standalone runs and orchestrated batch tests. It mirrors the page layout of `mostafa report .pdf`:

*   **Color Palette:** Premium dark blue (`#1A365D`) for table headers, slate gray (`#718096`) for labels, muted green (`#C6F6D5` background / `#22543D` text) for success/pass statuses, and muted red (`#FED7D7` background / `#9B2C2C` text) for failures/breaches.
*   **Header & Footer:** Features a neat uppercase header and horizontal separating lines. The footer displays the generation timestamp and a dynamically calculated page counter ("Page X of Y").
*   **Top Evaluation Banner:** A prominent status box that displays whether the system maintained stable performance (overall pass rate $\ge 85\%$) or breached limits.
*   **Summary Cards:** A 4-column flex grid showing key diagnostics: Path Success Rate, Mean Planning Time, Max Turn Angle, and Min Obstacle Clearance.
*   **Statistical Table:** Summarizes Min, Mean, and Max for every evaluated metric vector across all tests.
*   **Per-Test Table:** On page 2 (in PDF) or the second section (in HTML), a detailed breakdown table details the performance statistics of every executed test scenario, using colored status badges.

---

## 4. Execution & Validation Instructions

### Run Standalone Monitor
To start the monitor node independently:
```bash
ros2 run global_path_benchmarking benchmarking_node
```
In another terminal, run your path planner algorithm node, publish map data, start and goal poses, and plans. When you shut down the benchmarking node (Ctrl+C), it prints a summary console log and generates `benchmark_report.html`, `benchmark_report.pdf`, and `benchmark_history.csv` under the `reports/` folder.

### Run Orchestrated Batch Tests
To launch the full synthetic scenario pipeline:
```bash
ros2 launch global_path_benchmarking benchmark.launch.py
```
This automatically spins up the environment, executes all scenarios, cleans up, and aggregates the results into `reports/numerical_report.html` and `reports/numerical_report.pdf`.
