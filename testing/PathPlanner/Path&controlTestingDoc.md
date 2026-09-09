# 📋 PathPlanner & Control Tester: Implementation Tasks & Verification Guide (`Path&controlTestingDoc.md`)

> 💡 **Scope Notice:** This document is **strictly dedicated to the standalone PathPlanner & Control TESTER** (`testing/PathPlanner/`). It provides the exact technical implementation tasks, mathematical formulations, and step-by-step code edits required to make the test harness 100% self-sufficient, realistic in its mock physics simulation, and fully verified against our current `erc_path_planner` and Control stack.

---

## 🔍 System Verification & Compatibility Audit

Before starting implementation, here is how the tester directly interfaces with our current codebase:

| Subsystem Component | Current Codebase Entity | Tester Integration Point | Compatibility Status |
| :--- | :--- | :--- | :---: |
| **Global Path Planner** | `nav2_smac_planner::SmacPlannerHybrid` in `nav2_params.yaml` | `testing_node.py` feeds `/map`, Start pose, and 4 Waypoints; validates `/plan` Reeds-Shepp feasibility | ✅ Verified Compatible |
| **Local Trajectory Controller** | `nav2_mppi_controller::MPPIController` (2000 rollouts @ 20Hz) | `mock_rover_sim.py` ingests `/cmd_vel` @ 50Hz, calculates CTE, jerk, and rollout tracking | ✅ Verified Compatible |
| **Perception Costmap Bridge** | `costmap_bridge_node.cpp` | `mock_perception.py` publishes `/terrain/obstacle_features` ➔ `/bridge/pointcloud` ➔ `local_costmap` | ✅ Verified Compatible |
| **State Estimation & TFs** | Nav2 expects `map -> odom -> base_link` | `mock_rover_sim.py` publishes `/odometry/filtered` (50Hz) and continuous dynamic TF transforms | ✅ Verified Compatible |
| **Multi-Waypoint Actions** | `bt_navigator` (`navigate_through_poses_w_replanning_and_recovery.xml`) | `testing_node.py` executes `NavigateThroughPoses` Action Client for 4 sequential destinations | 🔄 Covered in Task 1 |

---

## 📊 Tester Implementation Progress Matrix

| ID | Task Name | Target Files | Key Focus Area | Status | Priority |
|:---|:---|:---|:---|:---:|:---:|
| **Task 1** | [Multi-Waypoint Action Client](#-task-1-tester-core-multi-waypoint--action-client-support-in-benchmarking-runner) | `testing_node.py`, `scenarios.yaml` | 4-waypoint route dispatch, leg split timing & tolerance checks | `[ ]` To Do | **High** |
| **Task 2** | [Mock Skid-Steer Kinematics Engine](#-task-2-tester-mock-mock-skid-steer-kinematics--actuator-model) | `mock_rover_sim.py`, `benchmark_config.yaml` | Skid-steer splitting, RPM conversion, acceleration slew rate & deadband | `[ ]` To Do | **High** |
| **Task 3** | [Terrain Slip & Drift Physics Engine](#-task-3-tester-mock-terrain-slip--noisy-odometry-physics-simulator) | `mock_rover_sim.py`, `benchmark_config.yaml` | Longitudinal & rotational wheel slip on sand/slopes, 50Hz EKF feedback | `[ ]` To Do | **Medium** |
| **Task 4** | [Dynamic 3D Obstacle Fault Injector](#-task-4-tester-perception-dynamic-3d-obstacle-fault-injector) | `mock_perception.py`, `drop_stone.py`, `scenarios.yaml` | Real-time path obstacle drops via CLI & RViz click, costmap bridge | `[x]` Done | **High** |
| **Task 5** | [Control Trajectory & Energy Evaluator](#-task-5-tester-metrics-advanced-control-trajectory--energy-evaluator) | `report_generator.py`, `testing_node.py` | Control effort ($J_u$), steering reversal hunting & heading RMSE | `[ ]` To Do | **Medium** |
| **Task 6** | [Live RViz2 Testing Visual Dashboard](#-task-6-tester-ui-unified-rviz2-visual-dashboard--telemetry-overlay) | `benchmark_view.rviz`, `live_test.launch.py` | 3D chassis model, MPPI rollout cloud, HUD telemetry & waypoint flags | `[ ]` To Do | **Low** |
| **Task 7** | [Headless Automated CI Test Runner](#-task-7-tester-ci-headless-automated-ci-test-runner) | `run_testing_suite.sh`, `benchmark.launch.py` | CLI flags (`--batch`, `--rviz`), exit status codes & cleanup traps | `[ ]` To Do | **Medium** |

---

## 📋 Granular Step-by-Step Task Specifications

---

### 🔹 Task 1: [Tester-Core] Multi-Waypoint & Action Client Support in Benchmarking Runner
* **Status:** `- [ ] To Do`
* **Target Files:**
  * [`testing/PathPlanner/global_path_benchmarking/testing_node.py`](global_path_benchmarking/testing_node.py)
  * [`testing/PathPlanner/config/scenarios.yaml`](config/scenarios.yaml)
* **What This Solves:**
  Currently, `testing_node.py` only tests single-point start-to-goal navigation (`NavigateToPose`). In rover competitions (ERC/URC), the rover must sequentially visit **4 distinct intermediate waypoints** ($W_1 \to W_2 \to W_3 \to W_4$) on a single mission run. This task upgrades the test runner to dispatch full multi-waypoint action goals and evaluate per-leg split times.

#### Detailed Code Implementation Steps:
1. **Add Waypoints to Scenarios in `config/scenarios.yaml`:**
   Add a `waypoints` list to all 9 scenarios containing 4 $[x, y, \theta]$ poses:
   ```yaml
   waypoints:
     - [ -2.0, -1.0, 0.0 ]   # W1 (Entry)
     - [  0.0,  0.5, 0.785 ] # W2 (Narrow Pass)
     - [  1.5,  2.0, 1.57 ]  # W3 (S-Bend)
     - [  3.0,  3.0, 0.0 ]   # W4 (Destination Goal)
   ```
2. **Implement Nav2 `NavigateThroughPoses` Action Client in `testing_node.py`:**
   ```python
   from rclpy.action import ActionClient
   from nav2_msgs.action import NavigateThroughPoses
   from geometry_msgs.msg import PoseStamped

   # Inside __init__:
   self._nav_through_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
   ```
3. **Construct Goal Dispatch Method:**
   ```python
   def send_waypoint_mission(self, waypoints_list):
       self._nav_through_client.wait_for_server(timeout_sec=5.0)
       goal_msg = NavigateThroughPoses.Goal()
       poses = []
       for pt in waypoints_list:
           p = PoseStamped()
           p.header.frame_id = "map"
           p.header.stamp = self.get_clock().now().to_msg()
           p.pose.position.x = float(pt[0])
           p.pose.position.y = float(pt[1])
           p.pose.orientation.w = 1.0
           poses.append(p)
       goal_msg.poses = poses
       self._send_goal_future = self._nav_through_client.send_goal_async(
           goal_msg, feedback_callback=self.waypoint_feedback_cb
       )
   ```
4. **Waypoint Feedback & Arrival Split Tracking:**
   * In `waypoint_feedback_cb(self, feedback_msg)`:
     * Check `feedback_msg.feedback.number_of_poses_remaining`.
     * When the remaining poses count decrements, log split elapsed time $t_{\text{split}}$, leg distance, and local CTE.
5. **Publish RViz Marker Pins:**
   * Publish cylinder/sphere markers with text labels on `/waypoints_markers` (`visualization_msgs/msg/MarkerArray`).

#### Terminal Verification Commands:
```bash
# 1. Run live test with waypoints
ros2 launch global_path_benchmarking live_test.launch.py scenario_id:=canyon_gate_passage use_rviz:=true

# 2. Monitor active Nav2 action in a separate terminal
ros2 action info /navigate_through_poses
```

#### Acceptance Criteria:
* `testing_node.py` successfully connects to `/navigate_through_poses` without action rejection.
* The mock rover clears $W_1 \to W_2 \to W_3 \to W_4$ sequentially with arrival distance tolerance $\le 0.15\text{ m}$.
* Output summary report records split metrics for each individual leg.

---

### 🔹 Task 2: [Tester-Mock] Mock Skid-Steer Kinematics & Actuator Model
* **Status:** `- [ ] To Do`
* **Target Files:**
  * [`testing/PathPlanner/mock/mock_rover_sim.py`](mock/mock_rover_sim.py)
  * [`testing/PathPlanner/config/benchmark_config.yaml`](config/benchmark_config.yaml)
* **What This Solves:**
  Currently, `mock_rover_sim.py` integrates velocities without enforcing physical motor constraints (acceleration limits, RPM limits, deadband). This task makes the simulator model realistic skid-steer motor dynamics so MPPI's commanded `/cmd_vel` is validated against physical rover limits.

#### Kinematic & Physical Constants:
* Track Width: $L = 0.65\text{ m}$
* Wheel Radius: $R = 0.15\text{ m}$
* Max Motor RPM: $\text{RPM}_{\max} = 120\text{ RPM}$ ($\approx 1.88\text{ m/s}$)
* Max Linear Acceleration: $a_{\max} = 1.2\text{ m/s}^2$
* Max Angular Acceleration: $\alpha_{\max} = 2.0\text{ rad/s}^2$

#### Detailed Code Implementation Steps:
1. **Implement Skid-Steer Velocity Splitting & RPM Conversion in `mock_rover_sim.py`:**
   ```python
   # Inside update_physics:
   v_left = self.vx - (self.wz * self.rover_width / 2.0)
   v_right = self.vx + (self.wz * self.rover_width / 2.0)

   # Convert linear velocity (m/s) to RPM
   rpm_left = (v_left * 60.0) / (2.0 * math.pi * self.wheel_radius)
   rpm_right = (v_right * 60.0) / (2.0 * math.pi * self.wheel_radius)
   ```
2. **Implement Acceleration Slew-Rate Limiter:**
   ```python
   # Limit dv and dw per timestep
   dt = (current_time - self.last_time).nanoseconds * 1e-9
   max_dv = self.max_linear_accel * dt
   max_dw = self.max_angular_accel * dt

   self.vx = np.clip(target_vx, self.prev_vx - max_dv, self.prev_vx + max_dv)
   self.wz = np.clip(target_wz, self.prev_wz - max_dw, self.prev_wz + max_dw)
   ```
3. **Add Actuator Deadband:**
   * If $|v_x| < 0.02\text{ m/s}$ and $|\omega_z| < 0.03\text{ rad/s}$, force $v_x = 0, \omega_z = 0$.
4. **Publish Diagnostic Topics:**
   * Publish `std_msgs/msg/Float32MultiArray` on `/rover/motor_rpm` ($[RPM_L, RPM_R]$).
   * Publish `sensor_msgs/msg/JointState` on `/rover/joint_states`.

#### Terminal Verification Commands:
```bash
# 1. Run mock simulator standalone
ros2 run global_path_benchmarking mock_rover_sim.py

# 2. Publish test twist command
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.2}}"

# 3. Verify RPM outputs
ros2 topic echo /rover/motor_rpm --once
```

#### Acceptance Criteria:
* Commanded velocity ramps smoothly without unphysical step changes.
* Wheel RPM calculations correctly match theoretical differential speeds and clamp at $\pm 120\text{ RPM}$.
* Output linear jerk remains within $\le 0.25\text{ m/s}^2$.

---

### 🔹 Task 3: [Tester-Mock] Terrain Slip & Noisy Odometry Physics Simulator
* **Status:** `- [ ] To Do`
* **Target Files:**
  * [`testing/PathPlanner/mock/mock_rover_sim.py`](mock/mock_rover_sim.py)
  * [`testing/PathPlanner/config/benchmark_config.yaml`](config/benchmark_config.yaml)
* **What This Solves:**
  Real rovers on Martian soil experience longitudinal and rotational slip. Adding slip and discrete sensor noise allows the tester to verify that MPPI's closed-loop tracking reacts to terrain drift without destabilizing.

#### Mathematical Model:
* Longitudinal Slip: $s_l = 0.15$ ($15\%$ slip degradation on commanded forward speed).
* Angular Slip: $s_a = 0.10$ ($10\%$ rotational drift).
* Gaussian Noise: $\sigma_v = 0.02\text{ m/s}$, $\sigma_\omega = 0.03\text{ rad/s}$.

#### Detailed Code Implementation Steps:
1. **Add Parameters to `mock_rover_sim.py`:**
   ```python
   self.declare_parameter('slip_ratio_linear', 0.15)
   self.declare_parameter('slip_ratio_angular', 0.10)
   self.declare_parameter('noise_std_v', 0.02)
   self.declare_parameter('noise_std_w', 0.03)
   ```
2. **Apply Slip and Noise in Kinematic Integration:**
   ```python
   # Apply slip degradation
   actual_vx = self.vx * (1.0 - self.slip_ratio_linear) + np.random.normal(0, self.noise_std_v)
   actual_wz = self.wz * (1.0 - self.slip_ratio_angular) + np.random.normal(0, self.noise_std_w)

   # Integrate position
   self.yaw += actual_wz * dt
   self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
   self.x += (actual_vx * math.cos(self.yaw)) * dt
   self.y += (actual_vx * math.sin(self.yaw)) * dt
   ```
3. **Publish Odometry and Dynamic TF:**
   * Publish `/odometry/filtered` (`nav_msgs/msg/Odometry`) at strictly **50 Hz**.
   * Broadcast continuous TF transforms `odom -> base_link` and `map -> odom`.

#### Terminal Verification Commands:
```bash
# Verify transform publishing rate
ros2 run tf2_ros tf2_echo odom base_link
```

#### Acceptance Criteria:
* TF and `/odometry/filtered` publish consistently at $50.0 \pm 1.0\text{ Hz}$.
* Mock rover drifts realistically under slip, and MPPI corrects steering to stay within $\overline{\text{CTE}} \le 0.12\text{ m}$.

---

### 🔹 Task 4: [Tester-Perception] Dynamic 3D Obstacle Fault Injector
* **Status:** `- [x] Completed`
* **Target Files:**
  * [`testing/PathPlanner/mock/mock_perception.py`](mock/mock_perception.py)
  * [`testing/PathPlanner/scripts/drop_stone.py`](scripts/drop_stone.py)
  * [`testing/PathPlanner/config/scenarios.yaml`](config/scenarios.yaml)
  * [`PathPlanning/erc_path_planner/config/nav2_params.yaml`](../../PathPlanning/erc_path_planner/config/nav2_params.yaml)
* **What This Solves:**
  Allows real-time injection of rock obstacles directly onto the rover's active path mid-motion, testing whether Nav2 (Smac Hybrid A* and MPPI controller) dynamically detects the new blockage and calculates an avoidance detour.

#### Implemented Features:
1. **Interactive CLI Dropper (`drop_stone.py` / `ros2 run global_path_benchmarking drop_stone`):**
   * Automatically samples rover live odometry (`/odometry/filtered`) and active route (`/plan`).
   * Calculates a waypoint **2.0m ahead** on the path and spawns an obstacle right in front of the rover.
   * Supports interactive loop mode (`-i`): every press of `[Enter]` drops a fresh stone.
   * Supports custom distance (`-d <meters>`) and specific coordinates (`x y`).
2. **RViz2 Mouse Clicking ("Publish Point"):**
   * Subscribed to `/clicked_point`. Selecting "Publish Point" in RViz and clicking anywhere on the ground instantly drops a boulder.
3. **Scenario YAML Scheduled Triggers:**
   * Time-triggered obstacle pop-ups in `config/scenarios.yaml` (e.g. canyon gate blockages).
4. **Costmap Pipeline & Height Filter Fix:**
   * Configured `min_obstacle_height: -0.5` and `max_obstacle_height: 2.5` in `nav2_params.yaml` to ensure points are accepted by ROS 2 Jazzy `ObstacleLayer`.
   * Sampled 5cm dense 2D bounding footprints in `costmap_bridge_node.cpp` to create solid lethal cost disks.

#### Terminal Commands:
```bash
# Auto-drop rock 2m ahead on active path:
./src/Autonmous-27/testing/PathPlanner/scripts/drop_stone.py

# Interactive continuous loop:
./src/Autonmous-27/testing/PathPlanner/scripts/drop_stone.py -i

# Or via ROS 2 entrypoint:
ros2 run global_path_benchmarking drop_stone
```

#### Acceptance Criteria:
* ✅ Dynamic obstacles appear immediately upon CLI trigger or RViz click.
* ✅ Obstacle footprint is populated in `global_costmap` and `local_costmap` with 0.85m inflation.
* ✅ Smac Hybrid A* replans within $\le 1.0\text{s}$ and MPPI steers safely around the obstacle with $\ge 0.35\text{m}$ clearance.

---

### 🔹 Task 5: [Tester-Metrics] Advanced Control Trajectory & Energy Evaluator
* **Status:** `- [ ] To Do`
* **Target Files:**
  * [`testing/PathPlanner/global_path_benchmarking/report_generator.py`](global_path_benchmarking/report_generator.py)
  * [`testing/PathPlanner/global_path_benchmarking/testing_node.py`](global_path_benchmarking/testing_node.py)
* **What This Solves:**
  Adds control-theoretic and energy efficiency metrics to the benchmark evaluation engine, verifying that MPPI produces smooth motor commands without oscillations or excessive battery drain.

#### Mathematical Formulations:
1. **Control Effort (Energy Integral):**
   $$J_u = \int_{0}^{T} \left( v_x(t)^2 + \omega_z(t)^2 \right) dt \approx \sum_{k=0}^{N-1} \left( v_k^2 + \omega_k^2 \right) \Delta t_k$$
2. **Steering Reversals (Oscillatory Hunting Count):**
   $$\text{Reversals} = \sum_{k=1}^{N-1} \mathbb{I}\left( \text{sign}\left(\frac{\Delta \omega_k}{\Delta t}\right) \neq \text{sign}\left(\frac{\Delta \omega_{k-1}}{\Delta t}\right) \right)$$
3. **Heading Tracking RMSE:**
   $$\text{RMSE}_{\theta} = \sqrt{\frac{1}{N}\sum_{k=0}^{N-1} \left( \text{wrap\_angle}(\theta_{\text{rover}}(k) - \theta_{\text{path}}(k)) \right)^2}$$

#### Detailed Code Implementation Steps:
1. **Log Telemetry Stream in `testing_node.py`:**
   * Maintain high-resolution telemetry log: $[(t_k, x_k, y_k, \theta_k, v_k, \omega_k, \text{cte}_k)]$.
2. **Implement Metric Calculation Functions in `report_generator.py`:**
   * `def compute_control_effort(telemetry): ...`
   * `def compute_steering_reversals(telemetry): ...`
   * `def compute_heading_rmse(telemetry): ...`
3. **Embed Metrics in HTML & PDF Reports:**
   * Add Energy and Smoothness scorecards in `reports/numerical_report.html` and `reports/numerical_report.pdf`.

#### Terminal Verification Commands:
```bash
ros2 launch global_path_benchmarking benchmark.launch.py scenario_id:=empty_straight
cat reports/results.json | grep -E "control_effort|steering_reversals|heading_rmse"
```

#### Acceptance Criteria:
* Metrics calculate cleanly without division by zero or NaN values.
* HTML/PDF reports render comparative bar charts and time-series plots for control smoothness.

---

### 🔹 Task 6: [Tester-UI] Unified RViz2 Visual Dashboard & Telemetry Overlay
* **Status:** `- [ ] To Do`
* **Target Files:**
  * [`testing/PathPlanner/rviz/benchmark_view.rviz`](rviz/benchmark_view.rviz)
  * [`testing/PathPlanner/launch/live_test.launch.py`](launch/live_test.launch.py)
* **What This Solves:**
  Provides an out-of-the-box, zero-configuration RViz2 interface showing live planning rollouts, robot state, costmap heatmaps, and on-screen HUD telemetry.

#### Configuration Elements in `benchmark_view.rviz`:
* **Global Plan:** Green line (`/plan` - `nav_msgs/msg/Path`).
* **MPPI Trajectory Rollouts:** Cyan multi-path rollout cloud (`/local_plan`).
* **3D Rover Body:** Real-time Rover chassis marker (`/rover_marker` on `base_link`).
* **Costmaps:** Overlay opacity $0.6$ showing lethal and inflated cost zones.
* **4 Waypoint Markers:** 3D numbered destination pins (`/waypoints_markers`).
* **On-Screen HUD Overlay:** Real-time velocity ($v_x$), CTE, and active waypoint index.

#### Terminal Verification Commands:
```bash
ros2 launch global_path_benchmarking live_test.launch.py use_rviz:=true
```

#### Acceptance Criteria:
* RViz2 loads in $< 3\text{ s}$ with zero unlinked TF or topic subscription errors.
* Visual trajectory tracking and costmap updates render smoothly at $\ge 30\text{ FPS}$.

---

### 🔹 Task 7: [Tester-CI] Headless Automated CI Test Runner
* **Status:** `- [ ] To Do`
* **Target Files:**
  * [`testing/PathPlanner/run_testing_suite.sh`](run_testing_suite.sh)
  * [`testing/PathPlanner/launch/benchmark.launch.py`](launch/benchmark.launch.py)
* **What This Solves:**
  Enables running the complete 9-scenario benchmark in non-interactive batch mode for automated regression testing and CI/CD pipelines.

#### Detailed Code Implementation Steps:
1. **Implement CLI Argument Parser in `run_testing_suite.sh`:**
   * Support `--batch`, `--rviz`, `--scenario <id>`, and `--output-dir <path>`.
2. **Implement Exit Status Code Propagation:**
   * Exit Code `0`: All 9 scenarios passed (Overall Score $\ge 85.0$, 0 collisions).
   * Exit Code `1`: Score $< 85.0$ or collision detected.
   * Exit Code `2`: Process crash or lifecycle timeout.
3. **Add Process Cleanup Traps:**
   ```bash
   cleanup() {
     pkill -9 -f "nav2|planner_server|controller_server|testing_node|mock_rover" || true
   }
   trap cleanup EXIT SIGINT SIGTERM
   ```

#### Terminal Verification Commands:
```bash
./testing/PathPlanner/run_testing_suite.sh --batch
echo "CI Exit Code: $?"
```

#### Acceptance Criteria:
* Batch mode executes all 9 scenarios headlessly without requiring user interaction.
* Correct exit code is returned to the shell and all background processes are cleanly terminated.
