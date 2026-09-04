# 🛰️ SLAM & EKF Testing Suite: Developer Implementation Tasks (A to Z)

This document contains the step-by-step developer task specifications for implementing the `slam_benchmarking` test suite inside `testing/SLAM/` from A to Z.

For the system architecture, scenario definitions, mathematical formulations, configuration schemas, and operational instructions, refer to [**`README.md`**](file:///e:/meseket/Autonmous-27/testing/SLAM/README.md).

---

## 📋 Comprehensive GitHub Projects Implementation Tasks

---

### Task 1: Package Scaffolding, Configuration Schemas & Build Manifests
- **Target Files**:
  - `testing/SLAM/package.xml`
  - `testing/SLAM/setup.py` & `testing/SLAM/setup.cfg`
  - `testing/SLAM/config/scenarios.yaml`
  - `testing/SLAM/config/benchmark_config.yaml`
  - Directory structure: `mock/`, `slam_benchmarking/`, `rviz/`, `launch/`, `reports/`, `test/`
- **Developer Instructions**:
  1. **ROS 2 Package Manifest (`package.xml`)**:
     - Package name: `slam_benchmarking`.
     - Build type: `ament_python`.
     - Dependencies:
       `rclpy`, `nav_msgs`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_geometry_msgs`, `builtin_interfaces`, `std_msgs`, `python3-numpy`, `python3-scipy`, `python3-matplotlib`, `python3-yaml`, `python3-jinja2`.
  2. **Python Setup Script (`setup.py`)**:
     - Declare executable entry points in `console_scripts`:
       - `mock_sensor_streamer = slam_benchmarking.mock_sensor_streamer:main`
       - `ground_truth_broadcaster = slam_benchmarking.ground_truth_broadcaster:main`
       - `benchmarking_node = slam_benchmarking.benchmarking_node:main`
       - `testing_node = slam_benchmarking.testing_node:main`
     - Register `data_files` to install `config/*.yaml`, `launch/*.launch.py`, `rviz/*.rviz`, and `templates/*.html` into the package share directory.
  3. **Configuration Files**:
     - `config/scenarios.yaml`: Define the 8 Mars Yard scenarios with durations, speeds, trajectory types (`straight`, `square`, `circle`, `slalom`, `slopes`), slip injection windows (`start_time`, `end_time`, `slip_wheel_speed`, `true_ground_speed`), and 6-DOF ArUco landmark coordinates `[id, x, y, z, roll, pitch, yaw]`.
     - `config/benchmark_config.yaml`: Define evaluation weights ($S_{\text{ate}}: 0.35, S_{\text{rpe}}: 0.20, S_{\text{slip}}: 0.20, S_{\text{loop}}: 0.15, S_{\text{timing}}: 0.10$), limits, and pass threshold ($85.0$).
- **Implementation Pitfall**: Ensure all non-Python data directories (`config`, `launch`, `rviz`) are recursively included in `setup.py` `data_files`, otherwise ROS 2 launch will fail to locate them in `install/`.
- **Acceptance Criteria**:
  ```bash
  colcon build --packages-select slam_benchmarking
  source install/setup.bash
  ros2 pkg prefix slam_benchmarking
  ```

---

### Task 2: Ground Truth Broadcaster & Parametric Kinematic Simulator
- **Target Files**:
  - `testing/SLAM/mock/ground_truth_broadcaster.py`
  - `testing/SLAM/slam_benchmarking/synthetic_data_generator.py`
- **Developer Instructions**:
  1. **Ground Truth Kinematics (`ground_truth_broadcaster.py`)**:
     - Inherits from `rclpy.node.Node` (`GroundTruthBroadcaster`).
     - Declares parameters: `scenario_id` (string), `update_rate` (default: 100.0 Hz).
     - Maintains internal simulation state: $\mathbf{s}(t) = [x, y, z, \dot{x}, \dot{y}, \dot{z}, \phi, \theta, \psi, \omega_x, \omega_y, \omega_z]^T$.
     - Evaluates closed-form parametric trajectories based on active scenario:
       - *Straight Line*: $x(t) = v \cdot t, y(t) = 0, z(t) = 0, \psi(t) = 0$.
       - *Square Loop*: Waypoint interpolation $(0,0) \to (10,0) \to (10,10) \to (0,10) \to (0,0)$ with smooth continuous-curvature fillet arcs at corners.
       - *Slalom / Sinusoidal*: $x(t) = v \cdot t, y(t) = A \sin(k x(t)), \psi(t) = \arctan(A k \cos(k x(t)))$.
       - *Slopes*: $z(t) = H \sin(k x(t))$, computing dynamic pitch angle $\theta(t) = -\arctan(\frac{dz}{dx})$.
  2. **Publishers & TF Broadcaster**:
     - `/ground_truth/pose` (`geometry_msgs/msg/PoseStamped` with `frame_id: "world"`).
     - `/ground_truth/path` (`nav_msgs/msg/Path` appended at 10 Hz).
     - TF transform: `world -> ground_truth_rover` broadcasted continuously at 100 Hz.
  3. **Verification Plot Generator (`synthetic_data_generator.py`)**:
     - Implements `generate_reference_plots()` to satisfy Mode 5 (generating PNG previews of ideal paths and slip windows before running tests).
- **Implementation Pitfall**: Use `scipy.spatial.transform.Rotation` to compute quaternions from Euler angles $(\phi, \theta, \psi)$ to eliminate gimbal lock on 3D sloped terrains.
- **Acceptance Criteria**:
  ```bash
  ros2 run slam_benchmarking ground_truth_broadcaster --ros-args -p scenario_id:=square_loop_closure
  ros2 topic hz /ground_truth/pose # Holds steady at 100.0 ± 1.0 Hz
  ```

---

### Task 3: Mock Sensor Streamer & Real-Time Fault/Slip Injector
- **Target Files**:
  - `testing/SLAM/mock/mock_sensor_streamer.py`
- **Developer Instructions**:
  1. **Node Architecture (`MockSensorStreamer`)**:
     - Subscribes to `/ground_truth/pose` to receive current kinematic state.
  2. **Wheel Odometry Publisher (`/wheel/odom_raw` @ 50 Hz)**:
     - Computes longitudinal speed $v_x = \dot{x} \cos\psi + \dot{y} \sin\psi$ and yaw rate $\omega_z = \dot{\psi}$.
     - **Slip State Machine**:
       - Checks if simulation time $t$ falls within scenario's `slip_injections: [{start_time, end_time, slip_wheel_speed, true_ground_speed}]`.
       - *During Slip*: Override $v_{\text{wheel}} = \text{slip\_wheel\_speed}$ ($1.5\text{ m/s}$) while ground truth is stationary.
       - *Normal State*: $v_{\text{wheel}} = v_x + \mathcal{N}(0, 0.005^2)$.
     - Publishes `nav_msgs/msg/Odometry` with frame `odom` and child frame `base_link`.
     - Initializes diagonal elements of `pose.covariance` and `twist.covariance` to $0.001$.
  3. **IMU Publisher (`/imu/data` @ 100 Hz)**:
     - Calculates body accelerations $\mathbf{a}_b = \mathbf{R}^T (\mathbf{a}_{\text{world}} + \mathbf{g})$ where $\mathbf{g} = [0, 0, 9.81]^T$.
     - Injects Gaussian white noise ($\sigma_a = 0.02\text{ m/s}^2, \sigma_g = 0.005\text{ rad/s}$) and constant bias drift ($b_g = 0.001\text{ rad/s}$).
     - Publishes `sensor_msgs/msg/Imu` with `frame_id: "imu_link"`.
  4. **ArUco Landmark PnP Streamer (`/perception/aruco_pose` @ 10 Hz)**:
     - For each marker in `scenarios.yaml`, calculates relative distance $d = \|\mathbf{p}_{\text{marker}} - \mathbf{p}_{\text{rover}}\|$.
     - If $d < 4.0\text{ m}$ and marker is within camera field of view ($\pm 40^\circ$), computes 6-DOF relative transformation into `camera_depth_optical_frame`.
     - Publishes `geometry_msgs/msg/PoseStamped` on `/perception/aruco_pose` and visual markers on `/perception/aruco_markers_vis`.
  5. **Synthetic Depth & RGB Feeds (`/camera/*` @ 30 Hz)**:
     - Generates synthetic $640\times 480$ 16-bit depth images (`16UC1`) containing a flat ground plane gradient.
     - Generates $640\times 480$ 8-bit BGR images (`bgr8`) with random high-contrast feature dots (corners) and embedded ArUco patterns.
     - When `featureless_lighting_drop` scenario is active, zeroes out frame buffers during the blackout window.
- **Implementation Pitfall**: In ROS 2, `nav_msgs/msg/Odometry` covariance fields are flattened 36-element arrays. Diagonal indices for linear $X$, linear $Y$, and angular $Z$ are `0`, `7`, and `35`.
- **Acceptance Criteria**:
  ```bash
  ros2 run slam_benchmarking mock_sensor_streamer --ros-args -p scenario_id:=severe_wheel_slip
  # Verify slip velocity spike:
  ros2 topic echo /wheel/odom_raw --field twist.twist.linear.x
  ```

---

### Task 4: Trajectory & SLAM Mathematical Evaluator Engine
- **Target Files**:
  - `testing/SLAM/slam_benchmarking/trajectory_evaluator.py`
  - `testing/SLAM/test/test_evaluator.py`
- **Developer Instructions**:
  1. **Trajectory Synchronizer & Alignment Engine**:
     - `add_ground_truth_pose(t, x, y, z, qx, qy, qz, qw)`
     - `add_estimated_pose(t, x, y, z, qx, qy, qz, qw)`
     - `add_raw_wheel_pose(t, x, y, z, qx, qy, qz, qw)`
     - `add_slip_event(t_injected, t_detected, covariance_val)`
     - `associate_trajectories(max_dt=0.02)`: Synchronizes time series via nearest-neighbor timestamp matching.
     - `align_trajectories_se3()`: Solves the Umeyama closed-form rigid transformation aligning estimated poses to ground truth.
  2. **Quantitative Metric Algorithms**:
     - **ATE**: Computes RMSE, Max, Mean, and Standard Deviation.
     - **RPE**: Computes windowed relative translation drift ($\%$ per meter) and rotational drift ($^\circ/\text{m}$).
     - **Slip Metrics**: Computes detection delay $T_{\text{slip\_latency}} = t_{\text{flag}} - t_{\text{injected}}$, True Positive Rate ($\text{TPR}$), and covariance inflation ratio $\frac{\text{Tr}(\mathbf{\Sigma}_{\text{slip}})}{\text{Tr}(\mathbf{\Sigma}_{\text{nominal}})}$.
     - **Loop Closure**: Measures residual offset upon revisiting origin $(0,0)$.
     - **Scoring Equation**:
       $$S_{\text{slam}} = 0.35 S_{\text{ate}} + 0.20 S_{\text{rpe}} + 0.20 S_{\text{slip}} + 0.15 S_{\text{loop}} + 0.10 S_{\text{timing}}$$
- **Implementation Pitfall**: Avoid trajectory extrapolation if estimated and ground truth buffers have slight timestamp offsets at the start/end of recording.
- **Acceptance Criteria**:
  ```bash
  pytest testing/SLAM/test/test_evaluator.py # All unit tests pass with known mathematical trajectory errors
  ```

---

### Task 5: Multi-Format Report Generator & Chart Compiler
- **Target Files**:
  - `testing/SLAM/slam_benchmarking/report_generator.py`
  - `testing/SLAM/slam_benchmarking/templates/report_template.html`
- **Developer Instructions**:
  1. **Matplotlib Plotting Pipelines**:
     - `plot_2d_trajectory(gt, est, raw, markers, out_png)`: 2D overhead map comparing Green Ground Truth, Blue EKF, Red Dashed Wheel Odometry, and Yellow ArUco markers.
     - `plot_error_curves(timestamps, ate_errors, out_png)`: Time-series plot of Euclidean error with highlighted slip injection windows.
     - `plot_covariance_timeline(timestamps, v_wheel, v_imu, cov_values, out_png)`: Dual-axis plot validating velocity discrepancy vs covariance inflation spikes.
  2. **HTML Dashboard Generator (Jinja2)**:
     - Render `report_template.html` with responsive Bootstrap cards, status badges (PASS/FAIL), sortable summary tables, and Base64-embedded charts.
  3. **PDF Exporter (WeasyPrint)**:
     - Compile rendered HTML into standalone `reports/slam_benchmark_report.pdf`.
  4. **Markdown & CSV Exporters**:
     - Write clean summary tables to `reports/slam_numerical_report.md`.
     - Export time-series telemetry to `reports/scenario_<id>.csv`.
- **Acceptance Criteria**:
  - Running generator produces valid `reports/slam_benchmark_report.html`, `reports/slam_benchmark_report.pdf`, `reports/slam_numerical_report.md`, and high-res PNG plots.

---

### Task 6: Master Benchmarking Orchestrator & Launch Infrastructure
- **Target Files**:
  - `testing/SLAM/slam_benchmarking/benchmarking_node.py`
  - `testing/SLAM/slam_benchmarking/testing_node.py`
  - `testing/SLAM/launch/benchmark.launch.py`
  - `testing/SLAM/launch/live_test.launch.py`
  - `testing/SLAM/rviz/slam_benchmark.rviz`
- **Developer Instructions**:
  1. **Batch Benchmarking Orchestrator (`benchmarking_node.py`)**:
     - Helper `clean_old_processes()` to terminate orphaned ROS 2 nodes before running.
     - Loads `scenarios.yaml`.
     - Iterates through scenarios sequentially:
       a. Brings up `rover_slam` nodes (`heuristic_slip_checker.py`, `ekf_node`, `rtabmap`).
       b. Starts `mock_sensor_streamer` with scenario parameters.
       c. Records `/odometry/filtered`, TF `map -> base_link`, and slip checker flags into memory buffers.
       d. On scenario completion, triggers `TrajectoryEvaluator` and transitions to next scenario.
     - Compiles summary report via `report_generator`.
  2. **Live Single Test Orchestrator (`testing_node.py` & `live_test.launch.py`)**:
     - Launches single scenario with continuous RViz path visualization and live TF broadcasts.
  3. **RViz Display Configuration (`rviz/slam_benchmark.rviz`)**:
     - Pre-configured RViz display layers (Green GT Path, Blue EKF Path, Red Slipping Wheel Path, Yellow ArUco markers, Flashing Red Slip sphere).
- **Implementation Pitfall**: Use `subprocess.Popen` with process group detachment (`preexec_fn=os.setsid`) to ensure clean shutdown between scenario transitions.
- **Acceptance Criteria**:
  ```bash
  # Live single test in RViz:
  ros2 launch slam_benchmarking live_test.launch.py scenario_id:=square_loop_closure use_rviz:=true
  # Batch headless benchmark:
  ros2 launch slam_benchmarking benchmark.launch.py clean:=true
  ```

---

### Task 7: Master Interactive CLI Runner & End-to-End Validation
- **Target Files**:
  - `testing/SLAM/run_testing_suite.sh`
- **Developer Instructions**:
  1. **Master Interactive Bash Script (`run_testing_suite.sh`)**:
     - Executable permissions: `chmod +x`.
     - Detects and sources active ROS 2 distro (`Humble` / `Jazzy`).
     - Builds packages: `colcon build --packages-select rover_slam slam_benchmarking`.
     - Sources install overlay: `source install/setup.bash`.
     - Displays interactive CLI menu:
       - `1) 🎮 Live Interactive Closed-Loop Test (Nav2/SLAM + Mock Feeder + Live RViz)`
       - `2) ⚡ Headless Automated Batch Benchmark (all 8 scenarios with full reports)`
       - `3) 📊 Visual Automated Batch Benchmark (watch all 8 scenarios live in RViz)`
       - `4) 🎯 Single Scenario Benchmark (headless or with RViz)`
       - `5) 🗺️  Verify Scenario Reference Trajectories (generate PNG plots)`
     - Traps `SIGINT` (Ctrl+C) to gracefully terminate child subprocesses.
  2. **Cross-Distro Compatibility**:
     - Verify on ROS 2 Humble and ROS 2 Jazzy.
- **Acceptance Criteria**:
  - Running `./testing/SLAM/run_testing_suite.sh` executes all 5 menu options smoothly without manual typing.
  - Complete batch benchmark run executes all 8 scenarios and generates full reports with zero unhandled exceptions.
