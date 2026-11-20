# Testing & Benchmarking Completion Plan

- **Workspace:** `Autonmous_Ws/testing/` (branch: `TestingValidation`)
- **Target Runtime:** `autonomous27:jazzy` Docker image (ROS 2 Jazzy), Gazebo world `final_world_R&A.world`
- **Status:** Draft — pending benchmarking criteria and Phase 0 sign-off
- **Purpose:** Living implementation roadmap and evaluation criteria specification to complete `testing/` step by step across mocks, module benchmarkers, and unified mission validation.

> **Core Principle:** The mission is the evaluation goal; module observers explain route quality, tracking, perception, and localization across the same mission. Dependencies remain strictly one-way (`testing/` → production). Passing a mocked or single-module stage never claims full-stack or hardware readiness.

---

## 1. Current State Audit

### 1.1 Existing Directory & Package Inventory

Currently `testing/` contains **six `ament_python` packages**, one documentation-only bringup folder (`validation_bringup/`), and legacy root shell wrappers:

| Location | ROS 2 Package | Current Implementation | Open Gaps & Limits |
| :--- | :--- | :--- | :--- |
| `testing/mocks/pathplanning/pathplanner_mock_tools/` | `pathplanner_mock_tools` | `mock_rover_sim` (50 Hz diff/skid-steer kinematics, slip/noise params, `/odometry/filtered`, TF, markers), `mock_perception` (features, cloud, LaserScan, opt-in `Detection3DArray` on `/perception/obstacles_only`), `mocked_planner.launch.py`, `test_planner_standalone.launch.py`, `custom_maps/` (4 maps), `maps/` (9 scenarios), `mocked_planner.rviz`, legacy `algo_node`, `drop_stone`, `mock_base`, `generate_benchmark_maps` | Marking-only perception (no camera clearing ray source); no mid-run fault injection (odom/TF/perception freeze); standalone `mock_rover_sim` defaults (`0.8×0.65 m`, `wheel_radius=0.15`, `track_width=0.65`, 15% slip) differ from Gazebo/production (`0.9×0.7 m`, `0.06 m`, `0.49 m`) unless overridden; CWD-relative map generation (`T07`) |
| `testing/mocks/perception/perception_test_tools/` | `perception_test_tools` | Deterministic RGB/depth/PointCloud2/CameraInfo feeder (`scene.py`, `sensor_inputs.py`) with 7 scenarios (`rocks_and_marker`, `ground_only`, `missing_depth`, `invalid_cloud`, `dropout`, `turn_occlusion`, `terrain_hazard`); offline floor+box generator (`terrain_scene.py`); `test_stack.launch.py`, `sensor_inputs.launch.py`, `legacy_map_anchor.launch.py`; 3 pytest files | Fixed camera fixture only; floor placed at `base_link` Z=0 (vs Rover URDF `-0.255 m`); frontal 0.21 m marker triggers `POSE_AMBIGUOUS` (no positive accepted-landmark scene); no structured JSON ground-truth metadata topic for automated accuracy scoring |
| `testing/mocks/slam/slam_mock_tools/` | `slam_mock_tools` | `rover_sensor_adapter` (reuses `mock_rover_sim` with TF off, `0.06 m` radius / `0.49 m` track, zero noise/slip, expands 2 side joints → 4 wheels + planar IMU with centripetal accel + `drop_imu`/`drop_wheels`); `image_gate` (real RGB dropout); legacy `mock_aruco_publisher` & `costmap_test_stub`; 1 unit test file | Nominal zero-noise/zero-slip adapter only; no deterministic slip-window, IMU bias, or rotated-mount injection; legacy stubs emit zero stamps and unset covariances (`S08`) |
| `testing/benchmarking/pathplanning/` | `global_path_benchmarking` | **Track 1 (Honest probes):** `navigation_probe.py` (real `ComputePathToPose`/`NavigateToPose`, `goal_id`, terminal status, monotonic latency, SE(2) SAT `swept_collision()` on independent truth YAML/PGM), `planner_map_probe.py` (`blocked-wall`, `too-narrow`, `wide-passage`), 47 passing pytest checks (+ 2 skipped ReportLab PDF tests).<br>**Track 2 (Legacy suite):** `testing_node.py`, `benchmarking_node.py`, `report_generator.py`, `benchmark_config.yaml`, `scenarios.yaml` (9 scenarios), `benchmark.launch.py`, `live_test.launch.py`, `run_testing_suite.sh` | Track 1 is single-shot CLI only (no batch suite, no `NavigateThroughPoses`, no CTE/smoothness/path summary reports).<br>Track 2 still has `P02`–`P08`, `T01`–`T03`, `T07`: stops on `/plan` without awaiting Nav2 execution, uses `0.3 m` vertex circles on `/global_costmap/costmap` instead of `0.9×0.7 m` swept collision on independent truth, conflates periodic replans with dynamic obstacle response, drops no-response trials, leaves `benchmark_config.yaml` unwired, and has mismatched controller score weights (`0.35/0.25/0.20/0.10/0.10` in YAML vs `0.30/0.25/0.20/0.15/0.10` in code) |
| `testing/benchmarking/slam/slam_test_tools/` | `slam_test_tools` | `ekf_validator.py` (cadence/finite/dropout monitor), `local_estimator_probe.py` (8 synthetic contracts + timestamp-aligned error), `local_mocked.launch.py`, `ekf_validation.launch.py`, `test_slam_standalone.launch.py`, 3 pytest files | No packaged Gazebo entity truth bridge (currently ad-hoc scripts outside repo); no ATE/RPE/slip-F1/loop-closure/map-sanity evaluator, clock-reset acceptance probe, or HTML/MD/PDF reports; `SLAMImprovments.md` Section 9 cases incomplete |
| `testing/benchmarking/perception/perception_benchmark_tools/` | `perception_benchmark_tools` | `terrain_benchmark.py`: pure-Python stage timing (`avg`, `min`, `max`, `p95`, `stddev`, FPS) and point counts over `synthetic_frames()` or `--bag` SQLite3 XYZ replay | Offline helper timing only; no live ROS accuracy observer, no 3D IoU / Hungarian matching / ground-segmentation IoU / ArUco 6-DoF accuracy scoring, and `--bag` mode does not replay TF transforms |
| `testing/validation_bringup/` | *(none)* | `README.md` architectural placeholder only | No shared ROS package, named `M1`–`M4` launch profiles, scoped process-group supervisor, or cross-module Mission Referee |
| `testing/*.sh` & `testing/LunchWorld&Rover` | *(none)* | Legacy Gazebo+Teleop scripts (`launch_world_and_rover.sh`, `LunchWorld&Rover`, `LunchWorld&Rover.sh`) | Duplicate scripts using unsafe global `pkill -9` (`T04`) |

### 1.2 Open Defect & Requirement Traceability Matrix

> **Important distinction:** Production bugs (`PP10` Smac boundary, `PP11` stale-input stopping, `PP12`/`S14` EKF drift, Patchwork++ ground misclassification) must be fixed in production packages, not masked by test tolerances. The `testing/` plan builds the **reproducible automated gates** that expose and verify them.

| Tracked ID / Source | Summary of Issue or Requirement | Target Phase |
| :--- | :--- | :--- |
| `T01`, `T04`, `T07` (`testing/README.md`) | Missing `matplotlib`/optional `reportlab` in `package.xml`; unsafe global `pkill`/`clean` wrappers; CWD-relative `maps/` & `reports/` output | Phase 0 & Phase 1 |
| `T02`, `T03` (`testing/README.md`) | `live_test.launch.py` / `mocked_planner.launch.py` target selection & duplicate bridge ownership guard | Phase 1 (verify/test) & Phase 2 |
| `P02`–`P08` (`benchmarking/pathplanning/VALIDATION.md`) | Await real Nav2 terminal result; endpoint + expected-no-path + `0.9×0.7 m` swept collision on independent truth; goal-ID/phase correlation for dynamic replans; wire `benchmark_config.yaml` and remove hardcoded 50/50 blending; one record per scheduled trial | Phase 1 |
| `PP10`, `PP11` (`PathPlanning/VALIDATION.md`) | Automated negative boundary gate (`blocked_wall`, `too_narrow_0.6m`) and stale-input fail-closed stopping test suite | Phase 1 (gate/probe) + Phase 2 (fault injection) |
| `S08`, `T05` (`SLAM/VALIDATION.md`, `testing/README.md`) | Legacy zero-stamp SLAM stubs (`mock_aruco_publisher`, `costmap_test_stub`); clock-reset & lateral-accel drift test cases | Phase 2 (mocks) & Phase 3 (SLAM evaluator) |
| `SLAMImprovments.md` §9 | Single encoder source selection, cadence mismatch, counter discontinuity, rotated IMU mount, gravity conventions, native `/perception/landmark_detections` (+1 ID offset) vs legacy singular fixture, inspection costmap marking + ray clearing + TTL | Phase 2 (mocks) & Phase 3 (SLAM benchmarker) |
| `Perception/INTEGRATION_REVIEW.md` | Non-ambiguous ArUco positive scene; Rover-height (`-0.255 m`) floor test fixture; live ground-segmentation & 3D obstacle accuracy observer; clearing & bridge contract checks | Phase 2 (mocks) & Phase 4 (Perception benchmarker) |
| `testing/validation_bringup/README.md` | Shared multi-module mission profiles (`M1`–`M4`), fresh RTAB-Map DB per run, Mission Referee & fault attribution | Phase 5 |
| `testing/mocks/perception/Readme.md` | Regression comparisons, headless CI outputs, stable GUI API contract | Phase 6 |

---

## 2. Benchmarking Criteria Framework

This section defines **how** every subsystem and mission is benchmarked, **what** is measured, and **what passes or fails**.

### 2.1 Two-Tier Evaluation Model (Gates vs. Metrics)

Every trial produces **two distinct layers** of evaluation—never a single weighted number that hides a safety or correctness failure:

1. **Tier A — Hard Acceptance Gates (Non-Compensatory)**
   - Boolean invariants that must **all** be true for `outcome == PASS`.
   - Any Tier A failure immediately forces `FAIL` (or `TIMEOUT` / `INVALID`), regardless of path length, speed, FPS, or any other metric.
   - Missing truth, missing required topics, duplicate TF/topic owners, or unconfigured/crashed nodes force `INCOMPLETE` or `BLOCKED`—never a silent `PASS` or fallback constant.

2. **Tier B — Graded Performance Metrics (Raw SI Physical Units First)**
   - Every metric is stored and reported in **unscaled SI physical units** (`s`, `m`, `deg`, `m/s`, `m/s²`, `rad/s`, `Hz`, `%`, `IoU`, `F1`).
   - **Optional Diagnostic Sub-Scores (0–100):** Computed *only* within a single mode (never blended across `plan-only` and `navigate`), *only* when all Tier A gates pass and all required measurements are finite (`not None`). If any required metric is unavailable, `diagnostic_score = null`.
   - **No Cross-Module Averaging:** Never blend Planner score and Controller score 50/50 when evaluating planning-only runs or when controller execution is incomplete.

3. **Threshold Governance Tags**
   - **`[CONFIG_LIMIT]`**: Derived directly from committed robot/Nav2/EKF parameters (`nav2_params.yaml`, URDF, `ekf.yaml`). Objective violation = defect.
   - **`[PROVISIONAL]`**: Engineering target baselines used for regression tracking until formally approved.
   - **`[RELEASE_GATE]`**: Signed-off mission or ERC competition requirement (none assigned until team/rulebook sign-off).

### 2.2 Mandatory Provenance Envelope

Every benchmark JSON, Markdown, HTML, and PDF output must carry a provenance header so mocked checks can never be mistaken for real-stack or hardware validation:

| Field | Required Values / Semantics |
| :--- | :--- |
| `schema_version` | Semantic version of the report schema (e.g. `"2.0.0"`) |
| `run_id` | Unique UUID / timestamped run slug |
| `git_commit` / `git_dirty` | Exact SHA and clean/dirty tree indicator |
| `runtime` | `ros_distro` (`jazzy`), `docker_image_id`, `host_or_container`, `use_sim_time` |
| `seed` | Fixed integer RNG seed used for scenario/noise generation |
| `evaluation_mode` | e.g., `planner_only`, `nav_closed_loop`, `slam_local`, `slam_global`, `perception_live`, `mission_full` |
| `dependency_manifest` | Explicit source per subsystem: `map`, `localization`, `perception`, `rover_plant`, `clock` ∈ `{REAL, MOCK, GAZEBO, ROSBAG, DISABLED}` |
| `truth_source` | Exact ground-truth origin (e.g., `independent_scenario_yaml`, `gazebo_dynamic_pose_info`, `mock_kinematic_shared_model`, `synthetic_scene_gt`) |
| `claim_scope` | Explicit machine-readable disclaimer stating what this run proves and what remains unestablished |

### 2.3 Trial Lifecycle, Correlation & Outcome Taxonomy

- **One-Trial-One-Record Rule:** Every scheduled scenario/trial in a run emits **exactly one** terminal trial record—even if the planner returns no path, the action server rejects/aborts, or a node times out. Repeated periodic `/plan` publications never multiply successful trial counts (`P08`).
- **Correlation Barriers (`P05`):** Each trial records explicit monotonic and ROS timestamps for phase boundaries:
  - `t_request_sent` (bound to Nav2 `goal_id` UUID or scenario event sequence ID)
  - `t_initial_response` (first correlated action/plan output)
  - `t_fault_or_obstacle_injected` (exact moment mock obstacle or fault parameter activates)
  - `t_post_event_response` (first replan/recovery correlated **strictly after** `t_fault_or_obstacle_injected` that alters geometry or handles the injected event)
  - `t_terminal_result` (Nav2 action completion, cancel, abort, or deadline)
- **Six Mutually Exclusive Outcomes:**

| Outcome | Definition |
| :--- | :--- |
| `PASS` | Every applicable Tier A gate passed, all required telemetry arrived within timeouts, and configured thresholds were met. |
| `FAIL` | Evidence was validly observed, and at least one Tier A gate or acceptance criterion failed (e.g., collision, false path on blocked map, drift above limit, failed stale-input stop). |
| `TIMEOUT` | Action/node did not reach a terminal state before the trial deadline (with cancellation requested cleanly). |
| `INVALID` | Produced output violated structural/mathematical sanity (empty path, NaN/Inf, non-monotonic stamps, non-PSD covariance, wrong frame ID, duplicate authoritative publisher). |
| `INCOMPLETE` | Execution ended before required evidence was gathered (e.g., missing truth map, dropped required sensor stream in a nominal test, aborted startup, missing ReportLab for requested PDF). |
| `BLOCKED` | External prerequisite prevented starting the run (e.g., missing ROS package, unavailable Gazebo world/entity ID, missing bag file). |

---

### 2.4 Module-Specific Evaluation Criteria

#### A. Global Path Planner (`erc_path_planner` / Smac Hybrid A*)

- **Reference Robot & Config Parameters (`[CONFIG_LIMIT]` from `nav2_params.yaml`):**
  - Footprint: `0.90 m × 0.70 m` (`[±0.45, ±0.35] m`)
  - Minimum turning radius: `0.80 m` (`REEDS_SHEPP`)
  - Grid resolution: `0.05 m`; Planner `tolerance`: `0.50 m`; Controller `xy_goal_tolerance`: `0.35 m`, `yaw_goal_tolerance`: `0.40 rad`
  - Max planning time limit: `5.0 s`
- **Test Categories:**
  1. *Positive static scenarios:* 9 Mars Yard scenarios (`scenarios.yaml`) + `wide_passage` (`2.4 m`).
  2. *Negative expected-infeasible scenarios:* `blocked_wall` (`0.0 m`), `too_narrow` (`0.6 m` < `0.70 m` rover width), `out_of_map`, `goal_in_obstacle`.
  3. *Dynamic obstacle replanning:* Static route blocked mid-run after initial path acceptance.

| Tier | Criterion / Metric | Requirement / Definition | Governance |
| :--- | :--- | :--- | :--- |
| **Tier A** | Action / Path Validity | Finite coordinates, `len(poses) >= 2`, `frame_id == "map"`, strictly correlated to request `goal_id`. | `[CONFIG_LIMIT]` |
| **Tier A** | Positive Terminal Status | Positive scenarios must terminate with Nav2 `STATUS_SUCCEEDED` (4). | `[CONFIG_LIMIT]` |
| **Tier A** | Negative Expected Rejection | Expected-infeasible scenarios must return `STATUS_ABORTED` (6) / error code (`204`/`207`) and **zero** path poses (`PP10` gate). | `[CONFIG_LIMIT]` |
| **Tier A** | Conservative Swept-Footprint Check | `swept_collision(path, truth_map, 0.90, 0.70) is False` against **independent** scenario YAML/PGM (never Nav2's output costmap). Unknown/out-of-bounds cells default blocked unless scenario explicitly permits unknown exploration. | `[CONFIG_LIMIT]` |
| **Tier A** | Endpoint Tolerance | Start error ≤ `0.20 m` (when `use_start=false`/from current pose) and final pose within configured planner tolerance (`0.50 m`) / goal tolerance (`0.35 m`). | `[CONFIG_LIMIT]` |
| **Tier A** | Dynamic Replan Safety | After `t_obstacle_injected`, newly received path must clear the updated truth map (`swept_collision is False`) within replan deadline. | `[CONFIG_LIMIT]` |
| **Tier B** | End-to-End & Internal Latency | `t_plan_received - t_request_sent` (`s`) and planner internal solve time (`s`). Target ≤ `2.0 s` (`[PROVISIONAL]`), hard cap `5.0 s` (`[CONFIG_LIMIT]`). | `[PROVISIONAL]` / `[CONFIG_LIMIT]` |
| **Tier B** | Path Length & Reference Ratio | Total polyline length $L$ (`m`), reference efficiency ratio $L / L_{\text{ref}}$ (against scenario `reference_path`, valid for mazes/detours), and Euclidean ratio $L / D_{\text{euc}}$ (for straight baselines). | `[PROVISIONAL]` |
| **Tier B** | Swept Minimum Clearance | Exact minimum distance (`m`) from the `0.90 × 0.70 m` swept rover polygon to nearest occupied truth cell. Target ≥ `0.15 m` margin beyond half-width. | `[PROVISIONAL]` |
| **Tier B** | Traversability Cost | Mean and integrated cost sampled on the **independent** scenario/slope truth grid `[0, 100]`. | `[PROVISIONAL]` |
| **Tier B** | Kinematic Curvature & Cusps | Minimum turning radius (`m`) computed strictly on forward/reverse continuous arcs (excluding Reeds-Shepp gear-change cusps, which are counted separately as `cusp_count`), plus mean/max turn deflection angle (`deg`). Target continuous $R_{\min} \ge 0.80\text{ m}$. | `[CONFIG_LIMIT]` |
| **Tier B** | Dynamic Replan Reaction Time | `t_replan_received - t_obstacle_injected` (`s`). Target ≤ `2.0 s`. | `[PROVISIONAL]` |

---

#### B. Local Controller & Navigation Execution (`erc_path_planner` / MPPI + Smoother + Costmap Bridge)

- **Test Modes:** `navigate_to_pose`, `navigate_through_poses` (4-waypoint routes), `cancel_mid_run`, `reverse_maneuver`, `fault_stale_inputs` (`PP11`), and optional Tier-2 `motor_rpm` actuation check.

| Tier | Criterion / Metric | Requirement / Definition | Governance |
| :--- | :--- | :--- | :--- |
| **Tier A** | Terminal Action Outcome | Run must await actual `NavigateToPose`/`NavigateThroughPoses` terminal result (`SUCCEEDED`, `ABORTED`, `CANCELED`)—receiving `/plan` never closes a controller trial (`P02`). | `[CONFIG_LIMIT]` |
| **Tier A** | Executed Swept-Footprint Collision | `swept_collision(executed_truth_trace, truth_map, 0.90, 0.70) is False` over the entire execution interval `[t_request_sent, t_terminal_result]`. | `[CONFIG_LIMIT]` |
| **Tier A** | Arrival Accuracy (Truth & Est.) | Final pose error vs goal on ground-truth trajectory (and estimated TF separately) within `0.35 m` XY and `0.40 rad` yaw (`SimpleGoalChecker`). | `[CONFIG_LIMIT]` |
| **Tier A** | Actuator Command Bounds | Every `/cmd_vel` sample stays within `[-0.50, 1.00] m/s` linear X, `|vy| <= 0.01 m/s` (non-holonomic), and `[-1.00, 1.00] rad/s` angular Z. | `[CONFIG_LIMIT]` |
| **Tier A** | Single Command Publisher | Only `velocity_smoother` publishes `/cmd_vel` (`PP04`, `PP06`); recovery/controller feed `/cmd_vel_nav`. | `[CONFIG_LIMIT]` |
| **Tier A** | Cancellation & Stale-Input Stopping | Explicit cancel transitions to `STATUS_CANCELED` and commands zero `/cmd_vel` within stopping deadline. Under `fault_stale_inputs` (paused localization/perception > max age), rover **must not** execute unguarded recovery spins (`PP11` gate). | `[CONFIG_LIMIT]` |
| **Tier B** | Cross-Track Error (CTE) | Mean, RMS, P95, and Max perpendicular distance (`m`) from truth trace to active `/plan` polyline. Provisional target: Mean ≤ `0.15 m`, Max ≤ `0.35 m`. | `[PROVISIONAL]` |
| **Tier B** | Heading Tracking RMSE | Root-mean-square heading error (`deg`) between rover yaw and nearest path tangent during forward tracking. | `[PROVISIONAL]` |
| **Tier B** | Execution Time & Stall Ratio | Total mission duration (`s`), fraction of time stalled (`|v| < 0.02 m/s` while active), and Nav2 recovery behavior trigger count. | `[PROVISIONAL]` |
| **Tier B** | Smoothness & Control Effort | Linear accel std (`m/s²`), angular accel std (`rad/s²`), RMS jerk, control effort $J_u = \int (v_x^2 + \omega_z^2)\,dt$, steering reversals, and reverse-motion command fraction. | `[PROVISIONAL]` |
| **Tier B** | Command Cadence & Continuity | Measured `/cmd_vel` rate (`Hz`, target `20.0 ± 2.0 Hz`) and max inter-command gap (`s`) while navigating. | `[CONFIG_LIMIT]` |
| **Tier B** | Executed Clearance | Minimum distance (`m`) between executed `0.90 × 0.70 m` footprint and independent obstacle truth. | `[PROVISIONAL]` |

*(Note: Once diagnostic controller scoring is wired in Phase 1, `benchmark_config.yaml` weights `0.35/0.25/0.20/0.10/0.10` and `report_generator.py` weights `0.30/0.25/0.20/0.15/0.10` will be reconciled so config is the single source of truth.)*

---

#### C. SLAM & State Estimation (`rover_slam`: Encoder + Slip + EKF + RTAB-Map)

- **Evaluation Separation:** Local odometry (`odom → base_link` from EKF) and Global SLAM (`map → base_link` from RTAB-Map + EKF) are **always evaluated separately**.
- **Truth Alignment Policy:** Compare against `/testing/slam/truth` (nominal consistency only—cannot certify physical accuracy) or Gazebo `/world/marsyard/dynamic_pose/info` (independent truth). Align trajectories using **one initial rigid XY+yaw transform** over the stationary baseline, matching timestamps within ≤ `0.05 s`, with **no scale fitting** and **no per-sample re-alignment**.

| Tier | Criterion / Metric | Requirement / Definition | Governance |
| :--- | :--- | :--- | :--- |
| **Tier A** | Exclusive TF & Sensor Ownership | Only EKF publishes `odom → base_link`; only RTAB-Map publishes `map → odom`; single `/imu/data` publisher in transformable `imu_link`; single wheel source (`joint_states` or ticks). | `[CONFIG_LIMIT]` |
| **Tier A** | Output Cadence & Freshness | `/odometry/filtered` cadence within `30.0 Hz ± 35%`, max gap ≤ `0.25 s`, finite pose/twist/quaternions, speed ≤ `3.0 m/s`. | `[CONFIG_LIMIT]` |
| **Tier A** | Covariance & Input Sanity | Rejects non-finite, negative-diagonal, asymmetric, or non-PSD covariance matrices without latching collision state (`S09`). | `[CONFIG_LIMIT]` |
| **Tier A** | Fault & Dropout Contracts | Missing wheels stop both `/wheel/odom_raw` and `/wheel/odom_filtered` (zero synthetic velocity); missing IMU inflates filtered wheel covariance (var ≥ `2.0`) & emits degraded diagnostics; recovery restores all settled diagnostics to `healthy`. | `[CONFIG_LIMIT]` |
| **Tier A** | Clock-Reset Recovery | After simulator/clock backward jump (`WorldControl.reset.all`), converter/slip baselines reset and fresh-stamp EKF + RTAB-Map resume within recovery window (currently **failing** in Gazebo—must fail cleanly until fixed). | `[CONFIG_LIMIT]` |
| **Tier A** | Map Growth Sanity | Global RTAB-Map occupancy grid bounds must stay physically bounded relative to explored truth bounding box (prevents runaway `79 × 1772` grid corruption seen in `PP12`). | `[CONFIG_LIMIT]` |
| **Tier B** | Local & Global ATE | Absolute Trajectory Error (RMSE, Mean, Max, P95 translation in `m` and yaw in `deg`) computed separately for `/odometry/filtered` and `map → base_link`. | `[PROVISIONAL]` |
| **Tier B** | Relative Pose Error (RPE) | Drift per distance (`%` translation error and `deg/m` heading drift over `1 m`, `5 m`, `10 m` sub-trajectories). | `[PROVISIONAL]` |
| **Tier B** | Sustained-Turn Planar Error | Max planar error (`m`) during steady circular/turn segments (explicitly tracks `S14` lateral-acceleration bias). | `[PROVISIONAL]` |
| **Tier B** | Slip Detector Classification | True-positive rate, false-positive rate (especially hard braking `S11`), F1 score, detection latency (`s`), and recovery latency (`s`) against injected slip windows. | `[PROVISIONAL]` |
| **Tier B** | Global Mapping & Loop Closure | Loop/proximity closure count, pre-vs-post closure ATE drop (`m`), RGB-dropout recovery time (`s`), and landmark adapter (+1 ID offset) pose residual on `/perception/landmark_detections`. | `[PROVISIONAL]` |
| **Tier B** | Inspection Costmap & Ray Clearing | Verification that obstacle clouds mark, camera rays clear removed obstacles, and memory TTL expires stale boxes without duplicate bridge publishers. | `[PROVISIONAL]` |

---

#### D. Perception (`terrain_geometry` + `marker_detection` + `perception_bringup` + `costmap_bridge`)

- **Test Modes:** `offline_timing` (`terrain_benchmark`), `live_terrain_only`, `live_markers_only`, `live_combined`, and `bridge_and_clearing`.

| Tier | Criterion / Metric | Requirement / Definition | Governance |
| :--- | :--- | :--- | :--- |
| **Tier A** | Fault & Degradation Semantics | `invalid_cloud` → unavailable diagnostics + unknown grid (never valid clear terrain); `missing_depth` → RGB/PnP continues with depth-fusion flag false; `dropout` → freshness timeout triggers unavailable state and recovers on resume. | `[CONFIG_LIMIT]` |
| **Tier A** | Calibration & Age Rejection | Invalid, wrong-resolution, stale, or future-stamped `CameraInfo` or RGB frames are rejected; clock reversal clears tracks and depth buffers. | `[CONFIG_LIMIT]` |
| **Tier A** | Ambiguous Marker Safety | `POSE_AMBIGUOUS` detections (such as frontal synthetic scene) must be rejected by `marker_action_interface_node` and `rtabmap_landmark_adapter`. | `[CONFIG_LIMIT]` |
| **Tier A** | Clean-Floor False-Positive Gate | `ground_only` scenario (at Rover URDF `-0.255 m` floor height) must produce **0** persistent obstacle boxes (currently **failing** with Patchwork++—remains a failing gate until fixed). | `[PROVISIONAL]` |
| **Tier A** | Bridge & Header Contract | `costmap_bridge_node` preserves target frame IDs, rejects NaN/zero quaternions, respects point-count caps, and leaves empty valid clouds empty. | `[CONFIG_LIMIT]` |
| **Tier B** | Ground Segmentation Quality | Point-level Ground Precision, Recall, IoU, and misclassified floor-point count on labeled scenes (flat, sloped, stepped, Rover-height). | `[PROVISIONAL]` |
| **Tier B** | 3D Obstacle Detection & Box Accuracy | Hungarian-matched (`linear_sum_assignment`, match cutoff `0.50 m`) Precision, Recall, F1, 3D centroid error (`m`), 3D axis-aligned IoU, dimension error (`m`), and false positives per frame (`FP/frame`). | `[PROVISIONAL]` |
| **Tier B** | Temporal Memory & Occlusion | Track ID consistency (ID switches), occlusion retention across `turn_occlusion`, and clearing latency once object leaves scene. | `[PROVISIONAL]` |
| **Tier B** | ArUco Detection & 6-DoF Pose | Detection rate vs distance (`1–8 m`) and angle (`0–60°`), ID classification accuracy, translation error (`m`), orientation error (`deg`), and accepted landmark rate on valid non-ambiguous fixtures. | `[PROVISIONAL]` |
| **Tier B** | Live End-to-End Runtime | ROS callback/publish latency (p50, p95, max `ms`), sustained output frequency (`Hz`), and offline stage compute breakdown. | `[PROVISIONAL]` |

---

#### E. Integrated Mission Validation (`validation_bringup`)

- **Integration Ladder:**
  - **`M0`**: Offline unit, math, schema, and mock-physics checks (no ROS graph).
  - **`M1`**: Isolated single real module + deterministic mocks (wall time, mocked dependencies explicitly flagged).
  - **`M2`**: Single real module + Gazebo `final_world_R&A.world` + native entity truth (`use_sim_time=true`).
  - **`M3`**: Full autonomous stack (`my_robot_description` + `rover_slam` + `perception_bringup` + `erc_path_planner`) in Gazebo with fresh RTAB-Map DB per run.
  - **`M4`**: Deterministic `rosbag2` regression replay (same sensor data before/after code or tuning changes).
- **Mission Tier A Gates:** All scheduled mission waypoints reached (or expected-blocked handled cleanly); zero body collisions against Gazebo world truth; zero unguarded motion during fault injection; single TF/topic ownership across entire mission; no node crashes.
- **Mission Tier B Metrics & Fault Attribution:** Total mission time (`s`), waypoints completed (`k/N`), traversed distance vs reference (`m`), Nav2 recovery count, plus **automated cross-module fault attribution** (correlating timestamps of mission failures against SLAM ATE spikes, Perception FP/FN bursts, Planner abort codes, and Controller CTE peaks).
- **Competition Scoring:** ERC rulebook-specific point formulas will live in a separate pluggable `mission_scoring.yaml` once the target rulebook edition and scoring weights are finalized.

---

### 2.5 Unified Result JSON Schema Contract

Every module and mission evaluator writes a single JSON schema (`schema_version: "2.0.0"`):

```json
{
  "schema_version": "2.0.0",
  "provenance": {
    "run_id": "20261120T150000Z_m1_canyon_gate",
    "git_commit": "0ba1d75",
    "git_dirty": false,
    "ros_distro": "jazzy",
    "docker_image_id": "sha256:d952a938...",
    "seed": 42,
    "evaluation_mode": "nav_closed_loop",
    "dependency_manifest": {
      "map": "MOCK",
      "localization": "MOCK",
      "perception": "MOCK",
      "rover_plant": "MOCK",
      "clock": "WALL"
    },
    "truth_source": "independent_scenario_yaml",
    "claim_scope": "Mocked closed-loop navigation; does not establish SLAM, perception, or physical accuracy."
  },
  "scenario": {
    "id": "canyon_gate_passage",
    "category": "positive_static",
    "expected_outcome": "PASS"
  },
  "outcome": "PASS",
  "gates": {
    "action_succeeded": { "passed": true, "detail": "STATUS_SUCCEEDED" },
    "swept_collision_free": { "passed": true, "detail": "min_clearance_m=0.25" },
    "within_goal_tolerance": { "passed": true, "detail": "truth_err_m=0.12" },
    "single_cmd_vel_publisher": { "passed": true, "detail": "/velocity_smoother" }
  },
  "metrics": {
    "planning_time_s": 0.41,
    "path_length_m": 8.42,
    "reference_length_ratio": 1.05,
    "swept_min_clearance_m": 0.25,
    "mean_cte_m": 0.048,
    "rms_cte_m": 0.061,
    "goal_error_truth_m": 0.12,
    "cmd_freq_hz": 20.0
  },
  "diagnostic_score": null,
  "phases": [
    { "name": "initial_nav", "goal_id": "a1b2...", "t_start": 10.0, "t_end": 28.4 }
  ],
  "artifacts": {
    "plot_png": "plots/canyon_gate_passage.png"
  }
}
```

### 2.6 Visual Inspection vs. Automated Gates

- **Automated Numerical Gates** always run headless inside Docker, require zero GUI, and control CI exit codes.
- **RViz Visual Inspection** is a separate, non-substitutable verification gate (recording fixed frame, displayed topics, and visual sanity of costmaps, footprints, point clouds, and TF frames).

---

## 3. Phased Implementation Plan

Work proceeds sequentially one phase at a time. Each phase requires plan/diff approval before code edits and must pass its Docker verification gate before starting the next phase.

### Phase 0 — Criteria Sign-Off, Unified Schema & Workspace Hygiene
- **Goal:** Lock the evaluation contract (`TESTING_PLAN.md`), introduce a shared Python result/provenance helper module, fix packaging dependencies, and remove unsafe process cleanup.
- **Files / Packages Touched:**
  - `testing/TESTING_PLAN.md` (sign-off status update)
  - Shared schema/provenance helper (installed in a lightweight shared module or mirrored contract initially until `validation_bringup` exists in Phase 5)
  - `testing/benchmarking/pathplanning/package.xml` (`T01`: add `python3-matplotlib`; document optional `python3-reportlab`)
  - `testing/launch_world_and_rover.sh`, `testing/LunchWorld&Rover`, `testing/LunchWorld&Rover.sh` (`T04`: remove duplicate wrappers and global `pkill -9`; convert remaining launcher to PID/process-group supervision)
  - `testing/benchmarking/pathplanning/reports/` (archive or relocate stale historical outputs outside package source per `T07`)
- **Exit Criteria:**
  - `colcon list --base-paths testing` discovers 6 clean packages with zero duplicates.
  - `package.xml` dependencies resolve cleanly in `autonomous27:jazzy`.
  - Launching and stopping the world/rover script leaves an unrelated dummy process untouched (`T04` verification).

---

### Phase 1 — Complete PathPlanner & Controller Benchmarker (`global_path_benchmarking`)
- **Goal:** Merge the honest probe architecture (`navigation_probe` / `planner_map_probe`) with the multi-scenario batch evaluator (`testing_node` / `benchmarking_node` / `report_generator`), resolving `P02`–`P08`, `T02`–`T03`, and `T07`.
- **Files / Packages Touched:**
  - `testing/benchmarking/pathplanning/global_path_benchmarking/`:
    - Unify `benchmarking_node.py` and `testing_node.py` around action-correlated execution (`ComputePathToPose`, `NavigateToPose`, `NavigateThroughPoses` for 4-waypoint routes) so trials never close on `/plan` arrival (`P02`).
    - Enforce one-trial-one-record accounting (`P08`) and explicit dynamic-event phase correlation bound to `goal_id` + `t_obstacle_injected` (`P05`).
    - Replace `0.3 m` vertex checks on `/global_costmap/costmap` with `swept_collision()` (`0.90 × 0.70 m`) on **independent** scenario truth YAML/PGM (`P04`, `P07`) and add endpoint/expected-infeasible checks (`P03`).
    - Wire `benchmark_config.yaml` (`interfaces`, `control_benchmarking`, `weights`), align YAML vs Python controller weights, support `planner_only` vs `nav_closed_loop` modes cleanly, and remove hardcoded 50/50 fallback blending (`P06`).
    - Resolve installed `scenarios.yaml`/`maps/` via `ament_index_python` and require an explicit `--output-dir` outside source (`T07`).
    - Upgrade `report_generator.py` to emit the v2.0.0 schema JSON, Markdown, CSV, HTML, and PNG/PDF trajectory overlays with provenance banners and Tier A gate tables.
  - `testing/benchmarking/pathplanning/config/scenarios.yaml` & `benchmark_config.yaml`: add negative scenarios (`blocked_wall`, `too_narrow_0.6m`, `out_of_map`), multi-waypoint profiles, and align weights.
  - `testing/benchmarking/pathplanning/launch/`: retire/replace duplicate `live_test.launch.py` bridge conflict (`T02`, `T03`) and support clean batch execution.
  - `testing/benchmarking/pathplanning/test/`: expand unit + ROS contract tests for P02–P08, `NavigateThroughPoses`, phase correlation, and v2.0.0 reports.
- **Exit Criteria:**
  - All existing 47 + new pytest checks pass in `autonomous27:jazzy`.
  - Batch run against `pathplanner_mock_tools` executes all positive, negative, dynamic, cancellation, and reverse scenarios from an arbitrary working directory into `/validation/pp_run/`, producing honest outcomes (including failing `PP10` negative gates until production Smac is fixed).

---

### Phase 2 — Complete Shared Dependency Mocks & Fault Injection
- **Goal:** Upgrade all three mock packages so every downstream benchmarker gets deterministic, realistic, fault-injectable inputs matching production rover geometry without duplicating mocks.
- **Files / Packages Touched:**
  - `testing/mocks/pathplanning/pathplanner_mock_tools/`:
    - Add explicit vehicle geometry presets (`production_rover`: `0.90 × 0.70 m`, `wheel_radius=0.06`, `track_width=0.49`) and seeded slip/noise profiles.
    - Add dynamic fault-injection parameters/services (`freeze_odom`, `drop_tf`, `cmd_vel_override`) to test `PP11` stale-input stopping and recovery.
    - Add a `map_clear_obstacle` / dynamic obstacle removal option alongside clear documentation that marking-only perception has no camera clearing rays.
  - `testing/mocks/perception/perception_test_tools/`:
    - Add configurable floor height (`ground_z_offset`, supporting both `0.0` legacy and `-0.255` Rover `base_link` height).
    - Add a non-ambiguous, tilted/off-axis ArUco scenario (`clear_aruco_nav`) for positive `marker_detection` & `rtabmap_landmark_adapter` acceptance tests.
    - Add dynamic moving-camera / moving-box live scene option + JSON ground-truth metadata publisher (`/testing/perception/ground_truth_json`) for automated 3D IoU & pose scoring.
  - `testing/mocks/slam/slam_mock_tools/`:
    - Upgrade `rover_sensor_adapter` with deterministic time-windowed slip injection, IMU bias/noise profiles, rotated IMU mount TF option, and encoder tick/4-count publishing modes (`SLAMImprovments.md` §9).
    - Fix or cleanly deprecate `mock_aruco_publisher` and `costmap_test_stub` (`S08`: proper ROS timestamps and valid PSD covariances if retained for smoke tests).
  - Tests in all three mock packages verifying kinematics, fault triggers, non-ambiguous ArUco PnP, and sensor adapter physics.
- **Exit Criteria:**
  - All mock unit and integration tests pass in `autonomous27:jazzy`.
  - `clear_aruco_nav` passes marker confirmation without `POSE_AMBIGUOUS`.
  - Fault-injection triggers deterministically reproduce `PP11` stale-input behavior for PathPlanner and slip windows for SLAM.

---

### Phase 3 — Build SLAM & State Estimation Benchmarker (`slam_test_tools`)
- **Goal:** Complete the SLAM evaluator so it measures local EKF and global RTAB-Map accuracy, slip detection, loop closure, map sanity, and clock-reset recovery in both mocked (`M1`) and Gazebo (`M2`) runs.
- **Files / Packages Touched:**
  - `testing/benchmarking/slam/slam_test_tools/`:
    - Package `gazebo_truth_bridge` (reading `/world/marsyard/dynamic_pose/info` with verified entity lookup) into `slam_test_tools` instead of ad-hoc `/validation/` scripts.
    - Build `slam_evaluator.py` (offline + live observer): initial rigid alignment, timestamp matching (`<= 0.05 s`), Local ATE, Global ATE, RPE (`1/5/10 m`), sustained-turn error (`S14`), slip Precision/Recall/F1 + braking FP (`S11`), loop/proximity closure accounting, map bounding-box sanity (`PP12`), and clock-reset recovery gate.
    - Cover remaining `SLAMImprovments.md` §9 integration tests (single encoder source, cadence mismatch, counter discontinuity, rotated IMU mount, gravity conventions, native `/perception/landmark_detections` +1 ID offset, inspection costmap marking + clearing + TTL).
    - Add unified v2.0.0 JSON + HTML/MD/PNG trajectory/drift plot generator.
- **Exit Criteria:**
  - Unit tests for ATE/RPE/alignment/slip-F1 pass on synthetic trajectories with known ground-truth errors.
  - Automated `M1` (`local_mocked.launch.py`) and `M2` (Gazebo `final_world_R&A.world`) runs emit v2.0.0 reports accurately flagging existing `S14` turn drift and failed clock-reset recovery.

---

### Phase 4 — Build Live Perception Accuracy & Runtime Benchmarker (`perception_benchmark_tools`)
- **Goal:** Extend `perception_benchmark_tools` from offline helper timing into a live ROS accuracy, ground-segmentation, marker-pose, and latency benchmarker.
- **Files / Packages Touched:**
  - `testing/benchmarking/perception/perception_benchmark_tools/`:
    - Add `live_perception_benchmark.py`: subscribes to `/terrain/obstacle_features`, `/perception/obstacles_only`, `/terrain/costmap`, `/perception/aruco_detections` (or landmark output), `/diagnostics`, and ground-truth JSON from `perception_test_tools` or Gazebo truth.
    - Implement Hungarian bipartite matching (`scipy.optimize.linear_sum_assignment`, 0.50 m cutoff), 3D bounding-box IoU, ground-segmentation Precision/Recall/IoU, `ground_only` FP/frame gate, ArUco 6-DoF translation/orientation error, occlusion retention (`turn_occlusion`), and live end-to-end ROS latency (p50/p95/max ms).
    - Add TF-aware `rosbag2` replay support (`M4`) so recorded Gazebo camera streams transform into `base_link` consistently.
    - Emit unified v2.0.0 JSON + Markdown/HTML/PNG confusion & latency reports.
- **Exit Criteria:**
  - Unit tests verify Hungarian matching, 3D IoU, and ArUco SE(3) error math on known synthetic boxes.
  - Live `M1` run against `perception_test_tools` completes across all scenarios, passes fault/ambiguity gates, and accurately fails/quantifies the open Patchwork++ floor misclassification (`-0.255 m` and `0.0 m`).

---

### Phase 5 — Shared Orchestration & Mission Benchmarker (`validation_bringup`)
- **Goal:** Create the `validation_bringup` ROS 2 package to run cohesive multi-module missions (`M1`–`M4`) with unified provenance, scoped cleanup, and cross-module root-cause attribution.
- **Files / Packages Touched:**
  - `testing/validation_bringup/` (`package.xml`, `setup.py`, `profiles/`, `launch/`, `validation_bringup/`):
    - Named launch profiles (`M1_planner_mocked`, `M1_slam_mocked`, `M1_perception_mocked`, `M2_gazebo_slam`, `M2_gazebo_perception`, `M3_gazebo_full_mission`, `M4_bag_replay`) enforcing single TF/topic owners and generating a unique fresh RTAB-Map DB per run.
    - Scoped process-group supervisor (no global `pkill`).
    - `mission_referee.py`: executes multi-waypoint mission profiles in `final_world_R&A.world`, runs all three module observers concurrently, computes Mission Tier A/B metrics, and generates a cross-module fault-attribution timeline.
    - Pluggable `config/mission_scoring.yaml` stub ready for ERC rulebook criteria.
- **Exit Criteria:**
  - All `M1`–`M3` profiles construct cleanly (`colcon build` + launch tests).
  - One end-to-end `M3` Gazebo mission run executes under a single supervisor command, cleanly shuts down only its own process groups, and produces a unified mission + 3-module report.

---

### Phase 6 — Regression Comparison, Headless CI & GUI Integration Contract
- **Goal:** Enable automated before/after comparisons across commits or parameter changes, headless Docker CI execution, and clean API hooks for the sibling `GUI` workspace.
- **Files / Packages Touched:**
  - `testing/validation_bringup/validation_bringup/compare_runs.py`: diffs two v2.0.0 JSON run folders (deltas on ATE, CTE, swept clearance, latency, IoU, and gate regressions).
  - `testing/run_ci_validation.sh` (or Python runner): headless Docker entrypoint running `M0` + `M1` suites, returning exit code `0` only if expected gates pass, outputting JUnit XML + v2.0.0 JSON bundle.
  - `testing/GUI_CONTRACT.md` (or section in `validation_bringup`): stable CLI invocation, real-time stdout/JSON progress events, and output directory contract for the `/home/saif/Desktop/MESEKET/GUI` frontend.
  - Cleanup of deprecated compatibility shims once callers migrate.
- **Exit Criteria:**
  - Comparing two test runs outputs an accurate regression diff report.
  - Headless Docker CI command runs `M0` + `M1` deterministically without host dependencies.

---

## 4. Open Decisions Needed Before Code Implementation

Please confirm or adjust these choices before we begin **Phase 0**:

| # | Decision | Recommended Default | Unlocks |
| :--- | :--- | :--- | :--- |
| **D1** | **Two-Tier Scoring Model:** Approve Tier A hard gates + raw SI metrics, making 0–100 diagnostic scores secondary & gated on Tier A pass? | **Yes (Recommended)** — prevents high speed/short path from masking collisions or drift. | Phase 0 & 1 |
| **D2** | **Provisional Threshold Governance:** Start with `[PROVISIONAL]` engineering baselines tied to `nav2_params.yaml` (`0.90×0.70 m`, `0.35 m` goal tol, `2.0 s` plan time), adding ERC rulebook points later in `mission_scoring.yaml`? | **Yes (Recommended)** — unblocks all testing without guessing unapproved ERC rules. | Phase 0 & 5 |
| **D3** | **Stale-Input Stopping Policy (`PP11`):** On stale `/odometry/filtered`, TF, or `/perception/obstacles_only`, should the system *stop-and-cancel* or *stop-and-wait*, and what max input age (`0.5 s` / `1.0 s`) applies? | **Stop-and-wait up to `1.0 s`, then cancel** (needs explicit confirmation for production vs test gate). | Phase 1 & 2 |
| **D4** | **Unknown-Space Policy (`P04`):** Should `swept_collision()` always treat unknown occupancy cells as blocked, or allow a per-scenario override for exploration tests? | **Blocked by default**, with explicit `allow_unknown: true` opt-in per scenario. | Phase 1 |
| **D5** | **Legacy Root Cleanup Scope:** Remove duplicate `testing/LunchWorld&Rover` / `testing/LunchWorld&Rover.sh` (keeping one fixed `launch_world_and_rover.sh`) and archive/remove stale files under `testing/benchmarking/pathplanning/reports/`? | **Remove duplicates & clear generated `reports/` from source** (keep `PathPlanning/VALIDATION.md` as historical record). | Phase 0 |
| **D6** | **Controller Diagnostic Score Weights:** Align `report_generator.py` (`0.30/0.25/0.20/0.15/0.10`) to `benchmark_config.yaml` (`cte: 0.35, smoothness: 0.25, control_effort: 0.20, heading_accuracy: 0.10, goal_accuracy: 0.10`)? | **Read dynamically from `benchmark_config.yaml`** with those values as defaults. | Phase 1 |

---

## 5. Phase Progress Tracker

| Phase | Scope | Status | Key Deliverables | Sign-Off |
| :--- | :--- | :--- | :--- | :--- |
| **Phase 0** | Criteria sign-off, v2.0.0 schema, `T01`/`T04`/`T07` hygiene | Implemented (static) | `schema.py`, `test_schema_v2.py`, clean `package.xml`, safe PGID `launch_world_and_rover.sh` | Pending Jazzy runtime |
| **Phase 1** | PathPlanner & Controller benchmarker (`P02`–`P08`) | Implemented (static) | `scenario_benchmark_runner.py`, v2.0.0 reports, `NavigateThroughPoses`, `benchmark.launch.py`, `test_scenario_runner_contract.py` | Pending Jazzy runtime |
| **Phase 2** | Shared dependency mocks & fault injection | Implemented (static) | Rover presets + freeze/drop faults, `clear_aruco_nav` + `-0.255 m` floor + GT JSON, SLAM slip/mount/tick modes, `S08` stamps/cov | Pending Jazzy runtime |
| **Phase 3** | SLAM & state estimation benchmarker | Planned | Packaged `gazebo_truth_bridge`, ATE/RPE/slip-F1/loop/map-sanity evaluator, §9 checks | Pending |
| **Phase 4** | Live Perception accuracy & runtime benchmarker | Planned | Hungarian + 3D IoU + ground IoU + ArUco pose accuracy + TF-aware bag replay | Pending |
| **Phase 5** | Shared `validation_bringup` & Mission Referee | Planned | `M1`–`M4` launch profiles, scoped supervisor, cross-module fault attribution | Pending |
| **Phase 6** | Run diffing, headless Docker CI & GUI contract | Planned | `compare_runs.py`, CI script, GUI API contract | Pending |
