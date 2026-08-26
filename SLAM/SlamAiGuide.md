# 🧠 SLAM Module AI Implementation & Architecture Guide (`SlamAiGuide.md`)

> 💡 **AI Assistant Directive:** This document is the primary reference and roadmap for AI assistants and human engineers developing the SLAM Subsystem (`rover_slam`). When completing a task or editing files, you **MUST update the task checkboxes (`- [✓]`) and validation milestone checkpoints** below.

---

## 📋 Metadata & Status Log
* **Subsystem:** Autonomous SLAM & State Estimation Module
* **Package Name:** `rover_slam`
* **Target Compute:** NVIDIA Jetson Orin Nano (8GB CUDA GPU)
* **Primary Sensors:** Intel RealSense D435 RGB-D Camera, Bosch BNO055 IMU, Wheel Encoders
* **ROS 2 Middleware:** ROS 2 Jazzy / Humble
* **Target Workspace:** `/home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws/SLAM/rover_slam`
* **Architecture:** Option B — Loosely-Coupled `robot_localization` EKF (100 Hz) + `RTAB-Map` Visual SLAM (1-5 Hz) + `nav2_costmap_2d` Server + Heuristic Slip Pre-Filter.
* **Current Status:** Package blueprint created; parallel task execution ready for Developer 1 and Developer 2.

---

## 🤖 1. System Prompt for AI Coding Assistants

Copy-paste the block below into your AI assistant session to prime it with full context:

```text
You are an expert ROS 2 Autonomous Systems Engineer assisting in implementing the `rover_slam` package for an ERC (European Rover Challenge) Mars Rover.

SYSTEM ARCHITECTURE SUMMARY (Option B: EKF + RTAB-Map + Pre-Filtering + Costmap 2D):
1. Hardware: Intel RealSense D435 (RGB-D), Bosch BNO055 (IMU), Wheel Encoders (Velocity), NVIDIA Jetson Orin Nano (Compute).
2. Local State Estimator: `robot_localization` (EKF node) fusing wheel ticks & IMU data. Publishes `/odometry/filtered` and `odom -> base_link` TF at 100 Hz.
3. Pre-Filtering & Slip Check: Custom heuristic slip checker node comparing wheel speed vs IMU angular/linear rate, dynamically publishing increased covariance to `/wheel/odom_raw` during wheel slip. RealSense depth filters enabled (Decimation, Spatial, Temporal, Hole-filling, Max range 4.0m).
4. Global Mapping Backend: `rtabmap_ros` node subscribing to `/odometry/filtered` and filtered depth images. Publishes static `/map` grid and `map -> odom` transform link at 1-5 Hz. Receives 6-DOF ArUco landmark poses (`/perception/aruco_pose`) as global graph constraints.
5. Navigation Costmap: `nav2_costmap_2d` server fusing static `/map` and dynamic obstacles (`/perception/obstacles_only`) into an inflated `/global_costmap/costmap` for Smac Hybrid A* and MPPI local planner.

DEVELOPMENT CONSTRAINTS:
- ROS 2 Distribution: Jazzy / Humble.
- Work Package Name: `rover_slam` located at `Autonmous_Ws/SLAM/rover_slam/`.
- Person 1 and Person 2 are working on separate, decoupled tracks.
- Always use precise ROS 2 message types (`nav_msgs/msg/Odometry`, `sensor_msgs/msg/Imu`, `sensor_msgs/msg/Image`, `geometry_msgs/msg/PoseStamped`).
- Maintain and update the task checklist and validation checkpoints in `SlamAiGuide.md` and `SlamAIWorkingGuide.html` upon completing any task.
```

---

## 📖 2. System Architecture & Context

### 2.1 The Coordinate Transform Hierarchy (TF Tree)
The system enforces a strict single-parent transform hierarchy:
$$\text{map} \xrightarrow{\text{RTAB-Map offset (1-5 Hz)}} \text{odom} \xrightarrow{\text{EKF local pose (100 Hz)}} \text{base\_link} \xrightarrow{\text{Static TF}} \text{camera\_link / imu\_link}$$

*   **`odom -> base_link` (Local Link):** High-frequency (100 Hz), smooth, zero coordinate jumps, outputted by `robot_localization` EKF. Used directly by local MPPI controller.
*   **`map -> odom` (Global Link):** Low-frequency (1–5 Hz), outputted by RTAB-Map. Absorbs global drift corrections and loop closures without jerking local wheel controllers.

### 2.2 Topic Data Pipeline Contract
| Topic Name | ROS 2 Message Type | Publisher Node | Subscriber Node(s) |
| :--- | :--- | :--- | :--- |
| `/wheel/ticks` / `/wheel/odom_raw` | `nav_msgs/msg/Odometry` | `encoder_ticks_to_odom.py` | `heuristic_slip_checker.py` / EKF |
| `/imu/data` | `sensor_msgs/msg/Imu` | BNO055 IMU Driver | `heuristic_slip_checker.py` / EKF |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | RealSense Driver | RTAB-Map / `aruco_detector_node.py` |
| `/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` | RealSense Driver | Depth Post-Processing Filter |
| `/camera/depth/filtered` | `sensor_msgs/msg/Image` | Depth Filter Node | RTAB-Map SLAM Node |
| `/perception/obstacles_only` | `sensor_msgs/msg/PointCloud2` | Perception Terrain Node | Nav2 Costmap 2D Server |
| `/perception/aruco_pose` | `geometry_msgs/msg/PoseStamped` | `aruco_detector_node.py` | RTAB-Map SLAM Node |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF Node (`robot_localization`) | RTAB-Map / Local Planner (MPPI) |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | Nav2 Costmap Server | Global Planner (Smac Hybrid A*) |

---

## 📂 3. Full Final Package Folder Structure (`rover_slam`)

```text
Autonmous_Ws/SLAM/
├── SlamAiGuide.md                  # [THIS FILE] Markdown AI Roadmap & Checkpoints
├── Slam_Docu/
│   ├── SlamAIWorkingGuide.html      # Interactive Glassmorphism AI Guide & Checklists
│   ├── saif SLAM.html               # Architecture & Theory Reference Report
│   └── slam_guide.html              # FIS + IESKF Study Reference Report
│
└── rover_slam/                      # Primary ROS 2 SLAM Package Root
    ├── CMakeLists.txt / setup.py    # Package build configuration
    ├── package.xml                   # ROS 2 dependencies
    ├── config/
    │   ├── ekf.yaml                  # robot_localization EKF config (Person 1)
    │   ├── rtabmap.yaml              # RTAB-Map parameters & loop closure settings (Person 1)
    │   ├── costmap_params.yaml       # Nav2 Costmap 2D layer & inflation settings (Person 2)
    │   ├── realsense_filters.yaml   # D435 depth filter parameters (Person 2)
    │   └── slam_visualization.rviz  # RViz 2 visualization dashboard config (Person 1)
    ├── launch/
    │   ├── slam_bringup.launch.py   # Master system launch file (Joint)
    │   ├── ekf.launch.py            # Local EKF & Slip Checker launch (Person 1)
    │   ├── rtabmap.launch.py        # RTAB-Map SLAM launch (Person 1)
    │   ├── static_transforms.launch.py # Static TF publishers for camera & sensors (Person 2)
    │   ├── vision_helper.launch.py  # Camera filters & ArUco detector launch (Person 2)
    │   └── costmap.launch.py        # Nav2 Costmap 2D server launch (Person 2)
    ├── rover_slam/                  # Python Node Package Directory
    │   ├── __init__.py
    │   ├── encoder_ticks_to_odom.py  # Node: Converts raw wheel ticks to nav_msgs/Odometry Twist (Person 1)
    │   ├── heuristic_slip_checker.py # Node: Compares wheels vs IMU & updates covariance (Person 1)
    │   └── costmap_test_stub.py      # Test Node: Simulates perception rock inputs for testing (Person 2)
    ├── test/
    │   └── test_slip_checker.py      # Unit test suite for slip pre-filter (Person 1)
    └── README.md
```

---

## 🎯 4. Milestone Validation Checkpoints (When & What to Test)

Use these explicit checkpoints to verify each milestone before merging code:

### 🚩 Checkpoint 1: Static Transforms & Package Infrastructure (After 1A & 2B.1)
* **Trigger:** Completed Task 1A (Package setup) and Task 2B.1 (`static_transforms.launch.py`).
* **Command to Run:**
  ```bash
  ros2 launch rover_slam static_transforms.launch.py
  ros2 run tf2_ros tf2_echo base_link camera_link
  ros2 run tf2_ros tf2_echo base_link imu_link
  ```
* **Expected Output:** TFs `base_link -> camera_link` and `base_link -> imu_link` are published continuously at 10 Hz with 0 transform errors.
* **Status:** `[ ] Pending`

### 🚩 Checkpoint 2: Sensor Pre-Processing & Depth Filtering (After 2A.1 & 1B)
* **Trigger:** Completed Task 2A.1 (`encoder_ticks_to_odom.py`) and Task 1B (`vision_helper.launch.py`).
* **Command to Run:**
  ```bash
  ros2 launch rover_slam vision_helper.launch.py
  ros2 topic hz /camera/depth/filtered
  ros2 topic echo /wheel/odom_raw
  ```
* **Expected Output:** `/wheel/odom_raw` outputs linear/angular velocity Twist at 50+ Hz. Depth image feed `/camera/depth/filtered` publishes clean depth images with decimation and 4m max-range clipping applied.
* **Status:** `[ ] Pending`

### 🚩 Checkpoint 3: Local EKF & Heuristic Slip Checker (After 2A.2, 2A.3, 3A & 2B.2)
* **Trigger:** Completed EKF setup (`ekf.launch.py`), slip checker (`heuristic_slip_checker.py`), and mock ArUco publisher.
* **Command to Run:**
  ```bash
  ros2 launch rover_slam ekf.launch.py
  ros2 topic pub /perception/aruco_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'camera_link'}, pose: {position: {x: 1.0}, orientation: {w: 1.0}}}" -1
  ros2 topic echo /odometry/filtered
  ```
* **Expected Output:**
  1. `/odometry/filtered` and `odom -> base_link` TF are published smoothly at 100 Hz.
  2. When simulated wheel spin occurs, `heuristic_slip_checker.py` increases covariance on `/wheel/odom_raw`.
  3. Verify receipt of the mock 6-DOF pose on `/perception/aruco_pose` without needing the real Perception module.
* **Status:** `[ ] Pending`

### 🚩 Checkpoint 4: RTAB-Map SLAM & Costmap 2D Server (After 4A, 5A, 4B & 5B)
* **Trigger:** Completed RTAB-Map setup (`rtabmap.launch.py`) and Costmap server (`costmap.launch.py` with `costmap_test_stub.py`).
* **Command to Run:**
  ```bash
  ros2 launch rover_slam rtabmap.launch.py
  ros2 launch rover_slam costmap.launch.py
  ros2 run tf2_ros tf2_echo map odom
  ros2 topic echo /global_costmap/costmap
  ```
* **Expected Output:** RTAB-Map publishes `map -> odom` TF frame and static `/map` grid. Costmap 2D server fuses `/map` + test obstacle point cloud and outputs inflated `/global_costmap/costmap`.
* **Status:** `[ ] Pending`

### 🚩 Checkpoint 5: Master System Bringup & Rosbag Benchmark (After 6.1, 6.2 & 7.1)
* **Trigger:** Completed Master bringup (`slam_bringup.launch.py`).
* **Command to Run:**
  ```bash
  ros2 launch rover_slam slam_bringup.launch.py
  ros2 run rviz2 -d src/rover_slam/config/slam_visualization.rviz
  ```
* **Expected Output:** Full TF tree (`map -> odom -> base_link -> camera_link`) is clean with 0 warnings. Robot trajectory renders smoothly at 100 Hz. Rocks appear correctly inflated on `/global_costmap/costmap`.
* **Status:** `[ ] Pending`

---

## 📝 5. Parallel Implementation Task Checklist

### ⚙️ Track A: Developer 1 (State Estimation & SLAM Backend)
- [✓] **Task 1A.1:** Initialize `rover_slam` ROS 2 package in `Autonmous_Ws/SLAM/rover_slam/`. (Package & setup ready)
- [✓] **Task 1A.2:** Create workspace subdirectories (`config/`, `launch/`, `rover_slam/`, `test/`).
- [ ] **Task 2A.1:** Implement `rover_slam/encoder_ticks_to_odom.py` (Differential drive kinematic math converting wheel ticks to `nav_msgs/msg/Odometry` velocity).
- [✓] **Task 2A.2:** Create `config/ekf.yaml` for `robot_localization` (`world_frame: odom`, `frequency: 100`, fuse wheel velocity & IMU yaw rate).
- [✓] **Task 2A.3:** Create `launch/ekf.launch.py` to start `ekf_node`.
- [ ] **Task 3A.1:** Develop `rover_slam/heuristic_slip_checker.py` (calculates speed difference $|V_{wheels} - V_{imu}| > 0.15\text{ m/s}$).
- [ ] **Task 3A.2:** Add dynamic covariance scaling to `heuristic_slip_checker.py` during wheel slip.
- [ ] **Task 3A.3:** Create `test/test_slip_checker.py` unit test suite.
- [✓] **Task 4A.1:** Create `config/rtabmap.yaml` (loop closure thresholds, memory management, GTSAM optimizer).
- [✓] **Task 4A.2:** Create `launch/rtabmap.launch.py` (`publish_tf: true`, subscribe to `/odometry/filtered` and filtered depth).
- [✓] **Task 5A.1:** Connect `/perception/aruco_pose` landmark topic into RTAB-Map's landmark channel in launch file.
- [✓] **Task 7A.1:** Create `config/slam_visualization.rviz` displaying TF tree, trajectory path, point clouds, and `/map`.

---

### 👁️ Track B: Developer 2 (Sensors Pre-Processing, Vision & Costmap)
- [✓] **Task 1B.1:** Create `config/realsense_filters.yaml` (Decimation, Spatial, Temporal, Hole-filling, 4.0m max depth).
- [✓] **Task 1B.2:** Create `launch/vision_helper.launch.py` launching `realsense2_camera` driver with filters enabled.
- [✓] **Task 2B.1:** Create `launch/static_transforms.launch.py` broadcasting `base_link -> camera_link` and `base_link -> imu_link`.
- [✓] **Task 3B.1:** Develop ArUco marker detector node (`aruco_detector_node.py`) & mock test publisher (`mock_aruco_publisher.py`) for `/perception/aruco_pose` landmark verification.
- [✓] **Task 4B.1:** Create `config/costmap_params.yaml` for `nav2_costmap_2d` (Static Layer, Obstacle Layer, Inflation Layer).
- [✓] **Task 4B.2:** Create `launch/costmap.launch.py` to start `nav2_costmap_2d` lifecycle nodes.
- [✓] **Task 5B.1:** Develop `rover_slam/costmap_test_stub.py` to publish synthetic obstacle point clouds and verify `/global_costmap/costmap` inflation output.

---

### 🤝 Track C: Joint Integration & System Verification (Developers 1 & 2 Together)
- [✓] **Task 6.1:** Create `launch/slam_bringup.launch.py` combining all sub-launch files.
- [✓] **Task 6.2:** Configure Nav2 lifecycle manager to automatically transition `costmap` and `rtabmap` nodes to `active`.
- [✓] **Task 7.1:** Execute end-to-end rosbag / Gazebo benchmark verification (verify 100 Hz EKF, slip covariance scaling, `map -> odom` loop closures, costmap inflation).
- [✓] **Task 7.2:** Update `README.md` and check off all completed tasks in `SlamAiGuide.md`.

---

## 🛠️ 6. AI Assistant Protocol for Document Maintenance
When an AI assistant completes any coding task:
1. Update the corresponding checkbox in `SlamAiGuide.md` to `- [✓]`.
2. Update the status in `SlamAIWorkingGuide.html` by setting `checked` attribute on the corresponding checkbox.
3. If new files are created, update Section 3 (Package Folder Structure) in both documents.
