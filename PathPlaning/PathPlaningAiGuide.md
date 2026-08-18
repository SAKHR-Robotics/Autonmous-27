# Path Planning Module: AI Implementation Guide

This document serves as the master guide and checklist for AI agents working on the Path Planning module for the ERC 2026 Rover. It includes the architectural context, the target folder structure, and a highly detailed breakdown of implementation tasks, specifically designed for **two developers working in parallel**.

*Note to AI: Update this document (using file editing tools) whenever a task is completed or the folder structure changes.*

---

## 1. The Big Story: Architecture
This implementation covers the full integration of both the **Smac Hybrid A* Global Planner** and the **MPPI Local Planner & Controller** via ROS 2 Nav2 plugins (`nav2_smac_planner` and `nav2_mppi_controller`).

**System Data Flow:**
*   **Inputs:** `/goal_pose` (Target), `/map` (Global static map), `/perception/obstacles` (Real-time dynamic obstacles), `/odometry/filtered` (High-speed 100 Hz EKF odometry), and TF transforms (`map -> odom -> base_link`).
*   **Outputs:** `/plan` (The route) and `/cmd_vel` (Motor commands).

---

## 2. Target Folder Structure
This is the directory structure that has been created for the implementation phase. 

```text
PathPlaning/
├── README.md
├── PathPlaningAiGuide.md
├── PathPlanner_Docu/
│   └── PathPlannerGuide.html
└── src/
    └── erc_path_planner/
        ├── package.xml                   # TO BE CREATED
        ├── CMakeLists.txt                # TO BE CREATED
        ├── config/
        │   └── nav2_params.yaml          # Core Smac, MPPI & Costmap configurations
        ├── launch/
        │   ├── path_planning.launch.py   # Main Nav2 bringup launcher
        │   └── rviz.launch.py            # Visualization launcher
        ├── rviz/
        │   └── nav2_default_view.rviz    # Saved RViz configuration
        ├── behavior_trees/
        │   └── custom_navigate.xml       # Custom Nav2 behavior tree
        └── src/
            └── costmap_bridge_node.cpp   # Converts perception topics to PointCloud2
```

---

## 3. Granular Implementation To-Do List (For 2 Developers)

### 🧑‍💻 Person A (Track A - Core Planners & Routing)

**Foundation & Package Setup:**
- [ ] **Task A-1.1:** Create the `erc_path_planner` package structure (directories: `config`, `launch`, `rviz`, `behavior_trees`, `src`). ( done )
- [ ] **Task A-1.2:** Write `package.xml` including all Nav2 dependencies (`nav2_bringup`, `nav2_smac_planner`, `nav2_mppi_controller`, `nav2_costmap_2d`, `nav2_bt_navigator`).(done)
- [ ] **Task A-2.1:** Write `CMakeLists.txt` to install `config`, `launch`, `rviz`, and `behavior_trees` directories.(done)
- [ ] **Check A-2.2 (Validation):** Run `colcon build --packages-select erc_path_planner`. Ensure 0 errors. (done)

**Planners Configuration:**
- [ ] **Task A-3.1:** Create `config/nav2_params.yaml`. (done)
- [ ] **Task A-3.2:** Configure `planner_server` inside `nav2_params.yaml` to use `SmacPlannerHybrid`.(done)
- [ ] **Task A-3.3:** Set Smac parameters (`motion_model_for_search: "REEDS_SHEPP"`, `minimum_turning_radius: 0.8`, `allow_unknown: true`).(done)
- [ ] **Task A-4.1:** Configure `controller_server` to use `MPPIController`.
- [ ] **Task A-4.2:** Set MPPI parameters (`batch_size: 2000`, `time_steps: 56`, `model_dt: 0.05`, limits, and critics).(done)
- [ ] **Check A-4.3 (Validation):** Run `yamllint config/nav2_params.yaml` to ensure no syntax errors.(done)

**Bringup & Launch:**
- [ ] **Task A-5.1:** Setup `bt_navigator` parameters in YAML to use standard navigation tree.
- [ ] **Task A-5.2:** Create `launch/path_planning.launch.py`.
- [ ] **Task A-6.1:** Include `nav2_bringup` in the launch file and pass the custom `nav2_params.yaml`.
- [ ] **Check A-6.2 (Validation):** Run `ros2 run nav2_util lifecycle_bringup` and `ros2 launch erc_path_planner path_planning.launch.py`. Verify active state.

---

### 🧑‍💻 Person B (Track B - Costmaps, Vision & Perception)

**Costmap Foundation:**
- [ ] **Task B-1.1:** Configure `global_costmap` in `nav2_params.yaml` to subscribe to `/map`.(done)
- [ ] **Task B-1.2:** Configure `local_costmap` to use a rolling window.(done)
- [ ] **Task B-2.1:** Add `obstacle_layer` to the `local_costmap`.(done)
- [ ] **Task B-2.2:** Add `inflation_layer` to both costmaps with rover-specific footprint/radius.(done)

**Perception Bridge Node:**
- [ ] **Task B-3.1:** Write the ROS 2 C++ boilerplate for `src/costmap_bridge_node.cpp`.(done)
- [ ] **Task B-3.2:** Add subscriber to `terrain_geometry_msgs/ObstacleFeatureArray`.(done)
- [ ] **Task B-4.1:** Add publisher for standard `sensor_msgs/PointCloud2`.(done)
- [ ] **Task B-4.2:** Implement the custom conversion logic inside the node's callback.(done)
- [ ] **Task B-5.1:** Update `CMakeLists.txt` to compile `costmap_bridge_node.cpp`.(done)
- [ ] **Check B-5.2 (Validation):** Build `costmap_bridge_node`.(done)
- [ ] **Check B-5.3 (Validation):** Publish dummy `ObstacleFeatureArray` via CLI, verify `PointCloud2` output.(done)

**Visualization:**
- [ ] **Task B-6.1:** Create `launch/rviz.launch.py`.(done)
- [ ] **Task B-6.2:** Configure RViz2 displays for Map, Costmaps, Plan, and Trajectories, and save to `rviz/nav2_default_view.rviz`. ( done )(done)
- [ ] **Check B-7.1 (Validation):** Launch `rviz.launch.py`, publish dummy `/map`, verify RViz loads properly.

---

### 🤝 Phase 4: Final Integration (Person A + Person B)

### 🤝 Phase 4: Progressive Integration & Main Branch Checkpoints

Because each task is merged to the `main` branch upon completion, use these 4 progressive integration checkpoints to validate the state of the `main` branch.

#### 🏁 Integration Checkpoint 1: Package Registration & Base Build
*Validate immediately after Task A-1 and Task A-2 are merged to main.*
- [ ] **Task INT-1.1:** Compile the workspace on the `main` branch.
- [ ] **Task INT-1.2:** Verify ROS 2 package registration.
- [ ] **Check INT-1.3 (Validation):**
  1. Run `colcon build --packages-select erc_path_planner`.
  2. Run `source install/setup.bash`.
  3. Run `ros2 pkg prefix erc_path_planner`.
  *Success Criteria:* Package builds successfully and returns the installation path.

#### 🏁 Integration Checkpoint 2: Parameter Server & Lifecycle Server Boot
*Validate after Track A planners (A-3 to A-6) are merged to main.*
- [ ] **Task INT-2.1:** Verify that all lifecycle nodes spin up and load parameters from YAML.
- [ ] **Check INT-2.2 (Validation):**
  1. Run `ros2 run nav2_util lifecycle_bringup`.
  2. Run `ros2 launch erc_path_planner path_planning.launch.py`.
  *Success Criteria:* `planner_server` and `controller_server` transition to the `active` state in the terminal without crashing due to YAML format errors.

#### 🏁 Integration Checkpoint 3: Costmaps & Perception Bridge Live Processing
*Validate after Track B costmaps & bridge node (B-1 to B-5) are merged to main.*
- [ ] **Task INT-3.1:** Run the costmap bridge node alongside the main Nav2 launcher.
- [ ] **Check INT-3.2 (Validation):**
  1. Run `ros2 run erc_path_planner costmap_bridge_node`.
  2. Publish a dummy message: `ros2 topic pub /perception/obstacles terrain_geometry_msgs/msg/ObstacleFeatureArray "{...}" -1`.
  3. Run `ros2 topic echo /bridge/pointcloud`.
  *Success Criteria:* The bridge node receives the custom message and publishes a standard PointCloud2, which is consumed by the costmap server.

#### 🏁 Integration Checkpoint 4: Complete Closed-Loop Navigation Loop
*Validate after Track B visualization (B-6, B-7) are merged, representing full project completion.*
- [ ] **Task INT-4.1:** Verify end-to-end routing and motor velocity commands.
- [ ] **Check INT-4.2 (Validation):**
  
  **Method 1: Using the Built-in Dummy Map (Recommended)**
  1. Build the updated package:
     `colcon build --packages-select erc_path_planner`
     `source install/setup.bash`
  2. Launch the path planner (which now automatically loads `config/dummy_map.yaml`):
     `ros2 launch erc_path_planner path_planning.launch.py`
  3. In a separate terminal, launch RViz:
     `ros2 launch erc_path_planner rviz.launch.py`
  4. Publish static localization transforms to mock Odometry and SLAM frames:
     `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom`
     `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link`
  5. In RViz, click **"2D Pose Estimate"** to set the starting position of the rover, then click **"Nav2 Goal"** to set a destination.
  *Success Criteria:* A green `/plan` path is drawn in RViz and `/cmd_vel` outputs valid velocity commands in response.

  **Method 2: Bypassing map_server entirely & manual publishing (Optional)**
  1. Launch the system with SLAM enabled to disable the built-in map_server:
     `ros2 launch erc_path_planner path_planning.launch.py slam:=True`
  2. Publish a dummy OccupancyGrid map manually via terminal:
     `ros2 topic pub /map nav_msgs/msg/OccupancyGrid "{header: {frame_id: 'map'}, info: {resolution: 0.1, width: 10, height: 10, origin: {position: {x: -5.0, y: -5.0, z: 0.0}}}, data: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"`
  3. Publish the static transforms and send goals in RViz as in Method 1.

---

## 4. Maintenance Notes
*   **Collaboration Rule:** Person A owns Planners and Launch files. Person B owns Costmaps and C++ Nodes.
*   **Completion Rule:** When a task is completed, do NOT check the box (leave it as `[ ]`). Instead, write `( done )` at the end of the task text.
*   If a task is blocked, document the blocker below the task using blockquotes (`> Blocker: ...`).
