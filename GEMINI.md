# Autonomous-27: Antigravity Workspace Directive & Architecture Map

ROS 2 (Jazzy/Humble) autonomous rover system for simulation, state estimation, perception, path planning, and motor control in the Mars Yard environment.

---

## 1. Antigravity Agent Directives & Guardrails

### Fast Navigation & Exclusion Rules (DO NOT SCAN)
- **NEVER recursively search or index `worlds/`**: Contains massive Gazebo simulation worlds, 3D rock meshes (`.dae`, `.obj`, `.stl`), heightmaps, and textures.
- **NEVER search build/install artifacts**: Always exclude `build/`, `install/`, `log/`, and `.git/`.
- **Target searches by subsystem**: Confine queries directly to the target package directory (`SLAM/rover_slam`, `Perception/marker_detection`, `PathPlanning/erc_path_planner`, `Rover/my_robot_description`).

### Build & Execution Standards
- **Symlink Builds**: Always build with symlinks to avoid re-compiling Python scripts:
  ```bash
  colcon build --symlink-install --packages-select <pkg_name>
  ```
- **Runtime Environment**:
  - Primary: Inside Docker container (`docker/run_docker.sh` -> workspace mounted at `/workspace`).
  - Host alternative: Sourcing `/opt/ros/jazzy/setup.bash` or `/opt/ros/humble/setup.bash` + `source install/setup.bash`.
- **Surgical Code Edits**:
  - Touch only targeted node logic or config parameters.
  - Never modify auto-generated URDF meshes or SDF collision geometries unless explicitly requested.
  - Avoid reformatting adjacent launch scripts or test stubs.

---

## 2. Subsystem Directory & Package Map

| Directory | ROS 2 Package | Primary Responsibility | Key Files & Nodes |
| :--- | :--- | :--- | :--- |
| [`Rover/my_robot_description`](file:///e:/SHAKR/Autonmous-27/Rover/my_robot_description) | `my_robot_description` | Robot kinematics, URDF/Xacro models, sensor frames, Gazebo spawn | `urdf/my_robot.urdf.xacro`<br>`launch/gazebo.launch.py`<br>`launch/spawn_rover.launch.py`<br>`scripts/teleop_gui.py` |
| [`SLAM/rover_slam`](file:///e:/SHAKR/Autonmous-27/SLAM/rover_slam) | `rover_slam` | State estimation, EKF fusion, RGB-D RTAB-Map SLAM, wheel slip detection | `rover_slam/heuristic_slip_checker.py`<br>`rover_slam/encoder_ticks_to_odom.py`<br>`config/ekf.yaml`<br>`config/rtabmap.yaml`<br>`launch/slam_bringup.launch.py` |
| [`Perception/marker_detection`](file:///e:/SHAKR/Autonmous-27/Perception/marker_detection) | `marker_detection` | ArUco detection, 6-DoF pose estimation, marker mapping, 3D obstacle tracking | `marker_detection/aruco_detector.py`<br>`marker_detection/depth_processor.py`<br>`marker_detection/marker_action_interface_node.py`<br>`launch/marker_detection.launch.py` |
| [`PathPlanning/erc_path_planner`](file:///e:/SHAKR/Autonmous-27/PathPlanning/erc_path_planner) | `erc_path_planner` | Nav2 integration, Smac Hybrid A* global planner, MPPI controller, costmap bridges | `config/nav2_params.yaml`<br>`src/costmap_bridge_node.cpp`<br>`behavior_trees/`<br>`launch/path_planning.launch.py` |
| [`Control`](file:///e:/SHAKR/Autonmous-27/Control) | N/A (Docs/Specs) | Skid-steer motor kinematics, hardware actuator interface specs | `Control/docs/` |
| [`testing`](file:///e:/SHAKR/Autonmous-27/testing) | Test Harnesses | Benchmarks, synthetic maps, launch orchestration | `launch_world_and_rover.sh`<br>`map_vs_odom_tf_explained.md`<br>`PathPlanner/` |
| [`docker`](file:///e:/SHAKR/Autonmous-27/docker) | Docker Compose | Pre-configured container (ROS 2, Nav2, RTAB-Map, ros_gz, RViz) | `docker-compose.yml`<br>`run_docker.sh`<br>`enter_docker.sh` |
| [`worlds`](file:///e:/SHAKR/Autonmous-27/worlds) | Simulation Assets | Mars Yard simulation worlds and models (**DO NOT PARSE/SEARCH**) | `world1.world`, `marsyard.world` |

---

## 3. Coordinate Frames & TF Tree (REP-105)

Standard transform chain:
```
map (Global World / RTAB-Map SLAM)
 └── odom (Smooth Local Odometry / robot_localization EKF)
      └── base_link (Rover Kinematic Center)
           ├── chassis
           ├── camera_link
           │    ├── camera_depth_frame -> camera_depth_optical_frame
           │    └── my_robot/camera_link/camera (Gazebo sensor bridge)
           └── imu_link
```

### Critical TF Rules & Gotchas:
1. **Never Duplicate Publishers**:
   - `odom ➔ base_link` MUST only be published by `robot_localization` EKF (`rover_slam/launch/ekf.launch.py`).
   - Do NOT allow Gazebo diff-drive plugin or raw wheel odom to publish the `odom ➔ base_link` TF directly when EKF is active.
2. **`map ➔ odom` Ownership**:
   - Dynamic `map ➔ odom` is published exclusively by RTAB-Map (`rover_slam/launch/rtabmap.launch.py`).
   - In `spawn_rover.launch.py` and `gazebo.launch.py`, `publish_map_tf:=false` by default to prevent dual-publisher conflicts. Only set `publish_map_tf:=true` for standalone teleop without SLAM.
3. **Camera Optical Frames**:
   - ROS image processing algorithms expect optical coordinates ($Z$ forward, $X$ right, $Y$ down).
   - Sensor mounting frames use standard body coordinates ($X$ forward, $Y$ left, $Z$ up).
   - `static_transforms.launch.py` owns the static bridge from `camera_depth_optical_frame ➔ my_robot/camera_link/camera`. `spawn_rover.launch.py` sets `publish_camera_tf:=false` by default to prevent duplicate parents.

---

## 4. Key Topic Contracts

| Topic | Type | Source Node | Consumer Nodes |
| :--- | :--- | :--- | :--- |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | Gazebo / RealSense | `Perception/depth_processor`, RTAB-Map |
| `/camera/color/image_raw` | `sensor_msgs/Image` | Gazebo / RealSense | `Perception/aruco_detector`, RTAB-Map |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo / BNO055 | `rover_slam/heuristic_slip_checker`, EKF |
| `/wheel/odom_raw` | `nav_msgs/Odometry` | Wheel Encoders | `rover_slam/heuristic_slip_checker`, EKF |
| `/odometry/filtered` | `nav_msgs/Odometry` | `robot_localization` EKF | RTAB-Map, Nav2 Controller, Nav2 Planner |
| `/map` | `nav_msgs/OccupancyGrid` | RTAB-Map | Nav2 Costmap Server |
| `/perception/obstacles_only` | `vision_msgs/Detection3DArray` | Perception Clustering | Nav2 Local Costmap |
| `/plan` | `nav_msgs/Path` | Nav2 Smac Planner | Nav2 MPPI Controller |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 MPPI / Teleop | Motor bridge / Gazebo plugin |

---

## 5. Primary Launch & Execution Playbook

### Clean Restart / Kill Zombie Processes
If Gazebo, RViz, or bridges freeze or hold ports:
```bash
pkill -9 -f gazebo; pkill -9 -f ign; pkill -9 -f gz; pkill -9 -f ros; pkill -9 -f rviz; killall -9 -q ruby gz server rviz2 parameter_bridge ros2 robot_state_publisher
```

### Build Command
```bash
# Clean build (single package)
colcon build --symlink-install --packages-select rover_slam

# Clean build (all packages)
colcon build --symlink-install
source install/setup.bash
```

### Launch Sequence
1. **Spawn Rover + Mars Yard Simulation**:
   ```bash
   ros2 launch my_robot_description gazebo.launch.py
   ```
2. **State Estimation & SLAM**:
   ```bash
   ros2 launch rover_slam slam_bringup.launch.py
   ```
3. **Perception (ArUco + Obstacle Clustering)**:
   ```bash
   ros2 launch marker_detection marker_detection.launch.py
   ```
4. **Nav2 Path Planning & Controller**:
   ```bash
   ros2 launch erc_path_planner path_planning.launch.py
   ```
5. **Teleoperation GUI (Manual Driving Verification)**:
   ```bash
   ros2 run my_robot_description teleop_gui.py
   ```

---

## 6. Diagnostics & Verification Cheatsheet

When verifying system state or debugging failures:

```bash
# Check TF Tree connectivity
ros2 run tf2_tools view_frames
# Inspect specific transform
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom

# Verify topic publish rates
ros2 topic hz /odometry/filtered
ros2 topic hz /camera/depth/color/points
ros2 topic hz /cmd_vel

# Inspect active node graph
ros2 node list
ros2 topic list
```
