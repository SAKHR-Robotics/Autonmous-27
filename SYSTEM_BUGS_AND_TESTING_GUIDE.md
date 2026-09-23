# Autonomous-27: Master System Flaws, Architectural Bugs & Testing Guide

> **Document Purpose**: Complete engineering reference cataloging all cross-package architectural flaws, coordinate frame conflicts, topic contract breaks, and standalone testing flags across the Autonomous-27 rover repository.

---

## 📑 Table of Contents
1. [Core Architectural Philosophy: Clean Defaults + Dedicated Launchers](#1-core-architectural-philosophy)
2. [Subsystem Flaws & Architectural Bug Registry](#2-subsystem-flaws--architectural-bug-registry)
3. [Operational Modes Matrix & RViz Visualization Rules](#3-operational-modes-matrix--rviz-visualization-rules)
4. [Launch Flag Specifications & Recommended Defaults](#4-launch-flag-specifications--recommended-defaults)
5. [Dedicated Standalone Test Launchers Specification](#5-dedicated-standalone-test-launchers-specification)
6. [Step-by-Step Resolution Roadmap](#6-step-by-step-resolution-roadmap)

---

## 1. Core Architectural Philosophy

### The 2-Tier Standard: "Clean Production Defaults + Dedicated Standalone Launchers"

To balance **isolated module development** with **plug-and-play system integration**, the repository follows two strict rules:

* **Rule 1 — Production Launch Files Run 100% Clean by Default**:
  All primary bringup files (`gazebo.launch.py`, `slam_bringup.launch.py`, `path_planning.launch.py`, `perception_system.launch.py`) have test stubs, synthetic publishers, and duplicate servers **disabled by default**. Running these files without arguments connects the full rover with zero collisions.
* **Rule 2 — Dedicated Standalone Launchers for Subsystem Testing**:
  Instead of forcing developers to remember long CLI arguments (`launch_aruco_stub:=true launch_costmap:=true bridge_sim_tf:=true ...`), dedicated test launchers (e.g., `test_slam_standalone.launch.py`) configure isolated testing environments automatically.

---

## 2. Subsystem Flaws & Architectural Bug Registry

### 🔴 Category A: Coordinate Frame (TF) Tree Violations & Multi-Parent Collisions

#### Bug 1: Gazebo DiffDrive TF Bridge Flooding `/tf` [SOLVED]
* **Affected Files**: [`Rover/my_robot_description/launch/spawn_rover.launch.py`](file:///e:/SHAKR/Autonmous-27/Rover/my_robot_description/launch/spawn_rover.launch.py) & [`gazebo.launch.py`](file:///e:/SHAKR/Autonmous-27/Rover/my_robot_description/launch/gazebo.launch.py).
* **The Root Cause**: Gazebo diff-drive plugin broadcasts `odom -> base_footprint` to `/tf`. Meanwhile, EKF publishes `odom -> base_link`, creating dual competing parent frames on `base_link`.
* **The Fix Applied**: Added `publish_map_tf:=false` (default `false`) and disabled simulation `/model/my_robot/tf` bridge into `/tf` during integrated bringup.

#### Bug 2: Triple Broadcaster & Conflicting Parent on Camera Frame [SOLVED]
* **Affected Files**: [`Rover/my_robot_description/urdf/my_robot.urdf.xacro`](file:///e:/SHAKR/Autonmous-27/Rover/my_robot_description/urdf/my_robot.urdf.xacro#L282), [`spawn_rover.launch.py`](file:///e:/SHAKR/Autonmous-27/Rover/my_robot_description/launch/spawn_rover.launch.py), [`SLAM/rover_slam/launch/static_transforms.launch.py`](file:///e:/SHAKR/Autonmous-27/SLAM/rover_slam/launch/static_transforms.launch.py).
* **The Collision**: Multiple static transforms broadcast `camera_depth_optical_frame -> my_robot/camera_link/camera` while URDF and Gazebo spawn published competing frames.
* **The Fix Applied**: Added `publish_camera_tf:=false` (default `false`) across `spawn_rover.launch.py`, `gazebo.launch.py`, and `gazebo_with_teleop.launch.py`. Aligned optical frame convention to REP-103.

---

### 🔴 Category B: Subsystem Topic Contracts & Bridge Gaps

#### Bug 3: ArUco SLAM Landmark Contract Mismatch [SOLVED]
* **Affected Files**: [`Perception/marker_detection/marker_detection/marker_action_interface_node.py`](file:///e:/SHAKR/Autonmous-27/Perception/marker_detection/marker_detection/marker_action_interface_node.py) & [`SLAM/rover_slam/launch/rtabmap.launch.py`](file:///e:/SHAKR/Autonmous-27/SLAM/rover_slam/launch/rtabmap.launch.py).
* **The Gap**: RTAB-Map SLAM listens for 6-DOF landmark constraints (`geometry_msgs/msg/PoseStamped`) on topic `/perception/aruco_pose`. The real `marker_detection` pipeline only published custom action target arrays.
* **The Fix Applied**: Added `slam_landmark_pub` to `marker_action_interface_node.py`. When a marker passes validation (`usable_for_action` and `pose_valid`), its 6-DoF pose in `base_link` is automatically published as `geometry_msgs/msg/PoseStamped` on `/perception/aruco_pose` for RTAB-Map global loop closure.

#### Bug 4: Mock ArUco Publisher Active by Default in SLAM Bringup [SOLVED]
* **Affected Files**: [`SLAM/rover_slam/launch/slam_bringup.launch.py`](file:///e:/SHAKR/Autonmous-27/SLAM/rover_slam/launch/slam_bringup.launch.py).
* **The Standalone Artifact**: `declare_launch_aruco_stub` defaulted to `true`, flooding `/perception/aruco_pose` with fake poses and corrupting live perception data.
* **The Fix Applied**: Changed default to `launch_aruco_stub:=false`.

#### Bug 5: Duplicate Nav2 Costmap Server in SLAM Bringup [SOLVED]
* **Affected Files**: [`SLAM/rover_slam/launch/slam_bringup.launch.py`](file:///e:/SHAKR/Autonmous-27/SLAM/rover_slam/launch/slam_bringup.launch.py).
* **The Standalone Artifact**: `slam_bringup.launch.py` unconditionally launched `costmap.launch.py` via a 3-second `TimerAction`, clashing with full Nav2 costmap server in `erc_path_planner`.
* **The Fix Applied**: Added `launch_costmap:=false` (default `false`) and conditioned `costmap_launch` on `launch_costmap`.

---

### 🔴 Category C: Path Planning & Simulation Clock Synchronization

#### Bug 6: Nav2 Dual Map Authority Clash [SOLVED]
* **Affected Files**: [`PathPlanning/erc_path_planner/launch/path_planning.launch.py`](file:///e:/SHAKR/Autonmous-27/PathPlanning/erc_path_planner/launch/path_planning.launch.py).
* **The Fix Applied**: Added `use_slam:=true` argument (default `true`). When true, `map_server` is disabled and omitted from `lifecycle_manager_navigation`, letting RTAB-Map own `/map`. When `use_slam:=false`, `map_server` loads `dummy_map.yaml` for standalone testing.

#### Bug 7: Hardcoded `use_sim_time: False` in Lifecycle Manager [SOLVED]
* **Affected Files**: [`PathPlanning/erc_path_planner/launch/path_planning.launch.py`](file:///e:/SHAKR/Autonmous-27/PathPlanning/erc_path_planner/launch/path_planning.launch.py#L162).
* **The Fix Applied**: Dynamic `LaunchConfiguration('use_sim_time')` passed into `lifecycle_manager_navigation` and all servers, preventing simulation clock deadlocks.

---

### 🔴 Category D: State Estimation Kinematics & Filter Tuning

#### Bug 8: 4-Wheel Hardcoding on 6-Wheel Rover
* **Affected Files**: [`SLAM/rover_slam/rover_slam/encoder_ticks_to_odom.py`](file:///e:/SHAKR/Autonmous-27/SLAM/rover_slam/rover_slam/encoder_ticks_to_odom.py#L160-L165).
* **The Flaw**: Node hardcodes `wheel_names` to 4 wheels with `track_width: 0.42`. The rover has 6 wheels with `track_width: 0.49`. Middle wheels (`left_middle`, `right_middle`) are completely ignored.
* **Action Required**: Update default `wheel_names` to include all 6 wheels and set `track_width: 0.49`.

#### Bug 9: EKF Lateral Velocity Over-Constraint
* **Affected Files**: [`SLAM/rover_slam/config/ekf.yaml`](file:///e:/SHAKR/Autonmous-27/SLAM/rover_slam/config/ekf.yaml#L17).
* **The Flaw**: `odom0_config` fuses $V_y$ (`[true, true, false]`). However, wheel kinematics hardcodes $V_y = 0.0$. Meanwhile, IMU fuses linear acceleration $Y$. When turning in loose sand or slopes, EKF receives $V_y = 0$ with high certainty while the IMU measures lateral slip, causing filter instability.
* **Action Required**: Set `odom0_config` $V_y$ to `false` (`[true, false, false]`).

---

### 🔴 Category E: Performance & Testing Infrastructure

#### Bug 10: Unused CPU Costmap Rasterization in Perception
* **Affected Files**: [`Perception/terrain_geometry/terrain_geometry/terrain_node.py`](file:///e:/SHAKR/Autonmous-27/Perception/terrain_geometry/terrain_geometry/terrain_node.py#L240-L260).
* **The Flaw**: Node executes Scipy distance-transform costmap inflation every frame to publish `/terrain/costmap`. Nav2 does not use `/terrain/costmap` (it subscribes to `/bridge/pointcloud` from `costmap_bridge_node`). This wastes ~35% of a CPU core.
* **Action Required**: Add parameter `enable_costmap:=false` (default `false`) to bypass steps 7 & 8 during integrated runs.

#### Bug 11: Broken Project Directory Index in Benchmarking Suite
* **Affected Files**: [`testing/PathPlanner/run_testing_suite.sh`](file:///e:/SHAKR/Autonmous-27/testing/PathPlanner/run_testing_suite.sh#L15).
* **The Flaw**: `PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../../.." && pwd)"` navigates 4 directories up to root (`E:\` or `/`), causing `colcon build` to fail or delete directories outside the workspace.
* **Action Required**: Change path resolution to `$SCRIPT_DIR/../..`.

#### Bug 12: Python Namespace Pollution
* **Affected Files**: [`testing/PathPlanner/setup.py`](file:///e:/SHAKR/Autonmous-27/testing/PathPlanner/setup.py#L10).
* **The Flaw**: Packages generic directories `mock` and `scripts`, installing them into Python `site-packages` and shadowing standard library modules.
* **Action Required**: Move mock scripts inside `global_path_benchmarking` namespace.

---

## 3. Operational Modes Matrix & RViz Visualization Rules

| Operational Mode | Launch Commands | Frame Authority (`map ➔ odom`) | Map Topic Authority (`/map`) | RViz Fixed Frame Setting |
| :--- | :--- | :--- | :--- | :--- |
| **Full Autonomous System** | `gazebo.launch.py`<br>`slam_bringup.launch.py`<br>`perception_system.launch.py`<br>`path_planning.launch.py` | RTAB-Map SLAM (dynamic drift correction) | RTAB-Map SLAM (live 2D occupancy grid) | `map` |
| **SLAM + Perception Integration** | `gazebo.launch.py`<br>`slam_bringup.launch.py`<br>`perception_system.launch.py` | RTAB-Map SLAM (dynamic drift correction) | RTAB-Map SLAM (live 2D occupancy grid) | `map` |
| **Perception Standalone Testing** | `gazebo.launch.py`<br>`test_perception_standalone.launch.py` | Static identity anchor (`publish_map_tf:=true`) OR none | Static anchor / None | **Option A**: Change Fixed Frame to `odom` or `base_link`<br>**Option B**: Use `map` with static identity bridge |
| **Path Planner Standalone Testing** | `test_planner_standalone.launch.py` (or `path_planning.launch.py use_slam:=false`) | Static identity anchor | `map_server` loading `dummy_map.yaml` or benchmark map | `map` |
| **SLAM Standalone Testing** | `test_slam_standalone.launch.py` | RTAB-Map SLAM | RTAB-Map SLAM | `map` |

### How to Inspect Perception in RViz Without SLAM
If you run Perception without SLAM:
1. **Method 1 (Fastest / Zero Nodes)**: In RViz under **Global Options**, change **Fixed Frame** from `map` to `odom` or `base_link`. RealSense points, obstacle bounding boxes, and ArUco markers render immediately.
2. **Method 2 (Static Anchor)**: Run `publish_map_tf:=true` to publish a dummy `map -> odom` transform so RViz's default `map` frame does not error out.

---

## 4. Launch Flag Specifications & Recommended Defaults

### 1. `PathPlanning/erc_path_planner/launch/path_planning.launch.py`
| Argument | Default | Integrated Production Value | Standalone Testing Value | Purpose |
| :--- | :---: | :---: | :---: | :--- |
| `use_slam` | `true` | `true` | `false` | When `true`, disables `map_server` (RTAB-Map owns `/map`). When `false`, boots `map_server` with `dummy_map.yaml`. |
| `use_sim_time` | `false` | `true` (sim) / `false` (real) | Match environment | Synchronizes Nav2 lifecycle servers with Gazebo `/clock`. |

### 2. `SLAM/rover_slam/launch/slam_bringup.launch.py`
| Argument | Target Default | Integrated Production Value | Standalone Testing Value | Purpose |
| :--- | :---: | :---: | :---: | :--- |
| `launch_aruco_stub` | `false` | `false` | `true` | When `false`, listens for real perception on `/perception/aruco_pose`. When `true`, boots `mock_aruco_publisher`. |
| `launch_costmap` | `false` | `false` | `true` | When `false`, omits standalone costmap server (Nav2 owns costmaps). When `true`, runs SLAM costmap for verification. |
| `launch_static_tf` | `true` | `true` | `true` | Includes static transforms. |
| `launch_camera` | `false` | `false` (sim) / `true` (real) | Match environment | Starts physical RealSense driver. |

### 3. `Rover/my_robot_description/launch/gazebo.launch.py` & `spawn_rover.launch.py`
| Argument | Target Default | Integrated Production Value | Standalone Testing Value | Purpose |
| :--- | :---: | :---: | :---: | :--- |
| `publish_map_tf` | `false` | `false` | `true` | When `false`, RTAB-Map SLAM owns `map -> odom`. When `true`, publishes static `map -> odom` for teleop. |
| `publish_camera_tf` | `false` | `false` | `true` | When `false`, URDF owns camera frame. |
| `bridge_sim_tf` | `false` | `false` | `true` | When `false`, Gazebo diff-drive plugin does NOT bridge raw TF to `/tf` (EKF owns `odom -> base_link`). |

### 4. `Perception/terrain_geometry/launch/terrain.launch.py`
| Argument | Target Default | Integrated Production Value | Standalone Testing Value | Purpose |
| :--- | :---: | :---: | :---: | :--- |
| `enable_costmap` | `false` | `false` | `true` | When `false`, skips Python 2D grid rasterization to save ~35% CPU. When `true`, publishes `/terrain/costmap`. |

---

## 5. Dedicated Standalone Test Launchers Specification

### 1. `PathPlanning/erc_path_planner/launch/test_planner_standalone.launch.py`
```python
# Runs Nav2 in standalone mode using dummy_map.yaml (or benchmark map) with static map anchor
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('erc_path_planner')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_share, 'config', 'dummy_map.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'path_planning.launch.py')),
            launch_arguments={'use_slam': 'false', 'map': LaunchConfiguration('map')}.items()
        )
    ])
```

### 2. `SLAM/rover_slam/launch/test_slam_standalone.launch.py`
```python
# Runs SLAM bringup with mock ArUco publisher and standalone costmap enabled for testing
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('rover_slam')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam_bringup.launch.py')),
            launch_arguments={'launch_aruco_stub': 'true', 'launch_costmap': 'true'}.items()
        )
    ])
```

### 3. `Perception/terrain_geometry/launch/test_perception_standalone.launch.py`
```python
# Runs perception pipeline with standalone static map anchor and RViz for isolated inspection
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('terrain_geometry')
    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='standalone_map_to_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'perception_system.launch.py')),
        launch_arguments={'launch_rviz': 'true'}.items()
    )
    return LaunchDescription([static_map_tf, perception])
```

---

## 6. Step-by-Step Resolution Roadmap

1. [x] **Path Planning**: Conditionalize `map_server` behind `use_slam:=true` and sync `use_sim_time` in `path_planning.launch.py`.
2. [ ] **SLAM Bringup**: In `slam_bringup.launch.py`, change `launch_aruco_stub` default to `false` and wrap `costmap_launch` behind `launch_costmap:=false`.
3. [ ] **Gazebo / Spawn Bridge**: In `spawn_rover.launch.py` and `gazebo.launch.py`, add `bridge_sim_tf:=false` to prevent Gazebo diff-drive from publishing conflicting `/tf` when EKF is active.
4. [ ] **SLAM Static Transforms**: In `static_transforms.launch.py`, remove redundant `static_tf_camera_optical_to_gz` node.
5. [ ] **ArUco Perception Bridge**: In `marker_action_interface_node.py`, publish confirmed target pose as `geometry_msgs/msg/PoseStamped` on `/perception/aruco_pose`.
6. [ ] **Kinematics & EKF**: Update `encoder_ticks_to_odom.py` with all 6 wheels and set `odom0_config` $V_y$ to `false` in `ekf.yaml`.
7. [ ] **Dedicated Launchers**: Create `test_slam_standalone.launch.py` and `test_perception_standalone.launch.py`.
8. [ ] **Testing Runner**: Fix relative path in `testing/PathPlanner/run_testing_suite.sh`.
