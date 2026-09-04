# Perception Subsystem: 3D Terrain Geometry, Obstacle Detection & ArUco Vision

The **Perception Subsystem** provides 3D spatial awareness, terrain geometry segmentation, obstacle extraction, and visual landmark/marker tracking for an autonomous Mars-analog ERC rover.

---

## 🏛️ 1. Subsystem Architecture & Packages

The Perception workspace contains two primary functional modules alongside their custom interface definition packages:

```
Perception/
├── docs/                                    # Architectural HTML & AI implementation guides
│   ├── perception.html                      # Interactive Glassmorphism Architecture Report
│   └── PerceptionAiGuide.md                 # Detailed AI Roadmap & Milestone Gates
│
├── rviz/                                    # 🌐 Centralized Master RViz View
│   └── perception_system_view.rviz          # Unified visualizer (Terrain + Obstacles + ArUco + TF)
│
├── terrain_geometry/                        # 🪨 Core 3D Point Cloud Processing Package
│   ├── terrain_geometry/                    # Python pipeline (ROI, Ground, Voxel, DBSCAN, Costmap)
│   ├── config/                              # Terrain configuration YAMLs
│   ├── launch/                              # Standalone terrain & RViz launchers
│   └── rviz/                                # 📦 Modular Package RViz View
│       └── terrain_geometry_view.rviz       # Dedicated Terrain Geometry display config
│
├── terrain_geometry_msgs/                   # 📜 Custom Terrain Message Definitions
│   └── msg/
│       ├── ObstacleFeature.msg              # Single obstacle centroid & 3D bounding box dimensions
│       └── ObstacleFeatureArray.msg         # Array of detected obstacle features
│
├── marker_detection/                        # 🎯 ArUco Vision & SolvePnP 3D Pose Package
│   ├── marker_detection/                    # Multi-stage detection, SolvePnP, Kalman tracking
│   ├── config/                              # Marker detection YAML parameters
│   ├── launch/                              # Marker detection launchers
│   └── rviz/                                # 📦 Modular Package RViz View
│       └── marker_detection_view.rviz       # Dedicated Marker Detection display config
│
└── marker_detection_msgs/                   # 📜 Custom Marker Message Definitions
    └── msg/
        ├── MarkerPose.msg                   # Filtered 6-DOF marker pose with covariance
        └── MarkerPoseArray.msg              # Array of detected active markers
```

---

## 📡 2. Input & Output Topics

### A. Subscriptions (Sensor Inputs)
*   **`/camera/depth/color/points`** (`sensor_msgs/msg/PointCloud2`): Raw 3D point cloud in the camera optical frame.
*   **`/camera/color/image_raw`** (`sensor_msgs/msg/Image`): Raw RGB camera feed for ArUco marker extraction.
*   **`/camera/camera_info`** (`sensor_msgs/msg/CameraInfo`): Camera intrinsic calibration matrix ($f_x, f_y, c_x, c_y$).
*   **`/tf` & `/tf_static`** (`tf2_msgs/msg/TFMessage`): Dynamic and static coordinate transforms (`camera_link` $\rightarrow$ `base_link`).

### B. Publications (Outputs to SLAM & Nav2)
*   **`/perception/local_bboxes`** (`vision_msgs/msg/Detection3DArray`): Raw single-frame 3D rock bounding boxes.
*   **`/perception/obstacles_only`** (`vision_msgs/msg/Detection3DArray`): Smoothed, persistent rock obstacles with blind-spot retention for the Nav2 Costmap Server.
*   **`/terrain/costmap`** (`nav_msgs/msg/OccupancyGrid`): Direct 2D inflated obstacle costmap at 5cm resolution.
*   **`/terrain/obstacle_features`** (`terrain_geometry_msgs/msg/ObstacleFeatureArray`): Obstacle telemetry (centroid, size dimensions, distance).
*   **`/terrain/obstacle_markers`** (`visualization_msgs/msg/MarkerArray`): 3D visual bounding boxes and text labels for RViz2.
*   **`/perception/aruco_pose`** (`geometry_msgs/msg/PoseStamped`): 6-DOF marker target pose for Saif SLAM loop closure / drift hard resets.

### C. Debug Publications (Active when `publish_debug_topics: True`)
*   **`/terrain/debug/ground_cloud`** (`sensor_msgs/msg/PointCloud2`): Ground planes separated by Patchwork++.
*   **`/terrain/debug/voxel_cloud`** (`sensor_msgs/msg/PointCloud2`): Point cloud filtered by Voxel Downsampling.
*   **`/terrain/debug/clustered_cloud`** (`sensor_msgs/msg/PointCloud2`): Points colored by DBSCAN Cluster IDs.

---

## 🖥️ 3. RViz2 Visualization Setup: Modular vs Centralized

To maintain a clean separation of concerns, the workspace provides both **modular package-level views** and a **centralized master perception view**:

| View Scope | Configuration Path | What It Displays |
| :--- | :--- | :--- |
| **Centralized System View** | `Perception/rviz/perception_system_view.rviz` | **Full Perception Subsystem**: Live RGB stream, 3D PointCloud, DBSCAN clusters, 2D Inflated Costmap, Persistent Obstacle BBoxes, and ArUco 3D target markers/TF frames simultaneously. |
| **Terrain Geometry View** | `Perception/terrain_geometry/rviz/terrain_geometry_view.rviz` | **Terrain Only**: PointCloud, ground separation, voxel grid, cluster markers, and `/terrain/costmap`. |
| **Marker Detection View** | `Perception/marker_detection/rviz/marker_detection_view.rviz` | **ArUco Only**: RGB image overlay with 2D corner detections, 3D marker coordinate axes, and `/marker_detection/targets`. |

---

## 🚀 4. Running & Launching

### A. Build the Workspace
```bash
cd /path/to/your/cloned/repository
source /opt/ros/humble/setup.bash # Or jazzy
colcon build
source install/setup.bash
```

### B. Launching Individual Modules (Modular)

**1. Terrain Geometry (Standalone):**
```bash
ros2 launch terrain_geometry terrain.launch.py
# Optional: Launch modular RViz
ros2 launch terrain_geometry rviz.launch.py
```

**2. Marker Detection (Standalone):**
```bash
ros2 launch marker_detection marker_detection.launch.py
# Optional: Launch modular RViz
ros2 launch marker_detection rviz.launch.py
```

### C. Launching the Full Perception Subsystem
```bash
ros2 launch terrain_geometry perception_system.launch.py
# Open Centralized System RViz:
rviz2 -d Perception/rviz/perception_system_view.rviz
```

---

## 🧪 5. Testing & Validation Workflows

### Method A: Testing in Gazebo Mars Yard Simulation (Recommended)
1. **Start World & Spawn Rover:**
   ```bash
   ros2 launch my_robot_description gazebo.launch.py world:=world1.world
   ```
2. **Launch Perception:**
   ```bash
   ros2 launch terrain_geometry perception_system.launch.py
   ```
3. **Drive the Rover:**
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
4. **Inspect RViz2:**
   Open `Perception/rviz/perception_system_view.rviz` in RViz2 to verify costmap updates and 3D bounding box stability.

### Method B: Testing with ROS 2 Bag Playback
1. **Publish Camera TF (if not in bag):**
   ```bash
   ros2 run tf2_ros static_transform_publisher 0.5 0 0.5 0 0 0 base_link camera_link
   ```
2. **Play Bag:**
   ```bash
   ros2 bag play /path/to/bag_file/
   ```
3. **Launch Perception:**
   ```bash
   ros2 launch terrain_geometry terrain.launch.py
   ```

---

## ⚠️ 6. Important Notes on QoS Configuration

RealSense camera drivers and Gazebo simulation plugins publish raw sensor streams (`/camera/depth/color/points`, `/camera/color/image_raw`) using **Best Effort** QoS. 

*   All perception input subscribers are configured to use `QoSReliabilityPolicy.BEST_EFFORT` to prevent silent message drops.
*   If viewing raw feeds in RViz2, make sure each display's **Reliability Policy** is set to **Best Effort**.

