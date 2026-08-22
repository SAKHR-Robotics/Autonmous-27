# 🚀 rover_slam: ERC Mars Rover SLAM & State Estimation Package

This ROS 2 package implements **Option B Architecture** for the ERC Autonomous Mars Rover:
* **Local State Estimator:** `robot_localization` EKF node publishing smooth `odom -> base_link` pose at 100 Hz.
* **Pre-Filtering:** RealSense D435 post-processing depth filters + `heuristic_slip_checker.py` dynamically scaling wheel covariance during sand slippage.
* **Global Mapping Backend:** `rtabmap_ros` publishing static `/map` grid and `map -> odom` transform offset (1-5 Hz), incorporating ArUco landmark 6-DOF poses.
* **Costmap Server:** `nav2_costmap_2d` server fusing static map & dynamic rock obstacle point clouds into `/global_costmap/costmap`.

## 📦 Package Layout
```text
rover_slam/
├── config/
│   ├── ekf.yaml                  # robot_localization EKF config
│   ├── rtabmap.yaml              # RTAB-Map parameters & loop closure settings
│   ├── costmap_params.yaml       # Nav2 Costmap 2D layer & inflation settings
│   ├── realsense_filters.yaml   # D435 depth filter parameters
│   └── slam_visualization.rviz  # RViz 2 dashboard configuration
├── launch/
│   ├── slam_bringup.launch.py   # Master system launch file
│   ├── ekf.launch.py            # Local EKF & Slip Checker launch
│   ├── rtabmap.launch.py        # RTAB-Map SLAM launch
│   ├── static_transforms.launch.py # Static TFs (base_link -> camera_link / imu_link)
│   ├── vision_helper.launch.py  # Camera depth filters & ArUco detector launch
│   └── costmap.launch.py        # Nav2 Costmap 2D server launch
├── rover_slam/
│   ├── encoder_ticks_to_odom.py  # Converts raw wheel ticks to nav_msgs/Odometry Twist
│   ├── heuristic_slip_checker.py # Compares wheel vs IMU & updates covariance
│   ├── aruco_detector_node.py    # OpenCV ArUco detector & PnP solver
│   └── costmap_test_stub.py      # Test stub simulating perception rock clouds
└── test/
    └── test_slip_checker.py      # Unit test suite
```

## 🛠️ System Dependencies Installation
Before building or running, install required system dependencies:
```bash
sudo apt update && sudo apt install -y \
  ros-humble-diagnostic-updater \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-nav2-costmap-2d \
  ros-humble-robot-localization \
  ros-humble-rtabmap-ros \
  ros-humble-rtabmap-slam
```

## 🚀 How to Build
```bash
cd ~/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
colcon build --packages-select rover_slam
source install/setup.bash
```

---

## 🧪 Step-by-Step Validation & Testing Suite

### 1️⃣ Test 1: Static Transforms Validation (Checkpoint 1)
Verify that static coordinate frames (`base_link -> camera_link` and `base_link -> imu_link`) broadcast correctly:
```bash
# Terminal 1:
ros2 launch rover_slam static_transforms.launch.py

# Terminal 2:
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link imu_link
```
*Expected Result:* 3D translation and rotation frames print continuously at 10 Hz with 0 lookup errors.

---

### 2️⃣ Test 2: RealSense Depth Filter Pipeline (Checkpoint 2)
Verify that camera depth post-processing filters (decimation, spatial, temporal, 4.0m range clipping) are active:
```bash
# Terminal 1:
ros2 launch rover_slam vision_helper.launch.py

# Terminal 2:
ros2 topic hz /camera/depth/filtered
```
*Expected Result:* Filtered depth images publish smoothly on `/camera/depth/filtered`.

---

### 3️⃣ Test 3: Standalone Nav2 Costmap 2D + Synthetic Obstacle Test (Checkpoint 4B)
Verify that `nav2_costmap_2d` ingests 3D point cloud rocks from `/perception/obstacles_only` and outputs an inflated safety costmap grid:
```bash
# Terminal 1: Launch Costmap 2D Server
ros2 launch rover_slam costmap.launch.py

# Terminal 2: Run Synthetic Rock Generator
ros2 run rover_slam costmap_test_stub

# Terminal 3: Echo the Costmap Output
ros2 topic echo /global_costmap/costmap
```
*Expected Result:* `/global_costmap/costmap` publishes streaming `nav_msgs/msg/OccupancyGrid` showing inflated rock obstacle cost zones (costs 100 -> 85 -> 50).

---

### 4️⃣ Test 4: ArUco Landmark Pose Topic Contract (Checkpoint 3B)
Verify that the SLAM landmark channel can ingest 6-DOF marker poses independently:
```bash
ros2 topic pub /perception/aruco_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'camera_link'}, pose: {position: {x: 1.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}" -1
```
*Expected Result:* Message publishes cleanly for RTAB-Map graph loop closure.

---

### 5️⃣ Test 5: Master SLAM Bringup (Checkpoint 5)
Run the full system integration launch combining all sub-systems:
```bash
ros2 launch rover_slam slam_bringup.launch.py
```
