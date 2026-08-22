# SLAM Subsystem (`rover_slam`) — ROS 2 Humble

This module contains the SLAM (Simultaneous Localization and Mapping) and state estimation pipeline for the Autonomous Mars Rover under **ROS 2 Humble**.

---

## 1. Architecture Overview (Option B)

1. **Local State Estimator (`robot_localization` EKF)**:
   - High-frequency ($100\,\text{Hz}$) continuous filtering.
   - Fuses wheel odometry (`/wheel/odom_raw`) and 6-DOF IMU (`/imu/data`).
   - Broadcasts dynamic `odom -> base_link` TF.
2. **Slip Detection & Pre-Filtering (`heuristic_slip_checker`)**:
   - Analyzes discrepancies between wheel velocity and IMU linear acceleration/yaw rates to detect sandy wheel slip.
   - Dynamically inflates wheel odometry covariance during slip events.
3. **Global Mapping Backend (`rtabmap_ros` / `rtabmap_slam`)**:
   - Visual SLAM processing RealSense D435 RGB-D depth and image streams.
   - Graph SLAM loop closure incorporating 6-DOF ArUco landmark poses (`/perception/aruco_pose`).
   - Publishes static occupancy grid (`/map`) and `map -> odom` correction TF.
4. **Local/Global Costmap Server (`nav2_costmap_2d`)**:
   - Nav2 2D Costmap server fusing static map layers and real-time obstacle point clouds (`/perception/obstacles_only`) with inflation layers.

---

## 2. Directory Structure

```text
SLAM/
├── README.md                   # This document
├── SlamAiGuide.md              # SLAM architecture & implementation guide
├── Slam_Docu/                  # Architecture documentation & diagrams
└── rover_slam/                 # ROS 2 Humble package
    ├── package.xml             # Package dependencies
    ├── setup.py                # Build setup & node entry points
    ├── config/                 # YAML configuration files
    │   ├── ekf.yaml            # robot_localization EKF config
    │   ├── rtabmap.yaml        # RTAB-Map parameters & GTSAM optimizer settings
    │   ├── costmap_params.yaml # Nav2 Costmap 2D layer & inflation parameters
    │   └── realsense_filters.yaml # RealSense D435 post-processing filters
    ├── launch/                 # ROS 2 Humble launch files
    │   ├── slam_bringup.launch.py   # Master SLAM bringup launcher
    │   ├── ekf.launch.py            # Local EKF & Slip Checker
    │   ├── rtabmap.launch.py        # RTAB-Map SLAM
    │   ├── static_transforms.launch.py # Positional TF broadcaster (base -> camera, base -> imu)
    │   ├── vision_helper.launch.py  # Camera driver & depth filters
    │   └── costmap.launch.py        # Nav2 Costmap 2D lifecycle node
    └── rover_slam/             # Python nodes
        ├── encoder_ticks_to_odom.py
        ├── heuristic_slip_checker.py
        ├── mock_aruco_publisher.py
        └── costmap_test_stub.py
```

---

## 3. How to Build & Run on ROS 2 Humble

```bash
# Sourcing ROS 2 Humble
source /opt/ros/humble/setup.bash

# Build the package
colcon build --symlink-install --packages-select rover_slam

# Source the workspace
source install/setup.bash

# Launch full SLAM subsystem
ros2 launch rover_slam slam_bringup.launch.py
```
