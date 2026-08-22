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

## 🛠️ How to Build & Run
```bash
# Build package
cd ~/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
colcon build --packages-select rover_slam
source install/setup.bash

# Run master bringup
ros2 launch rover_slam slam_bringup.launch.py
```
