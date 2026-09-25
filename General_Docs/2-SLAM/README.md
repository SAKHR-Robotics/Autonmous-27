# 🧭 SLAM & State Estimation Subsystem Diagrams (`General_Docs/2-SLAM/`)

This directory contains the visual pipeline graphs and granular block deep-dive diagrams for the **SLAM Subsystem** (multi-sensor state estimation, wheel slip rejection, 100Hz EKF fusion, and RTAB-Map SLAM).

---

## 📸 Diagram Gallery

### 1. Master Pipeline
![SLAM Master Pipeline](00_slam_master_pipeline.png)

---

### 2. Wheel Odometry Kinematics & Single-Wheel Slip Isolation
![Wheel Odometry Kinematics](01_encoder_ticks_to_odom_kinematics.png)

---

### 3. Heuristic Slip Checker & Dynamic Covariance Inflation
![Slip Checker](02_heuristic_slip_checker_and_covariance.png)

---

### 4. RealSense D435 Post-Processing Depth Filters
![RealSense Filters](03_realsense_depth_postprocessing_filters.png)

---

### 5. robot_localization EKF 100Hz Local Sensor Fusion
![EKF Fusion](04_robot_localization_ekf_100hz_fusion.png)

---

### 6. RTAB-Map Visual SLAM, Memory Graph & Loop Closures
![RTAB-Map SLAM](05_rtabmap_visual_slam_and_loop_closure.png)

---

### 7. Nav2 Costmap 2D Layering & Inflation Safety Corridor
![Costmap 2D](06_nav2_costmap_2d_layering.png)
