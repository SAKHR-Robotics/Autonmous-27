# Robot Description & Simulation (`my_robot_description`)

Autonomous 4-wheeled skid-steer Mars rover simulation package for ROS 2 Jazzy and Gazebo Harmonic/Ignition.

---

## 📌 Overview

This package defines the robot kinematics, URDF/Xacro models, sensor transforms, and Gazebo world spawning orchestration:

- **Robot Model**: 4-wheel skid-steer rover with chassis, suspension arms, drill motors, RealSense D435 RGB-D depth camera, and BNO055 IMU.
- **Simulation Bridge**: ROS-GZ parameter bridge routing `/cmd_vel`, `/model/my_robot/odometry`, `/camera/*`, and `/imu/data`.
- **TF Ownership**: Configured with `bridge_sim_tf:=false` in production so the EKF (`robot_localization`) holds sole transform authority over `odom ➔ base_link`.

---

## 🧪 Standalone Testing Guide (World + Rover + Teleop)

Test rover spawning, physics simulation, sensor data streams, and manual driving in isolation:

### Method A: Interactive Launcher (Recommended)
```bash
bash scripts/launch_sim.sh
# Select Option 1 for SLAM-compatible mode (bridge_sim_tf:=false)
# Select Option 2 for Standalone Teleop mode (bridge_sim_tf:=true, publish_map_tf:=true)
```

---

### Method B: Manual Step-by-Step Terminal Playbook

#### Step 1: Launch Mars Yard World & Spawn Rover
Open Terminal 1:
```bash
source install/setup.bash

# Standalone Teleop without SLAM:
ros2 launch my_robot_description gazebo.launch.py publish_map_tf:=true bridge_sim_tf:=true

# Or with custom world (e.g., empty world with sensors):
ros2 launch my_robot_description gazebo.launch.py world:=empty_with_sensors.sdf publish_map_tf:=true bridge_sim_tf:=true
```

#### Step 2: Launch Teleoperation GUI
Open Terminal 2:
```bash
source install/setup.bash
ros2 run my_robot_description teleop_gui.py
```
*(Provides interactive forward/reverse, turning sliders, and emergency stop buttons).*

#### Step 3: Sensor Verification
Open Terminal 3 to verify live sensor outputs as you drive:

```bash
# 1. Verify IMU publishes at 100 Hz
ros2 topic hz /imu/data

# 2. Verify RealSense RGB image stream
ros2 topic hz /camera/color/image_raw

# 3. Verify RealSense 3D point cloud stream
ros2 topic hz /camera/depth/color/points

# 4. Verify cmd_vel responsiveness
ros2 topic echo /cmd_vel
```

#### Step 4: Model Inspection in RViz2
To visualize kinematics and URDF frames without Gazebo physics:
```bash
ros2 launch my_robot_description display.launch.py
```

---

## ⚙️ Topic Contract

| Topic Name | Type | Direction | Purpose |
| :--- | :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Input | Motor velocity requests (linear $x$, angular $z$) |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | Output | 640x480 RGB camera stream |
| `/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` | Output | RealSense 3D depth point cloud |
| `/imu/data` | `sensor_msgs/msg/Imu` | Output | 100 Hz linear acceleration & angular velocity |
| `/model/my_robot/odometry` | `nav_msgs/msg/Odometry` | Output | Raw wheel contact odometry from Gazebo diff-drive plugin |
| `/tf` | `tf2_msgs/msg/TFMessage` | Output | Robot joint and body link transforms |

---

## 📐 Kinematic & Physical Specifications

- **Chassis Dimensions**: $0.4\times 0.4\times 0.15\text{ m}$ (center box).
- **Track Width**: $0.65\text{ m}$ (wheel separation lateral).
- **Wheel Base**: $0.4\text{ m}$ (wheel separation longitudinal).
- **Wheel Radius**: $0.15\text{ m}$.
- **Drive Configuration**: 4-wheel skid-steer (differential drive plugin).
- **Camera Mount**: Pitched down $15^\circ$, centered on front face ($x = +0.215\text{ m}$).
