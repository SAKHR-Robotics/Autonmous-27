# ERC 2026 Path Planning Module

This repository contains the ROS 2 Path Planning stack for the ERC 2026 Rover based on **Nav2**, **Smac Hybrid A\*** (Global Planner), and **MPPI** (Local Planner & Controller).

---

## 🛠️ System Prerequisites & Dependencies

Before building this module, you must install ROS 2 Navigation2 and its bringup packages for **ROS 2 Humble**:

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup -y
```

### ROS 2 Package Dependencies
- `rclcpp`
- `sensor_msgs`
- `vision_msgs` (Standard ROS 2 Vision Messages)
- `nav2_bringup`
- `nav2_smac_planner`
- `nav2_mppi_controller`
- `nav2_costmap_2d`
- `nav2_bt_navigator`

---

## 🏗️ Building the Package

```bash
# 1. Navigate to workspace root
cd /path/to/your/cloned/repository

# 2. Build the path planner package
colcon build --packages-select erc_path_planner

# 3. Source the workspace
source install/setup.bash
```

---

## 🚀 Execution & Testing

### 1. Launch Path Planner Bringup
```bash
ros2 launch erc_path_planner path_planning.launch.py
```

### 2. Launch RViz Dashboard
```bash
ros2 launch erc_path_planner rviz.launch.py
```

### 3. Run Perception Bridge Node
```bash
ros2 run erc_path_planner costmap_bridge_node
```
