# Interactive Modular Launch Scripts (`scripts/`)

This directory contains interactive bash orchestrators designed to streamline development, testing, and debugging across the autonomous rover stack.

---

## 📌 Available Scripts

| Script | Responsibility | Primary Modes & Options |
| :--- | :--- | :--- |
| [`launch_system.sh`](launch_system.sh) | **Master Control Dashboard** | • Option 1: Start All Systems (Sim ➔ SLAM ➔ Perception ➔ Planning ➔ Teleop)<br>• Option 2: Clean Restart (`pkill` hanging processes)<br>• Option 3-6: Launch individual subsystems |
| [`launch_sim.sh`](launch_sim.sh) | **Simulation & World Bringup** | • Option 1: Mars Yard + Rover (SLAM Mode, `bridge_sim_tf:=false`)<br>• Option 2: Mars Yard + Rover (Standalone Teleop, `bridge_sim_tf:=true`)<br>• Option 3: Mars Yard World Only<br>• Option 4: Empty World + Rover |
| [`launch_slam.sh`](launch_slam.sh) | **State Estimation & SLAM** | • Option 1: Full SLAM Stack (Slip Checker + EKF + RTAB-Map)<br>• Option 2: Standalone Test Launcher (`test_slam_standalone.launch.py`)<br>• Option 3: EKF & Slip Checker Only (No RTAB-Map) |
| [`launch_perception.sh`](launch_perception.sh) | **Perception & Clustering** | • Option 1: Full Perception (Rocks + ArUco Tags)<br>• Option 2: Standalone Test Launcher (`test_perception_standalone.launch.py`)<br>• Option 3: ArUco Tag Marker Detection Only<br>• Option 4: Terrain Point Cloud DBSCAN Node Only |
| [`launch_planning.sh`](launch_planning.sh) | **Nav2 Path Planning** | • Option 1: Full Nav2 with Live SLAM Map (`use_slam:=true`)<br>• Option 2: Standalone Nav2 with Dummy Map (`test_planner_standalone.launch.py`)<br>• Option 3: 3D Costmap Bridge Node Only |

---

## 🚀 Quick Usage Examples

### 1. Launch Master Dashboard
```bash
bash scripts/launch_system.sh
```

### 2. Run Clean Restart
If Gazebo, RViz, or DDS bridges freeze or hold ports:
```bash
bash scripts/launch_system.sh
# Select Option 2 (Clean Restart)
```
*(Executes: `pkill -9 -f gazebo; pkill -9 -f ign; pkill -9 -f gz; pkill -9 -f ros; pkill -9 -f rviz; killall -9 -q ruby gz server rviz2 parameter_bridge ros2 robot_state_publisher`).*

### 3. Launch Standalone Perception Testing
```bash
# Terminal 1: Simulation World & Rover
bash scripts/launch_sim.sh   # Select Option 1

# Terminal 2: Teleoperation GUI
ros2 run my_robot_description teleop_gui.py

# Terminal 3: Standalone Perception
bash scripts/launch_perception.sh # Select Option 2
```

### 4. Launch Standalone Nav2 Path Planning
```bash
# Terminal 1: Simulation World & Rover
bash scripts/launch_sim.sh   # Select Option 2

# Terminal 2: Teleoperation GUI
ros2 run my_robot_description teleop_gui.py

# Terminal 3: Standalone Nav2
bash scripts/launch_planning.sh # Select Option 2
```
