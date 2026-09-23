# ERC Path Planner (`erc_path_planner`)

ROS 2 Jazzy navigation and path planning package for the 4-wheel skid-steer autonomous rover in the European Rover Challenge (ERC) Mars Yard simulation.

---

## 📌 Overview & Architecture

`erc_path_planner` orchestrates global path search, dynamic obstacle avoidance, and local trajectory tracking using Nav2:

- **Global Path Planner**: `nav2_smac_planner::SmacPlannerHybrid` implementing Hybrid A* with Reeds-Shepp vehicle motion modeling and kinematic turning constraints.
- **Local Trajectory Controller**: `nav2_mppi_controller::MPPIController` running GPU/CPU-accelerated Model Predictive Path Integral optimization for real-time collision avoidance.
- **Costmap Obstacle Bridge (`costmap_bridge_node`)**: C++ node that bridges Perception with Nav2's native `ObstacleLayer`. Subscribes to `/perception/obstacles_only` (`vision_msgs/msg/Detection3DArray`) and samples the 3D bounding box footprint at $5\text{ cm}$ resolution into `sensor_msgs/msg/PointCloud2` on `/bridge/pointcloud`.
- **Costmap Layers**:
  - Global Costmap: Static layer (`/map`) + Obstacle layer (`/bridge/pointcloud`) + Inflation layer ($0.85\text{ m}$ radius).
  - Local Costmap: $10\times 10\text{ m}$ rolling window on `odom` + Obstacle layer + Inflation layer.

---

## 📂 Package Structure

```
erc_path_planner/
├── CMakeLists.txt              # C++ build definitions (links rclcpp, sensor_msgs, vision_msgs)
├── package.xml                 # Package manifest and dependencies
├── behavior_trees/             # Nav2 XML behavior trees
│   ├── navigate_to_pose_w_replanning_and_recovery.xml
│   └── navigate_through_poses_w_replanning_and_recovery.xml
├── config/
│   ├── nav2_params.yaml        # Full Smac Planner, MPPI, Costmap, and Lifecycle configuration
│   ├── dummy_map.yaml          # Static 50x50m map for standalone planner testing
│   └── dummy_map.pgm           # Occupancy grid bitmap for standalone mode
├── launch/
│   ├── path_planning.launch.py          # Production Nav2 bringup (integrates with SLAM /map)
│   ├── test_planner_standalone.launch.py# Standalone testing bringup (runs dummy_map + costmaps)
│   └── rviz.launch.py                   # Dedicated Navigation RViz2 visualizer
└── src/
    └── costmap_bridge_node.cpp # High-throughput 3D bounding box point cloud sampler
```

---

## 🧪 Standalone Testing Guide (World + Rover + Teleop + Planner)

You can test the planner standalone without running the full SLAM or perception modules:

### Method A: Interactive Script (Fastest)
```bash
bash scripts/launch_planning.sh
# Select Option 1 for Production (with SLAM) or Option 2 for Standalone (with dummy_map)
```

---

### Method B: Manual Step-by-Step Terminal Playbook

#### Step 1: Launch Mars Yard World & Spawn Rover
Open Terminal 1 and start Gazebo simulation:
```bash
source install/setup.bash
# When testing standalone without SLAM, pass publish_map_tf:=true to bridge static world coordinates
ros2 launch my_robot_description gazebo.launch.py publish_map_tf:=true
```

#### Step 2: Launch Teleoperation GUI
Open Terminal 2 to manually position the rover or test obstacle clearances:
```bash
source install/setup.bash
ros2 run my_robot_description teleop_gui.py
```

#### Step 3: Launch Standalone Path Planning
Open Terminal 3 to boot Nav2 with the standalone test launcher:
```bash
source install/setup.bash
ros2 launch erc_path_planner test_planner_standalone.launch.py
```
*(Loads `dummy_map.yaml`, boots Lifecycle Manager, Smac Planner, MPPI Controller, and `costmap_bridge_node`).*

#### Step 4: Verification & Navigation Goal Execution
1. In the opened RViz2 window, verify that:
   - Fixed Frame is set to `map`.
   - Global Costmap (`/global_costmap/costmap`) and Local Costmap (`/local_costmap/costmap`) render without errors.
   - Robot footprint renders matching the physical $0.9\times 0.7\text{ m}$ rover base.
2. Select the **2D Goal Pose** tool in the RViz2 top toolbar and click on an open position on the map.
3. Observe:
   - Global path (`/plan`) is generated via Reeds-Shepp curves.
   - MPPI candidate trajectories appear around the rover.
   - Rover begins moving toward the target and publishes velocity commands to `/cmd_vel`.

#### Step 5: Test Dynamic Obstacle Injection
In Terminal 4, inject a synthetic obstacle bounding box to verify real-time avoidance:
```bash
ros2 topic pub --once /perception/obstacles_only vision_msgs/msg/Detection3DArray "{
  header: {frame_id: 'map'},
  detections: [{
    id: 'test_rock_1',
    bbox: {
      center: {position: {x: 3.0, y: 0.0, z: 0.2}, orientation: {w: 1.0}},
      size: {x: 0.8, y: 0.8, z: 0.4}
    }
  }]
}"
```
Verify:
```bash
# Check sampled point cloud generation
ros2 topic hz /bridge/pointcloud

# Verify MPPI replans trajectory around the inflated rock
ros2 topic echo /plan
```

---

## ⚙️ Key Topics Contract

| Topic Name | Message Type | Direction | Purpose |
| :--- | :--- | :--- | :--- |
| `/perception/obstacles_only` | `vision_msgs/msg/Detection3DArray` | Input | 3D bounding boxes from Perception / Persistent Memory |
| `/bridge/pointcloud` | `sensor_msgs/msg/PointCloud2` | Output (Internal) | Dense 5cm sampled obstacle points for Nav2 `ObstacleLayer` |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Input | Static global grid from RTAB-Map SLAM (or `map_server` in standalone) |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | Input | Filtered smooth odometry from SLAM EKF |
| `/plan` | `nav_msgs/msg/Path` | Output | Global path generated by Smac Hybrid A* |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Output | Velocity commands published to rover motors |
