# ROS 2 Rover Development Skill

## Build Commands
Always use symlink-install for fast iteration:
```bash
colcon build --symlink-install --packages-select <pkg_name>
source install/setup.bash
```

## Primary Subsystems
- `Rover/my_robot_description`: URDF, Xacro, DiffDrive plugin, teleop GUI
- `SLAM/rover_slam`: Wheel encoder kinematics (`encoder_ticks_to_odom`), slip checking, EKF fusion
- `Perception/marker_detection`: ArUco target detection & 3D clustering
- `PathPlanning/erc_path_planner`: Nav2 Smac Planner & MPPI controller

## TF Frame Chain (REP-105)
`map` -> `odom` -> `base_link` -> `base_footprint`, sensors, wheel links.
- Only EKF publishes `odom` -> `base_link`.
- RTAB-Map publishes `map` -> `odom`.
