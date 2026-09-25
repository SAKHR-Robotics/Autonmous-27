# Original User Request

## 2026-09-23T01:24:26Z

This is a single self-contained fix; keep it small and focused. Refactor the Mars rover URDF/Xacro and Gazebo simulation models from a 6-wheel skid-steer architecture to a 4-wheel drive configuration, updating all associated kinematics, sensor frame definitions, diff-drive plugins, and telemetry nodes.

Working directory: e:/SHAKR/Autonmous-27
Integrity mode: development

## Requirements

### R1. URDF Kinematics & Structure Refactoring
Remove middle wheel links (`left_middle_wheel_link`, `right_middle_wheel_link`), middle suspension arm links (`left_middle_arm_link`, `right_middle_arm_link`), and their associated joints from `Rover/my_robot_description/urdf/my_robot.urdf.xacro`. Retain front (+0.15m) and rear (-0.15m) wheels and arms. Ensure the robot link/joint count and tree are coherent.

### R2. Gazebo Physics & DiffDrive Plugin Update
Update `Rover/my_robot_description/urdf/gazebo.xacro`:
- Remove Gazebo reference tags for middle arm and wheel links (`<gazebo reference="...middle...">`).
- In `<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">`, configure the plugin for 4 wheels by retaining only `left_front_wheel_joint`, `left_rear_wheel_joint`, `right_front_wheel_joint`, and `right_rear_wheel_joint`.

### R3. Telemetry, Odometry & RViz Configuration Alignment
- In `SLAM/rover_slam/rover_slam/encoder_ticks_to_odom.py`, ensure joint state matching matches the URDF joint names (`left_front_wheel_joint`, `right_front_wheel_joint`, `left_rear_wheel_joint`, `right_rear_wheel_joint` or configured mapping) and processes the 4 wheels correctly.
- Update RViz configs (`Rover/my_robot_description/rviz/robot_view.rviz`, `SLAM/rover_slam/config/slam_visualization.rviz`, and Perception RViz configs) to remove obsolete middle wheel/arm tree entries.
- Update documentation and package summaries in `Rover/my_robot_description/README.md` and `SLAM/README.md` to reflect the 4-wheel specification.

## Acceptance Criteria

### URDF & TF Verification
- [ ] Processing `urdf/my_robot.urdf.xacro` with `xacro` produces valid URDF with no references to middle wheels or middle arms.
- [ ] Total active wheel links equal exactly 4 (`left_front_wheel_link`, `left_rear_wheel_link`, `right_front_wheel_link`, `right_rear_wheel_link`).
- [ ] DiffDrive plugin in `gazebo.xacro` references only the 4 active wheel joints.

### Odometry & Kinematics Consistency
- [ ] `encoder_ticks_to_odom.py` executes without errors and correctly computes 4-wheel velocities and `/wheel/odom_raw`.
- [ ] No broken TF frames or missing joint warnings when launching robot state publisher with the updated URDF.
