# Testing & Benchmarking Subsystem

This subsystem provides simulation launch scripts, scenario verification, and automated path planning benchmarking suites for the Mars Rover workspace.

## Key Components

- **`launch_world_and_rover.sh`**:
  Main interactive launcher script that brings up Gazebo simulation with the Mars Yard world (rocks & ArUco markers), spawns the rover, and launches the Rover Teleop GUI with automatic process cleanup on exit.

- **`PathPlanner/` (`global_path_benchmarking`)**:
  ROS 2 Python benchmarking suite that evaluates global path planners across benchmark scenarios, generating automated PDF, HTML, and PNG benchmark performance reports.

- **`Perception/` & `SLAM/`**:
  Validation scripts and mock publishers for Perception and SLAM modules.
