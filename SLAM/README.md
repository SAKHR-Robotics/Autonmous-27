# SLAM Subsystem

The **SLAM** (Simultaneous Localization and Mapping) subsystem provides 3D environment mapping, sensor fusion odometry, and wheel slip detection for the Autonomous Mars Rover.

## Package Architecture

- **`rover_slam`** (Python ROS 2 Package):
  - **`encoder_ticks_to_odom.py`**: Reads raw 4-wheel encoder telemetry and calculates differential drive odometry.
  - **`heuristic_slip_checker.py`**: Monitors wheel speed discrepancies against IMU/GPS to detect wheel slip on loose terrain.
  - **`ekf_validator.py`**: Evaluates Extended Kalman Filter state convergence.

## Documentation
Refer to [`docs/`](docs/) for full node & topic reference matrices and RTAB-Map integration guides.
