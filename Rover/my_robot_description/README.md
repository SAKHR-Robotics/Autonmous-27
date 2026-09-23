# Robot Description Package

A 4-wheeled Mars rover style robot for ROS 2 Humble with Gazebo Ignition simulation support.

## Quick Start

### Build Workspace
```bash
cd ~/Desktop/ROARTASK/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Launch Options

**Gazebo Simulation** (recommended for testing movement):
```bash
ros2 launch my_robot_description gazebo.launch.py
```

**RViz Visualization** (for model inspection only):
```bash
ros2 launch my_robot_description display.launch.py
```

### Control the Robot

**GUI Controller** (sliders for easy control):
```bash
python3 robot_teleop_gui.py
```

**Command Line** (manual velocity commands):
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

## Package Files

### Launch Files
- **`gazebo.launch.py`**: Spawns robot in Gazebo with physics simulation and differential drive control
- **`display.launch.py`**: Visualizes robot model in RViz without physics

### URDF/Xacro Files
- **`my_robot.urdf.xacro`**: Main robot description with 4-wheeled Mars rover structure and IMU sensor
- **`macros.xacro`**: Reusable macros for inertia calculations and suspension components
- **`materials.xacro`**: Color definitions (red, black, gray)
- **`gazebo.xacro`**: Gazebo-specific properties, differential drive plugin, and IMU sensor plugin

### Control
- **`robot_teleop_gui.py`**: GUI with sliders for linear/angular velocity control (publishes to `/cmd_vel`)

## Validation
```bash
xacro src/my_robot_description/urdf/my_robot.urdf.xacro > /tmp/my_robot.urdf
check_urdf /tmp/my_robot.urdf
```

## Sensor Verification

### IMU Sensor
```bash
# Check IMU topic exists
ros2 topic list | grep imu

# View IMU data
ros2 topic echo /imu/data

# Check IMU frequency
ros2 topic hz /imu/data

# Run IMU verification script
python3 verify_imu.py
```

### Camera Sensor
```bash
# Check camera topics exist
ros2 topic list | grep camera

# View camera image data
ros2 topic echo /camera/image_raw

# View camera info data
ros2 topic echo /camera/camera_info

# Check camera frequency
ros2 topic hz /camera/image_raw

# Run camera verification script
python3 verify_camera.py
```

## Topics

**Published by Robot**:
- `/odom` - Odometry data (position, velocity)
- `/tf` - Transform tree
- `/joint_states` - Joint positions
- `/imu/data` - IMU sensor data (linear acceleration, angular velocity)
- `/camera/image_raw` - Camera image data (640x480 RGB)
- `/camera/camera_info` - Camera calibration information

**Subscribed by Robot**:
- `/cmd_vel` - Velocity commands (Twist messages)

## Robot Specifications

- **Base**: 0.4m × 0.4m × 0.15m (square, red)
- **Suspension Arms**: 4 gray rectangular links
- **Wheels**: 4 total (2 per side, radius=0.06m)
- **Drive System**: 4-wheel differential drive / skid-steer
- **Sensors**:
  - **IMU**: Positioned at base_link center, 100 Hz update rate
    - Linear acceleration noise: σ=0.01 m/s², bias=0.01 m/s²
    - Angular velocity noise: σ=0.001 rad/s, bias=0.0001 rad/s
  - **Camera**: Positioned on front face of base_link, centered, pitched down 15°
    - Resolution: 640×480 pixels
    - Field of view: 60° horizontal (1.047 radians)
    - Update rate: 30 Hz
    - Image format: RGB8
    - Position: Front face of robot (x=0.215m from base center)
- **Total Links**: 13 (base_footprint, base_link, imu_link, camera_link, my_robot/camera_link/camera, 4 arms, 4 wheels)
- **Total Joints**: 12 (1 fixed footprint, 1 fixed IMU, 1 fixed camera, 1 fixed camera optical, 4 fixed arms, 4 continuous wheels)
