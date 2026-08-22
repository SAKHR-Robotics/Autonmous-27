# Jazzy Porting Guide & To-Do Checklist (`JazzyToDo.md`)

This document provides a comprehensive breakdown of:
1. **What changes can be directly merged from `main`** (models, worlds, scripts, teleop fixes).
2. **What edits MUST be adapted specifically for ROS 2 Jazzy / Gazebo Harmonic**.
3. **Important pitfalls & troubleshooting details** (e.g. bash `&` splitting, process termination trap, model URI lookup).

---

## 1. Quick Summary of What to Pull vs. What to Edit

| Item / Feature | Action for `jazzy-port` | Details |
| :--- | :--- | :--- |
| **`worlds/models/`** | ✅ **Directly Pull from `main`** | All `aruco/` (1–15), `rocks/` (1–9), and `mars_yard/` models are 100% compatible. |
| **`worlds/worlds/final_world_RA.world`** | 🟡 **Pull + Adapt Plugins** | Rock and ArUco placements are identical, but world `<plugin>` tags need Jazzy naming. |
| **`testing/LunchWorld&Rover`** | ✅ **Directly Pull from `main`** | Already contains adaptive `$ROS_DISTRO` detection and automatic full cleanup traps. |
| **`Rover/.../teleop_gui.py`** | ✅ **Directly Pull from `main`** | 10 Hz continuous velocity stream loop and clean shutdown handling. |
| **`my_robot_description/urdf/gazebo.xacro`** | 🔴 **Adapt for Jazzy** | Keep the 6-wheel DiffDrive configuration, but update plugin library/class names to `gz::sim`. |
| **`worlds/CMakeLists.txt`** | ✅ **Directly Pull from `main`** | `install(DIRECTORY data models DESTINATION share/${PROJECT_NAME})`. |
| **`gazebo.launch.py` & `launch_map.launch.py`** | 🟡 **Pull + Verify Bridges** | Model paths are included; ensure bridge message types use `gz.msgs.<Type>`. |

---

## 2. Detailed Breakdown: What Comes from `main`

These components are identical between distributions and can be checked out directly from `main` or merged:

### A. All Models (`worlds/models/`)
- `worlds/models/mars_yard/`
- `worlds/models/rocks/` (`rock_1` through `rock_9`)
- `worlds/models/aruco/` (`aruco_1` through `aruco_15`)

### B. Testing Launchers (`testing/LunchWorld&Rover` & `.sh`)
- Sources `/opt/ros/jazzy/setup.bash` dynamically if running on Jazzy.
- Exports both `GZ_SIM_RESOURCE_PATH` and `IGN_GAZEBO_RESOURCE_PATH` to find models in both source and install locations.
- **Trap Cleanup**: Automatically kills all lingering Gazebo, parameter bridge, robot state publisher, and ROS 2 nodes upon exit.
- Uses quoted `"world:=final_world_RA.world"` to prevent bash `&` background operator splitting.

### C. Teleoperation GUI Fix (`Rover/my_robot_description/scripts/teleop_gui.py`)
- Continuous **10 Hz ROS 2 timer loop** streaming `/cmd_vel` so Gazebo doesn't time out.
- Handled `ExternalShutdownException` and `KeyboardInterrupt` for clean exit without python tracebacks.

---

## 3. Detailed Breakdown: What Needs Custom Edits for Jazzy

When working on the **`jazzy-port`** branch, make the following specific adaptations:

---

### Step 1: Update URDF Gazebo Plugins (`my_robot_description/urdf/gazebo.xacro`)

Ensure **all 6 wheels** remain driven in skid-steer mode, but change the plugin namespace to `gz::sim::systems::*`:

```xml
  <!-- =================================================== -->
  <!-- 6-Wheel Differential / Skid-Steer Drive Plugin (Jazzy) -->
  <!-- =================================================== -->
  <gazebo>
    <plugin filename="libgz-sim-diff-drive-system.so" name="gz::sim::systems::DiffDrive">
      <!-- 6-Wheel Joint Bindings -->
      <left_joint>left_front_wheel_joint</left_joint>
      <left_joint>left_middle_wheel_joint</left_joint>
      <left_joint>left_rear_wheel_joint</left_joint>
      <right_joint>right_front_wheel_joint</right_joint>
      <right_joint>right_middle_wheel_joint</right_joint>
      <right_joint>right_rear_wheel_joint</right_joint>
      
      <!-- Dimensions -->
      <wheel_separation>0.49</wheel_separation>
      <wheel_radius>0.06</wheel_radius>
      
      <!-- Torque & Limits -->
      <max_linear_acceleration>5.0</max_linear_acceleration>
      <max_angular_acceleration>5.0</max_angular_acceleration>
      <max_velocity>2.0</max_velocity>
      <min_velocity>-2.0</min_velocity>
      <max_wheel_torque>50</max_wheel_torque>
      
      <!-- Topic -->
      <topic>cmd_vel</topic>
      
      <!-- Odometry -->
      <odom_publish_frequency>50</odom_publish_frequency>
      <odom_topic>odom</odom_topic>
      <tf_topic>tf</tf_topic>
      <frame_id>odom</frame_id>
      <child_frame_id>base_footprint</child_frame_id>
    </plugin>
  </gazebo>

  <!-- =================================================== -->
  <!-- Other Gazebo System Plugins (Jazzy)                 -->
  <!-- =================================================== -->
  <gazebo>
    <plugin filename="libgz-sim-joint-state-publisher-system.so" name="gz::sim::systems::JointStatePublisher" />
    <plugin filename="libgz-sim-imu-system.so" name="gz::sim::systems::Imu" />
    <plugin filename="libgz-sim-sensors-system.so" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </gazebo>
```

---

### Step 2: Update World System Plugins (`worlds/worlds/final_world_RA.world`)

For pure ROS 2 Jazzy with Gazebo Harmonic, update the system plugin headers at the top of `final_world_RA.world`:

```xml
  <!-- World System Plugins for Gazebo Harmonic / Jazzy -->
  <plugin filename="libgz-sim-physics-system.so" name="gz::sim::systems::Physics" />
  <plugin filename="libgz-sim-user-commands-system.so" name="gz::sim::systems::UserCommands" />
  <plugin filename="libgz-sim-scene-broadcaster-system.so" name="gz::sim::systems::SceneBroadcaster" />
  <plugin filename="libgz-sim-contact-system.so" name="gz::sim::systems::Contact" />
  <plugin filename="libgz-sim-sensors-system.so" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
```

*(All rock and ArUco `<include><uri>model://...</uri></include>` placements underneath remain identical).*

---

### Step 3: Verify ROS-Gazebo Topic Bridges (`gazebo.launch.py`)

Verify that the bridge definitions in `Rover/my_robot_description/launch/gazebo.launch.py` use the `gz.msgs` namespace:

```python
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/image', '/camera/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/camera_info', '/camera/camera_info'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/depth_image', '/camera/depth/image_raw'),
            (f'/world/{world_name}/model/my_robot/link/camera_link/sensor/camera/points', '/camera/depth/color/points'),
        ],
        output='screen'
    )
```

---

## 4. Git Workflow to Apply to `jazzy-port`

```bash
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws

# 1. Switch to jazzy-port
git checkout jazzy-port

# 2. Grab the shared models, worlds, and scripts from main (or current working copy)
git checkout main -- worlds/models/ \
                    worlds/worlds/final_world_RA.world \
                    worlds/CMakeLists.txt \
                    testing/LunchWorld\&Rover \
                    testing/LunchWorld\&Rover.sh \
                    Rover/my_robot_description/scripts/teleop_gui.py

# 3. Apply the Jazzy plugin naming edits in gazebo.xacro and final_world_RA.world

# 4. Rebuild the workspace on your Jazzy environment
cd /home/saif/Desktop/MESEKET/Autonmous-27
colcon build --symlink-install

# 5. Test using the script
cd Autonmous_Ws/testing
./"LunchWorld&Rover"

# 6. Commit and push
cd /home/saif/Desktop/MESEKET/Autonmous-27/Autonmous_Ws
git add .
git commit -m "Complete Jazzy port: 6-wheel diff drive, final_world_RA, models, and teleop"
git push origin jazzy-port
```
