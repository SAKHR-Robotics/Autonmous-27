# ROS 2 Humble (Ignition) $\leftrightarrow$ ROS 2 Jazzy (Gazebo) Migration Guide

This directory contains rules and instructions for future AI models or developers to seamlessly convert the workspace packages between **ROS 2 Humble (using Ignition Gazebo)** and **ROS 2 Jazzy (using Gazebo Harmonic/Ionic)**.

---

## Quick Reference: Key Differences

| Feature | ROS 2 Humble / Ignition | ROS 2 Jazzy / Gazebo |
| :--- | :--- | :--- |
| **Command Line Tool** | `ign gazebo` | `gz sim` |
| **Bridge Prefix** | `ignition.msgs.<Type>` | `gz.msgs.<Type>` |
| **Bridge Package** | `ros_gz_bridge` (or `ros_ign_bridge`) | `ros_gz_bridge` |
| **Resource Path Env** | `IGN_GAZEBO_RESOURCE_PATH` | `GZ_SIM_RESOURCE_PATH` |
| **System Plugins Env** | `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` | `GZ_SIM_SYSTEM_PLUGIN_PATH` |

---

## File-by-File Changes Required

Follow these exact steps to transition the workspace between the two distributions:

### 1. URDF Plugins Config (`my_robot_description/urdf/gazebo.xacro`)
Update the `<plugin>` tags inside the `<gazebo>` blocks.

#### ➡️ Moving to ROS 2 Jazzy (Gazebo)
Change the filenames and names of the sensor and diff-drive plugins:
*   **Sensors:**
    ```xml
    - <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
    + <plugin filename="libgz-sim-sensors-system.so" name="gz::sim::systems::Sensors">
    ```
*   **Diff-Drive:**
    ```xml
    - <plugin filename="libignition-gazebo-diff-drive-system.so" name="ignition::gazebo::systems::DiffDrive">
    + <plugin filename="libgz-sim-diff-drive-system.so" name="gz::sim::systems::DiffDrive">
    ```

#### ⬅️ Moving to ROS 2 Humble (Ignition)
Revert the namespace and shared object library name:
*   **Sensors:**
    ```xml
    - <plugin filename="libgz-sim-sensors-system.so" name="gz::sim::systems::Sensors">
    + <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors">
    ```
*   **Diff-Drive:**
    ```xml
    - <plugin filename="libgz-sim-diff-drive-system.so" name="gz::sim::systems::DiffDrive">
    + <plugin filename="libignition-gazebo-diff-drive-system.so" name="ignition::gazebo::systems::DiffDrive">
    ```

---

### 2. World System Plugins (`worlds/worlds/*.world` SDF files)
Each world file includes system plugins for physics, rendering, contacts, and user interactions.

#### ➡️ Moving to ROS 2 Jazzy (Gazebo)
Change the plugin library filenames and XML class paths from `ignition-gazebo-...` to `gz-sim-...`:
```xml
- <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics" />
+ <plugin filename="libgz-sim-physics-system.so" name="gz::sim::systems::Physics" />

- <plugin filename="libignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands" />
+ <plugin filename="libgz-sim-user-commands-system" name="gz::sim::systems::UserCommands" />

- <plugin filename="libignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster" />
+ <plugin filename="libgz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster" />

- <plugin filename="libignition-gazebo-contact-system" name="ignition::gazebo::systems::Contact" />
+ <plugin filename="libgz-sim-contact-system.so" name="gz::sim::systems::Contact" />
```

#### ⬅️ Moving to ROS 2 Humble (Ignition)
Revert the system plugin library filenames and XML class paths back to `ignition-gazebo` prefixes.

---

### 3. Simulation Bridges (`worlds/launch/launch_map.launch.py`)
Update the clock and sensor message bridge configurations to use the correct namespace syntax.

#### ➡️ Moving to ROS 2 Jazzy (Gazebo)
*   **Clock Bridge:**
    ```python
    - arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    + arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    ```

#### ⬅️ Moving to ROS 2 Humble (Ignition)
*   **Clock Bridge:**
    ```python
    - arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    + arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']
    ```

---

### 4. Build System & Dependency Manifests (`package.xml`)
If you require distribution-specific packages:
*   In Humble, `ros_gz_sim` and `ros_gz_bridge` are used. Keep them in `<depend>ros_gz_sim</depend>` and `<depend>ros_gz_bridge</depend>`.
*   If transitioning to older ROS 2 distros, some systems may require `ros_ign_bridge`. For Jazzy, they should strictly target `ros_gz_bridge`.
