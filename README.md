# Autonomous Rover Simulation Workspace

This ROS 2 workspace contains packages for simulating and controlling the autonomous rover in the Mars Yard environment.

## Directory Structure

*   **[Rover/my_robot_description](Rover/my_robot_description)** - URDF model description, configurations, and spawn/launch scripts for the test rover.
*   **[worlds](worlds)** - The environment package containing worlds (including `world1.world` and others), local models (such as `mars_yard` terrain and `rocks`), launch files, and costmap data.

---

## Setup & Compilation

To build the workspace without keeping the output build artifacts in your Git repository, compile from **outside** the `Autonmous_Ws` folder (at the repository root directory where you cloned it):

```bash
# Go to the root repository directory (outside Autonmous_Ws)
cd /path/to/your/cloned/repository

# Sourcing standard ROS 2 (Jazzy)
source /opt/ros/jazzy/setup.bash

# Build the workspace packages
colcon build

# Source this workspace
source install/setup.bash
```

---

## 1. Launching the Rover into the World (One Command)

To launch the test rover spawned directly inside the Mars Yard simulation world, run the following single command:

```bash
ros2 launch my_robot_description gazebo.launch.py
```

> [!NOTE]
> By default, the launch file is configured to load `world1.world` (with rocks scaled up to 2.5x) and spawn the rover at a safe altitude (`z:=0.5`) to prevent collisions on startup.
>
> If you want to launch the rover in the **empty world** instead, run:
> ```bash
> ros2 launch my_robot_description gazebo.launch.py world:=empty_with_sensors.sdf
> ```

---

## 2. Launching the World Only (Without Rover)

To launch only the Mars Yard simulation world (with rocks) without spawning the rover, run:

```bash
ros2 launch worlds world1.launch.py
```

To launch the empty Mars Yard layout, run:
```bash
ros2 launch worlds launch_map.launch.py world:=marsyard.world
```

---

## Testing & Verification Guide

For a complete step-by-step tutorial on launching, driving the rover, starting the perception pipeline, and validating inputs/outputs in RViz2, refer to the local guide:
* **[testing_guide.md](General_Docu/testing_guide.md)**

---

## Git Workflow & Best Practices

To avoid conflicts and keep the repository clean, follow this workflow when working on the project:

### 1. Always Pull Before Editing
Before making any changes locally, pull the latest changes from the remote repository to ensure you are up-to-date:
```bash
git pull origin main
```

### 2. Make Your Edits
Perform your development, edit code files, and verify changes locally.

### 3. Stage and Commit Your Changes
Add your modified files (excluding ignored datasets and build outputs) and commit them with a descriptive message:
```bash
# Add modified/new files
git add .

# Commit with a clear note
git commit -m "Descriptive summary of the changes made"
```

### 4. Push to Remote
Publish your clean commits to the GitHub repository:
```bash
git push origin main
```
