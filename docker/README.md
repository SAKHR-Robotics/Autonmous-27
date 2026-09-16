# Autonomous Rover Docker Guide

This directory contains the Docker configuration for running the **Autonomous Rover (Autonomous-27)** workspace in **ROS 2 Jazzy** with full GUI (RViz2 & Gazebo) and GPU hardware acceleration.

---

## 📋 1. Host Machine Setup (Prerequisites)

If Docker is not installed on your host Linux machine, run these commands once:

### Step A: Install Docker & Docker Compose
```bash
sudo apt update
sudo apt install -y docker.io docker-compose-v2 docker-buildx-plugin
```

### Step B: Enable & Start Docker Service
```bash
sudo systemctl enable --now docker
```

### Step C: Run Docker Without `sudo`
```bash
sudo usermod -aG docker $USER
newgrp docker
```

*(Optional: For NVIDIA GPU hardware acceleration in Gazebo/RViz)*
```bash
sudo apt install -y nvidia-container-toolkit
sudo systemctl restart docker
```

---

## 🚀 2. Building & Running the Container

Navigate to this `docker/` folder and start the container:

```bash
cd ~/Desktop/MESEKET/Autonmous-27/Autonmous_Ws/docker
./run_docker.sh
```

*(On your first run, Docker will automatically download base images and compile the environment. This takes ~3-5 minutes).*

---

## 💻 3. What to Run INSIDE the Container

Once the script starts, your terminal prompt will change to:
`root@<hostname>:/workspace#`

### ⚡ Useful Aliases & Shortcuts (Pre-configured)

The container comes with built-in shortcuts:

| Shortcut | What it does | Command it runs |
| :--- | :--- | :--- |
| `help` / `al` | **List all aliases** | Prints colorful cheatsheet of all shortcuts |
| `bld` | **Build workspace** | `colcon build --symlink-install` |
| `bldpkg <pkg>` | **Build single package** | `colcon build --symlink-install --packages-select <pkg>` |
| `bldclean` | **Clean build artifacts** | `rm -rf /workspace/build/* /workspace/install/* /workspace/log/*` |
| `sros` | **Source ROS & Workspace** | `source /opt/ros/jazzy/setup.bash && source install/setup.bash` |
| `sb` / `src` | **Reload shell** | `source ~/.bashrc` |
| `edital` | **Edit aliases** | `nano ~/.bash_aliases` |
| `sim` | **Launch Rover & World** | `bash /workspace/testing/LunchWorld\&Rover.sh` |
| `rviz` | **Launch RViz2** | `ros2 launch my_robot_description rviz_only.launch.py` |
| `teleop` | **Drive Rover** | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |

---

### Step 1: Build the Workspace
```bash
bld
```
*(or `colcon build --symlink-install`)*

### Step 2: Source the Workspace
```bash
sros
```
*(or `source install/setup.bash`)*


### Step 3: Launch Nodes & Simulation

#### Launch Rover in Mars Yard Simulation (Gazebo + Sensors):
```bash
ros2 launch my_robot_description gazebo.launch.py
```

#### Launch RViz Visualization:
```bash
ros2 launch my_robot_description rviz_only.launch.py
```

#### Launch Perception (Terrain Geometry):
```bash
ros2 launch terrain_geometry terrain.launch.py use_sim_time:=true
```

#### Launch SLAM & State Estimation (EKF + RTAB-Map):
```bash
ros2 launch rover_slam slam_bringup.launch.py use_sim_time:=true launch_static_tf:=false
```

#### Drive the Rover (Keyboard Teleoperation):
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 🪟 4. Opening Additional Terminals inside the Running Container

To run multiple ROS 2 nodes simultaneously in separate terminal windows, open a new terminal on your laptop, navigate to the `docker/` folder, and run:

```bash
cd ~/Desktop/MESEKET/Autonmous-27/Autonmous_Ws/docker
./enter_docker.sh
```

This immediately connects you to the existing running container!

---

## 📁 5. How This Docker Setup Works

1. **`Dockerfile`**: Builds the custom environment on top of `ros:jazzy-desktop-full`. Installs Nav2, RTAB-Map, Robot Localization, Gazebo (`ros_gz`), OpenCV, PCL tools, and Python dependencies.
2. **`docker-compose.yml`**: Defines runtime parameters:
   - **`network_mode: host`**: Uses host network so ROS 2 DDS topics are shared across host, container, and external devices (like Jetson Orin Nano).
   - **`volumes: - ..:/workspace:rw`**: Bind-mounts the root workspace directory into `/workspace`. Code edits on your host editor (VS Code, etc.) appear inside the container instantly.
   - **`volumes: - /workspace/build`, `install`, `log`**: Anonymous volumes that keep compiled binaries inside Docker storage, keeping your host Git repo 100% clean.
   - **`DISPLAY` & `/tmp/.X11-unix`**: Shares the host X11 display socket so RViz2 and Gazebo windows open directly on your monitor.
   - **`/dev/dri` & `NVIDIA_*`**: Passes GPU graphics hardware acceleration directly into the container.
3. **`run_docker.sh`**: Grants X11 window permissions (`xhost +local:root`) and boots the container interactively.
