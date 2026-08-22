#!/usr/bin/env bash
# ==============================================================================
# Launch Mars Yard World (with ArUco Markers) and Rover in Gazebo Simulation
# ==============================================================================
# Script: LunchWorld&Rover
# Description: Automatically sources ROS 2 environment, prepares workspace,
#              and launches the Mars Yard world containing ArUco markers along
#              with the autonomous rover model and ROS-Gazebo bridges.
# ==============================================================================

set -e

# Determine script and project directory paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "========================================================================"
echo "🚀 Launching Rover & Mars Yard World with ArUco Markers"
echo "========================================================================"
echo "📁 Workspace root: $WORKSPACE_ROOT"

# Step 1: Source ROS 2 base installation
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "[1/3] Sourcing ROS 2 Humble (/opt/ros/humble/setup.bash)..."
        source /opt/ros/humble/setup.bash
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        echo "[1/3] Sourcing ROS 2 Jazzy (/opt/ros/jazzy/setup.bash)..."
        source /opt/ros/jazzy/setup.bash
    else
        echo "[1/3] Searching for ROS 2 installation..."
        for ros_setup in /opt/ros/*/setup.bash; do
            if [ -f "$ros_setup" ]; then
                source "$ros_setup"
                break
            fi
        done
    fi
else
    echo "[1/3] Using active ROS 2 environment (ROS_DISTRO=$ROS_DISTRO)..."
fi

# Step 2: Source local workspace build
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    echo "[2/3] Sourcing workspace install overlay ($WORKSPACE_ROOT/install/setup.bash)..."
    source "$WORKSPACE_ROOT/install/setup.bash"
else
    echo "[2/3] Workspace install/setup.bash not found. Building workspace first..."
    cd "$WORKSPACE_ROOT"
    colcon build --symlink-install
    source "$WORKSPACE_ROOT/install/setup.bash"
fi

# Step 3: Launch Gazebo simulation with world_Rotated_Aruco.world & Rover
echo "[3/3] Launching simulation..."
echo "      - World: world_Rotated_Aruco.world (with ArUco markers)"
echo "      - Robot: my_robot (with RealSense D435i, IMU, diff-drive)"
echo "========================================================================"

exec ros2 launch my_robot_description gazebo.launch.py world:=world_Rotated_Aruco.world "$@"
