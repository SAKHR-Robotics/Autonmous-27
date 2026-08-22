#!/usr/bin/env bash
# ==============================================================================
# Launch Final Mars Yard World (with Rocks & ArUco Markers), Rover, and Teleop
# ==============================================================================
# Script: LunchWorld&Rover
# World: final_world_RA.world / final_world_R&A.world
# Description: Directly launches Gazebo simulation with rover and the final
#              Mars Yard world containing both rocks and ArUco markers,
#              along with the continuous-stream Rover Teleop GUI.
#              Automatically cleans up ALL running Gazebo and ROS 2 processes
#              upon exit.
# ==============================================================================

set -e

# Determine script and project directory paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "========================================================================"
echo "🚀 Launching Rover, Final World (Rocks & ArUco) & Teleop GUI"
echo "========================================================================"
echo "📁 Workspace root: $WORKSPACE_ROOT"

# ==============================================================================
# Cleanup Function: Automatically terminates all Gazebo, ROS 2, and Teleop nodes
# ==============================================================================
cleanup() {
    echo ""
    echo "🛑 Cleaning up and terminating all Gazebo & ROS 2 simulation processes..."
    
    # 1. Terminate background Gazebo launch process
    if [ -n "$GZ_PID" ] && kill -0 "$GZ_PID" 2>/dev/null; then
        kill -SIGINT "$GZ_PID" 2>/dev/null || true
    fi
    
    # 2. Terminate background child processes
    pkill -P $$ 2>/dev/null || true
    
    # 3. Cleanly kill all lingering simulation nodes
    pkill -9 -f "ign gazebo" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    pkill -9 -f "robot_state_publisher" 2>/dev/null || true
    pkill -9 -f "teleop_gui.py" 2>/dev/null || true
    pkill -9 -f "ros_gz_sim" 2>/dev/null || true
    
    echo "✅ All processes terminated cleanly."
}
trap cleanup EXIT INT TERM

# Step 1: Source ROS 2 base installation
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "[1/4] Sourcing ROS 2 Humble (/opt/ros/humble/setup.bash)..."
        source /opt/ros/humble/setup.bash
    elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
        echo "[1/4] Sourcing ROS 2 Jazzy (/opt/ros/jazzy/setup.bash)..."
        source /opt/ros/jazzy/setup.bash
    else
        echo "[1/4] Searching for ROS 2 installation..."
        for ros_setup in /opt/ros/*/setup.bash; do
            if [ -f "$ros_setup" ]; then
                source "$ros_setup"
                break
            fi
        done
    fi
else
    echo "[1/4] Using active ROS 2 environment (ROS_DISTRO=$ROS_DISTRO)..."
fi

# Step 2: Source local workspace build
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    echo "[2/4] Sourcing workspace install overlay ($WORKSPACE_ROOT/install/setup.bash)..."
    source "$WORKSPACE_ROOT/install/setup.bash"
else
    echo "[2/4] Workspace install/setup.bash not found. Building workspace first..."
    cd "$WORKSPACE_ROOT"
    colcon build --symlink-install
    source "$WORKSPACE_ROOT/install/setup.bash"
fi

# Step 3: Configure Gazebo / Ignition model resource paths for both source and install locations
echo "[3/4] Configuring Gazebo resource paths for rocks, ArUco, and Mars Yard models..."
MODELS_DIR="$WORKSPACE_ROOT/Autonmous_Ws/worlds/models"
INSTALL_MODELS_DIR="$WORKSPACE_ROOT/install/worlds/share/worlds/models"

EXTRA_PATHS="$MODELS_DIR:$MODELS_DIR/rocks:$MODELS_DIR/aruco:$INSTALL_MODELS_DIR:$INSTALL_MODELS_DIR/rocks:$INSTALL_MODELS_DIR/aruco"

export GZ_SIM_RESOURCE_PATH="$EXTRA_PATHS:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="$EXTRA_PATHS:${IGN_GAZEBO_RESOURCE_PATH:-}"

# Step 4: Launch Gazebo Simulation in Background
echo "[4/4] Starting Gazebo simulation with rover and final_world_RA.world..."
# Use properly quoted world argument to prevent bash '&' splitting
ros2 launch my_robot_description gazebo.launch.py "world:=final_world_RA.world" "$@" &
GZ_PID=$!

# Wait briefly for ROS nodes and Gazebo to initialize
echo "⏳ Waiting for Gazebo simulation to initialize..."
sleep 4

# Launch Teleop GUI in Foreground
echo "🎮 Starting Rover Teleop GUI..."
if [ -f "$WORKSPACE_ROOT/Autonmous_Ws/Rover/my_robot_description/scripts/teleop_gui.py" ]; then
    python3 "$WORKSPACE_ROOT/Autonmous_Ws/Rover/my_robot_description/scripts/teleop_gui.py"
else
    ros2 run my_robot_description teleop_gui.py
fi

# When GUI is closed, cleanup trap automatically triggers
