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
if [ -d "$SCRIPT_DIR/../worlds" ]; then
    WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
    PROJECT_DIR="$WORKSPACE_ROOT"
elif [ -d "$SCRIPT_DIR/../../src/Autonmous-27/worlds" ]; then
    WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
    PROJECT_DIR="$WORKSPACE_ROOT/src/Autonmous-27"
else
    WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
    PROJECT_DIR="$WORKSPACE_ROOT"
fi

echo "========================================================================"
echo "🚀 Launching Rover, Final World (Rocks & ArUco) & Teleop GUI"
echo "========================================================================"
echo "📁 Workspace root: $WORKSPACE_ROOT"
echo "📁 Project dir:    $PROJECT_DIR"

# ==============================================================================
# Cleanup Function: Terminates ONLY processes owned by this launcher run
# ==============================================================================
GZ_PID=""
GZ_HAS_PGID=0

terminate_owned_group() {
    local sig="$1"
    if [ -n "$GZ_PID" ]; then
        if [ "$GZ_HAS_PGID" -eq 1 ]; then
            kill "-${sig}" -- "-${GZ_PID}" 2>/dev/null || true
        else
            kill "-${sig}" "${GZ_PID}" 2>/dev/null || true
        fi
    fi
}

cleanup() {
    echo ""
    echo "Stopping Gazebo and ROS 2 processes owned by this launcher..."

    if [ -n "$GZ_PID" ] && kill -0 "$GZ_PID" 2>/dev/null; then
        terminate_owned_group INT
        for _ in $(seq 1 25); do
            if ! kill -0 "$GZ_PID" 2>/dev/null; then
                break
            fi
            sleep 0.2
        done
        if kill -0 "$GZ_PID" 2>/dev/null; then
            terminate_owned_group TERM
            for _ in $(seq 1 15); do
                if ! kill -0 "$GZ_PID" 2>/dev/null; then
                    break
                fi
                sleep 0.2
            done
        fi
        if kill -0 "$GZ_PID" 2>/dev/null; then
            terminate_owned_group KILL
        fi
        wait "$GZ_PID" 2>/dev/null || true
    fi

    echo "Owned simulation processes stopped."
}
trap cleanup EXIT INT TERM

# Step 1: Source ROS 2 base installation (prefer Jazzy)
if [ -z "${ROS_DISTRO:-}" ]; then
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        echo "[1/4] Sourcing ROS 2 Jazzy (/opt/ros/jazzy/setup.bash)..."
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "[1/4] Sourcing ROS 2 Humble (/opt/ros/humble/setup.bash)..."
        source /opt/ros/humble/setup.bash
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
INSTALL_SETUP=""
if [ -f "/root/ros_build/install/setup.bash" ]; then
    INSTALL_SETUP="/root/ros_build/install/setup.bash"
elif [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    INSTALL_SETUP="$WORKSPACE_ROOT/install/setup.bash"
fi

if [ -n "$INSTALL_SETUP" ]; then
    echo "[2/4] Sourcing workspace install overlay ($INSTALL_SETUP)..."
    source "$INSTALL_SETUP"
else
    echo "[2/4] Workspace install/setup.bash not found. Building workspace first..."
    cd "$WORKSPACE_ROOT"
    colcon build --symlink-install
    if [ -f "/root/ros_build/install/setup.bash" ]; then
        INSTALL_SETUP="/root/ros_build/install/setup.bash"
    elif [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
        INSTALL_SETUP="$WORKSPACE_ROOT/install/setup.bash"
    fi
    if [ -n "$INSTALL_SETUP" ]; then
        source "$INSTALL_SETUP"
    fi
fi

# Step 3: Configure Gazebo / Ignition model resource paths for both source and install locations
echo "[3/4] Configuring Gazebo resource paths for rocks, ArUco, and Mars Yard models..."
MODELS_DIR="$PROJECT_DIR/worlds/models"
INSTALL_DIR="${INSTALL_SETUP%/*}"
INSTALL_MODELS_DIR="$INSTALL_DIR/worlds/share/worlds/models"

EXTRA_PATHS="$MODELS_DIR:$MODELS_DIR/rocks:$MODELS_DIR/aruco:$INSTALL_MODELS_DIR:$INSTALL_MODELS_DIR/rocks:$INSTALL_MODELS_DIR/aruco:/root/ros_build/install/worlds/share/worlds/models:$WORKSPACE_ROOT/install/worlds/share/worlds/models"

export GZ_SIM_RESOURCE_PATH="$EXTRA_PATHS:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="$EXTRA_PATHS:${IGN_GAZEBO_RESOURCE_PATH:-}"

# Optional world & TF flags (defaults preserve SLAM/EKF TF ownership)
WORLD_NAME="${WORLD_NAME:-final_world_R&A.world}"
PUBLISH_MAP_TF="${PUBLISH_MAP_TF:-false}"
PUBLISH_CAMERA_TF="${PUBLISH_CAMERA_TF:-false}"
BRIDGE_SIM_TF="${BRIDGE_SIM_TF:-false}"

# Step 4: Launch Gazebo Simulation in its own process group
echo "[4/4] Starting Gazebo simulation with rover and ${WORLD_NAME}..."
echo "TF Configuration: publish_map_tf=$PUBLISH_MAP_TF, publish_camera_tf=$PUBLISH_CAMERA_TF, bridge_sim_tf=$BRIDGE_SIM_TF"
echo "(Defaults preserve SLAM/Nav2 TF ownership; export BRIDGE_SIM_TF=true PUBLISH_MAP_TF=true for standalone teleop.)"
if command -v setsid >/dev/null 2>&1; then
    GZ_HAS_PGID=1
    setsid ros2 launch my_robot_description gazebo.launch.py \
        "world:=$WORLD_NAME" \
        "publish_map_tf:=$PUBLISH_MAP_TF" \
        "publish_camera_tf:=$PUBLISH_CAMERA_TF" \
        "bridge_sim_tf:=$BRIDGE_SIM_TF" \
        "$@" &
else
    GZ_HAS_PGID=0
    ros2 launch my_robot_description gazebo.launch.py \
        "world:=$WORLD_NAME" \
        "publish_map_tf:=$PUBLISH_MAP_TF" \
        "publish_camera_tf:=$PUBLISH_CAMERA_TF" \
        "bridge_sim_tf:=$BRIDGE_SIM_TF" \
        "$@" &
fi
GZ_PID=$!

# Wait briefly for ROS nodes and Gazebo to initialize
echo "⏳ Waiting for Gazebo simulation to initialize..."
sleep 4

# Launch Teleop GUI in Foreground
echo "🎮 Starting Rover Teleop GUI..."
if [ -f "$PROJECT_DIR/Rover/my_robot_description/scripts/teleop_gui.py" ]; then
    python3 "$PROJECT_DIR/Rover/my_robot_description/scripts/teleop_gui.py"
else
    ros2 run my_robot_description teleop_gui.py
fi

# When GUI is closed, cleanup trap automatically triggers
