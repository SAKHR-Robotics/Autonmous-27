#!/usr/bin/env bash
# ==============================================================================
# 🚀 Simulation Launcher: Mars Yard World & Rover (Modular & Interactive)
# ==============================================================================
# Usage:
#   ./scripts/launch_sim.sh                     # Interactive Menu
#   ./scripts/launch_sim.sh --mode prod         # Production (clean TF, for SLAM/Nav2)
#   ./scripts/launch_sim.sh --mode test         # Standalone testing with Teleop GUI
#   ./scripts/launch_sim.sh --world marsyard    # Specific world
#   ./scripts/launch_sim.sh --target world_only # Launch world without spawning rover
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source ROS 2 environment
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
fi

# Source workspace install
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
fi

# Configure Gazebo model paths
MODELS_DIR="$WORKSPACE_ROOT/worlds/models"
EXTRA_PATHS="$WORKSPACE_ROOT/worlds:$MODELS_DIR:$MODELS_DIR/rocks:$MODELS_DIR/aruco:$WORKSPACE_ROOT/install/worlds/share/worlds/models"
export GZ_SIM_RESOURCE_PATH="$EXTRA_PATHS:${GZ_SIM_RESOURCE_PATH:-}"
export IGN_GAZEBO_RESOURCE_PATH="$EXTRA_PATHS:${IGN_GAZEBO_RESOURCE_PATH:-}"

# Parse CLI arguments
MODE=""
WORLD="world1.world"
TARGET="full"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode|-m)
            MODE="$2"
            shift 2
            ;;
        --world|-w)
            WORLD="$2"
            shift 2
            ;;
        --target|-t)
            TARGET="$2"
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done

# If no mode specified, prompt interactively
if [ -z "$MODE" ]; then
    echo "========================================================================"
    echo "🪐 MARS YARD SIMULATION LAUNCHER"
    echo "========================================================================"
    echo "Select Operating Mode:"
    echo "  [1] Production (Clean defaults: Gazebo TF bridge OFF, ready for SLAM/EKF)"
    echo "  [2] Standalone Teleop (Gazebo TF bridge ON, static map TF ON, Teleop GUI)"
    echo "  [3] World Only (Launch simulation world without rover)"
    echo "  [4] Teleop GUI Only (Connect to existing simulation)"
    read -p "Choose option [1-4] (default 1): " choice
    case "$choice" in
        2) MODE="test" ;;
        3) MODE="prod"; TARGET="world_only" ;;
        4) TARGET="teleop_only" ;;
        *) MODE="prod" ;;
    esac
fi

# Execute selection
case "$TARGET" in
    world_only)
        echo "🌍 Launching Mars Yard world only (world: $WORLD)..."
        ros2 launch worlds world1.launch.py
        ;;
    teleop_only)
        echo "🎮 Launching Teleop GUI only..."
        ros2 run my_robot_description teleop_gui.py
        ;;
    full|*)
        if [ "$MODE" = "test" ]; then
            echo "🚀 Launching Rover Simulation in STANDALONE TELEOP mode..."
            ros2 launch my_robot_description gazebo_with_teleop.launch.py \
                world:="$WORLD" \
                publish_map_tf:=true \
                publish_camera_tf:=true \
                bridge_sim_tf:=true
        else
            echo "🚀 Launching Rover Simulation in PRODUCTION mode (Clean TF for EKF/SLAM)..."
            ros2 launch my_robot_description gazebo.launch.py \
                world:="$WORLD" \
                publish_map_tf:=false \
                publish_camera_tf:=false \
                bridge_sim_tf:=false
        fi
        ;;
esac
