#!/usr/bin/env bash
# ==============================================================================
# 👁️ Perception Subsystem Launcher (Modular & Interactive)
# ==============================================================================
# Usage:
#   ./scripts/launch_perception.sh                     # Interactive Menu
#   ./scripts/launch_perception.sh --mode prod         # Production (costmap bypass ON, RViz OFF)
#   ./scripts/launch_perception.sh --mode test         # Standalone (map anchor ON, costmap ON, RViz ON)
#   ./scripts/launch_perception.sh --node terrain      # Terrain geometry only
#   ./scripts/launch_perception.sh --node marker       # ArUco marker detection only
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

MODE=""
NODE="full"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode|-m)
            MODE="$2"
            shift 2
            ;;
        --node|-n)
            NODE="$2"
            shift 2
            ;;
        *)
            shift
            ;;
    esac
done

if [ -z "$MODE" ] && [ "$NODE" = "full" ]; then
    echo "========================================================================"
    echo "👁️ PERCEPTION SUBSYSTEM LAUNCHER"
    echo "========================================================================"
    echo "Select Operating Mode or Specific Pipeline:"
    echo "  [1] Production (Unified Terrain & ArUco, costmap bypass ON for Nav2)"
    echo "  [2] Standalone Testing (Static map anchor ON, 2D costmap ON, RViz ON)"
    echo "  [3] Terrain Geometry Only (Rock clustering & bounding boxes)"
    echo "  [4] ArUco Marker Detection & Tracking Only"
    echo "  [5] ArUco Action Interface Node Only (/perception/aruco_pose publisher)"
    read -p "Choose option [1-5] (default 1): " choice
    case "$choice" in
        2) MODE="test" ;;
        3) NODE="terrain" ;;
        4) NODE="marker" ;;
        5) NODE="action" ;;
        *) MODE="prod" ;;
    esac
fi

case "$NODE" in
    terrain)
        echo "🪨 Starting Terrain Geometry Pipeline only..."
        ros2 launch terrain_geometry terrain.launch.py use_sim_time:=true enable_costmap:=false
        ;;
    marker)
        echo "🏷️ Starting ArUco Marker Branch only..."
        ros2 launch marker_detection marker_branch.launch.py use_sim_time:=true
        ;;
    action)
        echo "🎯 Starting Marker Action Interface Node only..."
        ros2 run marker_detection marker_action_interface --ros-args -p use_sim_time:=true
        ;;
    full|*)
        if [ "$MODE" = "test" ]; then
            echo "👁️ Launching Perception in STANDALONE TESTING mode..."
            ros2 launch terrain_geometry test_perception_standalone.launch.py use_sim_time:=true
        else
            echo "👁️ Launching Perception in PRODUCTION mode (Clean defaults for Nav2)..."
            ros2 launch terrain_geometry perception_system.launch.py \
                use_sim_time:=true \
                enable_costmap:=false \
                launch_rviz:=false
        fi
        ;;
esac
