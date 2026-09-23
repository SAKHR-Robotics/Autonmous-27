#!/usr/bin/env bash
# ==============================================================================
# 🗺️ Path Planning & Navigation Launcher (Modular & Interactive)
# ==============================================================================
# Usage:
#   ./scripts/launch_planning.sh                     # Interactive Menu
#   ./scripts/launch_planning.sh --mode prod         # Production (with live SLAM map)
#   ./scripts/launch_planning.sh --mode test         # Standalone (dummy map + static anchor)
#   ./scripts/launch_planning.sh --node bridge       # Costmap bridge node only
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
    echo "🗺️ PATH PLANNING & NAV2 LAUNCHER"
    echo "========================================================================"
    echo "Select Operating Mode or Specific Component:"
    echo "  [1] Production Navigation (Integrated with live SLAM /map & Perception)"
    echo "  [2] Standalone Navigation Testing (dummy_map.yaml + static map anchor)"
    echo "  [3] Costmap Perception Bridge Node Only (Dense 3D PointCloud publisher)"
    echo "  [4] Smac Global Planner Server Only"
    echo "  [5] MPPI Local Controller Server Only"
    read -p "Choose option [1-5] (default 1): " choice
    case "$choice" in
        2) MODE="test" ;;
        3) NODE="bridge" ;;
        4) NODE="planner" ;;
        5) NODE="controller" ;;
        *) MODE="prod" ;;
    esac
fi

case "$NODE" in
    bridge)
        echo "🌉 Starting Costmap Perception Bridge Node only..."
        ros2 run erc_path_planner costmap_bridge_node --ros-args -p use_sim_time:=true
        ;;
    planner)
        echo "📐 Starting Smac Planner Server only..."
        ros2 run nav2_planner planner_server --ros-args --params-file "$WORKSPACE_ROOT/PathPlanning/erc_path_planner/config/nav2_params.yaml" -p use_sim_time:=true
        ;;
    controller)
        echo "🎮 Starting MPPI Controller Server only..."
        ros2 run nav2_controller controller_server --ros-args --params-file "$WORKSPACE_ROOT/PathPlanning/erc_path_planner/config/nav2_params.yaml" -p use_sim_time:=true
        ;;
    full|*)
        if [ "$MODE" = "test" ]; then
            echo "🗺️ Launching Nav2 Stack in STANDALONE TESTING mode (dummy map)..."
            ros2 launch erc_path_planner test_planner_standalone.launch.py use_sim_time:=true
        else
            echo "🗺️ Launching Nav2 Stack in PRODUCTION mode (RTAB-Map SLAM map)..."
            ros2 launch erc_path_planner path_planning.launch.py \
                use_slam:=true \
                use_sim_time:=true \
                autostart:=true
        fi
        ;;
esac
