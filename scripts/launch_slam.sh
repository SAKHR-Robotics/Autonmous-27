#!/usr/bin/env bash
# ==============================================================================
# 🧭 SLAM & State Estimation Launcher (Modular & Interactive)
# ==============================================================================
# Usage:
#   ./scripts/launch_slam.sh                     # Interactive Menu
#   ./scripts/launch_slam.sh --mode prod         # Production (integrated with live perception)
#   ./scripts/launch_slam.sh --mode test         # Standalone (mock ArUco + costmap + RViz)
#   ./scripts/launch_slam.sh --node ekf          # Run EKF node only
#   ./scripts/launch_slam.sh --node rtabmap      # Run RTAB-Map SLAM only
#   ./scripts/launch_slam.sh --node slip         # Run slip checker only
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
    echo "🧭 SLAM & STATE ESTIMATION LAUNCHER"
    echo "========================================================================"
    echo "Select Operating Mode or Specific Node:"
    echo "  [1] Production SLAM Bringup (Integrated with real Perception & Nav2)"
    echo "  [2] Standalone SLAM Testing (Mock ArUco + standalone Costmap + RViz)"
    echo "  [3] EKF State Estimation Only (launch/ekf.launch.py)"
    echo "  [4] RTAB-Map SLAM Only (launch/rtabmap.launch.py)"
    echo "  [5] Heuristic Slip Checker Only (rover_slam/heuristic_slip_checker)"
    echo "  [6] Wheel Encoder Odometry Only (rover_slam/encoder_ticks_to_odom)"
    read -p "Choose option [1-6] (default 1): " choice
    case "$choice" in
        2) MODE="test" ;;
        3) NODE="ekf" ;;
        4) NODE="rtabmap" ;;
        5) NODE="slip" ;;
        6) NODE="ticks" ;;
        *) MODE="prod" ;;
    esac
fi

case "$NODE" in
    ekf)
        echo "🧭 Starting EKF State Estimation Node only..."
        ros2 launch rover_slam ekf.launch.py use_sim_time:=true
        ;;
    rtabmap)
        echo "🗺️ Starting RTAB-Map SLAM only..."
        ros2 launch rover_slam rtabmap.launch.py use_sim_time:=true
        ;;
    slip)
        echo "🛞 Starting Heuristic Slip Checker only..."
        ros2 run rover_slam heuristic_slip_checker --ros-args -p use_sim_time:=true
        ;;
    ticks)
        echo "⚙️ Starting Wheel Encoder Ticks Odometry only..."
        ros2 run rover_slam encoder_ticks_to_odom --ros-args -p use_sim_time:=true
        ;;
    full|*)
        if [ "$MODE" = "test" ]; then
            echo "🧭 Launching SLAM in STANDALONE TESTING mode..."
            ros2 launch rover_slam test_slam_standalone.launch.py use_sim_time:=true
        else
            echo "🧭 Launching SLAM in PRODUCTION mode (Clean defaults for Nav2)..."
            ros2 launch rover_slam slam_bringup.launch.py \
                use_sim_time:=true \
                launch_aruco_stub:=false \
                launch_costmap:=false \
                launch_costmap_stub:=false \
                launch_rviz:=false
        fi
        ;;
esac
