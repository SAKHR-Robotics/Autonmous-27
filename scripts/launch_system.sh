#!/usr/bin/env bash
# ==============================================================================
# 🎛️ Master Interactive System Launcher: Autonomous-27 Rover
# ==============================================================================
# Unified control dashboard to launch any subsystem in Production or Testing mode,
# or execute clean system shutdowns.
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

show_menu() {
    clear
    echo "========================================================================"
    echo "🤖 AUTONOMOUS-27 ROVER: MASTER SYSTEM CONTROL DASHBOARD"
    echo "========================================================================"
    echo "📁 Workspace: $WORKSPACE_ROOT"
    echo "------------------------------------------------------------------------"
    echo "  [1] 🪐 Simulation: Mars Yard & Rover (Full / Teleop / World Only)"
    echo "  [2] 🧭 SLAM & State Estimation (Production / Standalone / EKF / RTAB)"
    echo "  [3] 👁️  Perception (Production / Standalone / Terrain / ArUco)"
    echo "  [4] 🗺️  Path Planning & Nav2 (Production / Standalone / Bridge)"
    echo "  [5] 🚀 Full System Stack (All nodes in Production sequence)"
    echo "  [6] 🛑 Clean Restart: Kill all zombie Gazebo, ROS 2 & RViz processes"
    echo "  [7] 🚪 Exit"
    echo "========================================================================"
}

while true; do
    show_menu
    read -p "Select module to launch [1-7]: " choice
    case "$choice" in
        1)
            echo ""
            "$SCRIPT_DIR/launch_sim.sh"
            break
            ;;
        2)
            echo ""
            "$SCRIPT_DIR/launch_slam.sh"
            break
            ;;
        3)
            echo ""
            "$SCRIPT_DIR/launch_perception.sh"
            break
            ;;
        4)
            echo ""
            "$SCRIPT_DIR/launch_planning.sh"
            break
            ;;
        5)
            echo ""
            echo "🚀 Launching Full Autonomous Stack in Production Sequence..."
            echo "Step 1: Simulation (Mars Yard + Rover)..."
            "$SCRIPT_DIR/launch_sim.sh" --mode prod &
            sleep 6
            echo "Step 2: SLAM & State Estimation..."
            "$SCRIPT_DIR/launch_slam.sh" --mode prod &
            sleep 4
            echo "Step 3: Perception..."
            "$SCRIPT_DIR/launch_perception.sh" --mode prod &
            sleep 3
            echo "Step 4: Path Planning & Nav2..."
            "$SCRIPT_DIR/launch_planning.sh" --mode prod
            break
            ;;
        6)
            echo ""
            echo "🛑 Killing zombie Gazebo, Ignition, ROS 2, and RViz processes..."
            pkill -9 -f gazebo 2>/dev/null || true
            pkill -9 -f ign 2>/dev/null || true
            pkill -9 -f gz 2>/dev/null || true
            pkill -9 -f ros 2>/dev/null || true
            pkill -9 -f rviz 2>/dev/null || true
            pkill -9 -f parameter_bridge 2>/dev/null || true
            pkill -9 -f robot_state_publisher 2>/dev/null || true
            pkill -9 -f teleop_gui 2>/dev/null || true
            echo "✅ Cleanup complete."
            read -p "Press [Enter] to return to menu..."
            ;;
        7)
            echo "👋 Exiting."
            exit 0
            ;;
        *)
            echo "Invalid choice. Please select 1-7."
            sleep 1
            ;;
    esac
done
