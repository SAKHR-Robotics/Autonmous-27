#!/usr/bin/env bash

# ==============================================================================
# 🚀 ERC 2026 Path Planner & MPPI Controller Master Test & Benchmark Runner
# ==============================================================================
# Interactive test launcher for testing the Path Planner (Smac Hybrid A*) and
# Controller (MPPI) with standalone Mock Rover kinematics, Mock Perception,
# real-time RViz path tracking, and automated metric reporting.
# ==============================================================================

set -e

# Project Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo "========================================================================"
echo "🧭 PATH PLANNER & MPPI CONTROLLER MASTER TESTING SUITE"
echo "========================================================================"
echo "Project Root: $PROJECT_ROOT"

# Step 1: Detect and Source ROS 2 Distro
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        echo "[INFO] Sourcing ROS 2 Jazzy..."
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "[INFO] Sourcing ROS 2 Humble..."
        source /opt/ros/humble/setup.bash
    else
        echo "[WARN] No standard /opt/ros/{humble,jazzy} found. Relying on active shell environment."
    fi
else
    echo "[INFO] Active ROS 2 Distro: $ROS_DISTRO"
fi

# Step 2: Build Workspace Packages
echo ""
echo "------------------------------------------------------------------------"
echo "📦 [1/3] Building packages (terrain_geometry_msgs, erc_path_planner, global_path_benchmarking)..."
echo "------------------------------------------------------------------------"
cd "$PROJECT_ROOT"
colcon build --packages-select terrain_geometry_msgs erc_path_planner global_path_benchmarking

# Step 3: Source Workspace Install
echo "------------------------------------------------------------------------"
echo "🔄 [2/3] Sourcing workspace overlay..."
echo "------------------------------------------------------------------------"
source "$PROJECT_ROOT/install/setup.bash"

# Step 4: Interactive Configuration Menu
echo "------------------------------------------------------------------------"
echo "📋 [3/3] Configure Test Session"
echo "------------------------------------------------------------------------"

# Check if arguments were passed directly from CLI (non-interactive shortcut)
if [ "$1" == "--headless" ] || [ "$1" == "-h" ] || [ "$1" == "--batch" ] || [ "$1" == "-b" ]; then
    MODE="2"
elif [ "$1" == "--verify" ] || [ "$1" == "-v" ]; then
    MODE="5"
elif [ -n "$1" ]; then
    MODE="4"
    CLI_SCENARIO="$1"
else
    echo "Select Testing Mode:"
    echo "  1) 🎮 Live Interactive Closed-Loop Test (Nav2 + Mock Rover + Live RViz)"
    echo "  2) ⚡ Headless Automated Batch Benchmark (Fast batch run across all 9 maps with full reports)"
    echo "  3) 📊 Visual Automated Batch Benchmark (Watch all 9 maps evaluated live in RViz)"
    echo "  4) 🎯 Single Scenario Benchmark (Headless or with RViz)"
    echo "  5) 🗺️  Verify Scenario Reference Maps & Paths (Generate PNG plots)"
    read -rp "Enter choice [1-5] (default: 1): " MODE
    MODE=${MODE:-1}
fi

SCENARIOS=(
    "empty_straight"
    "scattered_rocks_detour"
    "canyon_gate_passage"
    "marsyard_rough_slopes"
    "marsyard_labyrinth"
    "canyon_gate_blocked"
    "crater_field"
    "dead_end_trap"
    "snake_passage"
)

select_scenario() {
    echo ""
    echo "Available Test Scenarios / Maps:"
    for i in "${!SCENARIOS[@]}"; do
        printf "  %d) %s\n" "$((i+1))" "${SCENARIOS[$i]}"
    done
    read -rp "Select map/scenario [1-${#SCENARIOS[@]}] (default: 2 - scattered_rocks_detour): " SCENARIO_IDX
    SCENARIO_IDX=${SCENARIO_IDX:-2}
    SELECTED_SCENARIO="${SCENARIOS[$((SCENARIO_IDX-1))]}"
}

case "$MODE" in
    1)
        echo ""
        echo "=== 🎮 Live Interactive Closed-Loop Test ==="
        select_scenario

        read -rp "Enable Mock Rover Kinematics (50Hz Odom & dynamic TF)? [Y/n]: " USE_MOCK_ROVER
        USE_MOCK_ROVER=${USE_MOCK_ROVER:-Y}
        [[ "$USE_MOCK_ROVER" =~ ^[Yy]$ ]] && MOCK_ROVER_ARG="use_mock_rover:=true" || MOCK_ROVER_ARG="use_mock_rover:=false"

        read -rp "Enable Mock Perception Obstacles (/terrain/obstacle_features)? [Y/n]: " USE_MOCK_PERCEPTION
        USE_MOCK_PERCEPTION=${USE_MOCK_PERCEPTION:-Y}
        [[ "$USE_MOCK_PERCEPTION" =~ ^[Yy]$ ]] && MOCK_PERCEPTION_ARG="use_mock_perception:=true" || MOCK_PERCEPTION_ARG="use_mock_perception:=false"

        read -rp "Open RViz2 Live Visualizer? [Y/n]: " USE_RVIZ
        USE_RVIZ=${USE_RVIZ:-Y}
        [[ "$USE_RVIZ" =~ ^[Yy]$ ]] && RVIZ_ARG="use_rviz:=true" || RVIZ_ARG="use_rviz:=false"

        echo ""
        echo "========================================================================"
        echo "🚀 Launching Live Test: Scenario = $SELECTED_SCENARIO"
        echo "========================================================================"
        ros2 launch global_path_benchmarking live_test.launch.py \
            $MOCK_ROVER_ARG \
            $MOCK_PERCEPTION_ARG \
            $RVIZ_ARG
        ;;

    2)
        echo ""
        echo "=== ⚡ Headless Automated Batch Benchmark (Fast Mode) ==="
        read -rp "Clean stale test reports before running? [Y/n]: " CLEAN_REPORTS
        CLEAN_REPORTS=${CLEAN_REPORTS:-Y}
        [[ "$CLEAN_REPORTS" =~ ^[Yy]$ ]] && CLEAN_ARG="clean:=true" || CLEAN_ARG="clean:=false"

        echo "🚀 Launching headless benchmark across all 9 scenarios..."
        ros2 launch global_path_benchmarking benchmark.launch.py $CLEAN_ARG use_rviz:=false
        echo ""
        echo "========================================================================"
        echo "✅ Headless Benchmark Complete!"
        echo "📊 Evaluated both Global Planning (Smac) and Local Control (MPPI)."
        echo "📄 Interactive HTML Report: $SCRIPT_DIR/reports/numerical_report.html"
        echo "📄 Printable PDF Report:    $SCRIPT_DIR/reports/numerical_report.pdf"
        echo "📄 Markdown Summary:        $SCRIPT_DIR/reports/numerical_report.md"
        echo "========================================================================"
        ;;

    3)
        echo ""
        echo "=== 📊 Visual Automated Batch Benchmark (with RViz) ==="
        read -rp "Clean stale test reports before running? [Y/n]: " CLEAN_REPORTS
        CLEAN_REPORTS=${CLEAN_REPORTS:-Y}
        [[ "$CLEAN_REPORTS" =~ ^[Yy]$ ]] && CLEAN_ARG="clean:=true" || CLEAN_ARG="clean:=false"

        echo "🚀 Launching visual benchmark across all 9 scenarios with RViz..."
        ros2 launch global_path_benchmarking benchmark.launch.py $CLEAN_ARG use_rviz:=true
        echo ""
        echo "========================================================================"
        echo "✅ Visual Batch Benchmark Complete!"
        echo "📄 Interactive HTML Report: $SCRIPT_DIR/reports/numerical_report.html"
        echo "📄 Printable PDF Report:    $SCRIPT_DIR/reports/numerical_report.pdf"
        echo "========================================================================"
        ;;

    4)
        echo ""
        echo "=== 🎯 Single Scenario Benchmark ==="
        if [ -n "$CLI_SCENARIO" ]; then
            SELECTED_SCENARIO="$CLI_SCENARIO"
        else
            select_scenario
        fi

        read -rp "Open RViz2 live visualizer? [y/N]: " USE_RVIZ
        USE_RVIZ=${USE_RVIZ:-N}
        [[ "$USE_RVIZ" =~ ^[Yy]$ ]] && RVIZ_ARG="use_rviz:=true" || RVIZ_ARG="use_rviz:=false"

        echo "🚀 Benchmarking scenario: $SELECTED_SCENARIO..."
        ros2 launch global_path_benchmarking benchmark.launch.py scenario_id:="$SELECTED_SCENARIO" clean:=false $RVIZ_ARG
        ;;

    5)
        echo ""
        echo "=== 🗺️  Verify Scenario Reference Maps & Paths ==="
        echo "Generating PNG verification maps in reports/..."
        ros2 launch global_path_benchmarking benchmark.launch.py verify:=true
        echo "✅ Verification complete. Check $SCRIPT_DIR/reports/ for verify_scenario_*.png files."
        ;;

    *)
        echo "[ERROR] Invalid choice: $MODE. Exiting."
        exit 1
        ;;
esac

echo ""
echo "========================================================================"
echo "🏁 Session Finished."
echo "========================================================================"
