#!/usr/bin/env bash

# ==============================================================================
# ERC 2026 Path Planner & Controller Automated Benchmark Runner
# ==============================================================================
# This script builds the workspace from the project root (/home/saif/Desktop/MESEKET/Autonmous-27),
# sources the environment, executes all 9 benchmark test scenarios against
# erc_path_planner (Smac Hybrid A* + MPPI Controller), and compiles visual HTML/PDF reports.
# ==============================================================================

set -e

# Define paths
PROJECT_ROOT="/home/saif/Desktop/MESEKET/Autonmous-27"
WS_PATH="$PROJECT_ROOT/Autonmous_Ws"

echo "========================================================================"
echo "🚀 Starting Automated Path Planner & Controller Benchmark Suite..."
echo "========================================================================"

# Step 1: Navigate to Project Root
echo "[1/4] Navigating to project root: $PROJECT_ROOT"
cd "$PROJECT_ROOT"

# Step 2: Build Workspace Packages
echo "[2/4] Building packages (global_path_benchmarking, erc_path_planner, terrain_geometry_msgs)..."
colcon build --packages-select global_path_benchmarking erc_path_planner terrain_geometry_msgs

# Step 3: Source Workspace
echo "[3/4] Sourcing ROS 2 environment..."
source "$PROJECT_ROOT/install/setup.bash"

# Step 4: Execute Benchmark Suite
echo "[4/4] Executing Benchmark Scenarios..."
SCENARIO_ARG=""
if [ -n "$1" ]; then
    echo "[INFO] Running single scenario: $1"
    SCENARIO_ARG="scenario_id:=$1"
fi

ros2 launch global_path_benchmarking benchmark.launch.py clean:=true $SCENARIO_ARG

echo "========================================================================"
echo "✅ Benchmarking Complete!"
echo "Reports generated in: $WS_PATH/testing/PathPlanner/Global_path_benchmarking/reports/"
echo "========================================================================"
