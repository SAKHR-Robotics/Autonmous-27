#!/usr/bin/env bash
# Launch real production PathPlanner with test-owned dependencies or run probes.
# Build/source the Jazzy overlay first; see the package README.
set -euo pipefail

if [ "${ROS_DISTRO:-}" != "jazzy" ]; then
    echo "Run inside autonomous27:jazzy with the built Jazzy overlay sourced." >&2
    exit 1
fi

case "${1:-}" in
    --headless)
        shift
        exec ros2 launch pathplanner_mock_tools mocked_planner.launch.py use_rviz:=false "$@"
        ;;
    --benchmark)
        shift
        exec ros2 run global_path_benchmarking scenario_benchmark_runner "$@"
        ;;
    --map-probe)
        shift
        exec ros2 run global_path_benchmarking planner_map_probe "$@"
        ;;
    --verify|-b|-v|-e|-h|--clean)
        echo "Obsolete batch/cleanup flags are disabled." >&2
        echo "Use --headless, --benchmark --output-dir /validation/pp, or --map-probe." >&2
        exit 2
        ;;
    *)
        exec ros2 launch pathplanner_mock_tools mocked_planner.launch.py "$@"
        ;;
esac
