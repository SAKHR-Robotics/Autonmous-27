#!/usr/bin/env bash
# Launch real production PathPlanner with test-owned dependencies.
# Build/source the Jazzy overlay first; see the mock package README.
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
    --batch|--verify|--external|-b|-v|-e|-h)
        echo "Legacy batch scoring is not a validated mission test." >&2
        echo "Use navigation_probe or planner_map_probe; see PathPlanning/VALIDATION.md." >&2
        exit 2
        ;;
    *)
        exec ros2 launch pathplanner_mock_tools mocked_planner.launch.py "$@"
        ;;
esac
