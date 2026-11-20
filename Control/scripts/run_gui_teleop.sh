#!/usr/bin/env bash
# Run the existing GUI and simulation from any working directory.
set -e
CONTROL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROJECT_DIR="$(cd "$CONTROL_DIR/../.." && pwd)"

if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
fi
if [[ -z "${ROS_DISTRO:-}" ]]; then
    echo "Source the ROS environment first. The rover simulation targets Jazzy/Harmonic." >&2
    exit 2
fi
if [[ -f "$PROJECT_DIR/install/setup.bash" ]]; then
    source "$PROJECT_DIR/install/setup.bash"
elif [[ -f "$PROJECT_DIR/Autonmous_Ws/install/setup.bash" ]]; then
    source "$PROJECT_DIR/Autonmous_Ws/install/setup.bash"
fi
if ! gz sim --versions >/dev/null 2>&1; then
    echo "Gazebo Sim (gz sim) is missing. This rover targets Harmonic; this host may only have Fortress (ign gazebo)." >&2
    echo "Use the project's Jazzy/Harmonic environment; do not mix ROS distributions in one shell." >&2
    exit 2
fi
exec ros2 launch "$CONTROL_DIR/launch/gui_teleop.launch.py" "$@"
