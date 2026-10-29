#!/usr/bin/env bash
# Run from any directory. Pass ROS launch arguments after --.
# Examples: --mode test --use-sim-time true -- launch_rviz:=false
#           --node ticks -- input_mode:=ticks_array
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
if [[ -z "${ROS_DISTRO:-}" && -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
fi
if [[ -f "$WORKSPACE_ROOT/install/setup.bash" ]]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
fi
MODE=""
NODE="full"
CLOCK=""
EXTRA=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --mode|-m|--node|-n|--use-sim-time)
            [[ $# -ge 2 ]] || { echo "Missing value for $1" >&2; exit 2; }
            case "$1" in
                --mode|-m) MODE="$2" ;;
                --node|-n) NODE="$2" ;;
                --use-sim-time) CLOCK="$2" ;;
            esac
            shift 2 ;;
        --) shift; EXTRA=("$@"); break ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
if [[ -z "$MODE" && "$NODE" == full ]]; then
    echo "1: Production SLAM   2: Gazebo/test wrapper (external sensors required)"
    echo "3: Local EKF         4: RTAB-Map         5: Slip filter   6: Encoders"
    read -r -p "Select [1-6], default 1: " choice
    case "$choice" in
        2) MODE=test ;;
        3) NODE=ekf ;;
        4) NODE=rtabmap ;;
        5) NODE=slip ;;
        6) NODE=ticks ;;
        *) MODE=prod ;;
    esac
fi
MODE="${MODE:-prod}"
[[ "$MODE" == prod || "$MODE" == test ]] || { echo "mode must be prod or test" >&2; exit 2; }
if [[ -z "$CLOCK" ]]; then
    CLOCK=false
    [[ "$MODE" == test ]] && CLOCK=true
fi
[[ "$CLOCK" == true || "$CLOCK" == false ]] || { echo "clock must be true or false" >&2; exit 2; }
PACKAGE=rover_slam
case "$NODE" in
    full)
        LAUNCH=slam_bringup.launch.py
        if [[ "$MODE" == test ]]; then PACKAGE=slam_test_tools; LAUNCH=test_slam_standalone.launch.py; fi ;;
    ekf) LAUNCH=ekf.launch.py ;;
    rtabmap) LAUNCH=rtabmap.launch.py ;;
    slip) LAUNCH=slip.launch.py ;;
    ticks|encoder) LAUNCH=encoder.launch.py ;;
    rviz) LAUNCH=rviz.launch.py ;;
    *) echo "Unknown node: $NODE" >&2; exit 2 ;;
esac
exec ros2 launch "$PACKAGE" "$LAUNCH" "use_sim_time:=$CLOCK" "${EXTRA[@]}"
