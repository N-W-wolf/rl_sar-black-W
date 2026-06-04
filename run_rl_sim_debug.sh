#!/bin/bash
set -euo pipefail

if [[ -z "${ROS_DISTRO:-}" ]]; then
    echo "[ERROR] ROS environment not detected. Please source /opt/ros/<distro>/setup.bash first."
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

set +u
source install/setup.bash
set -u

RNAME="${1:-blackW}"
POLICY_CONFIG="${2:-himloco}"

PARAM_PID=""

cleanup() {
    if [[ -n "${PARAM_PID}" ]] && kill -0 "${PARAM_PID}" 2>/dev/null; then
        kill "${PARAM_PID}" 2>/dev/null || true
        wait "${PARAM_PID}" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

echo "[INFO] Starting parameter_blackboard for robot_name=${RNAME}, policy_config=${POLICY_CONFIG}"
ros2 run demo_nodes_cpp parameter_blackboard --ros-args \
    -p robot_name:="${RNAME}" \
    -p gazebo_model_name:="${RNAME}_gazebo" \
    -p policy_config:="${POLICY_CONFIG}" &
PARAM_PID=$!

sleep 1

echo "[INFO] Starting rl_sim in foreground. Keyboard input stays on this terminal."
echo "[INFO] Press Ctrl-C to stop rl_sim and clean up parameter_blackboard."
ros2 run rl_sar rl_sim --ros-args -p policy_config:="${POLICY_CONFIG}"
