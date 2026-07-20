#!/usr/bin/env bash
# Usage: arm_two_uav_sim.sh [uav1-id] [uav2-id] [settle-seconds]
# Simulation only: arm both vehicles, then enter COMMAND_CONTROL.  Prometheus
# commands its configured initial-position hover, so the coordinator waits for
# both aircraft to reach its fly_height gate before coverage begins.
set -euo pipefail
[[ $# -le 3 ]] || { echo "usage: $0 [uav1-id] [uav2-id] [settle-seconds]" >&2; exit 2; }

workspace=${PROMETHEUS_WS:-"$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"}
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"

uav1_id=${1:-1}
uav2_id=${2:-2}
settle_seconds=${3:-0.5}
[[ "$uav1_id" != "$uav2_id" ]] || { echo "UAV IDs must differ" >&2; exit 2; }

setup_topic() { printf '/uav%s/prometheus/setup' "$1"; }
require_topic() {
  local topics
  topics="$(rostopic list)"
  grep -Fxq "$(setup_topic "$1")" <<<"$topics" || {
    echo "missing $(setup_topic "$1"); start start_two_uav_sim.sh first" >&2
    exit 1
  }
}
arm() {
  rostopic pub -1 "$(setup_topic "$1")" prometheus_msgs/UAVSetup \
    '{cmd: 0, arming: true, px4_mode: "", control_state: ""}'
}
command_control() {
  rostopic pub -1 "$(setup_topic "$1")" prometheus_msgs/UAVSetup \
    '{cmd: 3, arming: true, px4_mode: "", control_state: "COMMAND_CONTROL"}'
}
wait_connected() {
  local id=$1 state deadline
  deadline=$((SECONDS + 30))
  while (( SECONDS < deadline )); do
    state="$(timeout 2s rostopic echo -n 1 "/uav${id}/prometheus/state" 2>/dev/null || true)"
    if grep -Eq '^connected: (True|true)$' <<<"$state" &&
       grep -Eq '^odom_valid: (True|true)$' <<<"$state"; then
      return 0
    fi
    sleep 0.5
  done
  echo "UAV${id} is not connected with valid odometry; do not arm. Check PX4/MAVROS/Gazebo first." >&2
  return 1
}
wait_armed() {
  local id=$1 state deadline
  deadline=$((SECONDS + 15))
  while (( SECONDS < deadline )); do
    state="$(timeout 2s rostopic echo -n 1 "/uav${id}/prometheus/state" 2>/dev/null || true)"
    if grep -Eq '^armed: (True|true)$' <<<"$state"; then
      return 0
    fi
    sleep 0.5
  done
  echo "UAV${id} did not report armed=true within 15 seconds" >&2
  return 1
}

require_topic "$uav1_id"
require_topic "$uav2_id"
wait_connected "$uav1_id"
wait_connected "$uav2_id"
# rostopic -1 keeps each publisher alive for three seconds.  Run the two
# independent requests concurrently so that it does not delay takeoff by 6 s.
arm "$uav1_id" &
arm1_pid=$!
arm "$uav2_id" &
arm2_pid=$!
wait_armed "$uav1_id"
wait_armed "$uav2_id"
wait "$arm1_pid"
wait "$arm2_pid"
sleep "$settle_seconds"
command_control "$uav1_id" &
control1_pid=$!
command_control "$uav2_id" &
control2_pid=$!
wait "$control1_pid"
wait "$control2_pid"
echo "Both UAVs armed and switched to COMMAND_CONTROL; waiting for automatic initial-height hover and cooperative gate."
