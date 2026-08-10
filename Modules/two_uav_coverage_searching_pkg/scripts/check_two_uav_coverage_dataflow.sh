#!/usr/bin/env bash
# Verify the live two-UAV coverage control chain after start_two_uav_coverage_sim.sh.
set -euo pipefail

[[ $# -le 2 ]] || { echo "usage: $0 [uav1-id] [uav2-id]" >&2; exit 2; }
uav1_id=${1:-1}
uav2_id=${2:-2}
workspace=${PROMETHEUS_WS:-"$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"}

source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash" --extend

rosnode list >/dev/null 2>&1 || {
  echo "ROS master is unavailable; run this in the same terminal/session as the simulation." >&2
  exit 1
}

check_stream() {
  local label=$1 topic=$2 attempt
  for attempt in 1 2 3; do
    if timeout 2s rostopic echo -n 1 "$topic" >/dev/null 2>&1; then
      printf 'OK   %-30s %s\n' "$label" "$topic"
      return 0
    fi
  done
  printf 'FAIL %-30s %s\n' "$label" "$topic" >&2
  return 1
}

check_controller() {
  local id=$1 info
  info="$(timeout 3s rostopic info "/uav${id}/prometheus/command" 2>/dev/null || true)"
  grep -Fq "/uav_control_main_${id}" <<<"$info" || {
    echo "FAIL UAV${id} controller does not subscribe to /uav${id}/prometheus/command" >&2
    return 1
  }
  echo "OK   UAV${id} controller subscriber"
}

status=0
for id in "$uav1_id" "$uav2_id"; do
  check_stream "UAV${id} state" "/uav${id}/prometheus/state" || status=1
  check_stream "UAV${id} control state" "/uav${id}/prometheus/control_state" || status=1
  check_stream "UAV${id} D435i source" "/uav${id}/camera/depth/color/points" || status=1
  check_stream "UAV${id} depth downsample" "/uav${id}/camera/depth/color/points_downsampled" || status=1
  check_stream "UAV${id} OctoMap cloud" "/uav${id}/camera/depth/color/points_octomap" || status=1
  check_stream "UAV${id} raw coverage cmd" "/uav${id}/prometheus/coverage_search/raw_command" || status=1
  check_stream "UAV${id} relayed command" "/uav${id}/prometheus/command" || status=1
  check_stream "UAV${id} local frontiers" "/uav${id}/prometheus/coverage_search/swarm_frontiers" || status=1
  check_stream "UAV${id} bridged peer state" "/uav${id}/two_uav/rx/state" || status=1
  check_controller "$id" || status=1
done

if ((status)); then
  echo "Coverage data flow is incomplete. Read the first FAIL from source to sink." >&2
else
  echo "Coverage data flow verified: state -> depth -> raw command -> coordinator -> controller, plus peer bridge."
fi
exit "$status"
