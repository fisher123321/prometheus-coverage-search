#!/usr/bin/env bash
# Usage: start_two_uav_coverage_sim.sh [uav1-id] [uav2-id] [fly-height-m] [capture-logs]
set -euo pipefail
[[ $# -le 4 ]] || { echo "usage: $0 [uav1-id] [uav2-id] [fly-height-m] [capture-logs]" >&2; exit 2; }

capture_logs=${4:-false}
[[ "$capture_logs" == true || "$capture_logs" == false ]] || {
  echo "capture-logs must be true or false" >&2
  exit 2
}
if [[ "$capture_logs" == true ]]; then
  desktop_dir="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
  [[ -n "$desktop_dir" && "$desktop_dir" != "$HOME" ]] || desktop_dir="$HOME/Desktop"
  log_dir="$desktop_dir/日志记录"
  mkdir -p "$log_dir"
  uav1_id=${1:-1}
  uav2_id=${2:-2}
  fly_height=${3:-1.5}
  run_id="coverage_uav${uav1_id}-${uav2_id}_h${fly_height}_$(date +%Y%m%d_%H%M%S)_pid$$"
  info_log="$log_dir/${run_id}_info.log"
  warn_log="$log_dir/${run_id}_warn.log"
  touch "$info_log" "$warn_log"
  echo "Normal log: $info_log"
  echo "Warning log: $warn_log"
  exec 3>>"$info_log"
  exec 4>>"$warn_log"
  set +e
  "$0" "${@:1:3}" false 2>&1 | tee /dev/tty |
      sed -E $'s/\x1B\\[[0-9;]*m//g' |
      while IFS= read -r line || [[ -n "$line" ]]; do
    if [[ "$line" == *"[WARN]"* || "$line" == *"[ERROR]"* || "$line" == *"[FATAL]"* ]]; then
      printf '%s\n' "$line" >&4
    else
      printf '%s\n' "$line" >&3
    fi
  done
  launcher_status=${PIPESTATUS[0]}
  exit "$launcher_status"
fi

workspace=${PROMETHEUS_WS:-"$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"}
mavros_ws=${PROMETHEUS_MAVROS:-"${HOME}/prometheus_mavros"}
[[ -f "$mavros_ws/devel/setup.bash" ]] || { echo "missing MAVROS setup: $mavros_ws/devel/setup.bash" >&2; exit 1; }
source /opt/ros/noetic/setup.bash
source "$mavros_ws/devel/setup.bash"
source "$workspace/devel/setup.bash" --extend

coverage_node="$workspace/devel/lib/prometheus_two_uav_coverage_search/two_uav_coverage_search_node"
[[ -x "$coverage_node" ]] || {
  echo "Missing coverage executable: $coverage_node" >&2
  echo "Build the package first: cmake --build $workspace/build/two_uav_coverage_searching_pkg -j1" >&2
  exit 1
}

uav1_id=${1:-1}
uav2_id=${2:-2}
fly_height=${3:-1.5}
[[ "$uav1_id" != "$uav2_id" ]] || { echo "UAV IDs must differ" >&2; exit 2; }

# A planner can publish valid commands while the corresponding Prometheus
# controller is absent.  In that case the aircraft will hover and it is easy
# to mistake this for a coverage-planning failure.  Refuse to start until each
# command topic is actually consumed by its controller.
require_controller() {
  local id=$1 info
  info="$(timeout 5s rostopic info "/uav${id}/prometheus/command" 2>/dev/null || true)"
  grep -Fq "/uav_control_main_${id}" <<<"$info" || {
    echo "UAV${id} has no /uav_control_main_${id} subscriber on /uav${id}/prometheus/command." >&2
    echo "Keep start_two_uav_sim.sh running, wait for MAVROS and uav_control, then retry." >&2
    return 1
  }
}

require_yaw_rate() {
  local id=$1 response yaw_rate
  response="$(timeout 5s rosservice call "/uav${id}/mavros/param/get" "param_id: 'MC_YAWRATE_MAX'" 2>/dev/null || true)"
  yaw_rate="$(awk '/real:/ { print $2; exit }' <<<"$response")"
  awk -v value="$yaw_rate" 'BEGIN { exit !(value > 0) }' || {
    echo "UAV${id} has MC_YAWRATE_MAX=${yaw_rate:-unreadable}; yaw control is disabled. Restart the simulator so uav_control_indoor.yaml restores it." >&2
    return 1
  }
}

require_controller "$uav1_id" || exit 1
require_controller "$uav2_id" || exit 1
require_yaw_rate "$uav1_id" || exit 1
require_yaw_rate "$uav2_id" || exit 1

# The two state streams must already be expressed in one common world frame.
# Do this before launching the bridge/coordinator so a per-vehicle PX4-local
# frame cannot silently turn into an indefinite WAIT_TAKEOFF hover.
read_position() {
  local id=$1 state position
  state="$(timeout 5s rostopic echo -n 1 "/uav${id}/prometheus/state" 2>/dev/null || true)"
  position="$(sed -n 's/^position: \[\([^]]*\)\]$/\1/p' <<<"$state")"
  [[ -n "$position" ]] || {
    echo "UAV${id} has no usable prometheus/state; arm and wait for valid odometry first." >&2
    return 1
  }
  awk -F',' '{gsub(/ /, "", $1); gsub(/ /, "", $2); if ($1 ~ /^-?[0-9.]+$/ && $2 ~ /^-?[0-9.]+$/) print $1, $2; else exit 1}' <<<"$position"
}

pos1="$(read_position "$uav1_id")" || exit 1
pos2="$(read_position "$uav2_id")" || exit 1
separation="$(awk -v x1="${pos1%% *}" -v y1="${pos1##* }" -v x2="${pos2%% *}" -v y2="${pos2##* }" 'BEGIN { printf "%.3f", sqrt((x1-x2)^2 + (y1-y2)^2) }')"
awk -v d="$separation" 'BEGIN { exit !(d >= 1.0) }' || {
  echo "UAV separation is ${separation} m (< 1.0 m); refusing cooperative start. Check shared world coordinates and spawn positions." >&2
  exit 1
}
echo "Verified shared-frame UAV separation: ${separation} m"
exec roslaunch prometheus_two_uav_coverage_search two_uav_coverage_sim_algorithm.launch \
  uav1_id:="$uav1_id" uav2_id:="$uav2_id" fly_height:="$fly_height"
