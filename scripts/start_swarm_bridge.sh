#!/usr/bin/env bash
# Usage on either onboard computer: start_swarm_bridge.sh <local-ip> <peer-ip> [port-base] [peer-port-base]
set -euo pipefail
[[ $# -ge 2 && $# -le 4 ]] || { echo "usage: $0 <local-ip> <peer-ip> [port-base] [peer-port-base]" >&2; exit 2; }
workspace=${PROMETHEUS_WS:-"$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"}
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
exec roslaunch prometheus_two_uav_coverage_search two_uav_bridge.launch \
  local_ip:="$1" peer_ip:="$2" port_base:="${3:-31000}" peer_port_base:="${4:-${3:-31000}}"
