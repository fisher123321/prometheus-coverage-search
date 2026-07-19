#!/usr/bin/env bash
# Usage on either onboard computer: start_two_uav_coverage.sh <uav-id> <peer-uav-id> [height]
set -euo pipefail
[[ $# -ge 2 && $# -le 3 ]] || { echo "usage: $0 <uav-id> <peer-uav-id> [height-m]" >&2; exit 2; }
workspace=${PROMETHEUS_WS:-"$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"}
source /opt/ros/noetic/setup.bash
source "$workspace/devel/setup.bash"
global_coverage_source=$(( $1 < $2 ? $1 : $2 ))
exec roslaunch prometheus_two_uav_coverage_search two_uav_onboard.launch \
  uav_id:="$1" peer_uav_id:="$2" fly_height:="${3:-1.5}" global_coverage_source:="$global_coverage_source"
