#!/usr/bin/env bash
# Usage: start_two_uav_sim.sh [uav1-id] [uav2-id] [fly-height-m]
set -euo pipefail
[[ $# -le 3 ]] || { echo "usage: $0 [uav1-id] [uav2-id] [fly-height-m]" >&2; exit 2; }

workspace=${PROMETHEUS_WS:-"$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"}
px4_root=${PROMETHEUS_PX4:-"${HOME}/prometheus_px4"}
mavros_ws=${PROMETHEUS_MAVROS:-"${HOME}/prometheus_mavros"}

[[ -f "$workspace/devel/setup.bash" ]] || { echo "missing workspace setup: $workspace/devel/setup.bash" >&2; exit 1; }
[[ -f "$px4_root/Tools/setup_gazebo.bash" ]] || { echo "missing PX4 setup: $px4_root/Tools/setup_gazebo.bash" >&2; exit 1; }
[[ -f "$mavros_ws/devel/setup.bash" ]] || { echo "missing MAVROS setup: $mavros_ws/devel/setup.bash" >&2; exit 1; }

source /opt/ros/noetic/setup.bash
source "$mavros_ws/devel/setup.bash"
source "$workspace/devel/setup.bash" --extend
# PX4's setup script expands these variables directly.  Seed them because this
# launcher intentionally runs with `set -u` and a fresh shell may not define them.
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
source "$px4_root/Tools/setup_gazebo.bash" "$px4_root" "$px4_root/build/amovlab_sitl_default"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:$px4_root:$px4_root/Tools/sitl_gazebo"

services="$(timeout 3s rosservice list 2>/dev/null || true)"
if grep -Fxq '/gazebo/get_world_properties' <<<"$services"; then
  echo "A Gazebo world is already running on this ROS master. Stop the old Gazebo/PX4 simulation first; refusing to mix instances." >&2
  exit 1
fi
nodes="$(timeout 3s rosnode list 2>/dev/null || true)"
for legacy_node in /coverage_search_node_1 /depth_cloud_downsample_1 /octomap_server_1 /tf_camera_d435i_1; do
  if grep -Fxq "$legacy_node" <<<"$nodes"; then
    echo "Legacy single-UAV coverage simulation node $legacy_node is still running. Stop coverage_search_P450.sh and its roscore first." >&2
    exit 1
  fi
done

uav1_id=${1:-1}
uav2_id=${2:-2}
fly_height=${3:-1.5}
[[ "$uav1_id" != "$uav2_id" ]] || { echo "UAV IDs must differ" >&2; exit 2; }
exec roslaunch prometheus_two_uav_coverage_search two_uav_sim_world.launch \
  uav1_id:="$uav1_id" uav2_id:="$uav2_id" fly_height:="$fly_height"
