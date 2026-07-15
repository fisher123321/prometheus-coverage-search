#!/bin/bash

# Start the coverage-search Gazebo scenario and the original Prometheus controller.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PROMETHEUS_WS="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
export PROMETHEUS_PX4="${PROMETHEUS_PX4:-${HOME}/prometheus_px4}"
export PROMETHEUS_MAVROS="${PROMETHEUS_MAVROS:-${HOME}/prometheus_mavros}"

if [ ! -f "${PROMETHEUS_WS}/devel/setup.bash" ]; then
    echo "Missing ${PROMETHEUS_WS}/devel/setup.bash; compile the workspace first."
    exit 1
fi

if [ ! -f "${PROMETHEUS_PX4}/Tools/setup_gazebo.bash" ]; then
    echo "Missing PX4 setup: ${PROMETHEUS_PX4}/Tools/setup_gazebo.bash"
    exit 1
fi

if [ ! -f "${PROMETHEUS_MAVROS}/devel/setup.bash" ]; then
    echo "Missing MAVROS setup: ${PROMETHEUS_MAVROS}/devel/setup.bash"
    exit 1
fi

source /opt/ros/noetic/setup.bash
source "${PROMETHEUS_MAVROS}/devel/setup.bash"
source "${PROMETHEUS_WS}/devel/setup.bash" --extend
source "${PROMETHEUS_PX4}/Tools/setup_gazebo.bash" \
       "${PROMETHEUS_PX4}" \
       "${PROMETHEUS_PX4}/build/amovlab_sitl_default"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:${PROMETHEUS_PX4}:${PROMETHEUS_PX4}/Tools/sitl_gazebo"

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch prometheus_gazebo sitl_coverage_search_1uav_P450.launch; exec bash"' \
--tab -e 'bash -c "sleep 6; roslaunch prometheus_uav_control uav_control_main_outdoor.launch joy_enable:=false; exec bash"'
