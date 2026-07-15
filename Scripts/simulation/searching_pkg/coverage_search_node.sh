#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PROMETHEUS_WS="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if [ ! -f "${PROMETHEUS_WS}/devel/setup.bash" ]; then
    echo "Missing ${PROMETHEUS_WS}/devel/setup.bash; compile the workspace first."
    exit 1
fi

gnome-terminal --tab -e 'bash -c "source /opt/ros/noetic/setup.bash; source ${PROMETHEUS_WS}/devel/setup.bash; roslaunch prometheus_coverage_search coverage_search.launch; exec bash"'
