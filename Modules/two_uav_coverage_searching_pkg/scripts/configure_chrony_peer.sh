#!/usr/bin/env bash
# Run manually on each industrial computer. It intentionally changes system time service config.
set -euo pipefail

usage() {
  echo "usage: $0 master | client <lowest-id-uav-ip>"
  exit 2
}

[[ $# -ge 1 ]] || usage
mode=$1
case "$mode" in
  master)
    config=$'local stratum 10\nallow 192.168.1.0/24\nmakestep 0.1 3\nrtcsync'
    ;;
  client)
    [[ $# -eq 2 ]] || usage
    config=$"server $2 iburst prefer\nmakestep 0.1 3\nrtcsync"
    ;;
  *) usage ;;
esac

sudo install -m 0644 /etc/chrony/chrony.conf "/etc/chrony/chrony.conf.two_uav_backup.$(date +%Y%m%d%H%M%S)"
printf '%s\n' "$config" | sudo tee /etc/chrony/conf.d/two_uav_coverage.conf >/dev/null
sudo systemctl restart chrony
echo "chrony configured as $mode. Run check_clock_offset.sh before every flight."
