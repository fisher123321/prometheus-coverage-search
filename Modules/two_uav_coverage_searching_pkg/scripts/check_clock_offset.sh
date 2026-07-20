#!/usr/bin/env bash
# Read-only preflight check. Chrony must already be configured and synchronized.
set -euo pipefail

limit_ms=${1:-20}
tracking=$(chronyc tracking)
printf '%s\n' "$tracking"
offset=$(awk -F': ' '/Last offset/ {gsub(/ seconds/, "", $2); v=$2; if (v<0) v=-v; print v}' <<<"$tracking")
rms=$(awk -F': ' '/RMS offset/ {gsub(/ seconds/, "", $2); v=$2; if (v<0) v=-v; print v}' <<<"$tracking")
[[ -n "$offset" && -n "$rms" ]] || { echo "Cannot parse chronyc tracking" >&2; exit 2; }
awk -v offset="$offset" -v rms="$rms" -v limit="$limit_ms" 'BEGIN {
  worst = (offset > rms ? offset : rms) * 1000.0
  if (worst >= limit) {
    printf "FAIL: clock offset %.3f ms is not below %.3f ms\n", worst, limit > "/dev/stderr"; exit 1
  }
  printf "PASS: clock offset %.3f ms is below %.3f ms\n", worst, limit
}'
