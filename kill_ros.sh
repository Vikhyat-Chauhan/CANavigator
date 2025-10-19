#!/usr/bin/env bash
# Kill all ROS/rviz/gz bridge processes for the current user (default),
# with a gentle TERM then a hard KILL if needed.
# Usage:
#   ./kill_ros.sh              # terminate then kill leftovers
#   ./kill_ros.sh -n           # dry run (show what would be killed)
#   ./kill_ros.sh -f           # force KILL immediately (skip TERM)
#   ./kill_ros.sh -u otheruser # target a specific user

set -euo pipefail

DRY_RUN=0
FORCE=0
TARGET_USER="${USER}"

while getopts ":nfu:" opt; do
  case "$opt" in
    n) DRY_RUN=1 ;;
    f) FORCE=1 ;;
    u) TARGET_USER="$OPTARG" ;;
    *) echo "Usage: $0 [-n] [-f] [-u user]"; exit 2 ;;
  esac
done

# Patterns chosen to match ROS 2 (Jazzy), RViz, Gazebo (gz), and ros_gz_bridge components.
# Add/remove patterns here if you run other tools.
PATTERNS=(
  "/opt/ros/.*/bin/ros2"
  #"/opt/ros/.*/lib/rviz2/rviz2"
  "ros_gz_bridge"
  "parameter_bridge"
  "/gz/transport.*/gz-transport-"
  "gzclient"
  "gzserver"
  "gz sim"
  "ignition"
  "rqt"=
)

# Find PIDs for a single pattern (owned by TARGET_USER)
find_pids() {
  local pattern="$1"
  pgrep -u "$TARGET_USER" -f "$pattern" || true
}

# Kill a set of PIDs with a signal
kill_pids() {
  local signal="$1"; shift
  local pids=("$@")
  [ "${#pids[@]}" -eq 0 ] && return 0
  if [ "$DRY_RUN" -eq 1 ]; then
    echo "[DRY] kill $signal ${pids[*]}"
  else
    kill "$signal" "${pids[@]}" 2>/dev/null || true
  fi
}

gather_pids() {
  local out=()
  for pat in "${PATTERNS[@]}"; do
    mapfile -t found < <(find_pids "$pat")
    if [ "${#found[@]}" -gt 0 ]; then
      echo "Matched '$pat': ${found[*]}"
      out+=("${found[@]}")
    fi
  done
  # De-dup
  printf "%s\n" "${out[@]}" | awk '!seen[$0]++'
}

main() {
  echo "Target user: $TARGET_USER"
  echo "Dry run: $DRY_RUN, Force kill: $FORCE"
  mapfile -t initial < <(gather_pids)

  if [ "${#initial[@]}" -eq 0 ]; then
    echo "No matching ROS/gz/bridge processes found for user '$TARGET_USER'."
    exit 0
  fi

  if [ "$FORCE" -eq 1 ]; then
    echo "Forcing immediate SIGKILL for ${#initial[@]} processes…"
    kill_pids -9 "${initial[@]}"
  else
    echo "Sending SIGTERM to ${#initial[@]} processes…"
    kill_pids -15 "${initial[@]}"
    # Give them a moment to exit
    [ "$DRY_RUN" -eq 1 ] || sleep 1.5
    echo "Re-checking for leftover processes…"
    mapfile -t remaining < <(gather_pids)
    if [ "${#remaining[@]}" -gt 0 ]; then
      echo "Sending SIGKILL to ${#remaining[@]} stubborn processes…"
      kill_pids -9 "${remaining[@]}"
    else
      echo "All processes exited cleanly."
    fi
  fi

  echo "Done."
}

main

