#!/usr/bin/env bash

# Open an RTAB-Map database without requiring users to manually type mapNNN.
set -euo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
workspace="${LUXI_WORKSPACE_ROOT:-$(cd "$(dirname "${script_path}")/../../.." && pwd)}"
maps_directory="${workspace}/maps/rtab_maps"

if [[ "${1:-}" == "--print-path" ]]; then
  print_path_only=true
  shift
else
  print_path_only=false
fi

if [[ $# -ge 1 ]]; then
  database_path="$1"
else
  database_name="$({ find "${maps_directory}" -maxdepth 1 -type f -name 'map*.db' -printf '%f\n'; } | sort -V | tail -n 1)"
  database_path="${maps_directory}/${database_name}"
fi

if [[ ! -s "${database_path}" ]]; then
  echo "No saved RTAB-Map database found: ${database_path}" >&2
  exit 3
fi

if [[ "${print_path_only}" == true ]]; then
  printf '%s\n' "${database_path}"
  exit 0
fi

if pgrep -f '/rtabmap_slam/lib/rtabmap_slam/rtabmap' >/dev/null; then
  echo "RTAB-Map is still running. Stop mapping before opening its database." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
if [[ -f "${workspace}/device/D435i/ros2_ws/install/setup.bash" ]]; then
  source "${workspace}/device/D435i/ros2_ws/install/setup.bash"
fi
if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi

viewer="${workspace}/device/D435i/ros2_ws/install/rtabmap/bin/rtabmap-databaseViewer"
if [[ ! -x "${viewer}" ]]; then
  viewer="$(command -v rtabmap-databaseViewer || true)"
fi
if [[ -z "${viewer}" || ! -x "${viewer}" ]]; then
  echo "rtabmap-databaseViewer is unavailable after loading the D435i workspace." >&2
  exit 4
fi

echo "Opening RTAB-Map database: ${database_path}"
exec "${viewer}" "${database_path}"
