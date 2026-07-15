#!/usr/bin/env bash

set -eo pipefail

database_path="${1:-${HOME}/.ros/luxi_rtab_map.db}"
output_directory="${2:-${HOME}/.ros/luxi_rtab_map_exports}"

if pgrep -f "/rtabmap_slam/lib/rtabmap_slam/rtabmap" >/dev/null; then
  echo "RTAB-Map is still running. Stop the mapping launch before exporting." >&2
  exit 2
fi

if [[ ! -f "${database_path}" ]]; then
  echo "Database does not exist: ${database_path}" >&2
  exit 3
fi

source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
set -u

mkdir -p "${output_directory}"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_name="luxi_rtab_map_${timestamp}"

rtabmap-export \
  --cloud \
  --opt 3 \
  --decimation 4 \
  --voxel 0.03 \
  --min_range 0.2 \
  --max_range 4.0 \
  --output "${output_name}" \
  --output_dir "${output_directory}" \
  "${database_path}"

output_path="${output_directory}/${output_name}_cloud.ply"
if [[ ! -s "${output_path}" ]]; then
  echo "The database has no exportable odometry poses: ${database_path}" >&2
  exit 4
fi

echo "Exported 3D map: ${output_path}"
