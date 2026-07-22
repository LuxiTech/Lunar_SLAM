#!/usr/bin/env bash

set -eo pipefail

maps_directory="/home/lunar/project/lunar_slam/maps"

if [[ $# -ge 1 ]]; then
  database_path="$1"
else
  database_path="$(find "${maps_directory}" -maxdepth 1 -type f -name 'map*.db' -printf '%f\n' \
    | sort -V | tail -n 1)"
  if [[ -n "${database_path}" ]]; then
    database_path="${maps_directory}/${database_path}"
  fi
fi

if [[ $# -ge 2 ]]; then
  output_directory="$2"
else
  database_name="$(basename "${database_path:-map}")"
  output_directory="${maps_directory}/${database_name%.db}_export"
fi

if pgrep -f "/rtabmap_slam/lib/rtabmap_slam/rtabmap" >/dev/null; then
  echo "RTAB-Map is still running. Stop the mapping launch before exporting." >&2
  exit 2
fi

if [[ ! -f "${database_path}" ]]; then
  echo "No saved map database found. Specify maps/mapNNN.db explicitly." >&2
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
