#!/usr/bin/env bash

set -euo pipefail

workspace="${LUXI_WORKSPACE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
rtab_maps_directory="${workspace}/maps/rtab_maps"
octo_maps_directory="${workspace}/maps/octo_maps"
database_path="${1:-}"

if [[ -z "${database_path}" ]]; then
  database_name="$(find "${rtab_maps_directory}" -maxdepth 1 -type f -name 'map*.db' -printf '%f\n' | sort -V | tail -n 1)"
  database_path="${rtab_maps_directory}/${database_name}"
fi
if [[ ! -s "${database_path}" ]]; then
  echo "Saved RTAB-Map database not found: ${database_path}" >&2
  exit 2
fi

output_directory="${2:-${octo_maps_directory}/$(basename "${database_path%.db}")_octomap}"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "${workspace}/device/D435i/ros2_ws/install/setup.bash" ]]; then
  source "${workspace}/device/D435i/ros2_ws/install/setup.bash"
fi
if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi
set -u
export LD_LIBRARY_PATH="${workspace}/3parts/octomap/install/lib:${LD_LIBRARY_PATH:-}"

ros2 run luxi_rtab_map export_3d_map.sh "${database_path}" "${output_directory}"
ply_path="$(find "${output_directory}" -maxdepth 1 -type f -name '*_cloud.ply' -printf '%T@ %p\n' | sort -n | tail -n 1 | cut -d' ' -f2-)"
if [[ -z "${ply_path}" || ! -s "${ply_path}" ]]; then
  echo "RTAB-Map export did not produce a cloud PLY file." >&2
  exit 3
fi
octomap_path="${output_directory}/$(basename "${database_path%.db}").bt"
ros2 run luxi_voxel_navigation ply_to_octomap "${ply_path}" "${octomap_path}" 0.10
echo "Offline OctoMap: ${octomap_path}"
