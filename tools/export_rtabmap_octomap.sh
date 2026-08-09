#!/usr/bin/env bash

sanitize_mvs_library_path() {
  local mvs_root="${MVS_ROOT:-/opt/MVS}"
  local entry
  local normalized_entry
  local -a entries=()
  local -a filtered_entries=()
  IFS=: read -r -a entries <<< "${LD_LIBRARY_PATH:-}"
  for entry in "${entries[@]}"; do
    [[ -n "${entry}" ]] || continue
    normalized_entry="${entry%/}"
    if [[ "${normalized_entry}" == "${mvs_root%/}/lib/aarch64" ||
          "${normalized_entry}" == "${mvs_root%/}/lib/64" ]]; then
      continue
    fi
    filtered_entries+=("${entry}")
  done
  LD_LIBRARY_PATH="$(IFS=:; echo "${filtered_entries[*]}")"
  export LD_LIBRARY_PATH
}

main() {
set -euo pipefail

local workspace="${LUXI_WORKSPACE_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
local rtab_maps_directory="${workspace}/maps/rtab_maps"
local octo_maps_directory="${workspace}/maps/octo_maps"
local filter_cloud=false
if [[ "${1:-}" == "--filter" ]]; then
  filter_cloud=true
  shift
fi
local database_path="${1:-}"

if [[ -z "${database_path}" ]]; then
  local database_name
  database_name="$(find "${rtab_maps_directory}" -maxdepth 1 -type f -name 'map*.db' -printf '%f\n' | sort -V | tail -n 1)"
  database_path="${rtab_maps_directory}/${database_name}"
fi
if [[ ! -s "${database_path}" ]]; then
  echo "Saved RTAB-Map database not found: ${database_path}" >&2
  exit 2
fi

local output_directory="${2:-${octo_maps_directory}/$(basename "${database_path%.db}")_octomap}"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi
set -u
sanitize_mvs_library_path
export LD_LIBRARY_PATH="${workspace}/3parts/octomap/install/lib:${LD_LIBRARY_PATH:-}"

ros2 run luxi_rtab_map export_3d_map.sh "${database_path}" "${output_directory}"
local ply_path
ply_path="$(find "${output_directory}" -maxdepth 1 -type f -name '*_cloud.ply' -printf '%T@ %p\n' | sort -n | tail -n 1 | cut -d' ' -f2-)"
if [[ -z "${ply_path}" || ! -s "${ply_path}" ]]; then
  echo "RTAB-Map export did not produce a cloud PLY file." >&2
  exit 3
fi
if [[ "${filter_cloud}" == true ]]; then
  local filtered_ply_path="${output_directory}/$(basename "${database_path%.db}")_filtered_cloud.ply"
  "${workspace}/tools/map_cloud_filter/map_cloud_filter" \
    "${ply_path}" "${filtered_ply_path}"
  ply_path="${filtered_ply_path}"
fi
local octomap_path="${output_directory}/$(basename "${database_path%.db}").bt"
ros2 run luxi_voxel_navigation ply_to_octomap "${ply_path}" "${octomap_path}" 0.10
echo "OctoMap source cloud: ${ply_path}"
echo "Offline OctoMap: ${octomap_path}"
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
