#!/usr/bin/env bash

set -eo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
workspace="${LUXI_WORKSPACE_ROOT:-$(cd "$(dirname "${script_path}")/../../.." && pwd)}"
maps_directory="${workspace}/maps/rtab_maps"
octo_maps_directory="${workspace}/maps/octo_maps"

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
  output_directory="${octo_maps_directory}/${database_name%.db}_octomap"
fi

if pgrep -f "/rtabmap_slam/lib/rtabmap_slam/rtabmap" >/dev/null; then
  echo "RTAB-Map is still running. Stop the mapping launch before exporting." >&2
  exit 2
fi

if [[ ! -f "${database_path}" ]]; then
  echo "No saved map database found. Specify maps/rtab_maps/mapNNN.db explicitly." >&2
  exit 3
fi

source /opt/ros/humble/setup.bash
if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi
if [[ -f "${workspace}/device/D435i/ros2_ws/install/setup.bash" ]]; then
  source "${workspace}/device/D435i/ros2_ws/install/setup.bash"
fi
set -u

# MVS ships an older libusb which lacks symbols required by ROS/PCL. Camera
# nodes need that directory, but the standalone exporter must resolve the
# system libusb instead.
export LD_LIBRARY_PATH="$(printf '%s' "${LD_LIBRARY_PATH:-}" \
  | tr ':' '\n' \
  | awk 'NF && $0 != "/opt/MVS/lib/aarch64" && !seen[$0]++' \
  | paste -sd: -)"

mkdir -p "${output_directory}"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_name="luxi_rtab_map_${timestamp}"

# Keep this aligned with the 4x4 / 8x8 CREStereo far-depth lattices. Decimation
# 2 preserves every far sample; decimation 4 would retain only one quarter.
rtabmap-export \
  --cloud \
  --opt 0 \
  --decimation 2 \
  --voxel 0.03 \
  --min_range 0.35 \
  --max_range 10.0 \
  --edge_bleeding_error 0.10 \
  --noise_radius 0.35 \
  --noise_k 3 \
  --output "${output_name}" \
  --output_dir "${output_directory}" \
  "${database_path}"

output_path="${output_directory}/${output_name}_cloud.ply"
if [[ ! -s "${output_path}" ]]; then
  echo "The database has no exportable odometry poses: ${database_path}" >&2
  exit 4
fi

echo "Exported 3D map: ${output_path}"
