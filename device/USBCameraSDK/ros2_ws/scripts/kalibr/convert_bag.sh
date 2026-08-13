#!/usr/bin/env bash
set -eo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 ROS2_BAG_DIR [ROS1_BAG]" >&2
  exit 2
fi

source_bag="$(realpath "$1")"
destination="${2:-${source_bag%_ros2}.bag}"
if [[ -n "${ROSBAGS_CONVERT:-}" ]]; then
  converter="${ROSBAGS_CONVERT}"
else
  converter="$(command -v rosbags-convert || true)"
fi

if [[ -z "${converter}" || ! -x "${converter}" ]]; then
  echo "rosbags-convert not found; install the Python 'rosbags' package or set ROSBAGS_CONVERT" >&2
  exit 1
fi
if [[ -e "${destination}" ]]; then
  echo "Destination already exists: ${destination}" >&2
  exit 1
fi

"${converter}" \
  --src "${source_bag}" \
  --dst "${destination}" \
  --src-typestore ros2_humble \
  --dst-typestore ros1_noetic

echo "Created ${destination}"
