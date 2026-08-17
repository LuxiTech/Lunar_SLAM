#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source /opt/ros/humble/setup.bash
if [[ ! -f "${workspace}/install/setup.bash" ]]; then
  echo "Workspace is not built: ${workspace}" >&2
  echo "Run: colcon build --symlink-install" >&2
  exit 1
fi
source "${workspace}/install/setup.bash"
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/install/usb_camera_bringup/share/usb_camera_bringup/config/fastdds_large_image.xml"

exec ros2 launch usb_camera_bringup stereo_imu_calibration.launch.py "$@"
