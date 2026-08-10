#!/usr/bin/env bash
set -eo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
data_dir="${KALIBR_DATA_DIR:-${workspace}/calibration/data}"
run_name="${1:-stereo_imu_$(date +%Y%m%d_%H%M%S)}"
output="${data_dir}/${run_name}_ros2"

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
export FASTDDS_DEFAULT_PROFILES_FILE="${workspace}/install/usb_camera_bringup/share/usb_camera_bringup/config/fastdds_large_image.xml"

declare -A expected_types=(
  [/left_camera/image]=sensor_msgs/msg/Image
  [/right_camera/image]=sensor_msgs/msg/Image
  [/imu/data]=sensor_msgs/msg/Imu
)
for topic in /left_camera/image /right_camera/image /imu/data; do
  actual="$(ros2 topic type "${topic}" 2>/dev/null || true)"
  if [[ "${actual}" != "${expected_types[${topic}]}" ]]; then
    echo "Topic check failed: ${topic}, expected ${expected_types[${topic}]}, got ${actual:-missing}" >&2
    exit 1
  fi
done

if [[ -e "${output}" ]]; then
  echo "Output already exists: ${output}" >&2
  exit 1
fi
mkdir -p "${data_dir}"

echo "Recording ${output}"
echo "Keep the checkerboard visible in both cameras and excite translation plus rotation about X/Y/Z."
echo "Record for 3-5 minutes, then press Ctrl+C once."
exec ros2 bag record \
  --storage sqlite3 \
  -o "${output}" \
  --topics /left_camera/image /right_camera/image /imu/data
