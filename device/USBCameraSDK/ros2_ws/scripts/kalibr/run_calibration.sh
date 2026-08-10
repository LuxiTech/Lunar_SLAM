#!/usr/bin/env bash
set -eo pipefail

if [[ $# -ne 1 && $# -ne 3 ]]; then
  echo "Usage: $0 ROS1_BAG [START_SEC END_SEC]" >&2
  exit 2
fi

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
bag="$(realpath "$1")"
data_dir="$(dirname "${bag}")"
bag_name="$(basename "${bag}")"
image="${KALIBR_IMAGE:-luxi-kalibr:rosbags}"
cams_config="${KALIBR_CAMS_CONFIG:-camchain_left.yaml}"
if [[ "${cams_config}" != "$(basename "${cams_config}")" || ! -f "${workspace}/calibration/kalibr/${cams_config}" ]]; then
  echo "Camera config not found in calibration/kalibr: ${cams_config}" >&2
  exit 1
fi
bag_range_args=""
if [[ $# -eq 3 ]]; then
  bag_range_args="--bag-from-to $2 $3"
fi

# Kalibr itself is ROS 1 software. These Noetic paths exist only inside the
# disposable calibration container and are never sourced on the NX host.
docker run --rm \
  --entrypoint /bin/bash \
  -v "${data_dir}:/data" \
  -v "${workspace}/calibration/kalibr:/config:ro" \
  "${image}" -lc \
  "source /opt/ros/noetic/setup.bash && \
   source /catkin_ws/devel/setup.bash && \
   rosbag info '/data/${bag_name}' && \
   rosrun kalibr kalibr_calibrate_imu_camera \
     --bag '/data/${bag_name}' \
     --cams '/config/${cams_config}' \
     --imu /config/imu_h30.yaml \
     --target /config/checkerboard_11x8_10mm.yaml \
     --bag-freq 10 \
     ${bag_range_args} \
     --max-iter 30 \
     --timeoffset-padding 0.20 \
     --dont-show-report"
