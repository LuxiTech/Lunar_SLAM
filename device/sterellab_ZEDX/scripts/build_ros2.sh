#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
workspace="$project_dir/ros2_ws"

"$script_dir/check_platform.sh"

if [[ ! -r /usr/local/zed/zed-config.cmake ]]; then
  printf 'ZED SDK is not installed. Run scripts/install_sdk.sh first.\n' >&2
  exit 1
fi

if [[ ! -x /usr/local/cuda/bin/nvcc ]]; then
  cat >&2 <<'EOF'
CUDA development tools are missing. Install the matching JetPack development
packages before building:
  sudo apt update
  sudo apt install nvidia-jetpack nvidia-jetpack-dev
EOF
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

rosdep update --rosdistro humble
rosdep install --from-paths \
  "$workspace/src/zed-ros2-wrapper" \
  "$workspace/src/zed-ros2-interfaces" \
  "$workspace/src/zed-ros2-description" \
  "$workspace/src/zed-ros2-examples/zed_display_rviz2" \
  --ignore-src -r -y \
  --rosdistro humble --skip-keys libopencv-dev

cd "$workspace"
colcon build \
  --symlink-install \
  --parallel-workers "$(nproc)" \
  --packages-up-to zed_ros2 zed_display_rviz2 \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda

printf '[OK] Workspace built: %s/install\n' "$workspace"
