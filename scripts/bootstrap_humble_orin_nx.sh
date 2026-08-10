#!/usr/bin/env bash
# Install the shared Ubuntu 22.04 / ROS 2 Humble prerequisites for Orin NX.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/bootstrap_humble_orin_nx.sh [options]

Options:
  --with-d435i           Also install the Humble RealSense wrapper.
  --build WORKSPACE       Build algorithm, hik, usb, or d435i after installation.
  --keep-build            Reuse generated build files instead of cleaning the old overlay.
  --with-tests            Enable tests when --build is used.
  --mvs-root PATH         Hikrobot MVS root forwarded to the Hik build.
  -h, --help              Show this help.

The script intentionally does not install the Hikrobot MVS SDK. Install its
ARM64 development package first when building the Hik workspace. Building the
USB workspace also builds the shared algorithm underlay first.
EOF
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
with_d435i=0
build_workspace=""
clean_build=1
with_tests=0
mvs_root=""

while (($#)); do
  case "$1" in
    --with-d435i)
      with_d435i=1
      shift
      ;;
    --build)
      (($# >= 2)) || die "--build requires a workspace name"
      build_workspace="$2"
      shift 2
      ;;
    --keep-build)
      clean_build=0
      shift
      ;;
    --with-tests)
      with_tests=1
      shift
      ;;
    --mvs-root)
      (($# >= 2)) || die "--mvs-root requires a value"
      mvs_root="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown option: $1 (use --help)"
      ;;
  esac
done

[[ "$(. /etc/os-release; printf '%s' "${VERSION_CODENAME:-}")" == "jammy" ]] ||
  die "this bootstrap supports Ubuntu 22.04 (jammy) only"

if [[ "$(uname -m)" != "aarch64" ]]; then
  printf 'warning: expected Orin NX architecture aarch64, found %s\n' "$(uname -m)" >&2
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  cat >&2 <<'EOF'
ROS 2 Humble is not installed. Add the official ROS 2 apt source for Ubuntu
22.04, then install ros-humble-ros-base and rerun this script.
EOF
  exit 1
fi

packages=(
  ros-humble-ros-base ros-dev-tools python3-rosdep
  ros-humble-camera-info-manager ros-humble-cv-bridge
  ros-humble-image-transport ros-humble-imu-filter-madgwick
  ros-humble-pcl-ros ros-humble-rtabmap-conversions ros-humble-rtabmap-msgs
  ros-humble-rtabmap-odom ros-humble-rtabmap-slam ros-humble-rtabmap-viz
  ros-humble-rviz2 ros-humble-tf2-geometry-msgs
  libopencv-dev libtinyxml2-dev libusb-1.0-0-dev libpcl-dev liboctomap-dev
  libopen3d-dev nlohmann-json3-dev
)
if ((with_d435i)) || [[ "${build_workspace}" == "d435i" ]]; then
  packages+=(ros-humble-realsense2-camera)
fi

sudo apt-get update
sudo apt-get install -y "${packages[@]}"
# Keep these tightly coupled Humble packages on one release.  A partial ROS apt
# upgrade can otherwise leave tf2_geometry_msgs expecting headers that an older
# tf2_ros package does not provide.
sudo apt-get install --only-upgrade -y \
  ros-humble-tf2 \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-sensor-msgs

if [[ -n "${build_workspace}" ]]; then
  build_args=(--workspace "${build_workspace}")
  ((clean_build)) && build_args+=(--clean)
  ((with_tests)) && build_args+=(--with-tests)
  [[ -n "${mvs_root}" ]] && build_args+=(--mvs-root "${mvs_root}")
  if [[ "${build_workspace}" == "usb" ]]; then
    algorithm_args=(--workspace algorithm)
    ((clean_build)) && algorithm_args+=(--clean)
    ((with_tests)) && algorithm_args+=(--with-tests)
    "${script_dir}/build_humble_orin_nx.sh" "${algorithm_args[@]}"
  fi
  "${script_dir}/build_humble_orin_nx.sh" "${build_args[@]}"
fi
