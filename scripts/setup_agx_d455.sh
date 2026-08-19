#!/usr/bin/env bash
# Configure JetPack 6 / Ubuntu 22.04 for the D455 mapping and navigation stack.
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/setup_agx_d455.sh [options]

Options:
  --skip-system       Do not install apt/ROS packages.
  --skip-realsense    Do not clone/build librealsense and realsense-ros.
  --skip-python       Do not install the HLoc/SuperPoint Python runtime.
  --skip-project      Do not build the project ROS workspace.
  --cpu-python        Install CPU PyTorch instead of the Jetson CUDA overlay.
  --gpu-index URL     Jetson CUDA wheel index (default: JP6 CUDA 12.6 index).
  --no-shell-hook     Do not add the project environment to ~/.bashrc.
  -h, --help          Show this help.

The script is safe to rerun. It never stores sudo credentials.
EOF
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

log() {
  printf '\n==> %s\n' "$*"
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
D455_WORKSPACE="${WORKSPACE}/device/D455/ros2_ws"
LIBREALSENSE_SOURCE="${D455_WORKSPACE}/3parts/librealsense"
LIBREALSENSE_INSTALL="${LIBREALSENSE_SOURCE}/install-rsusb"
REALSENSE_ROS_SOURCE="${D455_WORKSPACE}/src/realsense-ros"
ROS_DISTRO=humble
GPU_INDEX_URL="https://pypi.jetson-ai-lab.io/jp6/cu126"
ROS_APT_MIRROR="${LUXI_ROS_APT_MIRROR:-https://mirrors.ustc.edu.cn/ros2/ubuntu}"
PYTHON_RUNTIME=gpu
INSTALL_SYSTEM=1
BUILD_REALSENSE=1
INSTALL_PYTHON=1
BUILD_PROJECT=1
INSTALL_SHELL_HOOK=1

while (($#)); do
  case "$1" in
    --skip-system) INSTALL_SYSTEM=0; shift ;;
    --skip-realsense) BUILD_REALSENSE=0; shift ;;
    --skip-python) INSTALL_PYTHON=0; shift ;;
    --skip-project) BUILD_PROJECT=0; shift ;;
    --cpu-python) PYTHON_RUNTIME=cpu; shift ;;
    --gpu-index)
      (($# >= 2)) || die "--gpu-index requires a URL"
      GPU_INDEX_URL="$2"
      shift 2
      ;;
    --no-shell-hook) INSTALL_SHELL_HOOK=0; shift ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown option: $1 (use --help)" ;;
  esac
done

[[ "$(uname -m)" == "aarch64" ]] || die "this installer is intended for ARM64 Jetson"
[[ "$(. /etc/os-release && printf '%s' "${VERSION_CODENAME}")" == "jammy" ]] ||
  die "ROS 2 Humble setup requires Ubuntu 22.04 (jammy)"
[[ -f /etc/nv_tegra_release ]] || die "JetPack/L4T was not detected"

if ((INSTALL_SYSTEM)); then
  command -v sudo >/dev/null 2>&1 || die "sudo is required for system packages"
  log "Installing ROS 2 Humble repository and system dependencies"
  if [[ ! -f /usr/share/ros-apt-source/ros2.sources ]]; then
    sudo apt-get update
    sudo apt-get install -y curl ca-certificates gnupg lsb-release software-properties-common
    ros_source_deb="$(mktemp --suffix=.deb)"
    trap 'rm -f -- "${ros_source_deb:-}"' EXIT
    ros_source_url="$(
      curl --fail --silent --show-error \
        https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest |
        python3 -c '
import json, sys
assets = json.load(sys.stdin)["assets"]
matches = [a["browser_download_url"] for a in assets
           if a["name"].startswith("ros2-apt-source_")
           and a["name"].endswith(".jammy_all.deb")]
if len(matches) != 1:
    raise SystemExit("cannot resolve the current Jammy ROS apt source package")
print(matches[0])'
    )"
    curl --fail --location "${ros_source_url}" --output "${ros_source_deb}"
    sudo apt-get install -y "${ros_source_deb}"
  fi
  # packages.ros.org is often slow from mainland China. Keep the apt-source
  # package's embedded signing key and replace only its repository URI.
  sudo sed -i \
    -e 's/^Types: deb deb-src$/Types: deb/' \
    -e "s|^URIs: .*ros2/ubuntu$|URIs: ${ROS_APT_MIRROR}|" \
    /usr/share/ros-apt-source/ros2.sources
  sudo apt-get -o Acquire::Retries=3 -o Acquire::Languages=none update

  # JetPack 6 ships an all-in-one OpenCV 4.8 development package, while the
  # Jammy ROS 2 Humble RTAB-Map binaries are linked to Ubuntu OpenCV 4.5
  # (including aruco). Mixing the two leaves RTAB-Map with an unresolvable
  # -lopencv_aruco link and can load two OpenCV ABIs into one process. Use the
  # matching Jammy development packages for this ROS workspace.
  if dpkg-query -W -f='${Version}' libopencv-dev 2>/dev/null | grep -q '^4\.8\.0-1-g'; then
    sudo dpkg --remove --force-depends libopencv-dev opencv-licenses
  fi
  sudo apt-get -o APT::Default-Release=jammy --fix-broken install -y
  sudo apt-get -o APT::Default-Release=jammy install -y \
    libopencv-dev=4.5.4+dfsg-9ubuntu4 \
    libopencv-contrib-dev=4.5.4+dfsg-9ubuntu4
  sudo apt-mark hold libopencv-dev >/dev/null

  sudo apt-get install -y \
    build-essential cmake git ninja-build pkg-config python3-dev python3-pip \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    libgl1-mesa-dev libgtk-3-dev liboctomap-dev libopen3d-dev libpcl-dev \
    libssl-dev libudev-dev libusb-1.0-0-dev nlohmann-json3-dev \
    ros-humble-ros-base ros-humble-ament-cmake-python ros-humble-cv-bridge \
    ros-humble-diagnostic-msgs ros-humble-image-transport \
    ros-humble-imu-filter-madgwick ros-humble-octomap-msgs \
    ros-humble-pcl-ros ros-humble-rmw-fastrtps-cpp \
    ros-humble-rtabmap-conversions ros-humble-rtabmap-msgs \
    ros-humble-rtabmap-odom ros-humble-rtabmap-slam \
    ros-humble-rtabmap-sync ros-humble-rtabmap-viz ros-humble-rviz2 \
    ros-humble-tf2-geometry-msgs
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
fi

[[ -f /opt/ros/humble/setup.bash ]] ||
  die "/opt/ros/humble/setup.bash is missing; rerun without --skip-system"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u

clone_tag() {
  local repository="$1"
  local tag="$2"
  local destination="$3"
  if [[ ! -d "${destination}/.git" ]]; then
    [[ ! -e "${destination}" ]] || die "non-git path blocks clone: ${destination}"
    git clone --depth 1 --branch "${tag}" "${repository}" "${destination}"
  fi
  local current_tag
  current_tag="$(git -C "${destination}" describe --tags --exact-match 2>/dev/null || true)"
  [[ "${current_tag}" == "${tag}" ]] ||
    die "${destination} is at '${current_tag:-an untagged commit}', expected ${tag}"
}

if ((BUILD_REALSENSE)); then
  log "Fetching pinned D455 SDK and ROS wrapper"
  mkdir -p -- "${D455_WORKSPACE}/3parts" "${D455_WORKSPACE}/src"
  clone_tag https://github.com/IntelRealSense/librealsense.git v2.58.2 \
    "${LIBREALSENSE_SOURCE}"
  clone_tag https://github.com/IntelRealSense/realsense-ros.git 4.58.2 \
    "${REALSENSE_ROS_SOURCE}"

  log "Installing RealSense USB access rules"
  sudo install -m 0644 \
    "${LIBREALSENSE_SOURCE}/config/99-realsense-libusb.rules" \
    /etc/udev/rules.d/99-realsense-libusb.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=8086
  sudo udevadm settle

  log "Building librealsense 2.58.2 with the userspace RSUSB backend"
  cmake -S "${LIBREALSENSE_SOURCE}" -B "${LIBREALSENSE_SOURCE}/build-rsusb" \
    -G Ninja -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${LIBREALSENSE_INSTALL}" \
    -DFORCE_RSUSB_BACKEND=true -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false -DBUILD_UNIT_TESTS=false \
    -DBUILD_WITH_DDS=false -DBUILD_PYTHON_BINDINGS=false
  cmake --build "${LIBREALSENSE_SOURCE}/build-rsusb" --parallel "$(nproc)"
  cmake --install "${LIBREALSENSE_SOURCE}/build-rsusb"

  log "Building the D455 ROS 2 workspace"
  export CMAKE_PREFIX_PATH="${LIBREALSENSE_INSTALL}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${LIBREALSENSE_INSTALL}/lib:${LD_LIBRARY_PATH:-}"
  colcon --log-base "${D455_WORKSPACE}/log" build \
    --base-paths "${D455_WORKSPACE}/src" \
    --build-base "${D455_WORKSPACE}/build" \
    --install-base "${D455_WORKSPACE}/install" \
    --symlink-install --cmake-clean-cache \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
fi

if ((INSTALL_PYTHON)); then
  log "Initializing the learned localization/frontend sources"
  git -C "${WORKSPACE}" submodule update --init --recursive --depth 1 -- \
    3parts/hloc 3parts/lightglue
  if [[ "${PYTHON_RUNTIME}" == gpu ]]; then
    "${WORKSPACE}/scripts/setup_3parts.sh" --runtime gpu --skip-submodules \
      --gpu-index-url "${GPU_INDEX_URL}"
  else
    "${WORKSPACE}/scripts/setup_3parts.sh" --runtime cpu --skip-submodules
  fi
fi

if ((BUILD_PROJECT)); then
  log "Building the hardware-independent mapping/navigation workspace"
  colcon --log-base "${WORKSPACE}/log" build \
    --base-paths "${WORKSPACE}/project" \
      "${WORKSPACE}/3parts/D1-ROS2-SDK-Demo/ddt_msgs" \
    --build-base "${WORKSPACE}/build" \
    --install-base "${WORKSPACE}/install" \
    --symlink-install --cmake-clean-cache \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

if ((INSTALL_SHELL_HOOK)); then
  log "Installing an idempotent shell environment hook"
  shell_hook="source ${WORKSPACE}/scripts/luxi_env.sh"
  if ! grep -Fqx "${shell_hook}" "${HOME}/.bashrc" 2>/dev/null; then
    printf '\n# lunar_slam ROS 2 environment\n%s\n' "${shell_hook}" >> "${HOME}/.bashrc"
  fi
fi

log "Running installation checks"
# shellcheck disable=SC1091
source "${WORKSPACE}/scripts/luxi_env.sh"
if ((BUILD_REALSENSE)); then
  ros2 pkg prefix lunar_d455_bringup >/dev/null ||
    die "ROS package is unavailable: lunar_d455_bringup"
fi
if ((BUILD_PROJECT)); then
  for package in luxi_adapter luxi_rtab_map luxi_hloc luxi_location \
    luxi_voxel_navigation luxi_3d_navigation luxi_web_control; do
    ros2 pkg prefix "${package}" >/dev/null || die "ROS package is unavailable: ${package}"
  done
fi
if ((INSTALL_PYTHON)); then
  python_check=(python3 "${WORKSPACE}/project/luxi_hloc/scripts/check_hloc_environment.py")
  if [[ "${PYTHON_RUNTIME}" == gpu ]]; then
    python_check+=(--require-cuda)
  fi
  "${python_check[@]}"
fi

printf '\nAGX D455 environment is ready. Open a new terminal or run:\n  source %s/scripts/luxi_env.sh\n' \
  "${WORKSPACE}"
