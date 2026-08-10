#!/usr/bin/env bash
# Build one isolated Lunar SLAM workspace on Ubuntu 22.04 / ROS 2 Humble.
#
# Do not run `colcon build` from the repository root: it discovers the device
# workspaces recursively and finds duplicate H30 (yesense_*) packages.  This
# script provides the only supported build entry point and passes explicit
# package paths to colcon.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/build_humble_orin_nx.sh [options]

Options:
  --workspace NAME       Workspace to build: algorithm (default), hik, usb, or d435i.
  --clean                Remove this workspace's generated build/, install/, and log/ first.
  --with-tests           Enable CMake tests (disabled by default on Orin NX).
  --test                 Build with tests, then run this workspace's test suite.
  --build-type TYPE      CMake build type (default: Release).
  --mvs-root PATH        Hikrobot MVS installation root (default: /opt/MVS).
  --mvs-include-dir PATH Hikrobot MVS include directory (default: <mvs-root>/include).
  -h, --help             Show this help.

Examples:
  scripts/build_humble_orin_nx.sh --workspace algorithm
  scripts/build_humble_orin_nx.sh --workspace hik --mvs-root /opt/MVS
  scripts/build_humble_orin_nx.sh --workspace usb --clean
EOF
}

die() {
  printf 'error: %s\n' "$*" >&2
  exit 1
}

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
workspace_name="algorithm"
clean=0
build_testing=OFF
run_tests=0
build_type=Release
mvs_root="${MVS_ROOT:-/opt/MVS}"
mvs_include_dir="${MVS_INCLUDE_DIR:-}"

while (($#)); do
  case "$1" in
    --workspace)
      (($# >= 2)) || die "--workspace requires a value"
      workspace_name="$2"
      shift 2
      ;;
    --clean)
      clean=1
      shift
      ;;
    --with-tests)
      build_testing=ON
      shift
      ;;
    --test)
      build_testing=ON
      run_tests=1
      shift
      ;;
    --build-type)
      (($# >= 2)) || die "--build-type requires a value"
      build_type="$2"
      shift 2
      ;;
    --mvs-root)
      (($# >= 2)) || die "--mvs-root requires a value"
      mvs_root="$2"
      shift 2
      ;;
    --mvs-include-dir)
      (($# >= 2)) || die "--mvs-include-dir requires a value"
      mvs_include_dir="$2"
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

[[ -f /opt/ros/humble/setup.bash ]] ||
  die "ROS 2 Humble is not installed at /opt/ros/humble"

# A prior ROS overlay can leak CMake packages, Python modules and ament hooks
# from another distribution.  Always start the build from the Humble underlay.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
unset COLCON_DEFAULTS_FILE ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
# Humble's setup scripts read optional variables, so nounset must be disabled
# while sourcing them.
set +u
source /opt/ros/humble/setup.bash
set -u

declare -a package_paths=()
declare -a cmake_args=(-DBUILD_TESTING="${build_testing}" -DCMAKE_BUILD_TYPE="${build_type}")

case "${workspace_name}" in
  algorithm)
    workspace_dir="${repository_root}"
    package_paths=("${repository_root}"/project/*/)
    ;;
  hik)
    workspace_dir="${repository_root}/device/Hik_MV-CH120-60UC/ros2_ws"
    [[ -d "${workspace_dir}/src" ]] || die "Hik workspace is missing: ${workspace_dir}"
    mvs_include_dir="${mvs_include_dir:-${mvs_root}/include}"
    [[ -f "${mvs_include_dir}/MvCameraControl.h" ]] || die \
      "Hikrobot MVS development header not found: ${mvs_include_dir}/MvCameraControl.h"
    package_paths=(
      # The Hik tree contains an empty historical serial_ros2 placeholder.
      # Keep one real copy (vendored by the USB camera workspace) so colcon
      # never discovers duplicate packages during a root-level build.
      "${repository_root}/device/USBCameraSDK/ros2_ws/src/third_party/serial_ros2"
      "${workspace_dir}/src/third_party/yesense_ros2/yesense_interface"
      "${workspace_dir}/src/third_party/yesense_ros2/yesense_std_ros2"
      "${workspace_dir}/src/hikrobot_camera_driver"
      "${workspace_dir}/src/stereo_depth"
      "${workspace_dir}/src/hik_bringup"
    )
    cmake_args+=("-DMVS_ROOT=${mvs_root}" "-DMVS_INCLUDE_DIR=${mvs_include_dir}")
    ;;
  usb)
    workspace_dir="${repository_root}/device/USBCameraSDK/ros2_ws"
    [[ -d "${workspace_dir}/src" ]] || die "USB camera workspace is missing: ${workspace_dir}"
    algorithm_setup="${repository_root}/install/setup.bash"
    [[ -f "${algorithm_setup}" ]] || die \
      "Algorithm underlay is missing; build --workspace algorithm first"
    set +u
    source "${algorithm_setup}"
    set -u
    package_paths=(
      "${workspace_dir}/src/third_party/serial_ros2"
      "${workspace_dir}/src/third_party/yesense_ros2/yesense_interface"
      "${workspace_dir}/src/third_party/yesense_ros2/yesense_std_ros2"
      "${workspace_dir}/src/usb_camera_driver"
      "${workspace_dir}/src/usb_camera_bringup"
      "${workspace_dir}/src/lunar_usb_rtabmap_bringup"
    )
    ;;
  d435i)
    workspace_dir="${repository_root}/device/D435i/ros2_ws"
    [[ -d "${workspace_dir}/src" ]] || die "D435i workspace is missing: ${workspace_dir}"
    package_paths=(
      "${workspace_dir}/src/lunar_realsense_bringup"
      "${workspace_dir}/src/lunar_d435i_rtabmap_bringup"
    )
    ;;
  *)
    die "--workspace must be one of: algorithm, hik, usb, d435i"
    ;;
esac

for package_path in "${package_paths[@]}"; do
  [[ -f "${package_path}/package.xml" ]] ||
    die "ROS package manifest is missing: ${package_path}/package.xml"
done

if ((clean)); then
  rm -rf -- "${workspace_dir}/build" "${workspace_dir}/install" "${workspace_dir}/log"
fi

printf 'Building %s workspace with ROS 2 %s:\n' "${workspace_name}" "${ROS_DISTRO}"
printf '  %s\n' "${package_paths[@]}"
cd "${workspace_dir}"
colcon build --symlink-install --paths "${package_paths[@]}" --cmake-args "${cmake_args[@]}"

if ((run_tests)); then
  colcon test --paths "${package_paths[@]}" --event-handlers console_direct+
  colcon test-result --all --verbose
fi

printf '\nBuild complete. Activate it with:\n'
printf '  source /opt/ros/humble/setup.bash\n'
printf '  source %s/install/setup.bash\n' "${workspace_dir}"
