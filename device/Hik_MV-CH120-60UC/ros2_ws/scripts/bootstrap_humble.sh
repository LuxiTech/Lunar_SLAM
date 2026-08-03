#!/usr/bin/env bash
# Prepare the Hikrobot stereo workspace for Ubuntu 22.04 + ROS 2 Humble.
# Run this script from a terminal; it deliberately asks sudo for system setup.
# ROS 2 Humble setup scripts expand optional environment variables that may be
# unset, so do not enable `nounset` before sourcing them.
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"
mvs_root="${MVS_ROOT:-/opt/MVS}"
mvs_include_dir="${MVS_INCLUDE_DIR:-${mvs_root}/include}"

if [ "$(. /etc/os-release && printf '%s' "${VERSION_CODENAME}")" != "jammy" ]; then
  echo "This bootstrap is intended for Ubuntu 22.04 (jammy)." >&2
  exit 1
fi

if [ "${ros_distro}" != "humble" ]; then
  echo "Set ROS_DISTRO=humble (or unset it) before using this Ubuntu 22.04 bootstrap." >&2
  exit 1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
  if ! apt-cache show ros-humble-ros-base >/dev/null 2>&1; then
    source_deb="$(mktemp --suffix=.deb)"
    curl --fail --location \
      https://github.com/ros-infrastructure/ros-apt-source/releases/latest/download/ros2-apt-source_1.1.0.jammy_all.deb \
      --output "${source_deb}"
    sudo apt install -y "${source_deb}"
  fi
fi

# Install the complete dependency set even when ROS itself was preinstalled.
# Minimal ROS images commonly contain ros2 but omit cv_bridge and RTAB-Map.
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros-base ros-dev-tools python3-rosdep \
  ros-humble-camera-info-manager ros-humble-cv-bridge \
  ros-humble-image-transport ros-humble-pcl-ros \
  ros-humble-rtabmap-msgs ros-humble-rtabmap-odom \
  ros-humble-rtabmap-slam ros-humble-rtabmap-viz \
  ros-humble-rviz2 ros-humble-tf2-geometry-msgs \
  libopencv-dev libtinyxml2-dev libusb-1.0-0-dev

if [ ! -f "${mvs_include_dir}/MvCameraControl.h" ]; then
  cat >&2 <<EOF
Missing ${mvs_include_dir}/MvCameraControl.h.
Install the *development* package of Hikrobot MVS for Linux ARM64.  The
existing /opt/MVS runtime libraries are not enough to compile this driver.
Then rerun this script (use MVS_ROOT=/path/to/MVS and/or
MVS_INCLUDE_DIR=/path/to/MVS/include if needed).
EOF
  exit 2
fi

# Do not inherit an overlay built for a previous ROS distribution. In
# particular, an old install/ directory may contain package hooks that refer
# to deleted source-built RTAB-Map packages.
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH
source /opt/ros/humble/setup.bash
cd "${workspace_dir}"

# The repository vendors current Lyrical-era RTAB-Map source, but Humble has
# tested binary packages.  Build only this camera stack and the local H30
# driver, allowing CMake to resolve RTAB-Map from /opt/ros/humble.  Ignoring
# the vendored packages is essential: otherwise colcon prefers their source
# paths and generates hooks for packages intentionally not built here.
humble_system_packages=(
  diagnostic_aggregator diagnostic_common_diagnostics diagnostic_remote_logging
  diagnostic_updater diagnostics self_test octomap_msgs pcl_ros
  rtabmap rtabmap_conversions rtabmap_demos
  rtabmap_examples rtabmap_launch rtabmap_msgs rtabmap_odom rtabmap_python
  rtabmap_ros rtabmap_rviz_plugins rtabmap_slam rtabmap_sync rtabmap_util
  rtabmap_viz
)
colcon build --symlink-install \
  --packages-select serial yesense_interface yesense_std_ros2 \
  hikrobot_camera_driver stereo_depth hik_bringup \
  --packages-ignore "${humble_system_packages[@]}" \
  --cmake-args -DBUILD_TESTING=OFF \
  -DMVS_ROOT="${mvs_root}" -DMVS_INCLUDE_DIR="${mvs_include_dir}"

echo
echo "Build complete. Activate it with:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ${workspace_dir}/install/setup.bash"
echo "Then test the camera:"
echo "  ros2 launch hik_bringup camera_only.launch.py"
