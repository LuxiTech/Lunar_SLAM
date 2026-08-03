#!/usr/bin/env bash
# Install and build the EVK4 wrapper on Ubuntu 22.04 / ROS 2 Humble.
# ROS 2 Humble setup scripts expand optional environment variables that may be
# unset, so do not enable `nounset` before sourcing them.
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ "$(. /etc/os-release && printf '%s' "${VERSION_CODENAME}")" != "jammy" ]; then
  echo "This bootstrap is intended for Ubuntu 22.04 (jammy)." >&2
  exit 1
fi

if [ ! -f /opt/ros/humble/setup.bash ]; then
  cat >&2 <<'EOF'
ROS 2 Humble is not installed. Configure the official ROS apt source, then run:
  sudo apt update
  sudo apt install ros-humble-ros-base ros-dev-tools
EOF
  exit 1
fi

# The ROS packages include a compatible OpenEB build and set its plugin path.
# They are preferable to the old workspace-local Lyrical vendor installation.
sudo apt-get update
sudo apt-get install -y \
  ros-humble-metavision-driver \
  ros-humble-event-camera-msgs \
  ros-humble-event-camera-codecs \
  ros-humble-event-camera-renderer

source /opt/ros/humble/setup.bash
cd "${workspace_dir}"
colcon build --symlink-install --packages-select evk4_driver --cmake-args -DBUILD_TESTING=OFF

sudo install -m 0644 \
  "${workspace_dir}/evk4_driver/udev/99-prophesee-evk4.rules" \
  /etc/udev/rules.d/99-prophesee-evk4.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb

echo
echo "Build complete. Unplug/replug the EVK4, then run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source ${workspace_dir}/install/setup.bash"
echo "  ros2 launch evk4_driver evk4_image_view.launch.py"
