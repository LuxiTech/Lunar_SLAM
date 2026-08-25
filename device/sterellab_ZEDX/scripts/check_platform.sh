#!/usr/bin/env bash
set -euo pipefail

failures=0

check_equal() {
  local label="$1"
  local actual="$2"
  local expected="$3"

  if [[ "$actual" == "$expected" ]]; then
    printf '[OK] %s: %s\n' "$label" "$actual"
  else
    printf '[ERROR] %s: got %s, expected %s\n' "$label" "$actual" "$expected" >&2
    failures=$((failures + 1))
  fi
}

arch="$(uname -m)"
ubuntu_version="$(. /etc/os-release && printf '%s' "$VERSION_ID")"
l4t_release="$(sed -n 's/^# R\([0-9]*\).*REVISION: \([0-9.]*\).*/R\1.\2/p' /etc/nv_tegra_release 2>/dev/null || true)"
ros_distro="unset"

if [[ -r /opt/ros/humble/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
  ros_distro="${ROS_DISTRO:-unset}"
fi

check_equal 'architecture' "$arch" 'aarch64'
check_equal 'Ubuntu' "$ubuntu_version" '22.04'
check_equal 'Jetson Linux' "$l4t_release" 'R36.5.0'
check_equal 'ROS 2' "$ros_distro" 'humble'

if [[ -d /usr/local/zed ]]; then
  printf '[INFO] ZED SDK installation found: /usr/local/zed\n'
else
  printf '[INFO] ZED SDK is downloaded but not installed yet.\n'
fi

if (( failures > 0 )); then
  printf '[ERROR] Platform check failed; do not install these L4T-specific packages.\n' >&2
  exit 1
fi

printf '[OK] Platform matches JetPack 6.2.2 / L4T 36.5 / ROS 2 Humble.\n'
