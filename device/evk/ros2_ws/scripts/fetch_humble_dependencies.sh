#!/usr/bin/env bash
# Fetch the ROS 2 Humble source dependencies of evk4_driver.
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source_dir="${workspace_dir}/src"
mkdir -p "${source_dir}"

clone_humble() {
  local package="$1"
  local branch="$2"
  local destination="${source_dir}/${package}"
  if [ -e "${destination}" ]; then
    echo "Keeping existing ${destination}"
    return
  fi
  git clone --branch "${branch}" --depth 1 \
    "https://github.com/ros-event-camera/${package}.git" "${destination}"
}

clone_humble event_camera_msgs humble
clone_humble event_camera_codecs humble
clone_humble event_camera_renderer humble

# These two upstream repositories do not publish a Humble branch. Their
# release branches are the upstream compatibility line used with the Humble
# event-camera packages; pinning to the branch avoids accidentally consuming
# a newer rolling-only master revision.
clone_humble metavision_driver release
clone_humble openeb_vendor release

echo "Fetched EVK4 sources into ${source_dir}."
echo "Install OpenEB build prerequisites with rosdep, then build as described in README_humble.md."
