#!/usr/bin/env bash
# 加载 EVK4 HD 驱动运行环境。请使用 source 执行本脚本。
#
# Ubuntu 22.04 的默认目标是 ROS 2 Humble。ROS_DISTRO 可用于显式覆盖，
# 方便同一份驱动在其他发行版中进行验证。

_evk4_pkg_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_evk4_setup_ws="$(cd "${_evk4_pkg_dir}/../../.." && pwd)"

_evk4_ros_distro="${ROS_DISTRO:-humble}"
_evk4_ros_setup="/opt/ros/${_evk4_ros_distro}/setup.bash"
if [ ! -f "${_evk4_ros_setup}" ]; then
  echo "EVK4: ROS 2 ${_evk4_ros_distro} is not installed: ${_evk4_ros_setup}" >&2
  return 1 2>/dev/null || exit 1
fi
source "${_evk4_ros_setup}"

if [ -f "${_evk4_setup_ws}/.local_ros/opt/ros/${_evk4_ros_distro}/local_setup.bash" ]; then
  source "${_evk4_setup_ws}/.local_ros/opt/ros/${_evk4_ros_distro}/local_setup.bash"
fi

# Keep this list aligned with the EVK4 runtime chain. Source packages in
# dependency order so the standalone setup script exposes the renderer too.
for _pkg in event_camera_msgs metavision_driver event_camera_codecs event_camera_renderer evk4_driver; do
  if [ -f "${_evk4_setup_ws}/install/${_pkg}/share/${_pkg}/local_setup.bash" ]; then
    source "${_evk4_setup_ws}/install/${_pkg}/share/${_pkg}/local_setup.bash"
  fi
done

_evk4_local_ros="${_evk4_setup_ws}/.local_ros/opt/ros/${_evk4_ros_distro}"
_evk4_openeb_vendor="${_evk4_local_ros}/opt/openeb_vendor"
if [ -d "${_evk4_local_ros}" ]; then
  export AMENT_PREFIX_PATH="${_evk4_local_ros}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
  export CMAKE_PREFIX_PATH="${_evk4_local_ros}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
fi
if [ -d "${_evk4_openeb_vendor}" ]; then
  export CMAKE_PREFIX_PATH="${_evk4_openeb_vendor}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
  export LD_LIBRARY_PATH="${_evk4_openeb_vendor}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
  export MV_HAL_PLUGIN_PATH="${_evk4_openeb_vendor}/lib/metavision/hal/plugins${MV_HAL_PLUGIN_PATH:+:${MV_HAL_PLUGIN_PATH}}"
fi
export PATH="${_evk4_setup_ws}/.local_tools/usr/bin${PATH:+:${PATH}}"
export PYTHONPATH="${_evk4_setup_ws}/.local_tools/usr/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"

unset _evk4_pkg_dir
unset _evk4_setup_ws
unset _pkg
unset _evk4_ros_distro
unset _evk4_ros_setup
unset _evk4_local_ros
unset _evk4_openeb_vendor
