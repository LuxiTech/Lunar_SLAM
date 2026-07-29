#!/usr/bin/env bash
# 加载 EVK4 HD 驱动运行环境。请使用 source 执行本脚本。

_evk4_pkg_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_evk4_setup_ws="$(cd "${_evk4_pkg_dir}/../../.." && pwd)"

source /opt/ros/lyrical/setup.bash

if [ -f "${_evk4_setup_ws}/.local_ros/opt/ros/lyrical/local_setup.bash" ]; then
  source "${_evk4_setup_ws}/.local_ros/opt/ros/lyrical/local_setup.bash"
fi

# Keep this list aligned with the EVK4 runtime chain. Source packages in
# dependency order so the standalone setup script exposes the renderer too.
for _pkg in event_camera_msgs metavision_driver event_camera_codecs event_camera_renderer evk4_driver; do
  if [ -f "${_evk4_setup_ws}/install/${_pkg}/share/${_pkg}/local_setup.bash" ]; then
    source "${_evk4_setup_ws}/install/${_pkg}/share/${_pkg}/local_setup.bash"
  fi
done

export AMENT_PREFIX_PATH="${_evk4_setup_ws}/.local_ros/opt/ros/lyrical${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export CMAKE_PREFIX_PATH="${_evk4_setup_ws}/.local_ros/opt/ros/lyrical:${_evk4_setup_ws}/.local_ros/opt/ros/lyrical/opt/openeb_vendor${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${_evk4_setup_ws}/.local_ros/opt/ros/lyrical/opt/openeb_vendor/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export MV_HAL_PLUGIN_PATH="${_evk4_setup_ws}/.local_ros/opt/ros/lyrical/opt/openeb_vendor/lib/metavision/hal/plugins${MV_HAL_PLUGIN_PATH:+:${MV_HAL_PLUGIN_PATH}}"
export PATH="${_evk4_setup_ws}/.local_tools/usr/bin${PATH:+:${PATH}}"
export PYTHONPATH="${_evk4_setup_ws}/.local_tools/usr/lib/python3/dist-packages${PYTHONPATH:+:${PYTHONPATH}}"

unset _evk4_pkg_dir
unset _evk4_setup_ws
unset _pkg
