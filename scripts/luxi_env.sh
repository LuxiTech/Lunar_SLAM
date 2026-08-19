#!/usr/bin/env bash
# Source this file; do not execute it. It activates every overlay needed by D455.

_luxi_env_script="${BASH_SOURCE[0]}"
_luxi_env_root="$(cd -- "$(dirname -- "${_luxi_env_script}")/.." && pwd)"
_luxi_env_had_nounset=0
case "$-" in
  *u*) _luxi_env_had_nounset=1 ;;
esac

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'luxi_env: missing /opt/ros/humble/setup.bash\n' >&2
  return 1 2>/dev/null || exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f "${_luxi_env_root}/device/D455/ros2_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${_luxi_env_root}/device/D455/ros2_ws/install/setup.bash"
fi
if [[ -f "${_luxi_env_root}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${_luxi_env_root}/install/setup.bash"
fi
if ((_luxi_env_had_nounset)); then
  set -u
fi

export LUXI_WORKSPACE_ROOT="${_luxi_env_root}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

_luxi_rs_lib="${_luxi_env_root}/device/D455/ros2_ws/3parts/librealsense/install-rsusb/lib"
if [[ -d "${_luxi_rs_lib}" ]]; then
  case ":${LD_LIBRARY_PATH:-}:" in
    *":${_luxi_rs_lib}:"*) ;;
    *) export LD_LIBRARY_PATH="${_luxi_rs_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
  esac
fi

unset _luxi_env_script _luxi_env_root _luxi_rs_lib _luxi_env_had_nounset
