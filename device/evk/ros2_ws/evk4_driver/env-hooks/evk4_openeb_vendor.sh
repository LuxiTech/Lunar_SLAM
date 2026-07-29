# EVK4 HD 本地 OpenEB / MetaVision SDK 环境。
#
# 本 hook 由 install/setup.bash 自动加载。它把工作空间中的
# .local_ros/opt/ros/<ROS_DISTRO>/opt/openeb_vendor 加入运行环境。

_evk4_prepend_unique() {
  _evk4_var_name="$1"
  _evk4_value="$2"
  if [ -z "${_evk4_value}" ]; then
    return
  fi
  eval _evk4_current=\"\$${_evk4_var_name}\"
  case ":${_evk4_current}:" in
    *:"${_evk4_value}":*) ;;
    *)
      if [ -n "${_evk4_current}" ]; then
        eval export "${_evk4_var_name}=\"${_evk4_value}:${_evk4_current}\""
      else
        eval export "${_evk4_var_name}=\"${_evk4_value}\""
      fi
      ;;
  esac
  unset _evk4_var_name
  unset _evk4_value
  unset _evk4_current
}

_evk4_install_prefix="${AMENT_CURRENT_PREFIX:-${COLCON_CURRENT_PREFIX}}"
if [ -z "${_evk4_install_prefix}" ] && [ -n "${COLCON_PREFIX_PATH}" ]; then
  _evk4_install_prefix="${COLCON_PREFIX_PATH}/evk4_driver"
fi
if [ -z "${_evk4_install_prefix}" ] && [ -n "${BASH_SOURCE:-}" ]; then
  _evk4_install_prefix="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
fi
_evk4_ws="$(builtin cd "${_evk4_install_prefix}/../.." && pwd)"
_evk4_ros_distro="${ROS_DISTRO:-humble}"
_evk4_local_ros="${_evk4_ws}/.local_ros/opt/ros/${_evk4_ros_distro}"
_evk4_openeb="${_evk4_local_ros}/opt/openeb_vendor"

if [ -d "${_evk4_local_ros}" ]; then
  _evk4_prepend_unique AMENT_PREFIX_PATH "${_evk4_local_ros}"
  _evk4_prepend_unique CMAKE_PREFIX_PATH "${_evk4_local_ros}"
fi

if [ -d "${_evk4_openeb}" ]; then
  _evk4_prepend_unique CMAKE_PREFIX_PATH "${_evk4_openeb}"

  if [ -d "${_evk4_openeb}/bin" ]; then
    _evk4_prepend_unique PATH "${_evk4_openeb}/bin"
  fi

  if [ -d "${_evk4_openeb}/lib" ]; then
    _evk4_prepend_unique LD_LIBRARY_PATH "${_evk4_openeb}/lib"
  fi

  if [ -d "${_evk4_openeb}/lib/metavision/hal/plugins" ]; then
    _evk4_prepend_unique MV_HAL_PLUGIN_PATH "${_evk4_openeb}/lib/metavision/hal/plugins"
  fi
fi

unset -f _evk4_prepend_unique
unset _evk4_install_prefix
unset _evk4_ws
unset _evk4_local_ros
unset _evk4_openeb
unset _evk4_ros_distro
