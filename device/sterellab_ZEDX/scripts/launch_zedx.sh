#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
workspace="$project_dir/ros2_ws"

if [[ ! -r "$workspace/install/local_setup.bash" ]]; then
  printf 'Workspace is not built. Run scripts/build_ros2.sh first.\n' >&2
  exit 1
fi

# usermod changes are not reflected in already-open VS Code terminals.  Start a
# fresh process with the configured supplementary groups when only that stale
# group list prevents access to the ZED X IMU device.
if [[ -e /dev/spsc_bmi0 && ! -r /dev/spsc_bmi0 ]]; then
  imu_gid="$(stat -c '%g' /dev/spsc_bmi0)"
  run_user="$(id -un)"
  if [[ " $(id -G "$run_user") " == *" $imu_gid "* ]]; then
    exec sudo -n -u "$run_user" -H env \
      DISPLAY="${DISPLAY:-}" \
      XAUTHORITY="${XAUTHORITY:-}" \
      "$0" "$@"
  fi
fi

"$script_dir/prepare_container_runtime.sh"

# CUDA-based ZED applications on Jetson need a working X11/EGL display.  The
# development container may retain DISPLAY=:1 even though only X0 is mounted.
display_number="${DISPLAY:-}"
display_number="${display_number#:}"
display_number="${display_number%%.*}"
if [[ -z "${DISPLAY:-}" || ! -S "/tmp/.X11-unix/X${display_number}" ]]; then
  if [[ -S /tmp/.X11-unix/X0 ]]; then
    export DISPLAY=:0
  fi
fi
if [[ -r /tmp/.docker.xauth ]]; then
  export XAUTHORITY=/tmp/.docker.xauth
fi
if [[ -z "${XDG_RUNTIME_DIR:-}" || ! -d "${XDG_RUNTIME_DIR:-}" ]]; then
  export XDG_RUNTIME_DIR="/tmp/zedx-runtime-${UID}"
  mkdir -p "$XDG_RUNTIME_DIR"
  chmod 700 "$XDG_RUNTIME_DIR"
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$workspace/install/local_setup.bash"
set -u

if systemctl is-system-running --quiet 2>/dev/null; then
  daemon_active_cmd=(systemctl is-active --quiet zed_x_daemon)
else
  daemon_active_cmd=(sudo -n nsenter -t 1 -m -u -i -n -p
    systemctl is-active --quiet zed_x_daemon)
fi

if ! "${daemon_active_cmd[@]}"; then
  printf '[WARN] zed_x_daemon is not active; check the GMSL driver and reboot state.\n' >&2
fi

exec ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx "$@"
