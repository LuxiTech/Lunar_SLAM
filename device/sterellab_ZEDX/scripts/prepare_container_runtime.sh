#!/usr/bin/env bash
set -euo pipefail

# ZED X uses host-side Argus and IMU Unix sockets.  The supported Docker setup
# bind-mounts the whole host /tmp into the container.  The current lunar_slam
# container only shares /tmp/.X11-unix, so refresh hard links to the live host
# sockets there before every launch.  This also survives an nvargus-daemon
# restart, which replaces /tmp/argus_socket with a new inode.

if systemctl is-system-running --quiet 2>/dev/null; then
  exit 0
fi

if [[ -S /tmp/argus_socket && ! -L /tmp/argus_socket ]]; then
  # The container already uses the official /tmp:/tmp bind mount.
  exit 0
fi

if ! sudo -n true 2>/dev/null; then
  printf '%s\n' \
    '[ERROR] Container socket preparation needs passwordless sudo.' \
    'Recreate the container with the official ZED X mount: -v /tmp:/tmp' >&2
  exit 1
fi

if ! sudo -n nsenter -t 1 -m -u -i -n -p true 2>/dev/null; then
  printf '%s\n' \
    '[ERROR] Host namespaces are unavailable. Start Docker with --pid=host,' \
    '--ipc=host, --network=host and --privileged.' >&2
  exit 1
fi

socket_names=(argus_socket camsock nvscsock imu_daemon.sock)
lock_names=(.zed_enum_lock .zed_gmsl_list_lock)

for name in "${socket_names[@]}"; do
  if ! sudo -n nsenter -t 1 -m test -S "/tmp/$name"; then
    printf '[ERROR] Host socket is missing: /tmp/%s\n' "$name" >&2
    exit 1
  fi

  sudo -n nsenter -t 1 -m \
    ln -f "/tmp/$name" "/tmp/.X11-unix/.zed_host_$name"
  sudo -n ln -sfn ".X11-unix/.zed_host_$name" "/tmp/$name"
done

for name in "${lock_names[@]}"; do
  if sudo -n nsenter -t 1 -m test -e "/tmp/$name"; then
    sudo -n nsenter -t 1 -m \
      ln -f "/tmp/$name" "/tmp/.X11-unix/.zed_host_$name"
    sudo -n ln -sfn ".X11-unix/.zed_host_$name" "/tmp/$name"
  fi
done

if [[ ! -r /etc/systemd/system/zed_x_daemon.service ]]; then
  printf '%s\n' \
    '[ERROR] Missing /etc/systemd/system/zed_x_daemon.service in container.' \
    'Add the official bind mount for this file.' >&2
  exit 1
fi

if [[ ! -d /var/nvidia/nvcam/settings ]]; then
  printf '%s\n' \
    '[ERROR] Missing /var/nvidia/nvcam/settings in container.' \
    'Add the official bind mount for this directory.' >&2
  exit 1
fi

if [[ -e /dev/spsc_bmi0 && ! -r /dev/spsc_bmi0 ]]; then
  printf '%s\n' \
    '[ERROR] IMU device is not readable. Add the container user to the host' \
    'IMU device GID (currently shown by: stat -c %g /dev/spsc_bmi0).' >&2
  exit 1
fi

printf '[OK] ZED X host runtime sockets are available in the container.\n'
