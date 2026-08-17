#!/usr/bin/env bash
set -euo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
script_dir="$(dirname "${script_path}")"
source_rule="$(readlink -f "${script_dir}/../config/99-h30-imu.rules" 2>/dev/null || true)"
installed_rule="$(readlink -f "${script_dir}/../../share/usb_camera_bringup/config/99-h30-imu.rules" 2>/dev/null || true)"
if [[ -f "${source_rule}" ]]; then
  rule_source="${source_rule}"
else
  rule_source="${installed_rule}"
fi
rule_destination="/etc/udev/rules.d/99-h30-imu.rules"

if [[ ! -f "${rule_source}" ]]; then
  echo "Missing H30 udev rule: ${rule_source}" >&2
  exit 2
fi

sudo install -o root -g root -m 0644 "${rule_source}" "${rule_destination}"
sudo udevadm control --reload-rules
if [[ -e /sys/class/tty/ttyACM0 ]]; then
  sudo udevadm trigger --action=add /sys/class/tty/ttyACM0
fi
sudo udevadm settle

if [[ ! -e /dev/imu-H30 ]]; then
  echo "The /dev/imu-H30 alias was not created; reconnect this H30 and retry." >&2
  exit 1
fi

ls -l /dev/imu-H30
