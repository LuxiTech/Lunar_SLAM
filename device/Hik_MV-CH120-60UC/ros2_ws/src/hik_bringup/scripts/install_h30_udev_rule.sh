#!/usr/bin/env bash
# Install the project-owned persistent H30 udev rule. Run as the nvidia user.
set -euo pipefail

script_path="$(readlink -f "${BASH_SOURCE[0]}")"
package_dir="$(cd "$(dirname "${script_path}")/.." && pwd)"
rule_source="${package_dir}/config/99-h30-imu.rules"
rule_destination="/etc/udev/rules.d/99-h30-imu.rules"

if [[ ! -f "${rule_source}" ]]; then
  echo "Missing udev rule: ${rule_source}" >&2
  exit 2
fi

sudo install -o root -g root -m 0644 "${rule_source}" "${rule_destination}"
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add --subsystem-match=tty --sysname-match=ttyACM0
sudo udevadm settle

if [[ ! -e /dev/H30-imu ]]; then
  echo "Alias was not created. Reconnect the H30 USB cable, then rerun this script." >&2
  exit 1
fi

ls -l /dev/H30-imu
