#!/usr/bin/env bash
set -euo pipefail

workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
rule_file="/etc/udev/rules.d/99-prophesee-evk4.rules"
package_rule="${workspace}/src/evk/evk4_driver/udev/99-prophesee-evk4.rules"
cypress_rule="${workspace}/src/evk/metavision_driver/udev/rules.d/88-cyusb.rules"

if ! command -v sudo >/dev/null 2>&1; then
  echo "需要 sudo 才能安装 udev 规则。" >&2
  exit 1
fi

if [ ! -f "${package_rule}" ]; then
  echo "找不到 EVK4 udev 规则：${package_rule}" >&2
  exit 1
fi

sudo install -m 0644 "${package_rule}" "${rule_file}"

if [ -f "${cypress_rule}" ]; then
  sudo install -m 0644 "${cypress_rule}" /etc/udev/rules.d/88-cyusb.rules
fi

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --attr-match=idVendor=04b4 || true

echo "EVK4 udev 规则已安装。请拔插相机后检查："
echo "  lsusb | grep -i '04b4:00f5'"
echo "  ls -l /dev/bus/usb/002/003"
