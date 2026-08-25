#!/usr/bin/env bash
set -euo pipefail

failures=0

check_service() {
  local service="$1"
  local service_type
  local result
  local main_status

  if systemctl is-active --quiet "$service"; then
    printf '[OK] service active: %s\n' "$service"
    return
  fi

  service_type="$(systemctl show "$service" -p Type --value)"
  result="$(systemctl show "$service" -p Result --value)"
  main_status="$(systemctl show "$service" -p ExecMainStatus --value)"
  if [[ "$service_type" == 'oneshot' && "$result" == 'success' && "$main_status" == '0' ]]; then
    printf '[OK] oneshot service completed successfully: %s\n' "$service"
    return
  fi

  printf '[ERROR] service is not healthy: %s\n' "$service" >&2
  systemctl status "$service" --no-pager 2>&1 | sed -n '1,30p' >&2 || true
  failures=$((failures + 1))
}

if ! systemctl is-system-running --quiet; then
  printf '[ERROR] Run this script on the Jetson host, not in a container.\n' >&2
  exit 2
fi

if [[ "$(uname -r)" != '5.15.185-tegra' ]]; then
  printf '[ERROR] Unexpected kernel: %s\n' "$(uname -r)" >&2
  failures=$((failures + 1))
else
  printf '[OK] kernel: %s\n' "$(uname -r)"
fi

driver_version="$(dpkg-query -W -f='${Version}' stereolabs-zedlink-duo 2>/dev/null || true)"
if [[ "$driver_version" == '1.4.3-LI-MAX96712-L4T36.5.0' ]]; then
  printf '[OK] ZED Link Duo driver: %s\n' "$driver_version"
else
  printf '[ERROR] Unexpected ZED Link Duo driver: %s\n' "${driver_version:-not installed}" >&2
  failures=$((failures + 1))
fi

sdk_version="$(sed -n 's/set(PACKAGE_VERSION "\([^"]*\)")/\1/p' /usr/local/zed/zed-config-version.cmake 2>/dev/null)"
if [[ "$sdk_version" == '5.4.0' ]]; then
  printf '[OK] ZED SDK: %s\n' "$sdk_version"
else
  printf '[ERROR] Unexpected ZED SDK: %s\n' "${sdk_version:-not installed}" >&2
  failures=$((failures + 1))
fi

check_service driver_zed_loader
check_service zed_x_daemon
check_service IMU_Daemon

mapfile -t video_devices < <(find /dev -maxdepth 1 -name 'video*' -printf '%p\n' | sort -V)
if (( ${#video_devices[@]} > 0 )); then
  printf '[OK] video devices:\n'
  printf '  %s\n' "${video_devices[@]}"
else
  printf '[ERROR] No /dev/video* devices were created.\n' >&2
  failures=$((failures + 1))
fi

printf '%s\n' '--- recent ZED X kernel messages ---'
dmesg | grep -i zedx | tail -n 80 || true

if (( failures > 0 )); then
  printf '[ERROR] Post-reboot verification failed (%d checks).\n' "$failures" >&2
  exit 1
fi

printf '[OK] ZED Link Duo and ZED X host configuration is healthy.\n'
