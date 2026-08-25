#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"
installer="$project_dir/installers/ZED_SDK_Tegra_L4T36.5_v5.4.0.zstd.run"
expected_sha256='c2776f846072692a6207e6b8cf949992616e351fb513b96d444eec9c257dd090'

"$script_dir/check_platform.sh"

if [[ ! -x "$installer" ]]; then
  printf 'Installer is missing or is not executable: %s\n' "$installer" >&2
  exit 1
fi

printf '%s  %s\n' "$expected_sha256" "$installer" | sha256sum --check --status
printf '[OK] ZED SDK 5.4.0 installer checksum verified.\n'
printf '[INFO] Starting the official installer. Read and accept the Stereolabs license.\n'

exec "$installer" "$@"
