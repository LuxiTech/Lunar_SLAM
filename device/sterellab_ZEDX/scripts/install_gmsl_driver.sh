#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "$script_dir/.." && pwd)"

usage() {
  printf 'Usage: %s {mono|duo|quad}\n' "$(basename "$0")" >&2
  printf 'Select the physical ZED Link capture-card model, not the camera count.\n' >&2
}

if [[ $# -ne 1 ]]; then
  usage
  exit 2
fi

"$script_dir/check_platform.sh"

case "$1" in
  mono)
    package='stereolabs-zedlink-mono_1.4.3-SL-MAX9296-L4T36.5.0_arm64.deb'
    expected_sha256='42a7c6b220b6bbc42e6598f39750c57260ea4b6d3d0480bf157d1f7965081ecd'
    ;;
  duo)
    package='stereolabs-zedlink-duo_1.4.3-LI-MAX96712-L4T36.5.0_arm64.deb'
    expected_sha256='12c101be7a8963ab501e0382286e9da9287d44f40e6ca91258c8c05d52abc889'
    ;;
  quad)
    package='stereolabs-zedlink-quad_1.4.3-SL-MAX96712-L4T36.5.0_arm64.deb'
    expected_sha256='8603c99a923b250cb79fd5e39bc825cc0f38c8b93cfbe5582a89c02046cfc370'
    ;;
  *)
    usage
    exit 2
    ;;
esac

deb="$project_dir/installers/$package"
if [[ ! -f "$deb" ]]; then
  printf 'Driver package is missing: %s\n' "$deb" >&2
  exit 1
fi

printf '%s  %s\n' "$expected_sha256" "$deb" | sha256sum --check --status
printf '[OK] Driver checksum verified: %s\n' "$package"
printf '[INFO] Installing the %s driver. Do not install another capture-card variant.\n' "$1"

sudo apt-get install -y "$deb"

printf '[OK] Driver installation completed. Reboot before connecting or testing ZED X.\n'
