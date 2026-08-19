#!/usr/bin/env bash
# Backward-compatible USB camera bootstrap for Ubuntu 22.04 / ROS 2 Humble.
set -euo pipefail

repository_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
exec "${repository_root}/scripts/bootstrap_humble_orin_nx.sh" --build usb "$@"
