#!/usr/bin/env bash
# Backward-compatible Hik entry point.  Shared setup is maintained centrally
# so all device workspaces use the same Ubuntu 22.04 / ROS 2 Humble underlay.
set -euo pipefail

repository_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)"
exec "${repository_root}/scripts/bootstrap_humble_orin_nx.sh" \
  --build hik --mvs-root "${MVS_ROOT:-/opt/MVS}" "$@"
