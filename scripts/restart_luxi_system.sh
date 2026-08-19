#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
readonly WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly RESTART_LOG="${WORKSPACE}/log/luxi_system_restart.log"
readonly LAUNCH_LOG="${WORKSPACE}/log/luxi_web_control_restart.log"
readonly PID_FILE="/tmp/luxi_web_control_launch.pid"

BIND_ADDRESS="0.0.0.0"
HTTP_PORT="8080"
WEB_UI_MODE="map_portal"

while (( $# > 0 )); do
    case "$1" in
        --bind-address)
            BIND_ADDRESS="${2:-}"
            shift 2
            ;;
        --http-port)
            HTTP_PORT="${2:-}"
            shift 2
            ;;
        --web-ui-mode)
            WEB_UI_MODE="${2:-}"
            shift 2
            ;;
        *)
            echo "Unknown restart argument: $1" >&2
            exit 2
            ;;
    esac
done

python3 - "${BIND_ADDRESS}" "${HTTP_PORT}" "${WEB_UI_MODE}" <<'PY'
import ipaddress
import sys

address, raw_port, mode = sys.argv[1:]
ipaddress.ip_address(address)
port = int(raw_port)
if not 1 <= port <= 65535:
    raise SystemExit("http port is outside 1..65535")
if mode not in {"developer", "map_portal"}:
    raise SystemExit("invalid web UI mode")
PY

mkdir -p "${WORKSPACE}/log"
echo "[$(date --iso-8601=seconds)] Restart requested." >>"${RESTART_LOG}"

# The HTTP handler returns its accepted response before this delay expires.
sleep 2
bash "${WORKSPACE}/scripts/stop_luxi_system.sh" >>"${RESTART_LOG}" 2>&1

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"
set -u

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

setsid ros2 launch luxi_web_control lekiwi_web_control.launch.py \
    "bind_address:=${BIND_ADDRESS}" \
    "http_port:=${HTTP_PORT}" \
    "web_ui_mode:=${WEB_UI_MODE}" >"${LAUNCH_LOG}" 2>&1 &
launch_pid=$!
echo "${launch_pid}" >"${PID_FILE}"

probe_address="${BIND_ADDRESS}"
if [[ "${probe_address}" == "0.0.0.0" ]]; then
    probe_address="127.0.0.1"
fi
status_url="http://${probe_address}:${HTTP_PORT}/api/status"

for _ in {1..300}; do
    if curl --fail --silent --connect-timeout 0.2 --max-time 1 \
            "${status_url}" >/dev/null 2>&1; then
        echo "[$(date --iso-8601=seconds)] Restart complete: ${status_url}" \
            >>"${RESTART_LOG}"
        exit 0
    fi
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
        echo "Web launch exited during restart; see ${LAUNCH_LOG}." >&2
        exit 1
    fi
    sleep 0.2
done

echo "Web service did not return within 60 seconds; see ${LAUNCH_LOG}." >&2
exit 1
