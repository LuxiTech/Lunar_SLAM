#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
readonly WORKSPACE="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
readonly STATUS_URL="http://127.0.0.1:8080/api/status"
readonly STARTUP_TIMEOUT_SECONDS="${LUXI_STARTUP_TIMEOUT_SECONDS:-120}"

runtime_install="${LUXI_RUNTIME_INSTALL_PREFIX:-}"
if [[ -z "${runtime_install}" && \
      -f "${WORKSPACE}/runtime/lunar-client/install/local_setup.bash" ]]; then
    runtime_install="${WORKSPACE}/runtime/lunar-client/install"
fi
if [[ -z "${runtime_install}" ]]; then
    runtime_install="${WORKSPACE}/install"
fi
readonly RUNTIME_INSTALL="${runtime_install}"
unset runtime_install

runtime_setup=""
for setup_candidate in \
        "${RUNTIME_INSTALL}/setup.bash" \
        "${RUNTIME_INSTALL}/local_setup.bash"; do
    if [[ -r "${setup_candidate}" ]]; then
        runtime_setup="${setup_candidate}"
        break
    fi
done
readonly RUNTIME_SETUP="${runtime_setup}"
unset runtime_setup setup_candidate

d1_lan_dds_setup=""
for dds_candidate in \
        "${RUNTIME_INSTALL}/lib/slam_d1_bridge/setup_d1_lan_dds.sh" \
        "${RUNTIME_INSTALL}/slam_d1_bridge/lib/slam_d1_bridge/setup_d1_lan_dds.sh"; do
    if [[ -r "${dds_candidate}" ]]; then
        d1_lan_dds_setup="${dds_candidate}"
        break
    fi
done
readonly D1_LAN_DDS_SETUP="${d1_lan_dds_setup}"
unset d1_lan_dds_setup dds_candidate

launch_pid=""

post_json()
{
    local path="$1"
    local body="$2"
    curl --fail --silent --show-error --connect-timeout 1 --max-time 30 \
        -X POST -H "Content-Type: application/json" -d "${body}" \
        "http://127.0.0.1:8080${path}"
}

http_listener_belongs_to_launch()
{
    local listener_pid
    local ancestor_pid
    while read -r listener_pid; do
        [[ "${listener_pid}" =~ ^[0-9]+$ ]] || continue
        ancestor_pid="${listener_pid}"
        while [[ "${ancestor_pid}" =~ ^[0-9]+$ ]] && (( ancestor_pid > 1 )); do
            if [[ "${ancestor_pid}" == "${launch_pid}" ]]; then
                return 0
            fi
            ancestor_pid="$(
                awk '/^PPid:/{print $2}' \
                    "/proc/${ancestor_pid}/status" 2>/dev/null || true
            )"
        done
    done < <(
        ss -H -ltnp 'sport = :8080' 2>/dev/null |
            grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u
    )
    return 1
}

shutdown_launch()
{
    local status=$?
    trap - EXIT INT TERM
    post_json /api/estop '{"active":true}' >/dev/null 2>&1 || true
    post_json /api/stop \
        '{"client_id":"boot-service-stop","force":true}' >/dev/null 2>&1 || true
    if [[ "${launch_pid}" =~ ^[0-9]+$ ]] && kill -0 "${launch_pid}" 2>/dev/null; then
        kill -INT "${launch_pid}" 2>/dev/null || true
        wait "${launch_pid}" 2>/dev/null || true
    fi
    exit "${status}"
}
trap shutdown_launch EXIT INT TERM

if [[ -z "${RUNTIME_SETUP}" ]]; then
    echo "Luxi runtime overlay is missing in ${RUNTIME_INSTALL}." >&2
    exit 1
fi
if [[ -z "${D1_LAN_DDS_SETUP}" ]]; then
    echo "D1 LAN DDS setup is missing from ${RUNTIME_INSTALL}." >&2
    exit 1
fi

mkdir -p "${WORKSPACE}/log"

# NetworkManager may report network-online before the dedicated static D1
# address has appeared. Wait for the exact configured address so DDS never
# selects Wi-Fi, Docker or a VPN interface for robot traffic.
deadline=$((SECONDS + STARTUP_TIMEOUT_SECONDS))
until ip -4 -o addr show scope global | grep -Eq \
        '^[0-9]+: [^ ]+ +inet 192\.168\.123\.66/24([[:space:]]|$)'; do
    if (( SECONDS >= deadline )); then
        echo "Timed out waiting for D1 LAN address 192.168.123.66/24." >&2
        exit 1
    fi
    sleep 1
done

# shellcheck disable=SC1091
export LUXI_RUNTIME_INSTALL_PREFIX="${RUNTIME_INSTALL}"
source "${WORKSPACE}/scripts/luxi_env.sh"
# Restrict DDS discovery and D1 command traffic to loopback plus the dedicated
# 192.168.123.0/24 Ethernet interface.
# shellcheck disable=SC1090
source "${D1_LAN_DDS_SETUP}"

echo "Starting Luxi web control on 0.0.0.0:8080 with D455 profile."
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
    bind_address:=0.0.0.0 \
    http_port:=8080 \
    web_ui_mode:=map_portal \
    d1_start_script:="${RUNTIME_INSTALL}/lib/slam_d1_bridge/start_slam_d1_bridge.sh" \
    d1_stop_script:="${RUNTIME_INSTALL}/lib/slam_d1_bridge/stop_slam_d1_bridge.sh" \
    d1_control_log_path:="${WORKSPACE}/log/luxi_web_control_d1.log" \
    camera_workspace_setup:="${WORKSPACE}/device/D455/ros2_ws/install/setup.bash" \
    camera_log_path:="${WORKSPACE}/log/luxi_web_control_camera.log" \
    mapping_workspace_setup:="${RUNTIME_SETUP}" \
    mapping_log_path:="${WORKSPACE}/log/luxi_web_control_rtabmap.log" \
    navigation_workspace_setup:="${RUNTIME_SETUP}" \
    navigation_log_path:="${WORKSPACE}/log/luxi_web_control_navigation.log" \
    maps_root:="${WORKSPACE}/maps" \
    semantic_maps_root:="${WORKSPACE}/maps/semantic_maps" &
launch_pid=$!

deadline=$((SECONDS + STARTUP_TIMEOUT_SECONDS))
until http_listener_belongs_to_launch && \
        curl --fail --silent --connect-timeout 1 --max-time 2 \
            "${STATUS_URL}" >/dev/null 2>&1; do
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
        wait "${launch_pid}" || true
        echo "Luxi web launch exited before the HTTP endpoint was ready." >&2
        exit 1
    fi
    if (( SECONDS >= deadline )); then
        echo "Timed out waiting for ${STATUS_URL}." >&2
        exit 1
    fi
    sleep 1
done

# Clear any stale browser command before releasing the web software emergency
# stop. D1 posture/SDK control remains an explicit operator action and is never
# enabled here, so boot still cannot make the robot stand or move by itself.
post_json /api/stop \
    '{"client_id":"boot-service-stop","force":true}' >/dev/null
post_json /api/estop '{"active":false}' >/dev/null

# The old web instance may need a moment to release its managed camera during a
# service restart. Retry until this instance owns exactly one D455 pipeline.
deadline=$((SECONDS + STARTUP_TIMEOUT_SECONDS))
while true; do
    post_json /api/camera/start '{"profile":"d455"}' >/dev/null 2>&1 || true
    if http_listener_belongs_to_launch && \
        curl --fail --silent --connect-timeout 1 --max-time 2 "${STATUS_URL}" |
        python3 -c '
import json
import sys

camera = json.load(sys.stdin).get("camera", {})
raise SystemExit(0 if camera.get("profile") == "d455"
                 and camera.get("ready") is True else 1)
'; then
        break
    fi
    if ! kill -0 "${launch_pid}" 2>/dev/null; then
        wait "${launch_pid}" || true
        echo "Luxi web launch exited while starting D455." >&2
        exit 1
    fi
    if (( SECONDS >= deadline )); then
        echo "Timed out waiting for the managed D455 pipeline." >&2
        exit 1
    fi
    sleep 2
done

echo "Luxi boot service ready: http://127.0.0.1:8080 (D455 ready, zero command, emergency stop released)."
wait "${launch_pid}"
