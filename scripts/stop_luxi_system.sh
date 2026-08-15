#!/usr/bin/env bash

set -euo pipefail

readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
readonly CURRENT_UID="$(id -u)"
# Match the complete Luxi/D1 package families instead of maintaining a fragile
# executable-by-executable list. External sensor and RTAB-Map processes are
# included explicitly because they are installed outside this workspace.
readonly PROCESS_PATTERN='[/]lib[/](luxi_[^/[:space:]]+|slam_d1_bridge|rtabmap_(slam|odom|sync|viz)|realsense2_camera|imu_filter_madgwick|yesense_std_ros2|stereo_depth|hikrobot_camera_driver)[/]|[r]os2[[:space:]]+launch[[:space:]]+(luxi_[^[:space:]]+|slam_d1_bridge|lunar_realsense_bringup|hik_bringup)[[:space:]]|__node:=[l]uxi_sensor_container|__node:=[s]ensor_imu_filter|__node:=[b]ase_to_sensor_tf|__node:=[h]ik_base_to_left_camera_tf|__node:=[h]ik_left_camera_to_imu_tf|[s]ensor_bringup\.launch\.py|[d]435i\.launch\.py|[s]tereo_camera_bringup\.launch\.py|[s]aved_map_navigation\.launch\.py|[r]gbd_mapping(_learned)?\.launch\.py|[s]lam_d1_bridge\.launch\.py|[l]ekiwi_web_control\.launch\.py|[w]eb_control\.launch\.py'
readonly PID_FILES=(
    "/tmp/d1_web_control.pid"
    "/tmp/luxi_web_control_launch.pid"
    "/tmp/slam_d1_bridge_d15041873.pid"
)

WEB_URLS=("http://127.0.0.1:8080")
while read -r local_address; do
    [[ -n "${local_address}" ]] || continue
    WEB_URLS+=("http://${local_address}:8080")
done < <(
    ip -4 -o addr show scope global 2>/dev/null |
        awk '{sub(/\/.*/, "", $4); print $4}'
)

WEB_URL=""
for candidate_url in "${WEB_URLS[@]}"; do
    if curl --fail --silent --connect-timeout 0.2 --max-time 1 \
            "${candidate_url}/api/status" >/dev/null 2>&1; then
        WEB_URL="${candidate_url}"
        break
    fi
done

protected_pids=("$$")
ancestor_pid="${PPID}"
while [[ "${ancestor_pid}" =~ ^[0-9]+$ ]] && (( ancestor_pid > 1 )); do
    protected_pids+=("${ancestor_pid}")
    ancestor_pid="$(awk '/^PPid:/{print $2}' "/proc/${ancestor_pid}/status" 2>/dev/null || true)"
done

post_if_available()
{
    local path="$1"
    local body='{}'
    [[ -n "${WEB_URL}" ]] || return 0
    if [[ "${path}" == "/api/stop" ]]; then
        body='{"client_id":"system-safety-stop","force":true}'
    fi
    # Mapping/navigation shutdown can wait for child process groups and a map
    # database flush, so allow more time after the live endpoint is identified.
    curl --fail --silent --show-error --connect-timeout 0.2 --max-time 15 \
        -X POST -H "Content-Type: application/json" -d "${body}" \
        "${WEB_URL}${path}" >/dev/null 2>&1 || true
    return 0
}

request_robot_shutdown()
{
    local response
    local status
    local attempt

    # The endpoint starts the safe zero -> lie-down -> SDK-release transition.
    # A missing/disabled web service is harmless; process cleanup still follows.
    [[ -n "${WEB_URL}" ]] || return 0
    response="$(
        curl --fail --silent --show-error --connect-timeout 0.2 --max-time 2 \
            -X POST -H "Content-Type: application/json" \
            -d '{"active":false}' \
            "${WEB_URL}/api/robot/control" 2>/dev/null || true
    )"
    [[ -n "${response}" ]] || return 0

    for ((attempt=0; attempt<900; attempt++)); do
        status="$(
            curl --fail --silent --connect-timeout 0.2 --max-time 1 \
                "${WEB_URL}/api/status" 2>/dev/null || true
        )"
        [[ -n "${status}" ]] || return 0
        if python3 -c '
import json, sys
try:
    control = json.load(sys.stdin).get("robot_control", {})
except (json.JSONDecodeError, OSError):
    raise SystemExit(1)
raise SystemExit(0 if not control.get("transitioning", False)
                 and not control.get("active", False) else 1)
' <<<"${status}"; then
            return 0
        fi
        sleep 0.1
    done

    echo "D1 safe shutdown did not finish within 90 seconds; forcing local process cleanup." >&2
}

matching_pids()
{
    local candidate
    local protected
    while read -r candidate; do
        [[ -n "${candidate}" ]] || continue
        for protected in "${protected_pids[@]}"; do
            if [[ "${candidate}" == "${protected}" ]]; then
                continue 2
            fi
        done
        echo "${candidate}"
    done < <(pgrep -u "${CURRENT_UID}" -f "${PROCESS_PATTERN}" || true)
}

signal_matches()
{
    local signal_name="$1"
    mapfile -t pids < <(matching_pids)
    if (( ${#pids[@]} == 0 )); then
        return
    fi
    kill -s "${signal_name}" "${pids[@]}" 2>/dev/null || true
}

wait_for_exit()
{
    local attempts="$1"
    local attempt
    for ((attempt=0; attempt<attempts; attempt++)); do
        if [[ -z "$(matching_pids)" ]]; then
            return 0
        fi
        sleep 0.1
    done
    return 1
}

remove_stale_pid_files()
{
    local pid_file
    local recorded_pid
    for pid_file in "${PID_FILES[@]}"; do
        [[ -f "${pid_file}" ]] || continue
        recorded_pid="$(head -n 1 "${pid_file}" 2>/dev/null || true)"
        if [[ ! "${recorded_pid}" =~ ^[0-9]+$ ]] || \
                ! kill -0 "${recorded_pid}" 2>/dev/null; then
            rm -f -- "${pid_file}"
        fi
    done
}

post_if_available /api/stop
post_if_available /api/mapping/stop
post_if_available /api/navigation/stop
request_robot_shutdown
sleep 2

signal_matches INT
if ! wait_for_exit 50; then
    echo "Luxi processes did not stop after SIGINT; sending SIGTERM." >&2
    signal_matches TERM
    wait_for_exit 30 || true
fi

remaining="$(matching_pids)"
if [[ -n "${remaining}" ]]; then
    echo "Luxi processes are still running:" >&2
    process_list="$(tr '\n' ',' <<<"${remaining}" | sed 's/,$//')"
    ps -o pid,ppid,pgid,stat,cmd -p "${process_list}" >&2
    exit 1
fi

remove_stale_pid_files

listener="$(ss -H -ltnp 'sport = :8080' 2>/dev/null || true)"
if [[ -n "${listener}" ]]; then
    echo "Port 8080 is still occupied by a non-Luxi process:" >&2
    echo "${listener}" >&2
    exit 1
fi

echo "Luxi/D1 processes stopped, stale PID files removed, and port 8080 is available."
