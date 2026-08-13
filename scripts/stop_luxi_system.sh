#!/usr/bin/env bash

set -euo pipefail

readonly WEB_URL="http://127.0.0.1:8080"
readonly CURRENT_UID="$(id -u)"
readonly PROCESS_PATTERN='[/](luxi_web_control/lib/luxi_web_control/web_control_node|luxi_3d_navigation/lib/luxi_3d_navigation/velocity_command_mux_node|luxi_visual_frontend/lib/luxi_visual_frontend/visual_odometry_node|rtabmap_slam/rtabmap|luxi_adapter/lib/luxi_adapter/sensor_adapter_node|realsense2_camera/lib/realsense2_camera/realsense2_camera_node|imu_filter_madgwick/lib/imu_filter_madgwick/imu_filter_madgwick_node|yesense_std_ros2/lib/yesense_std_ros2/yesense_node_publisher|stereo_depth/lib/stereo_depth/stereo_depth_node|hikrobot_camera_driver/lib/hikrobot_camera_driver/stereo_node)|__node:=[l]uxi_sensor_container|[s]ensor_bringup\.launch\.py|[d]435i\.launch\.py|[s]tereo_camera_bringup\.launch\.py|[l]ekiwi_web_control\.launch\.py|[w]eb_control\.launch\.py'

protected_pids=("$$")
ancestor_pid="${PPID}"
while [[ "${ancestor_pid}" =~ ^[0-9]+$ ]] && (( ancestor_pid > 1 )); do
    protected_pids+=("${ancestor_pid}")
    ancestor_pid="$(awk '/^PPid:/{print $2}' "/proc/${ancestor_pid}/status" 2>/dev/null || true)"
done

post_if_available()
{
    local path="$1"
    curl --fail --silent --show-error \
        -X POST -H "Content-Type: application/json" -d '{}' \
        "${WEB_URL}${path}" >/dev/null 2>&1 || true
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

post_if_available /api/stop
post_if_available /api/mapping/stop
post_if_available /api/navigation/stop
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

listener="$(ss -H -ltnp 'sport = :8080' 2>/dev/null || true)"
if [[ -n "${listener}" ]]; then
    echo "Port 8080 is still occupied by a non-Luxi process:" >&2
    echo "${listener}" >&2
    exit 1
fi

echo "Luxi processes stopped and port 8080 is available."
