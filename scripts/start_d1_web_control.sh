#!/usr/bin/env bash

set -euo pipefail

readonly WORKSPACE="${LUXI_WORKSPACE_ROOT:-/home/nvidia/Desktop/lunar_-slam}"
readonly WEB_URL="http://127.0.0.1:8080"
readonly WEB_LOG="${WORKSPACE}/log/d1_web_control.log"
readonly WEB_PID_FILE="/tmp/d1_web_control.pid"

ROBOT_IP="${D1_ROBOT_IP:-192.168.123.49}"
ROBOT_NS="${ROBOT_NS:-d15041873}"
D1_SSH_USER="${D1_SSH_USER:-robot}"

RUN_MOTION_TEST=false
ASSUME_YES=false
AUTO_RECOVER_DDS=true
while (($# > 0)); do
    case "$1" in
        --robot-ns)
            if (($# < 2)); then
                echo "--robot-ns requires a value." >&2
                exit 2
            fi
            ROBOT_NS="$2"
            shift 2
            ;;
        --robot-ns=*)
            ROBOT_NS="${1#*=}"
            shift
            ;;
        --robot-ip)
            if (($# < 2)); then
                echo "--robot-ip requires a value." >&2
                exit 2
            fi
            ROBOT_IP="$2"
            shift 2
            ;;
        --robot-ip=*)
            ROBOT_IP="${1#*=}"
            shift
            ;;
        --motion-test)
            RUN_MOTION_TEST=true
            shift
            ;;
        --yes)
            ASSUME_YES=true
            shift
            ;;
        --no-dds-recovery)
            AUTO_RECOVER_DDS=false
            shift
            ;;
        *)
            echo "Usage: $0 [--robot-ns NAME] [--robot-ip IPV4] [--motion-test] [--yes] [--no-dds-recovery]" >&2
            exit 2
            ;;
    esac
done

if [[ ! "${ROBOT_NS}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
    echo "Invalid robot namespace '${ROBOT_NS}'; use one ROS name token." >&2
    exit 2
fi
if ! python3 -c \
    'import ipaddress, sys; assert ipaddress.ip_address(sys.argv[1]).version == 4' \
    "${ROBOT_IP}" 2>/dev/null; then
    echo "Invalid robot IPv4 address '${ROBOT_IP}'." >&2
    exit 2
fi
readonly ROBOT_IP ROBOT_NS
readonly COMMAND_TOPIC="/${ROBOT_NS}/command/user_command"

if [[ "${ASSUME_YES}" != true ]]; then
    read -r -p \
        "Confirm D1 is on level ground, the area is clear, and an operator holds the emergency stop [y/N]: " \
        answer
    if [[ ! "${answer}" =~ ^[Yy]$ ]]; then
        echo "Cancelled."
        exit 0
    fi
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE}/install/setup.bash"
set -u

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROBOT_NS
export SLAM_D1_WORKSPACE="${WORKSPACE}"

# Never silently reuse a web process configured for another robot.
if existing_web_status="$(curl --fail --silent "${WEB_URL}/api/status" 2>/dev/null)"; then
    existing_robot_ns="$(
        python3 -c \
            'import json, sys; print(json.load(sys.stdin).get("robot_namespace", ""))' \
            <<<"${existing_web_status}" 2>/dev/null || true
    )"
    if [[ "${existing_robot_ns}" != "${ROBOT_NS}" ]]; then
        echo "Web control on port 8080 targets '${existing_robot_ns:-unknown}', not '${ROBOT_NS}'." >&2
        echo "Stop the existing web control before selecting another robot." >&2
        exit 1
    fi
fi

safe_web_stop()
{
    curl --fail --silent --show-error \
        -X POST -H "Content-Type: application/json" \
        -d '{}' "${WEB_URL}/api/stop" >/dev/null 2>&1 || true
}

safe_robot_stop()
{
    safe_web_stop
    ros2 run slam_d1_bridge stop_slam_d1_bridge.sh --yes >/dev/null 2>&1 || true
}

startup_failed()
{
    local status=$?
    trap - ERR INT TERM
    echo "D1 startup or motion test failed; stopping motion and lowering the robot." >&2
    safe_robot_stop
    exit "${status}"
}

command_subscriber_available()
{
    local topic_info
    topic_info="$(
        timeout 10s ros2 topic info --no-daemon --spin-time 5.0 \
            "${COMMAND_TOPIC}" 2>&1 || true
    )"
    grep -Eq 'Subscription count: [1-9][0-9]*' <<<"${topic_info}"
}

restart_remote_bringup_if_idle()
{
    echo "Target D1 is visible but its control participant is missing; checking safe DDS recovery..." >&2
    ssh -T \
        -o BatchMode=yes \
        -o ConnectTimeout=5 \
        -o ConnectionAttempts=1 \
        "${D1_SSH_USER}@${ROBOT_IP}" \
        bash -s -- "${ROBOT_NS}" <<'REMOTE_RECOVERY'
set -euo pipefail

expected_namespace="$1"
set +u
# shellcheck disable=SC1091
source /opt/d1_ros2/env.sh >/dev/null
set -u

if [[ "${ROBOT_NS:-}" != "${expected_namespace}" ]]; then
    echo "Remote D1 namespace '${ROBOT_NS:-unknown}' does not match '${expected_namespace}'." >&2
    exit 1
fi

fsm_output="$(
    timeout 6s ros2 topic echo --once \
        "/${expected_namespace}/rl_controller/fsm" 2>&1 || true
)"
if ! grep -Eq '^data: idle$' <<<"${fsm_output}"; then
    echo "Refusing DDS recovery because the remote D1 is not confirmed idle." >&2
    exit 1
fi

main_pid="$(systemctl show d1_bringup.service -p MainPID --value)"
if [[ ! "${main_pid}" =~ ^[1-9][0-9]*$ ]] || [[ ! -r "/proc/${main_pid}/cmdline" ]]; then
    echo "d1_bringup.service has no readable active MainPID." >&2
    exit 1
fi
command_line="$(tr '\0' ' ' <"/proc/${main_pid}/cmdline")"
if [[ "${command_line}" != *"ros2"*"ddt_bringup"*"d1_robots.launch.py"* ]]; then
    echo "Refusing to signal unexpected process ${main_pid}: ${command_line}" >&2
    exit 1
fi

# The system service has Restart=on-failure. A supervised failure recreates
# every DDS participant after eth0 has acquired its wired address.
kill -KILL "${main_pid}"

for _ in {1..60}; do
    sleep 0.5
    new_pid="$(systemctl show d1_bringup.service -p MainPID --value)"
    if [[ "${new_pid}" == "0" || "${new_pid}" == "${main_pid}" ]]; then
        continue
    fi
    if ! systemctl is-active --quiet d1_bringup.service; then
        continue
    fi
    topic_info="$(
        timeout 3s ros2 topic info --no-daemon --spin-time 1.0 \
            "/${expected_namespace}/command/user_command" 2>&1 || true
    )"
    if grep -Eq 'Subscription count: [1-9][0-9]*' <<<"${topic_info}"; then
        echo "Remote d1_bringup restarted with PID ${new_pid}; control subscriber is active."
        exit 0
    fi
done

echo "d1_bringup restarted but its control subscriber did not recover in time." >&2
exit 1
REMOTE_RECOVERY
}

ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null

if ! command_subscriber_available; then
    discovered_robot_namespace_lines="$(
        timeout 10s ros2 topic list --no-daemon --spin-time 5.0 2>/dev/null |
            sed -nE 's#^/(d[0-9]+)/.*#\1#p' | sort -u || true
    )"
    if [[ "${AUTO_RECOVER_DDS}" == true ]] && \
            grep -Fxq "${ROBOT_NS}" <<<"${discovered_robot_namespace_lines}"; then
        if restart_remote_bringup_if_idle; then
            for _ in {1..30}; do
                if command_subscriber_available; then
                    break
                fi
                sleep 0.5
            done
        fi
    fi
fi

if ! command_subscriber_available; then
    discovered_robot_namespaces="$(
        timeout 10s ros2 topic list --no-daemon --spin-time 5.0 2>/dev/null |
            sed -nE 's#^/(d[0-9]+)/.*#\1#p' | sort -u | paste -sd, - || true
    )"
    echo "D1 command subscriber is not available on ${COMMAND_TOPIC}." >&2
    if [[ -n "${discovered_robot_namespaces}" ]]; then
        echo "DDS currently exposes D1 namespace(s): ${discovered_robot_namespaces}." >&2
    else
        echo "No D1 DDS namespace was discovered on ROS domain ${ROS_DOMAIN_ID}." >&2
        echo "Check d1_bringup.service, ROS_DOMAIN_ID=42 and ROS_LOCALHOST_ONLY=0 on the robot." >&2
    fi
    exit 1
fi

# All checks above are read-only. From this point onward, a failure invokes
# the supervised stop/lie-down recovery because control state may have changed.
trap startup_failed ERR INT TERM

mkdir -p "${WORKSPACE}/log"
if ! curl --fail --silent "${WEB_URL}/api/status" >/dev/null 2>&1; then
    setsid ros2 launch luxi_web_control lekiwi_web_control.launch.py \
        robot_namespace:="${ROBOT_NS}" \
        bind_address:=0.0.0.0 http_port:=8080 >"${WEB_LOG}" 2>&1 &
    web_pid=$!
    echo "${web_pid}" >"${WEB_PID_FILE}"
    for _ in {1..50}; do
        if curl --fail --silent "${WEB_URL}/api/status" >/dev/null 2>&1; then
            break
        fi
        if ! kill -0 "${web_pid}" 2>/dev/null; then
            echo "Web control exited during startup. See ${WEB_LOG}." >&2
            false
        fi
        sleep 0.1
    done
fi

curl --fail --silent --show-error \
    -X POST -H "Content-Type: application/json" \
    -d '{"active":false}' "${WEB_URL}/api/estop" >/dev/null
safe_web_stop

invert_value="$(ros2 param get /d1_velocity_command_mux invert_angular_z 2>/dev/null || true)"
if [[ "${invert_value}" != *"False"* ]]; then
    echo "D1 velocity mux is missing or invert_angular_z is not false." >&2
    false
fi

ros2 run slam_d1_bridge start_slam_d1_bridge.sh --yes

for _ in {1..30}; do
    if ros2 topic info /cmd_vel 2>/dev/null | grep -Eq 'Subscription count: [1-9][0-9]*'; then
        break
    fi
    sleep 0.1
done
if ! ros2 topic info /cmd_vel 2>/dev/null | grep -Eq 'Subscription count: [1-9][0-9]*'; then
    echo "slam_d1_bridge did not subscribe to /cmd_vel." >&2
    false
fi

send_web_command_for()
{
    local linear_x="$1"
    local angular_z="$2"
    local duration_ms="$3"
    local start_ns end_ns now_ns
    start_ns="$(date +%s%N)"
    end_ns=$((start_ns + duration_ms * 1000000))

    while true; do
        now_ns="$(date +%s%N)"
        if ((now_ns >= end_ns)); then
            break
        fi
        curl --fail --silent --show-error \
            -X POST -H "Content-Type: application/json" \
            -d "{\"linear_x\":${linear_x},\"linear_y\":0.0,\"angular_z\":${angular_z}}" \
            "${WEB_URL}/api/cmd_vel" >/dev/null
        sleep 0.1
    done
    safe_web_stop
}

if [[ "${RUN_MOTION_TEST}" == true ]]; then
    echo "Running supervised low-speed D1 motion test..."
    echo "Forward: 0.05 m/s for 1 second (maximum nominal travel 5 cm)."
    send_web_command_for 0.05 0.0 1000
    sleep 1
    echo "Backward: -0.05 m/s for 1 second."
    send_web_command_for -0.05 0.0 1000
    sleep 1
    echo "Left turn: 0.10 rad/s for 1 second."
    send_web_command_for 0.0 0.10 1000
    sleep 1
    echo "Right turn: -0.10 rad/s for 1 second."
    send_web_command_for 0.0 -0.10 1000
    sleep 1
    safe_web_stop
    echo "Motion test complete; the command is zero."
fi

trap - ERR INT TERM
web_access_ip="$(
    ip -4 route get 1.1.1.1 2>/dev/null |
        sed -nE 's/.*[[:space:]]src[[:space:]]+([^[:space:]]+).*/\1/p' |
        head -n 1
)"
if [[ -z "${web_access_ip}" ]]; then
    web_access_ip="$(
        ip -o -4 address show up scope global 2>/dev/null |
            awk '$2 != "eno1" {sub(/\/.*/, "", $4); print $4; exit}'
    )"
fi
if [[ -n "${web_access_ip}" ]]; then
    echo "D1 web control is enabled: http://${web_access_ip}:8080"
else
    echo "D1 web control is enabled on port 8080; use an NX LAN address."
fi
echo "The 192.168.123.51 interface is the isolated D1 control link, not the normal browser entry."
echo "Stop and lower the robot with: ros2 run slam_d1_bridge stop_slam_d1_bridge.sh"
