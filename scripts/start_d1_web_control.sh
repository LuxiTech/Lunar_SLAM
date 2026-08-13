#!/usr/bin/env bash

set -euo pipefail

readonly WORKSPACE="/home/nvidia/Desktop/lunar_slam"
readonly WEB_URL="http://127.0.0.1:8080"
readonly WEB_LOG="${WORKSPACE}/log/d1_web_control.log"
readonly WEB_PID_FILE="/tmp/d1_web_control.pid"
readonly ROBOT_IP="192.168.123.49"
readonly ROBOT_NS="d15041873"
readonly COMMAND_TOPIC="/${ROBOT_NS}/command/user_command"

RUN_MOTION_TEST=false
ASSUME_YES=false
for argument in "$@"; do
    case "${argument}" in
        --motion-test)
            RUN_MOTION_TEST=true
            ;;
        --yes)
            ASSUME_YES=true
            ;;
        *)
            echo "Usage: $0 [--motion-test] [--yes]" >&2
            exit 2
            ;;
    esac
done

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
export ROBOT_NS
export SLAM_D1_WORKSPACE="${WORKSPACE}"

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
trap startup_failed ERR INT TERM

ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null

topic_info="$(ros2 topic info "${COMMAND_TOPIC}" 2>/dev/null || true)"
if ! grep -Eq 'Subscription count: [1-9][0-9]*' <<<"${topic_info}"; then
    echo "D1 command subscriber is not available on ${COMMAND_TOPIC}." >&2
    false
fi

mkdir -p "${WORKSPACE}/log"
if ! curl --fail --silent "${WEB_URL}/api/status" >/dev/null 2>&1; then
    setsid ros2 launch luxi_web_control lekiwi_web_control.launch.py \
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
echo "D1 web control is enabled: http://192.168.123.51:8080"
echo "Stop and lower the robot with: ros2 run slam_d1_bridge stop_slam_d1_bridge.sh"
