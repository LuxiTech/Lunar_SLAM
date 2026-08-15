#!/usr/bin/env bash

# Copyright 2026 ROS 2 Developer
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail

readonly DEFAULT_ROBOT_NS="d15041873"
readonly DEFAULT_ROS_DOMAIN_ID="42"
readonly SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
readonly DEFAULT_WORKSPACE="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"

ROBOT_NS="${ROBOT_NS:-${DEFAULT_ROBOT_NS}}"
if [[ ! "${ROBOT_NS}" =~ ^[A-Za-z_][A-Za-z0-9_]*$ ]]; then
    echo "Invalid ROBOT_NS '${ROBOT_NS}'; use one ROS name token." >&2
    exit 2
fi
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-${DEFAULT_ROS_DOMAIN_ID}}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
SLAM_D1_WORKSPACE="${SLAM_D1_WORKSPACE:-${DEFAULT_WORKSPACE}}"
D1_DISCOVERY_SPIN_TIME="${D1_DISCOVERY_SPIN_TIME:-30.0}"
D1_SERVICE_TIMEOUT="${D1_SERVICE_TIMEOUT:-30s}"

readonly PID_FILE="/tmp/slam_d1_bridge_${ROBOT_NS}.pid"
readonly COMMAND_TOPIC="/${ROBOT_NS}/command/user_command"
readonly TELEOP_NODE="/${ROBOT_NS}/teleop_command"
readonly PARAMETER_SERVICE="${TELEOP_NODE}/set_parameters"

ASSUME_YES=false
if [[ "${1:-}" == "--yes" ]]; then
    ASSUME_YES=true
elif [[ $# -ne 0 ]]; then
    echo "Usage: $0 [--yes]" >&2
    exit 2
fi

if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
    echo "ROS 2 Humble setup not found." >&2
    exit 1
fi
if [[ ! -f "${SLAM_D1_WORKSPACE}/install/setup.bash" ]]; then
    echo "Workspace is not built: ${SLAM_D1_WORKSPACE}/install/setup.bash" >&2
    exit 1
fi

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "${SLAM_D1_WORKSPACE}/install/setup.bash"
set -u
export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/setup_d1_lan_dds.sh"

echo "Robot namespace : ${ROBOT_NS}"
echo "ROS domain ID    : ${ROS_DOMAIN_ID}"
echo "D1 LAN interface : ${D1_LAN_INTERFACE} (${D1_LAN_ADDRESS})"
echo "Command topic    : ${COMMAND_TOPIC}"

topic_info="$(
    ros2 topic info --no-daemon --spin-time "${D1_DISCOVERY_SPIN_TIME}" -v \
        "${COMMAND_TOPIC}" 2>/dev/null || true
)"
if grep -Eq 'Node name: http_ros_gateway' <<<"${topic_info}"; then
    echo "http_ros_gateway is still publishing ${COMMAND_TOPIC}; stop it before running this script." >&2
    exit 1
fi

if [[ "${ASSUME_YES}" != true ]]; then
    read -r -p \
        "Confirm the area is clear and the robot may safely lie down [y/N]: " \
        answer
    if [[ ! "${answer}" =~ ^[Yy]$ ]]; then
        echo "Cancelled."
        exit 0
    fi
fi

if [[ -f "${PID_FILE}" ]]; then
    bridge_pid="$(<"${PID_FILE}")"
    if [[ "${bridge_pid}" =~ ^[0-9]+$ ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
        echo "Stopping slam_d1_bridge PID ${bridge_pid}..."
        kill -INT -- "-${bridge_pid}" 2>/dev/null || kill -INT "${bridge_pid}" 2>/dev/null || true
        for _ in {1..50}; do
            if ! kill -0 "${bridge_pid}" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done
        if kill -0 "${bridge_pid}" 2>/dev/null; then
            echo "Bridge did not stop after 5 seconds; refusing to publish competing commands." >&2
            exit 1
        fi
    else
        echo "Removing stale bridge PID file."
    fi
    rm -f "${PID_FILE}"
else
    echo "Bridge PID file not found; continuing with zero and lie-down commands."
fi

publish_for()
{
    local duration="$1"
    local fsm_mode="$2"
    local result=0

    timeout --signal=INT --kill-after=2s "${duration}" ros2 topic pub -r 20 \
        "${COMMAND_TOPIC}" \
        ddt_msgs/msg/UserCommand \
        "{fsm_mode: ${fsm_mode}, pose: {orientation: {w: 1.0}}}" || result=$?
    if [[ ${result} -ne 0 && ${result} -ne 124 && ${result} -ne 130 ]]; then
        return "${result}"
    fi
}

set_sdk_mode()
{
    local enabled="$1"
    local response
    local result=0
    response="$(
        timeout --signal=INT --kill-after=2s "${D1_SERVICE_TIMEOUT}" ros2 service call \
            "${PARAMETER_SERVICE}" \
            rcl_interfaces/srv/SetParameters \
            "{parameters: [{name: use_sdk, value: {type: 1, bool_value: ${enabled}}}]}"
    )" || result=$?
    if grep -q 'successful=True' <<<"${response}"; then
        return 0
    fi
    echo "D1 use_sdk=${enabled} request failed (exit ${result}): ${response}" >&2
    return 1
}

echo "Publishing loco with zero velocity for 1 second..."
publish_for 1s loco

echo "Lying down for 10 seconds..."
publish_for 10s transform_down

set_sdk_mode false
echo "Robot is commanded down and SDK control is released."
