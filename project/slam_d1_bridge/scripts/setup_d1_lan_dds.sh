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

# This file must be sourced after the ROS 2 workspace setup file. It selects the
# host address used to reach the D1 and generates a Fast DDS profile that allows
# only loopback and the wired 192.168.123.0/24 control LAN.

d1_robot_ip="${D1_ROBOT_IP:-192.168.123.49}"
d1_route="$(ip -4 route get "${d1_robot_ip}" 2>/dev/null | head -n 1)"
d1_lan_interface="$(awk '{for (i = 1; i <= NF; ++i) if ($i == "dev") print $(i + 1)}' <<<"${d1_route}")"
d1_lan_address="$(awk '{for (i = 1; i <= NF; ++i) if ($i == "src") print $(i + 1)}' <<<"${d1_route}")"

if [[ -z "${d1_lan_interface}" || -z "${d1_lan_address}" ]]; then
    echo "Cannot resolve the wired route to D1 ${d1_robot_ip}." >&2
    return 1 2>/dev/null || exit 1
fi
if [[ ! "${d1_lan_address}" =~ ^192\.168\.123\.[0-9]+$ ]]; then
    echo "Refusing D1 control: ${d1_robot_ip} resolves via ${d1_lan_interface} with source ${d1_lan_address}, not 192.168.123.0/24." >&2
    return 1 2>/dev/null || exit 1
fi
if ! ip -4 -o addr show dev "${d1_lan_interface}" scope global |
    awk '{print $4}' | grep -Fxq "${d1_lan_address}/24"; then
    echo "Refusing D1 control: ${d1_lan_interface} does not own ${d1_lan_address}/24." >&2
    return 1 2>/dev/null || exit 1
fi

if [[ -n "${D1_FASTDDS_PROFILE_TEMPLATE:-}" ]]; then
    d1_profile_template="${D1_FASTDDS_PROFILE_TEMPLATE}"
else
    d1_package_prefix="$(ros2 pkg prefix slam_d1_bridge 2>/dev/null)"
    d1_profile_template="${d1_package_prefix}/share/slam_d1_bridge/config/fastdds_lan_only.xml.in"
fi
if [[ ! -r "${d1_profile_template}" ]]; then
    echo "Fast DDS LAN-only template is missing: ${d1_profile_template}" >&2
    return 1 2>/dev/null || exit 1
fi

d1_profile_address="${d1_lan_address//./_}"
d1_profile_path="${D1_FASTDDS_PROFILE_FILE:-/tmp/lunar_slam_fastdds_lan_${UID}_${d1_profile_address}.xml}"
d1_profile_temp="$(mktemp "${d1_profile_path}.XXXXXX")"
sed "s/@D1_LAN_ADDRESS@/${d1_lan_address}/g" \
    "${d1_profile_template}" >"${d1_profile_temp}"
chmod 600 "${d1_profile_temp}"
mv -f "${d1_profile_temp}" "${d1_profile_path}"

export D1_LAN_INTERFACE="${d1_lan_interface}"
export D1_LAN_ADDRESS="${d1_lan_address}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${d1_profile_path}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset FASTDDS_BUILTIN_TRANSPORTS

unset d1_robot_ip d1_route d1_lan_interface d1_lan_address
unset d1_package_prefix d1_profile_template d1_profile_address
unset d1_profile_path d1_profile_temp
