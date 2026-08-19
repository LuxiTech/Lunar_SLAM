#!/usr/bin/env bash

_lunar_client_root="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
source /opt/ros/humble/setup.bash
source "${_lunar_client_root}/install/local_setup.bash"
unset _lunar_client_root
