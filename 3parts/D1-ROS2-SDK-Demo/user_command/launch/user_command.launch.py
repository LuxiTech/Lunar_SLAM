#!/usr/bin/python3
#
# Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
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

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

d1_namespace = 'd1'

unique_id_file = '/proc/device-tree/serial-number'
try:
    with open(unique_id_file, 'r') as file:
        unique_id = file.read().strip()
        unique_id = unique_id.replace('\x00', '')[6:]
        d1_namespace = 'd1'+unique_id
except FileNotFoundError:
        d1_namespace='d1'
except Exception as e:
        d1_namespace='d1'
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('user_command'),
        'config',
        'param.yaml'
    )

    return LaunchDescription([
        Node(
            package='user_command',
            executable='user_command_node',
            name='user_command_node',
            namespace=d1_namespace,
            output='screen',
            parameters=[config]
        )
    ])
