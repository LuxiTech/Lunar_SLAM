# Copyright 2026 lunar
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

"""Unit tests for keyboard-to-velocity mappings."""

from luxi_navigation.keyboard_teleop import STOP_COMMAND, command_for_key


def test_wasd_bindings():
    assert command_for_key("w", 0.2, 0.6).linear_x == 0.2
    assert command_for_key("W", 0.2, 0.6).linear_x == 0.2
    assert command_for_key("s", 0.2, 0.6).linear_x == -0.2
    assert command_for_key("a", 0.2, 0.6).angular_z == 0.6
    assert command_for_key("d", 0.2, 0.6).angular_z == -0.6


def test_stop_and_unknown_bindings():
    assert command_for_key(" ", 0.2, 0.6) == STOP_COMMAND
    assert command_for_key("x", 0.2, 0.6) == STOP_COMMAND
    assert command_for_key("?", 0.2, 0.6) is None
