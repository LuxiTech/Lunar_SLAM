#!/usr/bin/env python3
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

"""Publish safe ``Twist`` commands from WASD keyboard input."""

from __future__ import annotations

import os
import queue
import select
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy


HELP_TEXT = """
WASD robot control
------------------
  W: forward             S: reverse
  A: rotate left         D: rotate right
  Space or X: stop       Q: stop and quit

Hold a movement key to keep moving. The node automatically stops when
keyboard commands time out.
""".strip()


@dataclass(frozen=True)
class VelocityCommand:
    """Planar velocity represented by the fields used in ``Twist``."""

    linear_x: float
    angular_z: float


STOP_COMMAND = VelocityCommand(0.0, 0.0)


def command_for_key(
    key: str, linear_speed: float, angular_speed: float
) -> Optional[VelocityCommand]:
    """Translate one keyboard key to a planar velocity command."""
    bindings = {
        "w": VelocityCommand(linear_speed, 0.0),
        "s": VelocityCommand(-linear_speed, 0.0),
        "a": VelocityCommand(0.0, angular_speed),
        "d": VelocityCommand(0.0, -angular_speed),
        " ": STOP_COMMAND,
        "x": STOP_COMMAND,
    }
    return bindings.get(key.lower())


class KeyboardTeleop(Node):
    """Read keyboard input and continuously publish velocity commands."""

    def __init__(
        self, parameter_overrides: Optional[list[Parameter]] = None
    ) -> None:
        super().__init__(
            "luxi_keyboard_teleop",
            parameter_overrides=parameter_overrides or [],
        )
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("command_timeout", 0.6)
        self.declare_parameter("stop_publish_count", 3)
        self.declare_parameter("keyboard_enabled", True)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.stop_publish_count = int(self.get_parameter("stop_publish_count").value)
        self.keyboard_enabled = bool(self.get_parameter("keyboard_enabled").value)
        self._validate_parameters()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.current_command = STOP_COMMAND
        self.last_movement_key_time: Optional[float] = None
        self._timed_out = False
        self._stopping = False
        self._remaining_stop_messages = 0
        self.shutdown_ready = False
        self._key_queue: queue.Queue[Optional[str]] = queue.Queue()
        self._reader_stop = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None

        self.timer = self.create_timer(1.0 / self.publish_rate, self._timer_callback)
        self.get_logger().info(
            f"Keyboard teleop ready: topic={self.cmd_vel_topic} "
            f"linear={self.linear_speed:.3f} m/s "
            f"angular={self.angular_speed:.3f} rad/s "
            f"timeout={self.command_timeout:.2f} s"
        )

        if self.keyboard_enabled:
            self.get_logger().info(f"\n{HELP_TEXT}")
            self._reader_thread = threading.Thread(
                target=self._keyboard_reader,
                name="keyboard-reader",
                daemon=True,
            )
            self._reader_thread.start()

    def _validate_parameters(self) -> None:
        if not self.cmd_vel_topic:
            raise ValueError("cmd_vel_topic cannot be empty")
        if self.linear_speed <= 0.0:
            raise ValueError("linear_speed must be greater than zero")
        if self.angular_speed <= 0.0:
            raise ValueError("angular_speed must be greater than zero")
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self.command_timeout <= 0.0:
            raise ValueError("command_timeout must be greater than zero")
        if self.stop_publish_count < 1:
            raise ValueError("stop_publish_count must be at least one")

    def apply_key(self, key: str, timestamp: Optional[float] = None) -> bool:
        """Apply one key; return ``True`` when the key requests shutdown."""
        normalized = key.lower()
        if normalized in {"q", "\x03"}:
            self.request_stop()
            return True

        command = command_for_key(key, self.linear_speed, self.angular_speed)
        if command is None:
            return False

        self.current_command = command
        self._timed_out = False
        if command == STOP_COMMAND:
            self.last_movement_key_time = None
        else:
            self.last_movement_key_time = (
                timestamp if timestamp is not None else time.monotonic()
            )
        return False

    def enforce_timeout(self, timestamp: Optional[float] = None) -> bool:
        """Stop stale motion and return whether a timeout was applied."""
        if self.current_command == STOP_COMMAND or self.last_movement_key_time is None:
            return False
        now = timestamp if timestamp is not None else time.monotonic()
        if now - self.last_movement_key_time <= self.command_timeout:
            return False

        self.current_command = STOP_COMMAND
        self.last_movement_key_time = None
        if not self._timed_out:
            self.get_logger().warn("Keyboard command timed out; publishing stop.")
        self._timed_out = True
        return True

    def publish_once(self) -> Twist:
        """Publish the current command and return the generated message."""
        message = Twist()
        message.linear.x = self.current_command.linear_x
        message.angular.z = self.current_command.angular_z
        self.publisher.publish(message)
        return message

    def request_stop(self) -> None:
        """Begin a controlled shutdown that publishes repeated zero commands."""
        self.current_command = STOP_COMMAND
        self.last_movement_key_time = None
        if not self._stopping:
            self._stopping = True
            self._remaining_stop_messages = self.stop_publish_count
            self._reader_stop.set()

    def _timer_callback(self) -> None:
        if not self._stopping:
            try:
                key = self._key_queue.get_nowait()
            except queue.Empty:
                key = ""

            if key is None:
                self.request_stop()
            elif key:
                self.apply_key(key)

        self.enforce_timeout()
        self.publish_once()

        if self._stopping:
            self._remaining_stop_messages -= 1
            if self._remaining_stop_messages <= 0:
                self.shutdown_ready = True
                self.timer.cancel()

    def _keyboard_reader(self) -> None:
        try:
            file_descriptor = sys.stdin.fileno()
        except (AttributeError, OSError):
            self.get_logger().error("Standard input has no usable file descriptor.")
            self._key_queue.put(None)
            return

        is_terminal = os.isatty(file_descriptor)
        old_settings = termios.tcgetattr(file_descriptor) if is_terminal else None
        try:
            if is_terminal:
                tty.setcbreak(file_descriptor)

            while not self._reader_stop.is_set():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue
                key = sys.stdin.read(1)
                if key == "":
                    self._key_queue.put(None)
                    break
                self._key_queue.put(key)
        except Exception as error:  # Terminal failures must result in a stop.
            self.get_logger().error(f"Keyboard input failed: {error}")
            self._key_queue.put(None)
        finally:
            if old_settings is not None:
                termios.tcsetattr(file_descriptor, termios.TCSADRAIN, old_settings)

    def destroy_node(self) -> bool:
        self._reader_stop.set()
        self.current_command = STOP_COMMAND
        if rclpy.ok():
            self.publish_once()
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=0.3)
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    """Run the keyboard teleoperation node."""
    rclpy.init(args=args)
    node: Optional[KeyboardTeleop] = None
    try:
        node = KeyboardTeleop()
        while rclpy.ok() and not node.shutdown_ready:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        if node is not None:
            node.request_stop()
            for _ in range(node.stop_publish_count):
                node.publish_once()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
