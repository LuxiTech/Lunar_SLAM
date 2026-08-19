"""Stop the max-performance mapping graph before Jetson thermal throttling."""

from pathlib import Path

import rclpy
from rclpy.node import Node


def _thermal_temperatures(root: Path = Path("/sys/class/thermal")) -> dict[str, float]:
    temperatures = {}
    for zone in root.glob("thermal_zone*"):
        try:
            name = (zone / "type").read_text().strip()
            temperatures[name] = int((zone / "temp").read_text()) / 1000.0
        except (OSError, ValueError):
            continue
    return temperatures


def _active_throttles(root: Path = Path("/sys/class/thermal")) -> list[str]:
    active = []
    for device in root.glob("cooling_device*"):
        try:
            name = (device / "type").read_text().strip()
            state = int((device / "cur_state").read_text())
        except (OSError, ValueError):
            continue
        if "throttle-alert" in name and state > 0:
            active.append(name)
    return active


class JetsonHealthGuard(Node):
    """Monitor all SoC zones and request launch shutdown on unsafe state."""

    def __init__(self) -> None:
        super().__init__("jetson_health_guard")
        self.warning_temp = float(self.declare_parameter(
            "warning_temp_c", 85.0
        ).value)
        self.stop_temp = float(self.declare_parameter(
            "stop_temp_c", 92.0
        ).value)
        self.fail_on_throttle = bool(self.declare_parameter(
            "fail_on_throttle", True
        ).value)
        if not 0.0 < self.warning_temp < self.stop_temp:
            raise ValueError("warning_temp_c must be positive and below stop_temp_c")
        self.exit_code = 0
        self.last_report_ns = 0
        self.timer = self.create_timer(1.0, self._check)
        self.get_logger().info(
            f"Jetson health guard: warn={self.warning_temp:.1f} C, "
            f"stop={self.stop_temp:.1f} C, throttle guard="
            f"{'on' if self.fail_on_throttle else 'off'}"
        )

    def _stop(self, reason: str) -> None:
        if self.exit_code:
            return
        self.exit_code = 2
        self.get_logger().fatal(reason)
        self.timer.cancel()
        rclpy.shutdown()

    def _check(self) -> None:
        temperatures = _thermal_temperatures()
        if not temperatures:
            self._stop("No readable Jetson thermal zones; stopping protected profile")
            return
        hottest_name, hottest = max(temperatures.items(), key=lambda item: item[1])
        throttles = _active_throttles() if self.fail_on_throttle else []
        if hottest >= self.stop_temp:
            self._stop(
                f"Thermal safety stop: {hottest_name}={hottest:.1f} C "
                f">= {self.stop_temp:.1f} C"
            )
            return
        if throttles:
            self._stop("Jetson thermal throttling active: " + ", ".join(throttles))
            return
        now_ns = self.get_clock().now().nanoseconds
        if hottest >= self.warning_temp:
            self.get_logger().warn(
                f"Jetson temperature warning: {hottest_name}={hottest:.1f} C",
                throttle_duration_sec=5.0,
            )
        elif now_ns - self.last_report_ns >= 10_000_000_000:
            self.get_logger().info(
                f"Jetson healthy: hottest {hottest_name}={hottest:.1f} C"
            )
            self.last_report_ns = now_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JetsonHealthGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
