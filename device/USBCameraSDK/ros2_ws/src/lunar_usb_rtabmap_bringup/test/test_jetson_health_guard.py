"""Unit tests for Jetson sysfs health parsing."""

from pathlib import Path

from lunar_usb_rtabmap_bringup.jetson_health_guard import (
    _active_throttles,
    _thermal_temperatures,
)


def test_reads_temperatures_and_throttle_states(tmp_path: Path):
    zone = tmp_path / "thermal_zone0"
    zone.mkdir()
    (zone / "type").write_text("gpu-thermal\n")
    (zone / "temp").write_text("76500\n")
    device = tmp_path / "cooling_device0"
    device.mkdir()
    (device / "type").write_text("gpu-throttle-alert\n")
    (device / "cur_state").write_text("1\n")

    assert _thermal_temperatures(tmp_path) == {"gpu-thermal": 76.5}
    assert _active_throttles(tmp_path) == ["gpu-throttle-alert"]
