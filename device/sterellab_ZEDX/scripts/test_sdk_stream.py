#!/usr/bin/env python3
"""Open one ZED X and validate image, depth, timestamps, and IMU data."""

import sys
import time

import numpy as np
import pyzed.sl as sl


def main() -> int:
    devices = sl.Camera.get_device_list()
    print(f"detected_devices={len(devices)}")
    for device in devices:
        print(
            "device="
            f"model:{device.camera_model},"
            f"serial:{device.serial_number},"
            f"state:{device.camera_state},"
            f"path:{device.path}"
        )

    if not devices:
        print("ERROR: ZED SDK did not enumerate a camera", file=sys.stderr)
        return 1

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD1200
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.METER
    init.set_from_serial_number(devices[0].serial_number)

    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"ERROR: camera.open failed: {status}", file=sys.stderr)
        return 1

    try:
        info = camera.get_camera_information()
        print(
            "opened_camera="
            f"model:{info.camera_model},"
            f"serial:{info.serial_number},"
            f"firmware:{info.camera_configuration.firmware_version},"
            f"resolution:{info.camera_configuration.resolution.width}x"
            f"{info.camera_configuration.resolution.height},"
            f"fps:{info.camera_configuration.fps}"
        )

        runtime = sl.RuntimeParameters()
        left = sl.Mat()
        depth = sl.Mat()
        sensors = sl.SensorsData()
        timestamps = []
        successes = 0
        last_error = sl.ERROR_CODE.SUCCESS
        deadline = time.monotonic() + 15.0

        while successes < 60 and time.monotonic() < deadline:
            last_error = camera.grab(runtime)
            if last_error != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.005)
                continue

            timestamps.append(
                camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            )
            successes += 1

        if successes < 30:
            print(
                f"ERROR: only {successes} frames grabbed; last_error={last_error}",
                file=sys.stderr,
            )
            return 1

        image_status = camera.retrieve_image(left, sl.VIEW.LEFT)
        depth_status = camera.retrieve_measure(depth, sl.MEASURE.DEPTH)
        sensor_status = camera.get_sensors_data(sensors, sl.TIME_REFERENCE.IMAGE)

        if image_status != sl.ERROR_CODE.SUCCESS:
            print(f"ERROR: retrieve_image failed: {image_status}", file=sys.stderr)
            return 1
        if depth_status != sl.ERROR_CODE.SUCCESS:
            print(f"ERROR: retrieve_measure failed: {depth_status}", file=sys.stderr)
            return 1

        image_array = left.get_data()
        depth_array = depth.get_data()
        finite_depth = np.isfinite(depth_array) & (depth_array > 0)
        valid_ratio = float(np.count_nonzero(finite_depth)) / float(depth_array.size)

        elapsed_ns = timestamps[-1] - timestamps[0]
        measured_fps = (len(timestamps) - 1) * 1e9 / elapsed_ns if elapsed_ns else 0.0

        print(
            f"stream=frames:{successes},measured_fps:{measured_fps:.2f},"
            f"monotonic_timestamps:{all(b > a for a, b in zip(timestamps, timestamps[1:]))}"
        )
        print(f"left_image=shape:{image_array.shape},dtype:{image_array.dtype}")
        print(
            f"depth=shape:{depth_array.shape},dtype:{depth_array.dtype},"
            f"valid_ratio:{valid_ratio:.4f}"
        )
        print(f"imu_status={sensor_status}")

        if measured_fps < 25.0:
            print(f"ERROR: measured FPS is too low: {measured_fps:.2f}", file=sys.stderr)
            return 1
        if valid_ratio <= 0.0:
            print("ERROR: no valid depth pixels", file=sys.stderr)
            return 1
        if sensor_status != sl.ERROR_CODE.SUCCESS:
            print(f"ERROR: IMU retrieval failed: {sensor_status}", file=sys.stderr)
            return 1

        print("RESULT=PASS")
        return 0
    finally:
        camera.close()


if __name__ == "__main__":
    raise SystemExit(main())
