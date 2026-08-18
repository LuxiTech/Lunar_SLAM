# D455 bringup for lunar_slam

This package starts only an Intel RealSense D455 and publishes the vendor
streams consumed by `luxi_adapter`.  Depth is aligned into the D455 wide-FOV
RGB optical frame at `848x480@15 Hz`; infrared and point-cloud streams are off
by default to leave USB and CPU headroom for mapping.

## Build

Build the SDK first using `ros2_ws/3parts/README.md`, then:

```bash
cd /home/lunar/project/lunar_slam/device/D455/ros2_ws
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH="$PWD/3parts/librealsense/install-rsusb:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$PWD/3parts/librealsense/install-rsusb/lib:$LD_LIBRARY_PATH"
colcon build --symlink-install
```

## Run and verify

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D455/ros2_ws/install/setup.bash
ros2 launch lunar_d455_bringup d455.launch.py
```

In another terminal with the same environment:

```bash
ros2 run lunar_d455_bringup check_d455_topics
```

The launch uses `device_type=d455`, so it will not silently fall back to a
connected D435i.  Use a USB 3 port; the mapping profile is not intended for
USB 2 operation.
