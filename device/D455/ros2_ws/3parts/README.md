# D455 RealSense SDK

The official RealSense SDK is checked out locally at `librealsense` using the
`v2.58.2` tag.  It is kept outside the ROS package discovery path by
`COLCON_IGNORE` and installed into `librealsense/install-rsusb`.

Build the SDK without requiring a patched kernel module:

```bash
cd /home/lunar/project/lunar_slam/device/D455/ros2_ws/3parts/librealsense
cmake -S . -B build-rsusb \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PWD/install-rsusb" \
  -DFORCE_RSUSB_BACKEND=true \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_UNIT_TESTS=false \
  -DBUILD_WITH_DDS=false
cmake --build build-rsusb --parallel
cmake --install build-rsusb
```

Upstream source: <https://github.com/realsenseai/librealsense.git>

