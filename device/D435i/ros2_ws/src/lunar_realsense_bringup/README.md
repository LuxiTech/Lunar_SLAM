# D435i bringup for lunar_slam

This workspace vendors the official Intel RealSense ROS2 wrapper at tag `4.58.2`.

## Build

```bash
cd /home/lunar/project/lunar_slam/device/D435i/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run camera and RViz

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_realsense_bringup d435i.launch.py
```

The launch file enables:

- RGB: `/camera/camera/color/image_raw`
- Depth: `/camera/camera/depth/image_rect_raw`
- Point cloud: `/camera/camera/depth/color/points`
- TF: `camera_link` and optical frames
- IMU: gyro + accel combined with linear interpolation

On arm64, the RealSense wrapper exposes the point cloud filter as
`pointcloud__neon_`; the launch file handles that automatically.

Default color/depth profiles are `640x480@30fps`; the current D435i has been
verified on USB 3.2. If it is connected through USB2, reduce both profiles:

```bash
ros2 launch lunar_realsense_bringup d435i.launch.py \
  color_profile:=640,480,15 \
  depth_profile:=640,480,15
```

## Headless smoke test

Run the camera without RViz in one terminal:

```bash
ros2 launch lunar_realsense_bringup d435i.launch.py rviz:=false
```

Run the topic checker in another terminal:

```bash
ros2 run lunar_realsense_bringup check_realsense_topics
```
