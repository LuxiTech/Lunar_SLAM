# Luxi Adapter

`luxi_adapter` is the only hardware-facing package consumed by the project
algorithms. It starts exactly one profile selected by
`config/sensor_bringup.yaml`, then publishes the hardware-independent contract:

```text
/sensors/rgbd/color/image_raw
/sensors/rgbd/depth/image_raw
/sensors/rgbd/color/camera_info
/sensors/rgbd/color/image_raw/compressed
/sensors/imu/data_raw
/sensors/imu/data
/tf
/tf_static
```

The depth stream must be registered to the color camera. All image, camera-info
and IMU messages must contain their original driver timestamps and frame IDs.

## D435i profile

The default `d435i` profile reads its driver setup path, ROS domain and all
topic names from `config/sensor_bringup.yaml`. It starts the existing D435i
driver, relays the vendor topics into the stable contract, filters the canonical
raw IMU with Madgwick, and publishes the configured `base_link` to
`camera_link` static transform.

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_adapter sensor_bringup.launch.py
```

Only one `hardware_profile` may be selected. To add a camera, create a profile
launch file named `<profile>.launch.py`, set that name in the YAML file, and
make it publish the same contract. Do not add hardware-specific topic names to
`luxi_rtab_map`, `luxi_web_control`, or navigation packages.

## Hik stereo + H30 profile

The Hik profile preserves the calibrated sensor pipeline: rectified left colour,
depth and CameraInfo are all 512x375, while the camera driver itself publishes
1024x750 images. The H30 AHRS orientation is already valid, so it is relayed
unchanged to both canonical IMU topics without a second filter.

Start the hardware pipeline first. It deliberately contains no RTAB-Map node,
so Lunar SLAM is the only mapping backend:

```bash
source /opt/ros/humble/setup.bash
source ~/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws/install/setup.bash
ros2 launch hik_bringup hik_sensor_only.launch.py
```

Then start the adapter in a second terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/lunar_slam/install/setup.bash
ros2 launch luxi_adapter sensor_bringup.launch.py \
  config:=~/lunar_slam/install/luxi_adapter/share/luxi_adapter/config/hik_sensor_bringup.yaml
```

The adapter relays `/stereo/left/image_rect_color`, `/stereo/depth`,
`/stereo/left/camera_info`, `/imu/data` and the Hik JPEG preview into the
hardware-independent `/sensors/*` contract. It preserves timestamps and frame
IDs; it does not start a second camera, IMU driver or TF publisher.

## D435i hardware acceptance

Run the automated adapter test before connecting hardware:

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select luxi_adapter
colcon test-result --verbose
```

After the D435i is physically enumerated by `lsusb` and `/dev/video*`, start
the profile and leave it running:

```bash
ros2 launch luxi_adapter sensor_bringup.launch.py
```

In a second terminal, source the same two setup files and verify every
algorithm-facing input. Each `echo --once` command must return one message:

```bash
ros2 topic echo --once /sensors/rgbd/color/image_raw
ros2 topic echo --once /sensors/rgbd/depth/image_raw
ros2 topic echo --once /sensors/rgbd/color/camera_info
ros2 topic echo --once /sensors/imu/data_raw
ros2 topic echo --once /sensors/imu/data
ros2 run tf2_ros tf2_echo base_link camera_link
```

Then start `luxi_rtab_map`; confirm it reports `Canonical RGB-D input topics
are ready.` and that the web page shows the RGB preview. The D435i profile is
accepted only when all six checks pass. Re-run the same checks after a USB
unplug/replug: the profile keeps the driver running and the canonical topics
must resume without changing any mapping or web configuration.
