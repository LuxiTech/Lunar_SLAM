# Hik MV-CH120-60UC — Ubuntu 22.04 / ROS 2 Humble

This workspace is adapted for Ubuntu 22.04 (Jammy), ROS 2 Humble and ARM64.
It uses the MVS SDK at `/opt/MVS` by default; set `MVS_ROOT` only if it is
installed elsewhere.

## First-time setup

Install the **Linux ARM64 development** edition of Hikrobot MVS, not merely
its runtime. Before building, this file must exist:

```bash
test -f /opt/MVS/include/MvCameraControl.h
```

The present machine has `/opt/MVS/lib/aarch64/libMvCameraControl.so`, but the
required header is absent, so MVS must be reinstalled or its development files
must be restored. This is intentionally a build-time error rather than a
misleading link failure.

If the development headers are already available in a separate SDK checkout,
keep the installed runtime and pass its include directory explicitly, for
example:

```bash
MVS_INCLUDE_DIR=/path/to/MVS/include bash scripts/bootstrap_humble.sh
```

After installing MVS, run:

```bash
cd /home/nvidia/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws
bash scripts/bootstrap_humble.sh
```

The script asks for sudo only to install ROS 2 Humble and system dependencies.
It builds the camera packages and the bundled H30 IMU driver, while using the
needed Humble binary RTAB-Map packages rather than the checkout's Lyrical-era
source tree.

## Run

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws/install_humble/setup.bash

# Camera only
ros2 launch hik_bringup camera_only.launch.py

# Stereo images, depth and RViz
ros2 launch hik_bringup stereo_camera_bringup.launch.py

# Camera, H30 IMU and RTAB-Map
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py
```

The launch files pass the MVS library directory only to the camera process.
This prevents MVS's bundled `libusb` from replacing the system `libusb` used by
PCL and RTAB-Map.
