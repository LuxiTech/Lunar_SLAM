# EVK4 HD — Ubuntu 22.04 / ROS 2 Humble

`evk4_driver` now defaults to ROS 2 Humble and no longer assumes a private
Lyrical installation. The EVK4 and OpenEB SDK are not connected on this
machine, so the following commands prepare the software but cannot prove a
live event stream until the camera is attached.

## Install and build

```bash
cd /home/nvidia/lunar_slam/device/evk/ros2_ws
bash scripts/bootstrap_humble.sh
```

The script requires the official ROS apt source and sudo. It installs the
Humble binary `metavision_driver` stack, builds only this repository's
`evk4_driver` wrapper, and installs its USB udev rule. Do not reuse the old
`.local_ros/opt/ros/lyrical` tree: its binaries are not ABI-compatible with
Humble on Ubuntu 22.04. `scripts/fetch_humble_dependencies.sh` remains only
for source-based OpenEB development.

## Verify and run

After connecting the EVK4 and installing the udev rule (see `udev/`), run:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch evk4_driver info.launch.py
ros2 launch evk4_driver evk4_driver.launch.py
ros2 launch evk4_driver evk4_image_view.launch.py
```
