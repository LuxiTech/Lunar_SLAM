# Luxi Adapter

`luxi_adapter` is the only hardware-facing package consumed by the project
algorithms. `sensor_bringup.launch.py` starts exactly one profile selected by
the public `hardware` launch argument, then publishes the hardware-independent
contract:

```text
/sensors/rgbd/rgbd_image
/sensors/rgbd/color/image_raw
/sensors/rgbd/depth/image_raw
/sensors/rgbd/color/camera_info
/sensors/rgbd/color/image_raw/compressed  # optional preview transport
/sensors/imu/data_raw
/sensors/imu/data
/tf
/tf_static
```

The atomic RGBD packet is the algorithm input; split image topics are retained
for compatibility and preview consumers. The depth stream must be registered
to the color camera. All image, camera-info and IMU messages must contain their
original driver timestamps and frame IDs.

## Select one hardware profile

The only hardware choice required at runtime is:

```bash
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=hik
# or
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d435i
# or
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d455
# or
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=zedx
```

Names are case-insensitive. `hardware:=hik` selects `launch/hik.launch.py` and
`config/hik_sensor_bringup.yaml`; `hardware:=d435i` selects
`launch/d435i.launch.py` and the current default `config/sensor_bringup.yaml`.
`hardware:=d455` selects `launch/d455.launch.py` and
`config/d455_sensor_bringup.yaml`.
`hardware:=zedx` selects `launch/zedx.launch.py` and
`config/zedx_sensor_bringup.yaml`.
An explicit `config:=...` is an optional full-profile override, and launch fails
immediately if that file declares a different `hardware_profile`.

To add a future sensor named `<profile>`, add
`launch/<profile>.launch.py` and `config/<profile>_sensor_bringup.yaml`. The
profile must produce the contract above. Do not add its native topics or a new
hardware branch to `luxi_visual_frontend`, `luxi_rtab_map`, RViz, localization,
navigation or web packages.

After the selected profile is ready, all cameras use the same algorithm launch:

```bash
ros2 launch luxi_rtab_map rgbd_mapping_learned.launch.py \
  new_map:=true load_saved_map:=false
```

## D435i profile

The default `d435i` profile reads its driver setup path, ROS domain and all
topic names from `config/sensor_bringup.yaml`. It starts the existing D435i
driver, relays the vendor topics into the stable contract and filters the
canonical raw IMU with Madgwick. The profile waits for an explicit calibration
request from the web page and does not publish `base_link` to `camera_link`
before that request succeeds. Put the robot on level ground, keep it stationary,
then click `一键水平校准`. The C++ calibrator collects 200 samples and publishes
one static transform whose roll and pitch align gravity with the robot's vertical
axis. Translation and yaw remain configured values because gravity cannot
estimate them.

The current camera-forward translation is the midpoint of the measured range,
`camera_x=0.14 m`. Replace it with a measured value when available. Do not move
the robot until the page reports `校准完成`; motion or an abnormal acceleration
magnitude restarts the sample window automatically. Mapping and localization
are blocked while calibration is missing or in progress.

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d435i
```

## D455 wide-FOV profile

The D455 profile starts the independently pinned workspace under
`device/D455/ros2_ws`.  It requires a D455 (`device_type=d455`) and publishes
aligned color/depth at `848x480@15 Hz`. This retains the wide image while
avoiding the UVC watchdog gaps observed at 30 Hz on the deployed Jetson.
Unlike D435i's approximately 69 by 42
degree RGB field of view, D455 RGB is approximately 90 by 65 degrees, so
aligning depth to color no longer crops the useful depth view to the narrow
D435i RGB image.  The adapter republishes the exact same `/sensors/*` contract;
mapping and navigation launches do not change.

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d455
```

The front-mounted D455 is measured at `camera_x=0.18 m` from the robot rotation
center. Measure y/z and yaw again if the bracket changes; stationary web
calibration determines only roll and pitch.

## ZED X GMSL2 profile

The `zedx` profile sources the pinned ZED SDK 5.4 / ROS 2 Wrapper 5.4 workspace
under `device/sterellab_ZEDX`, refreshes the container's host Argus/IMU sockets,
and opens camera serial `45570700`. The camera grabs `HD1200@30` internally and
publishes registered RGB/depth at `960x600@10 Hz` using `NEURAL`. ZED
positional tracking, its `map -> odom`/`odom -> camera` TF and native point cloud
publication are disabled: Luxi's learned frontend remains the only odometry TF
owner and RTAB-Map remains the map TF owner.

ZED raw IMU is relayed through the same Madgwick filter used by the RealSense
profiles. The ZED Wrapper publishes the camera-to-IMU and camera optical TF
tree, while this profile publishes only `base_link -> zed_camera_link`.

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=zedx
```

The checked-in `base_link -> zed_camera_link` transform is identity for bench
testing because the final robot mounting pose has not been provided. Measure
and update `camera_x/y/z` and `camera_roll/pitch/yaw` in
`config/zedx_sensor_bringup.yaml` before judging trajectory accuracy or using
the camera for navigation.

After the canonical topics pass, start the common mapper without a second ZED
driver or ZED RViz instance:

```bash
ros2 launch luxi_rtab_map rgbd_mapping_learned.launch.py \
  rviz:=false new_map:=true load_saved_map:=false
```

The completed Jetson/ZED X acceptance measurements and expected warnings are
recorded in [zedx_test_report.md](zedx_test_report.md).

## Hik stereo + H30 profile

The HIK profile is optional.  A D435i-only host does not need the HIK MVS SDK,
`hik_bringup`, or the H30 serial driver in order to build `luxi_adapter`.
Before selecting `hardware:=hik`, build/install the packages in
`device/Hik_MV-CH120-60UC/ros2_ws/src`; the launch resolver will report a
missing HIK package or launch file if that optional stack is unavailable.

The Hik profile preserves the calibrated 1024x750 rectified colour, depth and
CameraInfo generated by the VPI stereo pipeline. The H30 AHRS orientation is
already valid, so it is relayed unchanged to both canonical IMU topics without
a second filter.

Start the Hik hardware profile with the same public selector:

```bash
cd /home/nvidia/Desktop/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=hik
```

Then use the common algorithm command shown above. The compatibility launch
`luxi_rtab_map/hik_mapping.launch.py` still combines those two steps, but it is
not required by the hardware-independent pipeline.

The Hik package only produces calibrated RGB-D/IMU data; its RTAB-Map odometry
configuration is not used. The adapter consumes the native synchronized
`/stereo/rgbd_image`, normalizes its nested headers once, and forwards it as
the hardware-independent atomic `/sensors/rgbd/rgbd_image` contract while
relaying `/imu/data`. Original pixels, frame IDs and calibration are preserved;
the learned frontend consumes the atomic packet without re-synchronizing split
topics.

For the Hik profile, `stereo_node`, `stereo_depth_node` and `luxi_adapter` are
C++ components in `/luxi_sensor_container` with intra-process communication.
The camera component starts first and the depth/adapter components load after
the MVS initialization window. Raw left/right images and native RGB-D therefore
do not cross a DDS process boundary; split canonical images are still published
on demand for RViz and other compatibility consumers.

## RealSense hardware acceptance

Run the automated adapter test before connecting hardware:

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select luxi_adapter
colcon test-result --verbose
```

After the selected camera is physically enumerated by `lsusb` and
`/dev/video*`, start its profile and leave it running:

```bash
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d435i
# D455 alternative: hardware:=d455
# ZED X alternative: hardware:=zedx
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
