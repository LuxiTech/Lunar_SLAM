# USB Stereo Bringup

Run `ros2 launch usb_camera_bringup stereo_camera.launch.py` after sourcing the workspace.
The camera parameters are in `usb_camera_driver/config/stereo_camera.yaml`.

The H30 configuration is `h30_imu.yaml`. Install its device-specific udev rule
with `ros2 run usb_camera_bringup install_h30_udev_rule.sh`; the resulting port
is `/dev/imu-H30` at 921600 baud. Do not enable SLAM fusion while either IMU
covariance starts with `-1`.
