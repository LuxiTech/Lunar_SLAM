# D435i ROS workspace source

This directory keeps project-specific D435i bringup and RTAB-Map integration
packages. Upstream SDK and ROS driver source trees are intentionally excluded
from this branch.

Download upstream dependencies as needed:

```text
Intel RealSense ROS:
https://github.com/IntelRealSense/realsense-ros.git

RTAB-Map:
https://github.com/introlab/rtabmap.git

RTAB-Map ROS:
https://github.com/introlab/rtabmap_ros.git
```

Suggested local paths:

```text
device/D435i/ros2_ws/src/realsense-ros
device/D435i/ros2_ws/src/rtabmap
device/D435i/ros2_ws/src/rtabmap_ros
```

Do not commit the downloaded upstream source trees.
