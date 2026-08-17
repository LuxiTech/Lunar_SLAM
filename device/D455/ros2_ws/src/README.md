# D455 ROS 2 workspace source

This workspace pins the official RealSense ROS wrapper to tag `4.58.1` and
the SDK to tag `v2.58.2`.  The local `lunar_d455_bringup` package applies the
D455 mapping profile and verifies the streams consumed by `luxi_adapter`.

Upstream source: <https://github.com/realsenseai/realsense-ros.git>

The selected `848x480@30` color/depth profiles retain the D455 wide RGB field
of view after depth-to-color alignment while keeping the RGB-D mapping load
bounded.  Do not replace the D455 profile with the D435i `640x480` profile.

