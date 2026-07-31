# luxi_visual_frontend

This ROS 2 package replaces RTAB-Map's high-rate ORB odometry with a CUDA
SuperPoint + LightGlue RGB-D frontend. Accepted frames publish both:

- `/luxi_visual_frontend/odom` (`nav_msgs/Odometry`), the level-A interface;
- `/luxi_visual_frontend/rgbd_image` (`rtabmap_msgs/RGBDImage`) containing
  SuperPoint keypoints, metric 3D points and compressed float descriptors, the
  level-B interface.

The package does not run NetVLAD. NetVLAD remains isolated in `luxi_hloc` for
low-rate global retrieval.

## Shared model files

The node reuses the workspace-level assets instead of duplicating weights in a
ROS install tree:

```text
3parts/hloc/third_party/SuperGluePretrainedNetwork/models/weights/superpoint_v1.pth
3parts/hloc_models/hub/checkpoints/superpoint_lightglue_v0-1_arxiv.pth
```

The verified SHA-256 values on this target are:

- SuperPoint: `52b6708629640ca883673b5d5c097c4ddad37d8048b33f09c8ca0d69db12c40e`;
- LightGlue: `6ff7040d0a497fc6639337946d7538dae07428c18f77a067a0b5a960e7cc551a`.

Check the CUDA runtime and weights after building:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run luxi_visual_frontend check_visual_frontend_environment.py
```

Start only the frontend when debugging:

```bash
ros2 launch luxi_visual_frontend visual_odometry.launch.py
```

Production mapping is started through
`luxi_rtab_map/rgbd_mapping_learned.launch.py`, which makes the frontend the
only `odom -> base_link` publisher and configures RTAB-Map to consume external
odometry and external local features.

See `docs/test_report.md` for the D435i, RTAB-Map and web end-to-end results.
