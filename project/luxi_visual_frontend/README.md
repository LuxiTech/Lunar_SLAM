# luxi_visual_frontend

This ROS 2 package replaces RTAB-Map's high-rate ORB odometry with a CUDA
SuperPoint + LightGlue RGB-D frontend. Accepted frames publish both:

- `/luxi_visual_frontend/odom` (`nav_msgs/Odometry`), the level-A interface;
- `/luxi_visual_frontend/rgbd_image` (`rtabmap_msgs/RGBDImage`) containing
  SuperPoint keypoints, metric 3D points and compressed float descriptors, the
  level-B interface.

The production input is the adapter's atomic `/sensors/rgbd/rgbd_image`
packet. Its best-effort queue keeps only the newest frame so slower learned
inference cannot accumulate camera latency.

## SuperPoint inference acceleration

The frontend selects a fixed-shape TensorRT engine when one is available and
falls back to the original PyTorch model otherwise. TensorRT runs only the
dense SuperPoint convolution network; NMS, descriptor sampling and geometric
verification remain unchanged. Production uses TensorRT FP16 for SuperPoint
and PyTorch AMP FP16 for LightGlue. TensorRT I/O stays FP32, preserving the
original post-processing and RTAB descriptor format.

Build the HIK `1024x750 -> 800x586` engine once on the target Jetson:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run luxi_visual_frontend build_superpoint_tensorrt.py \
  --height 586 --width 800 --precision fp16
```

For a D435i `640x480` stream, build an additional shape-specific engine:

```bash
ros2 run luxi_visual_frontend build_superpoint_tensorrt.py \
  --height 480 --width 640 --precision fp16
```

TensorRT plans are GPU/TensorRT-version specific and are therefore generated
locally under `3parts/hloc_models/tensorrt/`, not committed to Git.  The
`superpoint_backend` parameter accepts `auto` (default), `pytorch` or
`tensorrt`; `superpoint_precision` accepts `fp32`, `fp16` or `int8`. Forced
`tensorrt` mode reports a missing or incompatible engine as an error instead of
silently changing inference backends. Diagnostics expose both active paths,
for example `superpoint_backend=tensorrt_fp16` and
`lightglue_backend=pytorch_amp_fp16`.

On Jetson Orin NX, CUDA Graph is enabled for the fixed-shape TensorRT enqueue
and for the common LightGlue temporal path (up to 512 points, first three
layers). It does not change SuperPoint resolution, NMS, thresholds, feature
limit or weights. Frames above 512 points and LightGlue layers after the
captured prefix automatically retain the original adaptive FlashAttention
path. Diagnostics append `_cudagraph` to the active backend and report
`extract_seconds`, `match_seconds` and `match_layers`.

INT8 building is supported only for controlled experiments and requires at
least 16 representative normalized images in an NPZ `images` array. The
current 32-frame HIK calibration was rejected by the geometric quality gate
(inlier ratio 0.815 versus FP16 0.949), so production must remain FP16.

The validated HIK settings retain `resize_max=800` (`800x586` network input),
cap features at 1024 and use NMS radius 4. Reducing the long edge to 720 lost
16.7% of correct matches; 640 lost 29.8%, so lowering input resolution is not a
safe production optimization.

Odometry tracking and RTAB feature publication have independent rates. On the
HIK + Orin NX profile the frontend tracks the newest frame at 5 Hz, while
`rgbd_features_rate` defaults to 1 Hz for RTAB-Map's 1 Hz detector. This keeps
the combined stereo and learned-frontend GPU duty in range without changing
feature quality, avoids rebuilding a full RGB-D feature message on every
tracking update, and retains one-frame margin for rejected updates.

## Visual and IMU rotation policy

Filtered IMU orientation is not treated as an absolute yaw measurement. PnP
keeps authority over heading, while IMU gravity checks roll/pitch consistency.
The fixed-rotation RGB-D translation path is used only when its full relative
rotation agrees with PnP. During a short visual gap, keyframe reseeding
propagates relative IMU rotation from the last accepted visual pose instead of
reapplying orientation accumulated since startup. Rejected frames are not
published to RTAB-Map.

The learned mapping launch disables direct backend IMU input by default, so
the same yaw is not fused once in this frontend and again in RTAB-Map. The
diagnostic topic exposes both `imu_rotation_error_deg` and the yaw-independent
`imu_gravity_error_deg`.

Camera/base-frame conversion uses a unit-quaternion rigid rotation. Its
orthogonality, determinant and quaternion round trip are regression-tested;
otherwise a non-rigid transform can turn an ordinary yaw crossing into a large
published odometry jump.

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
