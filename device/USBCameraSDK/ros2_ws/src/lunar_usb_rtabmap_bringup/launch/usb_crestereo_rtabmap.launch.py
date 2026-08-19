"""Primary pre-trained CREStereo/H30 USB chain feeding Luxi and RTAB-Map."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_BASE_SPEC = importlib.util.spec_from_file_location(
    "lunar_usb_rtabmap_vpi_launch", Path(__file__).with_name("usb_rtabmap.launch.py")
)
_BASE = importlib.util.module_from_spec(_BASE_SPEC)
_BASE_SPEC.loader.exec_module(_BASE)


def _sensor_actions():
    use_imu = LaunchConfiguration("use_imu")
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("usb_camera_bringup"),
                "launch",
                "stereo_crestereo_rgbd.launch.py",
            ])
        ),
        launch_arguments={
            "start_imu": use_imu,
            "camera_params": LaunchConfiguration("camera_params"),
            "calibration_file": LaunchConfiguration("calibration_file"),
            "crestereo_params": LaunchConfiguration("crestereo_params"),
            "model_family": LaunchConfiguration("model_family"),
            "model_path": LaunchConfiguration("crestereo_model_path"),
            "left_right_model_path": LaunchConfiguration(
                "crestereo_left_right_model_path"
            ),
            "execution_provider": LaunchConfiguration("execution_provider"),
            # Preserve full visual context without making distant learned
            # depth dense: 0.4-4 m stays the navigation-quality main layer,
            # while 4-6 m and 6-10 m are progressively sparse previews.
            "point_cloud_max_depth_m": "10.0",
            "point_cloud_far_sparse_start_m": "4.0",
            "point_cloud_medium_max_depth_m": "6.0",
            "point_cloud_medium_sparse_pixel_step": "4",
            "point_cloud_far_sparse_pixel_step": "8",
        }.items(),
    )
    adapter = Node(
        package="luxi_adapter",
        executable="sensor_adapter_node",
        name="luxi_adapter",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("lunar_usb_rtabmap_bringup"),
                "config",
                "usb_adapter.yaml",
            ]),
            {
                "enable_imu": ParameterValue(use_imu, value_type=bool),
                # CREStereo publishes a freshest-frame RGB-D stream.  Match it
                # end to end so RViz/network clients cannot back-pressure the
                # learned depth or visual odometry processes.
                "reliable_rgbd_input": False,
                "reliable_image_output": False,
            },
        ],
        output="screen",
    )
    mounting_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="usb_base_to_left_camera_tf",
        arguments=[
            "--x", LaunchConfiguration("camera_x"),
            "--y", LaunchConfiguration("camera_y"),
            "--z", LaunchConfiguration("camera_z"),
            "--qx", LaunchConfiguration("camera_qx"),
            "--qy", LaunchConfiguration("camera_qy"),
            "--qz", LaunchConfiguration("camera_qz"),
            "--qw", LaunchConfiguration("camera_qw"),
            "--frame-id", LaunchConfiguration("base_frame"),
            "--child-frame-id", LaunchConfiguration("camera_frame"),
        ],
        output="screen",
    )
    return [camera, adapter, mounting_tf]


def _fast_foundation_engine_error(engine_path):
    """Return why a TensorRT engine cannot safely be handed to the node."""
    if engine_path.suffix.lower() != ".engine":
        return "the file name must end in .engine"
    if not engine_path.is_file():
        return "the file does not exist or is not a regular file"
    try:
        size = engine_path.stat().st_size
    except OSError as exc:
        return f"the file metadata cannot be read: {exc}"
    if size <= 0:
        return "the engine file is empty"
    return ""


def _validate_crestereo_models(context):
    """Fail before camera capture when the selected learned model is unavailable."""
    model_family = context.launch_configurations.get("model_family", "crestereo")
    if model_family == "fast_foundation_stereo":
        engine_path = Path(
            LaunchConfiguration("crestereo_model_path").perform(context)
        ).expanduser()
        engine_error = _fast_foundation_engine_error(engine_path)
        if engine_error:
            raise RuntimeError(
                "Fast-FoundationStereo TensorRT engine is invalid: "
                f"{engine_path} ({engine_error}). Install a Jetson-compatible "
                "engine with "
                "`python3 scripts/install_fast_foundation_stereo_engine.py "
                "--engine /absolute/path/to/model.engine --output-name "
                "ffs_320x192_i4_d96.engine`, or pass "
                "`model_path:=/absolute/path/to/model.engine` to "
                "usb_fast_foundation_stereo_rtabmap.launch.py."
            )
        return []
    if model_family != "crestereo":
        raise RuntimeError(
            "unsupported learned stereo model_family: "
            f"{model_family!r}; expected 'crestereo' or "
            "'fast_foundation_stereo'"
        )
    required_models = (
        ("CREStereo refinement model", "crestereo_model_path"),
        ("CREStereo left/right model", "crestereo_left_right_model_path"),
    )
    missing = []
    for label, argument in required_models:
        model_path = Path(LaunchConfiguration(argument).perform(context)).expanduser()
        if not model_path.is_file():
            missing.append(f"{label}: {model_path}")
    if missing:
        raise RuntimeError(
            "CREStereo model preflight failed; rebuild usb_camera_driver "
            "before starting mapping (colcon build --packages-select "
            "usb_camera_driver --symlink-install): "
            + "; ".join(missing)
        )
    return []


def _mapping_arguments():
    """Use one learned tracker for pose and features in the model profile.

    The classic RTAB-Map F2M profile is tuned for VPI's sparse, low-noise near
    depth.  CREStereo is much denser but temporally noisier, and F2M can reset
    repeatedly while the camera turns.  Those reset poses made the same scene
    appear twice in opposite directions.  SuperPoint/LightGlue already runs
    in this profile and publishes an odometry/RGB-D pair with the exact same
    sensor stamp, so let that geometrically verified pair own model odometry.
    Rejected learned frames are not inserted into RTAB-Map.
    """
    arguments = _BASE._mapping_arguments().copy()
    rtabmap_arguments = arguments["rtabmap_args"]
    replacements = {
        # Dense learned depth needs more frame-to-frame overlap than the VPI
        # profile.  The former 0.12 m / 0.12 rad spacing made object surfaces
        # visibly separate while the D1 was walking.
        # Reflective corridors produce many visually similar floor/wall
        # views. Do not accept a weak appearance-only closure which can fold
        # an otherwise continuous learned-odometry trajectory.
        "--Rtabmap/LoopThr 0.2": "--Rtabmap/LoopThr 0.25",
        "--RGBD/OptimizeMaxError 3.0": "--RGBD/OptimizeMaxError 1.0",
        "--Vis/MinInliers 20": "--Vis/MinInliers 30",
        "--RGBD/LinearUpdate 0.12": "--RGBD/LinearUpdate 0.06",
        "--RGBD/AngularUpdate 0.12": "--RGBD/AngularUpdate 0.05",
        "--Kp/MaxDepth 3.0": "--Kp/MaxDepth 4.0",
        # The 4-10 m sparse layer is useful for visual context and loop
        # closure, but passive-stereo uncertainty there must not become a
        # navigation obstacle. Keep the local occupancy grid D435-like.
        "--Grid/RangeMax 3.0": "--Grid/RangeMax 4.0",
        # A 35 cm / 3-neighbor gate accepted the dense radial ghost sheets in
        # map100. Keep only locally supported near-map geometry for planning;
        # the separate 4-10 m layer remains available as sparse context.
        "--Grid/NoiseFilteringRadius 0.12": "--Grid/NoiseFilteringRadius 0.12",
        "--Grid/NoiseFilteringMinNeighbors 8": "--Grid/NoiseFilteringMinNeighbors 8",
    }
    for original, replacement in replacements.items():
        if original not in rtabmap_arguments:
            raise RuntimeError(f"missing inherited RTAB parameter: {original}")
        rtabmap_arguments = rtabmap_arguments.replace(original, replacement)
    # White corridor walls and reflected floor streaks should not satisfy the
    # traversable-ground classifier merely because MaxGroundAngle was loose.
    rtabmap_arguments += (
        " --Optimizer/Robust true"
        " --Grid/MaxGroundAngle 25.0"
        # Segment in the gravity-aligned map frame. On a walking D1, doing it
        # in the pitching/rolling base frame turns level floor into obstacles.
        " --Grid/MapFrameProjection true"
        # Later valid observations clear isolated learned-depth ghosts along
        # the camera ray instead of preserving every false return forever.
        " --Grid/RayTracing true"
        # Geometry above the robot is retained in the colored context cloud,
        # but ceilings and high pipework cannot block a ground route.
        " --Grid/MaxObstacleHeight 0.8"
    )
    arguments.update({
        "visual_odometry": "false",
        "odom_topic": "/luxi_visual_frontend/odom",
        "subscribe_odom_info": "false",
        "visual_frontend_publish_tf": "true",
        "visual_frontend_target_rate": LaunchConfiguration(
            "crestereo_frontend_target_rate"
        ),
        "visual_frontend_upstream_rate_limited": "true",
        "visual_frontend_sensor_poll_rate_multiplier": LaunchConfiguration(
            "crestereo_frontend_poll_multiplier"
        ),
        "visual_frontend_max_keypoints": LaunchConfiguration(
            "crestereo_frontend_max_keypoints"
        ),
        "visual_frontend_lightglue_cuda_graph_layers": LaunchConfiguration(
            "crestereo_lightglue_cuda_graph_layers"
        ),
        "visual_frontend_lightglue_cuda_graph_keypoints": LaunchConfiguration(
            "crestereo_lightglue_cuda_graph_keypoints"
        ),
        "visual_frontend_rgbd_features_rate": LaunchConfiguration(
            "crestereo_mapping_rate"
        ),
        # Refresh a stale reference before lighting/viewpoint changes make PnP
        # bridge a large baseline. Motion still creates an earlier keyframe at
        # 0.10 m or 8 degrees.
        "visual_frontend_keyframe_max_age": "3.0",
        # Keep the dense RTAB map at 4 m, but let PnP sample the first metre of
        # the sparse context layer. In the low-mounted corridor view almost
        # all textured wall features sit just beyond 4 m; a hard 4 m tracking
        # gate left only 26 depth matches and starved otherwise strong RGB
        # correspondences.
        "visual_frontend_maximum_depth": "5.0",
        "visual_frontend_depth_sampling_radius": "3",
        # Current CRE depth is not used for frame-to-frame translation: PnP
        # keeps metric scale from the reference depth and H30 constrains tilt.
        "visual_frontend_use_depth_translation_refinement": "false",
        "visual_frontend_stationary_maximum_median_pixel_motion": "0.75",
        "visual_frontend_stationary_maximum_rotation_deg": "0.50",
        # Mapping is driven manually by the web controller. Subscribe to its
        # continuously published command directly, so startup/teardown of the
        # navigation mux cannot create a short false-motion window. Saved-map
        # navigation still uses the final /cmd_vel mux output independently.
        "visual_frontend_motion_hint_topic": "/d1/cmd_vel_standard",
        "visual_frontend_stationary_hint_maximum_median_pixel_motion": "2.0",
        # CRE metric translation is noisy on glossy floors. A fresh zero D1
        # command may therefore hold pose from image motion + H30 rotation
        # without asking the corrupted depth translation to validate itself.
        "visual_frontend_stationary_hint_maximum_translation": "0.0",
        # Lock only base height. H30 roll/pitch remains in the camera pose to
        # compensate D1 trunk motion instead of flattening the 3-D cloud.
        "visual_frontend_constrain_vertical_translation": "true",
        "visual_frontend_mapping_depth_minimum": "0.4",
        "visual_frontend_mapping_depth_dense_maximum": "4.0",
        # Keep navigation geometry dense only to 4 m. The distant tiers remain
        # useful as sparse colored context/loop-closure structure; RTAB's
        # Grid/RangeMax below stays 4 m so they cannot become nav obstacles.
        "visual_frontend_mapping_depth_medium_maximum": "6.0",
        "visual_frontend_mapping_depth_medium_sparse_pixel_step": "4",
        "visual_frontend_mapping_depth_far_maximum": "10.0",
        "visual_frontend_mapping_depth_far_sparse_pixel_step": "8",
        # Passive stereo is unreliable on the textureless reflective floor.
        # H30 supplies the gravity direction, but this stage is filter-only:
        # it may remove an unverified sheet behind the support plane and may
        # never rewrite measured depth or synthesize missing floor.
        "visual_frontend_mapping_ground_prior_enabled": "true",
        "visual_frontend_mapping_ground_camera_height": "0.35",
        "visual_frontend_mapping_ground_below_tolerance": "0.05",
        "visual_frontend_mapping_ground_surface_tolerance": "0.04",
        # Preserve small coherent slopes/uneven ground. A return more than
        # 0.12 m behind the H30 support plane is too uncertain for walking
        # navigation: reject it as unknown instead of snapping/filling it.
        # Thus a real pit remains a non-traversable observation, never a
        # fabricated flat floor.
        "visual_frontend_mapping_ground_maximum_correction": "0.12",
        # H30 attitude and the physical mount can retain roughly 1-2 degrees
        # of roll/pitch error. Convert that angular bound to a range-growing
        # height tolerance so valid 3-4 m floor is not erased.
        "visual_frontend_mapping_ground_angular_uncertainty_degrees": "2.0",
        "visual_frontend_mapping_ground_minimum_up_alignment": "0.55",
        "visual_frontend_mapping_ground_minimum_row_ratio": "0.52",
        "visual_frontend_mapping_ground_reject_unverified_below_plane": "true",
        "visual_frontend_mapping_ground_repair_all_below_plane": "false",
        "visual_frontend_mapping_ground_hole_fill_radius": "0",
        "visual_frontend_mapping_ground_hole_fill_minimum_support": "0",
        "visual_frontend_mapping_ground_hole_fill_maximum_depth": "0.0",
        "visual_frontend_mapping_ground_filter_only": "true",
        "sensor_rgbd_topic": LaunchConfiguration(
            "crestereo_sensor_rgbd_topic"
        ),
        "rtabmap_args": rtabmap_arguments,
        "rtabmap_start_delay": "3.0",
    })
    return arguments


def _mapping():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("luxi_rtab_map"),
                "launch",
                "rgbd_mapping.launch.py",
            ])
        ),
        launch_arguments=_mapping_arguments().items(),
    )


def _launch_profile(context):
    removed = [
        name for name in ("mode", "depth_backend")
        if name in context.launch_configurations
    ]
    if removed:
        raise RuntimeError(
            "CREStereo has its own experimental launch; removed arguments: "
            + ", ".join(removed)
        )
    # Do not add the VPI F2M continuity postprocessor here: learned odometry
    # publishes /odom -> /base_link itself and shares its accepted-frame stamp
    # with /luxi_visual_frontend/rgbd_image.
    return [
        *_sensor_actions(),
        TimerAction(period=3.0, actions=[_mapping(), _BASE._cloud_preview()]),
    ]


def generate_launch_description():
    model_default = PathJoinSubstitution([
        FindPackageShare("usb_camera_driver"),
        "models",
        "crestereo_combined_iter2_360x640.onnx",
    ])
    left_right_model_default = PathJoinSubstitution([
        FindPackageShare("usb_camera_driver"),
        "models",
        "crestereo_init_iter2_180x320_fp16.onnx",
    ])
    return LaunchDescription([
        *_BASE._common_launch_arguments(),
        DeclareLaunchArgument(
            "crestereo_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_driver"),
                "config",
                "crestereo_depth_mapping_quality.yaml",
            ]),
        ),
        DeclareLaunchArgument("model_family", default_value="crestereo"),
        DeclareLaunchArgument("crestereo_model_path", default_value=model_default),
        DeclareLaunchArgument(
            "crestereo_left_right_model_path",
            default_value=left_right_model_default,
        ),
        DeclareLaunchArgument("execution_provider", default_value="cuda"),
        DeclareLaunchArgument("crestereo_frontend_target_rate", default_value="3.0"),
        DeclareLaunchArgument("crestereo_frontend_poll_multiplier", default_value="3.0"),
        DeclareLaunchArgument("crestereo_frontend_max_keypoints", default_value="1024"),
        DeclareLaunchArgument("crestereo_lightglue_cuda_graph_layers", default_value="3"),
        DeclareLaunchArgument("crestereo_lightglue_cuda_graph_keypoints", default_value="512"),
        DeclareLaunchArgument("crestereo_mapping_rate", default_value="2.0"),
        DeclareLaunchArgument(
            "crestereo_sensor_rgbd_topic",
            default_value="/sensors/rgbd/rgbd_image",
        ),
        OpaqueFunction(function=_validate_crestereo_models),
        OpaqueFunction(function=_launch_profile),
    ])
