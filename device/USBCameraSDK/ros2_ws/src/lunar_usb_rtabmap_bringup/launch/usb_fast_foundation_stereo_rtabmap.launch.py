"""Fast-FoundationStereo TensorRT/H30 USB chain feeding Luxi and RTAB-Map."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


DEFAULT_ENGINE_PATH = (
    "~/.cache/luxi/fast_foundation_stereo/ffs_320x192_i4_d96.engine"
)

_BASE_SPEC = importlib.util.spec_from_file_location(
    "lunar_usb_crestereo_launch",
    Path(__file__).with_name("usb_crestereo_rtabmap.launch.py"),
)
_BASE = importlib.util.module_from_spec(_BASE_SPEC)
_BASE_SPEC.loader.exec_module(_BASE)


def _forwarded_arguments():
    """Reuse the validated learned-depth RTAB profile with an FFS engine."""
    common_names = [action.name for action in _BASE._BASE._common_launch_arguments()]
    forwarded = {name: LaunchConfiguration(name) for name in common_names}
    forwarded.update({
        "model_family": "fast_foundation_stereo",
        "crestereo_params": LaunchConfiguration("depth_params"),
        "crestereo_model_path": LaunchConfiguration("model_path"),
        # FFS runs the swapped pair through the same serialized session for
        # left-right consistency. Never allocate a second engine copy.
        "crestereo_left_right_model_path": "",
        "execution_provider": "tensorrt",
        "crestereo_frontend_target_rate": "4.0",
        "crestereo_frontend_poll_multiplier": "4.0",
        "crestereo_frontend_max_keypoints": "1024",
        "crestereo_lightglue_cuda_graph_layers": "3",
        "crestereo_lightglue_cuda_graph_keypoints": "512",
        "crestereo_mapping_rate": "2.0",
        "crestereo_sensor_rgbd_topic": "/sensors/rgbd/rgbd_image",
    })
    return forwarded


def _validate_fast_foundation_engine(context):
    """Reject an unusable engine before the camera and H30 claim hardware."""
    engine_path = Path(LaunchConfiguration("model_path").perform(context)).expanduser()
    engine_error = _BASE._fast_foundation_engine_error(engine_path)
    if engine_error:
        raise RuntimeError(
            "Fast-FoundationStereo TensorRT engine is invalid: "
            f"{engine_path} ({engine_error}). Install a Jetson-compatible "
            "engine with "
            "`python3 scripts/install_fast_foundation_stereo_engine.py "
            "--engine /absolute/path/to/model.engine --output-name "
            "ffs_320x192_i4_d96.engine`, or restart this launch with "
            "`model_path:=/absolute/path/to/model.engine`."
        )
    return []


def generate_launch_description():
    return LaunchDescription([
        *_BASE._BASE._common_launch_arguments(),
        DeclareLaunchArgument(
            "depth_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_driver"),
                "config",
                "fast_foundation_stereo_depth_mapping_quality.yaml",
            ]),
            description=(
                "Shared disparity-confidence and mapping filter profile used "
                "after Fast-FoundationStereo inference."
            ),
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=DEFAULT_ENGINE_PATH,
            description=(
                "Serialized Fast-FoundationStereo TensorRT engine built for "
                "this Jetson."
            ),
        ),
        OpaqueFunction(function=_validate_fast_foundation_engine),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("lunar_usb_rtabmap_bringup"),
                    "launch",
                    "usb_crestereo_rtabmap.launch.py",
                ])
            ),
            launch_arguments=_forwarded_arguments().items(),
        ),
    ])
