"""Run saved-map localization/navigation with Fast-FoundationStereo and H30."""

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
    "lunar_usb_crestereo_saved_map_navigation_launch",
    Path(__file__).with_name("usb_crestereo_saved_map_navigation.launch.py"),
)
_BASE = importlib.util.module_from_spec(_BASE_SPEC)
_BASE_SPEC.loader.exec_module(_BASE)


def _fast_foundation_engine_error(engine_path):
    """Return why a TensorRT engine cannot safely be used for navigation."""
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


def _forwarded_arguments():
    """Reuse the saved-map stack while replacing only its depth producer."""
    forwarded = {
        argument.name: LaunchConfiguration(argument.name)
        for argument in _BASE._common_launch_arguments()
    }
    forwarded.update({
        "model_family": "fast_foundation_stereo",
        "crestereo_params": LaunchConfiguration("depth_params"),
        "crestereo_model_path": LaunchConfiguration("model_path"),
        # The reverse consistency pass reuses the same TensorRT session.
        "crestereo_left_right_model_path": "",
        "execution_provider": "tensorrt",
    })
    return forwarded


def _validate_fast_foundation_engine(context):
    """Reject an unusable engine before localization claims camera or H30."""
    engine_path = Path(LaunchConfiguration("model_path").perform(context)).expanduser()
    engine_error = _fast_foundation_engine_error(engine_path)
    if engine_error:
        raise RuntimeError(
            "Fast-FoundationStereo TensorRT engine is invalid: "
            f"{engine_path} ({engine_error}). Install a validated Jetson "
            "engine with "
            "`python3 scripts/install_fast_foundation_stereo_engine.py "
            "--engine /absolute/path/to/ffs_320x192_i4_d96.engine`, or "
            "restart this "
            "launch with `model_path:=/absolute/path/to/model.engine`."
        )
    return []


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        *_BASE._common_launch_arguments(),
        DeclareLaunchArgument(
            "depth_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_driver"),
                "config",
                "fast_foundation_stereo_depth_mapping_quality.yaml",
            ]),
            description=(
                "Fast-FoundationStereo disparity-confidence profile used "
                "during saved-map localization."
            ),
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=DEFAULT_ENGINE_PATH,
            description=(
                "Serialized Fast-FoundationStereo TensorRT engine validated "
                "for this Jetson."
            ),
        ),
        OpaqueFunction(function=_validate_fast_foundation_engine),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("lunar_usb_rtabmap_bringup"),
                    "launch",
                    "usb_crestereo_saved_map_navigation.launch.py",
                ])
            ),
            launch_arguments=_forwarded_arguments().items(),
        ),
    ])
