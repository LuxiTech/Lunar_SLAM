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
            "model_path": LaunchConfiguration("crestereo_model_path"),
            "execution_provider": LaunchConfiguration("execution_provider"),
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
        "--Kp/MaxDepth 3.0": "--Kp/MaxDepth 4.0",
        "--Grid/RangeMax 3.0": "--Grid/RangeMax 10.0",
        "--Grid/NoiseFilteringRadius 0.12": "--Grid/NoiseFilteringRadius 0.35",
        "--Grid/NoiseFilteringMinNeighbors 8": "--Grid/NoiseFilteringMinNeighbors 3",
    }
    for original, replacement in replacements.items():
        if original not in rtabmap_arguments:
            raise RuntimeError(f"missing inherited RTAB parameter: {original}")
        rtabmap_arguments = rtabmap_arguments.replace(original, replacement)
    arguments.update({
        "visual_odometry": "false",
        "odom_topic": "/luxi_visual_frontend/odom",
        "subscribe_odom_info": "false",
        "visual_frontend_publish_tf": "true",
        "visual_frontend_target_rate": "6.0",
        "visual_frontend_upstream_rate_limited": "true",
        "visual_frontend_rgbd_features_rate": "1.0",
        # Keep metric pose constraints inside the reliable near layer. The
        # depth image sent to RTAB retains more room-shape support at 4-6 m
        # and a coarser context layer at 6-10 m.
        "visual_frontend_maximum_depth": "4.0",
        "visual_frontend_mapping_depth_minimum": "0.4",
        "visual_frontend_mapping_depth_dense_maximum": "4.0",
        "visual_frontend_mapping_depth_medium_maximum": "6.0",
        "visual_frontend_mapping_depth_medium_sparse_pixel_step": "4",
        "visual_frontend_mapping_depth_far_maximum": "10.0",
        "visual_frontend_mapping_depth_far_sparse_pixel_step": "8",
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
        "crestereo_init_iter2_180x320_fp16.onnx",
    ])
    return LaunchDescription([
        *_BASE._common_launch_arguments(),
        DeclareLaunchArgument("crestereo_model_path", default_value=model_default),
        DeclareLaunchArgument("execution_provider", default_value="cuda"),
        OpaqueFunction(function=_launch_profile),
    ])
