"""Run saved-map navigation with the current USB learned-depth/H30 sensor."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _common_launch_arguments():
    """Arguments shared by CRE and Fast-FoundationStereo navigation."""
    return [
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("octomap_path", default_value=""),
        DeclareLaunchArgument("cloud_path", default_value=""),
        DeclareLaunchArgument("hloc_map_directory", default_value=""),
        DeclareLaunchArgument("semantic_path", default_value=""),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/navigation/cmd_vel"),
        DeclareLaunchArgument(
            "body_imu_topic",
            default_value="/d15041873/imu_sensor_broadcaster/imu",
        ),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("wait_for_camera", default_value="true"),
        DeclareLaunchArgument("camera_wait_timeout", default_value="60.0"),
        DeclareLaunchArgument(
            "camera_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_driver"),
                "config",
                "stereo_camera.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_driver"),
                "config",
                "stereo_opencv.yaml",
            ]),
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "camera_frame", default_value="left_camera_optical_frame"
        ),
        # Keep saved-map localization in exactly the same robot frame used to
        # build the map.  The stereo midpoint is (+0.20, 0, +0.20) m from the
        # D1 trunk center; the left optical center is half of the calibrated
        # 89.963 mm baseline toward robot-left.
        DeclareLaunchArgument("camera_x", default_value="0.20"),
        DeclareLaunchArgument("camera_y", default_value="0.044982"),
        DeclareLaunchArgument("camera_z", default_value="0.20"),
        DeclareLaunchArgument("camera_qx", default_value="-0.5"),
        DeclareLaunchArgument("camera_qy", default_value="0.5"),
        DeclareLaunchArgument("camera_qz", default_value="-0.5"),
        DeclareLaunchArgument("camera_qw", default_value="0.5"),
    ]


def _navigation_action():
    """Create the saved-map stack only after sensor health has passed."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("luxi_3d_navigation"),
                "launch",
                "saved_map_navigation.launch.py",
            ])
        ),
        launch_arguments={
            "database_path": LaunchConfiguration("database_path"),
            "octomap_path": LaunchConfiguration("octomap_path"),
            "cloud_path": LaunchConfiguration("cloud_path"),
            "hloc_map_directory": LaunchConfiguration("hloc_map_directory"),
            "semantic_path": LaunchConfiguration("semantic_path"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "body_imu_topic": LaunchConfiguration("body_imu_topic"),
        }.items(),
    )


def _on_sensor_health_exit(event, _context):
    """Start navigation only for a successful RGB-D/H30 health process."""
    if event.returncode != 0:
        raise RuntimeError(
            "RGB-D/H30 synchronization or IMU health check failed. "
            "Saved-map localization and navigation were not started."
        )
    return [
        LogInfo(msg="Saved-map RGB-D/H30 synchronization check passed."),
        _navigation_action(),
    ]


def _launch_navigation_after_sensor_health(context):
    """Run a non-blocking sensor check, then create the localization stack."""
    enabled = LaunchConfiguration("wait_for_camera").perform(context).lower()
    if enabled not in {"1", "true", "yes", "on"}:
        return [
            LogInfo(msg="Saved-map sensor synchronization check is disabled."),
            _navigation_action(),
        ]

    timeout = float(LaunchConfiguration("camera_wait_timeout").perform(context))
    require_imu = LaunchConfiguration("use_imu").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    health_check = ExecuteProcess(
        cmd=[
            "ros2", "run", "luxi_rtab_map", "sensor_sync_check", "--ros-args",
            "-p", "rgbd_topic:=/sensors/rgbd/rgbd_image",
            "-p", "imu_topic:=/sensors/imu/data",
            "-p", f"require_imu:={'true' if require_imu else 'false'}",
            "-p", f"timeout_sec:={timeout}",
        ],
        name="saved_map_sensor_sync_check",
        output="screen",
    )
    # Register before starting the checker so a fast preflight failure cannot
    # race past the event handler. ExecuteProcess leaves the launch event loop
    # free to start and service the stereo, FFS and H30 processes above.
    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=health_check,
                on_exit=_on_sensor_health_exit,
            )
        ),
        health_check,
    ]


def _crestereo_launch_arguments():
    """Compatibility defaults for the original CREStereo navigation mode."""
    return [
        DeclareLaunchArgument("model_family", default_value="crestereo"),
        DeclareLaunchArgument(
            "crestereo_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_driver"),
                "config",
                "crestereo_depth_mapping_quality.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "crestereo_model_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_driver"),
                "models",
                "crestereo_init_iter2_360x640.onnx",
            ]),
        ),
        DeclareLaunchArgument(
            "crestereo_left_right_model_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_driver"),
                "models",
                "crestereo_init_iter2_180x320_fp16.onnx",
            ]),
        ),
        DeclareLaunchArgument("execution_provider", default_value="cuda"),
    ]


def generate_launch_description() -> LaunchDescription:
    """Start one sensor producer and the saved-map localization stack."""
    use_imu = LaunchConfiguration("use_imu")
    sensor = IncludeLaunchDescription(
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
    return LaunchDescription([
        *_common_launch_arguments(),
        *_crestereo_launch_arguments(),
        sensor,
        adapter,
        mounting_tf,
        # Build the checker only after the sensor actions are registered. It
        # runs as an asynchronous launch process, allowing those sensor nodes
        # to publish while the check waits. HLoc/VO/navigation are created only
        # by the successful OnProcessExit branch.
        TimerAction(
            period=3.0,
            actions=[
                OpaqueFunction(
                    function=_launch_navigation_after_sensor_health
                )
            ],
        ),
    ])
