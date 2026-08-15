"""Run saved-map navigation with the current USB CREStereo/H30 sensor."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
            "--frame-id", "base_link",
            "--child-frame-id", "left_camera_optical_frame",
        ],
        output="screen",
    )
    navigation = IncludeLaunchDescription(
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
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("octomap_path", default_value=""),
        DeclareLaunchArgument("cloud_path", default_value=""),
        DeclareLaunchArgument("hloc_map_directory", default_value=""),
        DeclareLaunchArgument("semantic_path", default_value=""),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/navigation/cmd_vel"),
        DeclareLaunchArgument("use_imu", default_value="true"),
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
        DeclareLaunchArgument("execution_provider", default_value="cuda"),
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_qx", default_value="-0.5"),
        DeclareLaunchArgument("camera_qy", default_value="0.5"),
        DeclareLaunchArgument("camera_qz", default_value="-0.5"),
        DeclareLaunchArgument("camera_qw", default_value="0.5"),
        sensor,
        adapter,
        mounting_tf,
        navigation,
    ])
