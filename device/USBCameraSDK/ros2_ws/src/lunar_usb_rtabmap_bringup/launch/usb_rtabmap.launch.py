"""Launch one complete USB stereo RTAB-Map profile.

This device-owned entry mirrors ``lunar_d435i_rtabmap_bringup`` while keeping
the sensor adapter, visual frontend and RTAB-Map implementation reusable.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


STABLE_ODOM_ARGS = (
    "--Odom/ResetCountdown 3 "
    "--Odom/GuessMotion false "
    "--Odom/FilteringStrategy 1 "
    "--Odom/KalmanProcessNoise 0.001 "
    "--Odom/KalmanMeasurementNoise 0.03 "
    "--Odom/ImageDecimation 1 "
    "--Odom/KeyFrameThr 0.50 "
    "--Odom/VisKeyFrameThr 100 "
    "--OdomF2M/MaxSize 700 "
    "--OdomF2M/BundleAdjustment 1 "
    "--OdomF2M/BundleAdjustmentMaxFrames 5 "
    "--OdomF2M/BundleAdjustmentMaxKeyFramesPerFeature 3 "
    "--OdomF2M/BundleAdjustmentMinMotion 1.5 "
    "--Vis/DepthAsMask true "
    "--Vis/MinInliers 20 "
    "--Vis/MinInliersDistribution 0.005 "
    "--Vis/Iterations 300 "
    "--Vis/PnPReprojError 1.5 "
    "--Vis/CorGuessWinSize 120 "
    "--Vis/FeatureType 8 "
    "--Vis/MaxFeatures 800 "
    "--Vis/GridRows 2 "
    "--Vis/GridCols 3 "
    "--GFTT/MinDistance 6 "
    "--ORB/ScaleFactor 1.2 "
    "--ORB/NLevels 4 "
    "--Vis/MinDepth 0.4 "
    "--Vis/MaxDepth 3.0"
)

STABLE_RTABMAP_ARGS = (
    "--Rtabmap/DetectionRate 1.0 "
    "--Rtabmap/LoopThr 0.2 "
    "--RGBD/OptimizeMaxError 4.0 "
    "--RGBD/LinearUpdate 0.12 "
    "--RGBD/AngularUpdate 0.12 "
    "--Kp/MinDepth 0.4 "
    "--Kp/MaxDepth 3.0 "
    "--Grid/RangeMin 0.4 "
    "--Grid/RangeMax 3.0 "
    "--Grid/DepthDecimation 2 "
    "--Grid/PreVoxelFiltering true "
    "--Grid/NoiseFilteringRadius 0.12 "
    "--Grid/NoiseFilteringMinNeighbors 8"
)

LEARNED_RTABMAP_ARGS = (
    "--Rtabmap/DetectionRate 1.0 "
    "--Rtabmap/LoopThr 0.2 "
    "--RGBD/OptimizeMaxError 3.0 "
    # Reject local-space hypotheses whose view direction is too different.
    # Replaying the hand-held benchmark at 24 degrees reduced graph roll/pitch
    # closure from 5.63 to 0.66 degrees while keeping 18.9 mm translation.
    "--RGBD/ProximityAngle 24.0 "
    "--RGBD/LinearUpdate 0.12 "
    "--RGBD/AngularUpdate 0.12 "
    "--Kp/DetectorStrategy 1 "
    "--Kp/NNStrategy 1 "
    "--Kp/MaxFeatures 2048 "
    "--Kp/MinDepth 0.4 "
    "--Kp/MaxDepth 3.0 "
    "--Vis/FeatureType 1 "
    "--Vis/MaxFeatures 2048 "
    "--Vis/MinInliers 20 "
    "--Mem/UseOdomFeatures true "
    "--Grid/RangeMin 0.4 "
    "--Grid/RangeMax 3.0 "
    "--Grid/DepthDecimation 2 "
    "--Grid/PreVoxelFiltering true "
    "--Grid/NoiseFilteringRadius 0.12 "
    "--Grid/NoiseFilteringMinNeighbors 8"
)

def _sensor_actions(mode):
    # Both USB profiles share the same calibrated H30 input. The driver marks
    # unhealthy gyro/orientation packets unavailable, and the Luxi frontend
    # ignores those samples rather than applying a corrupt rotation prior.
    use_imu_argument = LaunchConfiguration("use_imu")
    enable_imu_parameter = ParameterValue(use_imu_argument, value_type=bool)
    depth_backend = (
        "opencv_cuda_sgm" if mode == "stable" else "vpi_ofa_pva_vic"
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("usb_camera_bringup"),
                "launch",
                "stereo_rgbd.launch.py",
            ])
        ),
        launch_arguments={
            "start_imu": use_imu_argument,
            "depth_backend": depth_backend,
        }.items(),
    )
    adapter_config = PathJoinSubstitution([
        FindPackageShare("lunar_usb_rtabmap_bringup"),
        "config",
        "usb_adapter.yaml",
    ])
    adapter = Node(
        package="luxi_adapter",
        executable="sensor_adapter_node",
        name="luxi_adapter",
        parameters=[
            adapter_config,
            {
                "enable_imu": enable_imu_parameter,
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


def _mapping_arguments(mode):
    arguments = {
        "rviz": LaunchConfiguration("rviz"),
        "rviz_display": LaunchConfiguration("rviz_display"),
        "rviz_rgb_topic": "/usb_stereo/left/image_preview",
        "rviz_depth_topic": "/usb_stereo/depth_preview",
        "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
        "database_path": LaunchConfiguration("database_path"),
        "new_map": LaunchConfiguration("new_map"),
        "overwrite_existing_database": LaunchConfiguration(
            "overwrite_existing_database"
        ),
        "load_saved_map": "false",
        "wait_for_camera": LaunchConfiguration("wait_for_camera"),
        "camera_wait_timeout": LaunchConfiguration("camera_wait_timeout"),
        "use_imu": LaunchConfiguration("use_imu"),
        "force_3dof": LaunchConfiguration("planar_mode"),
        "rgbd_sync": "false",
        "approx_sync": "false",
        "always_process_most_recent_frame": "true",
        "gen_depth_decimation": "2",
        "map_filter_radius": "0.15",
        "map_filter_angle": "12.0",
        "cloud_subtract_filtering": "true",
        "cloud_subtract_filtering_min_neighbors": "3",
    }
    if mode == "stable":
        arguments.update({
            "rgbd_topic": "/sensors/rgbd/rgbd_image",
            "odom_topic": "/rtabmap/odom",
            "odom_output_topic": "/rtabmap/odom_raw",
            "publish_odom_tf": "false",
            "odom_args": STABLE_ODOM_ARGS,
            "rtabmap_args": STABLE_RTABMAP_ARGS,
        })
    else:
        arguments.update({
            "learned_frontend": "true",
            # Luxi supplies learned local features; RTAB-Map F2M owns the
            # metric odometry so motion is optimized over a five-frame local
            # map instead of accumulating independent two-frame PnP poses.
            "visual_odometry": "true",
            "icp_odometry": "false",
            "odom_topic": "/rtabmap/odom",
            "odom_output_topic": "/rtabmap/odom",
            "subscribe_odom_info": "true",
            "publish_odom_tf": "true",
            "subscribe_rgbd": "true",
            "sensor_rgbd_topic": "/sensors/rgbd/rgbd_image",
            "rgbd_topic": "/luxi_visual_frontend/rgbd_image",
            "odom_rgbd_topic": "/sensors/rgbd/rgbd_image",
            "rtabmap_start_delay": "3.0",
            "visual_frontend_minimum_depth": "0.4",
            "visual_frontend_maximum_depth": "3.0",
            "visual_frontend_depth_sampling_radius": "2",
            "visual_frontend_depth_sampling_minimum_valid": "5",
            "visual_frontend_use_depth_translation_refinement": "true",
            "visual_frontend_rgbd_features_rate": "1.0",
            "visual_frontend_publish_tf": "false",
            "visual_frontend_camera_to_imu_time_offset": "0.0",
            "visual_frontend_superpoint_cuda_graph": "false",
            "odom_args": STABLE_ODOM_ARGS,
            "rtabmap_args": LEARNED_RTABMAP_ARGS,
        })
    return arguments


def _mapping(mode):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("luxi_rtab_map"),
                    "launch",
                    "rgbd_mapping.launch.py",
                ]
            )
        ),
        launch_arguments=_mapping_arguments(mode).items(),
    )


def _cloud_preview(mode):
    return Node(
        package="luxi_rtab_map",
        executable="odom_cloud_stabilizer",
        name=(
            "usb_odom_cloud_stabilizer"
            if mode == "stable"
            else "usb_learned_cloud_stabilizer"
        ),
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{
            "cloud_input": "/usb_stereo/points",
            "odom_input": (
                "/rtabmap/odom"
                if mode == "stable"
                else "/rtabmap/odom"
            ),
            "cloud_output": "/rtabmap/usb_points_stable",
            "map_cloud_input": "/rtabmap/cloud_map",
            "map_cloud_output": "/rtabmap/cloud_map_visual",
            "occupancy_grid_input": "/rtabmap/map",
            "occupancy_grid_output": "/rtabmap/map_visual",
            "base_frame": "base_link",
            "history_size": 1,
            "sync_queue_size": 20,
        }],
    )


def _planar_odometry():
    return Node(
        package="luxi_rtab_map",
        executable="planar_odometry",
        name="usb_planar_odometry",
        output="screen",
        parameters=[{
            "input_topic": "/rtabmap/odom_raw",
            "output_topic": "/rtabmap/odom",
            "publish_tf": True,
            "constrain_to_planar": ParameterValue(
                LaunchConfiguration("planar_mode"), value_type=bool
            ),
        }],
    )


def _launch_profile(context):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in {"stable", "vpi_learned"}:
        raise RuntimeError(
            f"Unsupported USB mapping mode '{mode}'; "
            "use stable or vpi_learned."
        )
    delayed_actions = [_mapping(mode), _cloud_preview(mode)]
    if mode == "stable":
        delayed_actions.insert(0, _planar_odometry())
    return [
        *_sensor_actions(mode),
        TimerAction(period=3.0, actions=delayed_actions),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="vpi_learned",
            description="USB mapping profile: stable or vpi_learned.",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_display", default_value=""),
        DeclareLaunchArgument("rtabmap_viz", default_value="false"),
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("new_map", default_value="true"),
        DeclareLaunchArgument(
            "overwrite_existing_database", default_value="false"
        ),
        DeclareLaunchArgument("camera_wait_timeout", default_value="60.0"),
        DeclareLaunchArgument("wait_for_camera", default_value="true"),
        DeclareLaunchArgument(
            "use_imu",
            default_value="true",
            description=(
                "Enable the calibrated H30 by default; the startup health "
                "gate refuses to start odometry if any IMU axis is invalid."
            ),
        ),
        DeclareLaunchArgument(
            "planar_mode",
            default_value="false",
            description=(
                "Enable x/y/yaw constraints only for a level ground robot."
            ),
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "camera_frame", default_value="left_camera_optical_frame"
        ),
        DeclareLaunchArgument("camera_x", default_value="0.0"),
        DeclareLaunchArgument("camera_y", default_value="0.0"),
        DeclareLaunchArgument("camera_z", default_value="0.0"),
        DeclareLaunchArgument("camera_qx", default_value="-0.5"),
        DeclareLaunchArgument("camera_qy", default_value="0.5"),
        DeclareLaunchArgument("camera_qz", default_value="-0.5"),
        DeclareLaunchArgument("camera_qw", default_value="0.5"),
        OpaqueFunction(function=_launch_profile),
    ])
