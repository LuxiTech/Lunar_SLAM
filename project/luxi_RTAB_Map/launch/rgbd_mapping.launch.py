"""Launch the hardware-independent RTAB-Map RGB-D mapping pipeline.

Hardware is provided exclusively by ``luxi_adapter``. This launch consumes its
canonical RGB-D, filtered-IMU and TF interface without knowing a camera model.
"""

import os
import re
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _workspace_root() -> Path:
    configured = os.environ.get("LUXI_WORKSPACE_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for candidate in (start, *start.parents):
            if (candidate / "project").is_dir() and (candidate / "maps").is_dir():
                return candidate
    raise RuntimeError("Cannot locate lunar_slam; set LUXI_WORKSPACE_ROOT")


MAPS_DIRECTORY = str(_workspace_root() / "maps" / "rtab_maps")


def _prepare_database_path(context: object) -> list[LogInfo]:
    """Select a database without silently deleting or racing another launch."""
    configured_path = LaunchConfiguration("database_path").perform(context).strip()
    new_map = LaunchConfiguration("new_map").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    allow_overwrite = LaunchConfiguration("overwrite_existing_database").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    if configured_path:
        database_path = os.path.abspath(os.path.expanduser(configured_path))
        os.makedirs(os.path.dirname(database_path), exist_ok=True)
        if new_map and os.path.exists(database_path) and not allow_overwrite:
            raise RuntimeError(
                f"Refusing to delete existing RTAB-Map database: {database_path}. "
                "Use new_map:=false to continue it, choose a new path, or explicitly set "
                "overwrite_existing_database:=true."
            )
    else:
        os.makedirs(MAPS_DIRECTORY, exist_ok=True)
        existing_indices = [
            int(match.group(1))
            for filename in os.listdir(MAPS_DIRECTORY)
            if (match := re.fullmatch(r"map(\d+)\.db(?:-(?:shm|wal))?", filename))
        ]
        map_index = max(existing_indices, default=0) + 1
        while True:
            database_path = os.path.join(MAPS_DIRECTORY, f"map{map_index:03d}.db")
            try:
                descriptor = os.open(database_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o664)
                os.close(descriptor)
                break
            except FileExistsError:
                map_index += 1

    context.launch_configurations["database_path"] = database_path
    return [LogInfo(msg=f"RTAB-Map database: {database_path}")]


def _wait_for_camera_inputs(context: object) -> list[LogInfo]:
    enabled = LaunchConfiguration("wait_for_camera").perform(context).lower()
    if enabled not in {"1", "true", "yes", "on"}:
        return [LogInfo(msg="Synchronized sensor input check is disabled.")]

    timeout = float(LaunchConfiguration("camera_wait_timeout").perform(context))
    require_imu = LaunchConfiguration("use_imu").perform(context).lower() in {
        "1", "true", "yes", "on"
    }
    command = [
        "ros2", "run", "luxi_rtab_map", "sensor_sync_check", "--ros-args",
        "-p", f"rgbd_topic:={LaunchConfiguration('sensor_rgbd_topic').perform(context)}",
        "-p", f"imu_topic:={LaunchConfiguration('imu_topic').perform(context)}",
        "-p", f"require_imu:={'true' if require_imu else 'false'}",
        "-p", f"timeout_sec:={timeout}",
    ]
    try:
        result = subprocess.run(command, timeout=timeout + 5.0, check=False)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError(
            f"Sensor synchronization check exceeded {timeout:.1f}s."
        ) from error
    if result.returncode != 0:
        raise RuntimeError(
            "RGB-D/IMU synchronization check failed. "
            "RTAB-Map was not started; inspect the active luxi_adapter profile and trigger source."
        )
    return [LogInfo(msg="Canonical RGB-D/IMU synchronization check passed.")]


def generate_launch_description() -> LaunchDescription:
    mvs_root = os.path.abspath(os.environ.get("MVS_ROOT", "/opt/MVS"))
    mvs_library_dirs = {
        os.path.join(mvs_root, "lib", "aarch64"),
        os.path.join(mvs_root, "lib", "64"),
    }
    rtabmap_environment = {
        "LD_LIBRARY_PATH": os.pathsep.join(
            entry
            for entry in os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
            if entry and os.path.abspath(entry) not in mvs_library_dirs
        )
    }
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("luxi_rtab_map"), "rviz", "rgbd_mapping.rviz"]
    )
    effective_rtabmap_args = PythonExpression(
        [
            "'",
            LaunchConfiguration("rtabmap_args"),
            "' + (' --delete_db_on_start' if '",
            LaunchConfiguration("new_map"),
            "'.lower() in ('true', '1', 'yes', 'on') else '')",
        ]
    )

    rgbd_sync = Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="rgbd_sync",
        namespace="rtabmap",
        output="screen",
        additional_env=rtabmap_environment,
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("rgbd_sync"),
            "'.lower() in ('true', '1', 'yes', 'on') and '",
            LaunchConfiguration("rgbd_sync_component"),
            "'.lower() not in ('true', '1', 'yes', 'on')",
        ])),
        parameters=[{
            "approx_sync": ParameterValue(
                LaunchConfiguration("approx_sync"), value_type=bool),
            "approx_sync_max_interval": LaunchConfiguration("approx_sync_max_interval"),
            "topic_queue_size": LaunchConfiguration("topic_queue_size"),
            "sync_queue_size": LaunchConfiguration("sync_queue_size"),
            "qos": LaunchConfiguration("qos"),
            "qos_camera_info": LaunchConfiguration("qos"),
        }],
        remappings=[
            ("rgb/image", LaunchConfiguration("rgb_topic")),
            ("depth/image", LaunchConfiguration("depth_topic")),
            ("rgb/camera_info", LaunchConfiguration("camera_info_topic")),
            ("rgbd_image", LaunchConfiguration("rgbd_topic")),
        ],
    )
    rgbd_sync_component = LoadComposableNodes(
        target_container=LaunchConfiguration("rgbd_sync_container"),
        condition=IfCondition(LaunchConfiguration("rgbd_sync_component")),
        composable_node_descriptions=[ComposableNode(
            package="rtabmap_sync",
            plugin="rtabmap_sync::RGBDSync",
            name="rgbd_sync",
            namespace="rtabmap",
            parameters=[{
                "approx_sync": ParameterValue(
                    LaunchConfiguration("approx_sync"), value_type=bool),
                "approx_sync_max_interval": LaunchConfiguration("approx_sync_max_interval"),
                "topic_queue_size": LaunchConfiguration("topic_queue_size"),
                "sync_queue_size": LaunchConfiguration("sync_queue_size"),
                "qos": LaunchConfiguration("qos"),
                "qos_camera_info": LaunchConfiguration("qos"),
            }],
            remappings=[
                ("rgb/image", LaunchConfiguration("rgb_topic")),
                ("depth/image", LaunchConfiguration("depth_topic")),
                ("rgb/camera_info", LaunchConfiguration("camera_info_topic")),
                ("rgbd_image", LaunchConfiguration("rgbd_topic")),
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )],
    )
    rgbd_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        namespace="rtabmap",
        output="screen",
        additional_env=rtabmap_environment,
        condition=IfCondition(LaunchConfiguration("visual_odometry")),
        parameters=[{
            "frame_id": LaunchConfiguration("base_frame"),
            "odom_frame_id": "odom",
            "publish_tf": ParameterValue(
                LaunchConfiguration("publish_odom_tf"), value_type=bool),
            "wait_for_transform": LaunchConfiguration("wait_for_transform"),
            "wait_imu_to_init": ParameterValue(
                LaunchConfiguration("use_imu"), value_type=bool),
            "always_check_imu_tf": False,
            "approx_sync": ParameterValue(
                LaunchConfiguration("approx_sync"), value_type=bool),
            "approx_sync_max_interval": LaunchConfiguration("approx_sync_max_interval"),
            "topic_queue_size": LaunchConfiguration("topic_queue_size"),
            "sync_queue_size": LaunchConfiguration("sync_queue_size"),
            "qos": LaunchConfiguration("qos"),
            "qos_camera_info": LaunchConfiguration("qos"),
            "qos_imu": LaunchConfiguration("qos_imu"),
            "subscribe_rgbd": True,
            "always_process_most_recent_frame": True,
        }],
        remappings=[
            ("rgbd_image", LaunchConfiguration("rgbd_topic")),
            ("odom", LaunchConfiguration("odom_topic")),
            ("imu", LaunchConfiguration("imu_topic")),
        ],
        arguments=[
            LaunchConfiguration("odom_args"), "--ros-args", "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )
    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        namespace="rtabmap",
        output="screen",
        on_exit=Shutdown(reason="RTAB-Map process exited"),
        additional_env=rtabmap_environment,
        parameters=[{
            "subscribe_depth": False,
            "subscribe_rgbd": True,
            "subscribe_rgb": False,
            "subscribe_stereo": False,
            "subscribe_odom_info": ParameterValue(
                LaunchConfiguration("subscribe_odom_info"), value_type=bool),
            "frame_id": LaunchConfiguration("base_frame"),
            "map_frame_id": LaunchConfiguration("map_frame"),
            "odom_frame_id": "",
            "publish_tf": True,
            "wait_for_transform": LaunchConfiguration("wait_for_transform"),
            "database_path": LaunchConfiguration("database_path"),
            "approx_sync": ParameterValue(
                LaunchConfiguration("approx_sync"), value_type=bool),
            "approx_sync_max_interval": LaunchConfiguration("approx_sync_max_interval"),
            "topic_queue_size": LaunchConfiguration("topic_queue_size"),
            "sync_queue_size": LaunchConfiguration("sync_queue_size"),
            "qos_image": LaunchConfiguration("qos"),
            "qos_camera_info": LaunchConfiguration("qos"),
            "qos_odom": LaunchConfiguration("qos_odom"),
            "qos_imu": LaunchConfiguration("qos_imu"),
            "map_always_update": ParameterValue(
                LaunchConfiguration("map_always_update"), value_type=bool),
            "cloud_output_voxelized": ParameterValue(
                LaunchConfiguration("cloud_output_voxelized"), value_type=bool),
            "Mem/IncrementalMemory": ParameterValue(
                PythonExpression([
                    "'false' if '", LaunchConfiguration("localization"),
                    "'.lower() in ('true', '1', 'yes', 'on') else 'true'",
                ]),
                value_type=str,
            ),
            "Mem/InitWMWithAllNodes": ParameterValue(
                PythonExpression([
                    "'true' if '", LaunchConfiguration("localization"),
                    "'.lower() in ('true', '1', 'yes', 'on') else 'false'",
                ]),
                value_type=str,
            ),
        }],
        remappings=[
            ("rgbd_image", LaunchConfiguration("rgbd_topic")),
            ("odom", LaunchConfiguration("odom_topic")),
            ("imu", LaunchConfiguration("imu_topic")),
        ],
        arguments=[
            effective_rtabmap_args, "--ros-args", "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    rtabmap_viz = Node(
        package="rtabmap_viz",
        executable="rtabmap_viz",
        name="rtabmap_viz",
        namespace="rtabmap",
        output="screen",
        additional_env=rtabmap_environment,
        condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
        parameters=[{
            "subscribe_rgbd": True,
            "subscribe_odom_info": ParameterValue(
                LaunchConfiguration("visual_odometry"), value_type=bool),
            "frame_id": LaunchConfiguration("base_frame"),
            "wait_for_transform": LaunchConfiguration("wait_for_transform"),
            "qos_image": LaunchConfiguration("qos"),
            "qos_odom": LaunchConfiguration("qos"),
        }],
        remappings=[
            ("rgbd_image", LaunchConfiguration("rgbd_topic")),
            ("odom", LaunchConfiguration("odom_topic")),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="luxi_mapping_rviz",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
        additional_env=rtabmap_environment,
    )

    learned_frontend = Node(
        package="luxi_visual_frontend",
        executable="visual_odometry_node",
        name="luxi_visual_frontend",
        output="screen",
        parameters=[
            LaunchConfiguration("visual_frontend_config"),
            {
                "minimum_depth": ParameterValue(
                    LaunchConfiguration("visual_frontend_minimum_depth"),
                    value_type=float,
                ),
                "maximum_depth": ParameterValue(
                    LaunchConfiguration("visual_frontend_maximum_depth"),
                    value_type=float,
                ),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "use_imu_rotation": ParameterValue(
                    LaunchConfiguration("use_imu"), value_type=bool),
                "camera_to_imu_time_offset": ParameterValue(
                    LaunchConfiguration("visual_frontend_camera_to_imu_time_offset"),
                    value_type=float,
                ),
            },
        ],
        condition=IfCondition(LaunchConfiguration("learned_frontend")),
    )

    delayed_rtabmap = TimerAction(
        period=LaunchConfiguration("rtabmap_start_delay"),
        actions=[rgbd_sync, rgbd_sync_component, rgbd_odometry, rtabmap, rtabmap_viz],
    )
    delayed_rviz = TimerAction(period=2.0, actions=[rviz])
    publish_map = TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/rtabmap/rtabmap/publish_map",
                    "rtabmap_msgs/srv/PublishMap",
                    "{global_map: false, optimized: true, graph_only: false}",
                ],
                condition=IfCondition(LaunchConfiguration("load_saved_map")),
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="false"),
            DeclareLaunchArgument(
                "localization",
                default_value="false",
                description="Use an existing RTAB-Map database for localization without extending it.",
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value="",
                description="Existing database to continue; empty creates the next rtab_maps/mapNNN.db.",
            ),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("rgb_topic", default_value="/sensors/rgbd/color/image_raw"),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/sensors/rgbd/depth/image_raw",
            ),
            DeclareLaunchArgument("camera_info_topic", default_value="/sensors/rgbd/color/camera_info"),
            DeclareLaunchArgument(
                "sensor_rgbd_topic", default_value="/sensors/rgbd/rgbd_image"),
            DeclareLaunchArgument("learned_frontend", default_value="false"),
            DeclareLaunchArgument(
                "visual_frontend_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("luxi_visual_frontend"),
                        "config",
                        "visual_odometry.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "visual_frontend_minimum_depth", default_value="0.2"),
            DeclareLaunchArgument(
                "visual_frontend_maximum_depth", default_value="6.0"),
            DeclareLaunchArgument(
                "visual_frontend_camera_to_imu_time_offset", default_value="0.0"),
            DeclareLaunchArgument("visual_odometry", default_value="true"),
            DeclareLaunchArgument("icp_odometry", default_value="false"),
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument("rgbd_sync", default_value="true"),
            DeclareLaunchArgument("rgbd_sync_component", default_value="false"),
            DeclareLaunchArgument("rgbd_sync_container", default_value="/luxi_sensor_container"),
            DeclareLaunchArgument("subscribe_rgbd", default_value="false"),
            DeclareLaunchArgument("rgbd_topic", default_value="rgbd_image"),
            DeclareLaunchArgument("qos", default_value="2"),
            DeclareLaunchArgument("qos_imu", default_value="2"),
            DeclareLaunchArgument("qos_odom", default_value="2"),
            DeclareLaunchArgument("approx_sync", default_value="true"),
            DeclareLaunchArgument("topic_queue_size", default_value="20"),
            DeclareLaunchArgument("sync_queue_size", default_value="10"),
            DeclareLaunchArgument("approx_sync_max_interval", default_value="0.05"),
            DeclareLaunchArgument("wait_for_transform", default_value="0.5"),
            DeclareLaunchArgument(
                "rtabmap_start_delay",
                default_value="0.5",
                description="Delay RTAB startup while an optional frontend initializes.",
            ),
            DeclareLaunchArgument("publish_odom_tf", default_value="true"),
            DeclareLaunchArgument("subscribe_odom_info", default_value="true"),
            DeclareLaunchArgument("map_always_update", default_value="false"),
            DeclareLaunchArgument("cloud_output_voxelized", default_value="true"),
            DeclareLaunchArgument("use_imu", default_value="true"),
            DeclareLaunchArgument("imu_topic", default_value="/sensors/imu/data"),
            DeclareLaunchArgument("wait_for_camera", default_value="true"),
            DeclareLaunchArgument(
                "camera_wait_timeout",
                default_value="60.0",
                description="Maximum time to wait for canonical sensor messages after hardware startup or reconnect.",
            ),
            DeclareLaunchArgument("new_map", default_value="false"),
            DeclareLaunchArgument("overwrite_existing_database", default_value="false"),
            DeclareLaunchArgument("load_saved_map", default_value="true"),
            DeclareLaunchArgument(
                "rtabmap_args",
                default_value=(
                    "--Rtabmap/DetectionRate 1 "
                    "--Rtabmap/LoopThr 0.2 "
                    "--RGBD/LinearUpdate 0.1 "
                    "--RGBD/AngularUpdate 0.1 "
                    "--Kp/MinDepth 0.2 "
                    "--Kp/MaxDepth 4.5 "
                    "--Grid/DepthDecimation 2"
                ),
            ),
            DeclareLaunchArgument(
                "odom_args",
                default_value=(
                    "--Odom/ResetCountdown 0 "
                    "--Odom/GuessMotion false "
                    "--Odom/FilteringStrategy 1 "
                    "--Odom/KalmanProcessNoise 0.001 "
                    "--Odom/KalmanMeasurementNoise 0.05 "
                    "--Odom/ImageDecimation 2 "
                    "--Odom/KeyFrameThr 0.5 "
                    "--Odom/VisKeyFrameThr 200 "
                    "--OdomF2M/MaxSize 1500 "
                    "--OdomF2M/BundleAdjustment 0 "
                    "--Vis/MinInliers 10 "
                    "--Vis/CorGuessWinSize 120 "
                    "--Vis/FeatureType 2 "
                    "--Vis/MaxFeatures 1000 "
                    "--Vis/MinDepth 0.2 "
                    "--Vis/MaxDepth 4.5"
                ),
            ),
            DeclareLaunchArgument("log_level", default_value="warn"),
            OpaqueFunction(function=_prepare_database_path),
            OpaqueFunction(function=_wait_for_camera_inputs),
            learned_frontend,
            delayed_rtabmap,
            delayed_rviz,
            publish_map,
        ]
    )
