"""Launch the hardware-independent RTAB-Map RGB-D mapping pipeline.

Hardware is provided exclusively by ``luxi_adapter``. This launch consumes its
canonical RGB-D, filtered-IMU and TF interface without knowing a camera model.
"""

import os
import re
import signal
import subprocess
import time

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


TEST_PROCESS_MARKERS = (
    "d435i_rtabmap.launch.py",
    "rgbd_mapping_test.launch.py",
    "luxi_rtab_map_node",
    "d435i_rtabmap.rviz",
    "/rtabmap_launch/share/rtabmap_launch/launch/config/rgbd.rviz",
    "rgbd_mapping.rviz",
    "rtabmap_util/point_cloud_xyzrgb",
    "/rtabmap_sync/rgbd_sync",
    "/rtabmap_odom/rgbd_odometry",
    "/rtabmap_slam/rtabmap",
    "luxi_visual_frontend/visual_odometry_node",
)

MAPS_DIRECTORY = "/home/lunar/project/lunar_slam/maps/rtab_maps"


def _prepare_database_path(context: object) -> list[LogInfo]:
    """Use the next numbered map database unless the caller selected one."""
    configured_path = LaunchConfiguration("database_path").perform(context).strip()
    if configured_path:
        database_path = os.path.abspath(os.path.expanduser(configured_path))
        os.makedirs(os.path.dirname(database_path), exist_ok=True)
    else:
        os.makedirs(MAPS_DIRECTORY, exist_ok=True)
        existing_indices = []
        for filename in os.listdir(MAPS_DIRECTORY):
            match = re.fullmatch(r"map(\d+)\.db(?:-(?:shm|wal))?", filename)
            if match:
                existing_indices.append(int(match.group(1)))
        map_index = max(existing_indices, default=0) + 1
        database_path = os.path.join(MAPS_DIRECTORY, f"map{map_index:03d}.db")

    context.launch_configurations["database_path"] = database_path
    return [LogInfo(msg=f"RTAB-Map database: {database_path}")]


def _wait_for_camera_inputs(context: object) -> list[LogInfo]:
    enabled = LaunchConfiguration("wait_for_camera").perform(context).lower()
    if enabled not in {"1", "true", "yes", "on"}:
        return [LogInfo(msg="D435i RGB-D input check is disabled.")]

    timeout = float(LaunchConfiguration("camera_wait_timeout").perform(context))
    topics = (
        LaunchConfiguration("rgb_topic").perform(context),
        LaunchConfiguration("depth_topic").perform(context),
        LaunchConfiguration("camera_info_topic").perform(context),
    )
    if LaunchConfiguration("use_imu").perform(context).lower() in {"1", "true", "yes", "on"}:
        topics += (LaunchConfiguration("imu_topic").perform(context),)
    deadline = time.monotonic() + timeout

    for topic in topics:
        # ``ros2 topic echo`` exits immediately while a topic has no known
        # type. Retry until the shared deadline: a single call would make the
        # advertised 15-second startup grace period ineffective after a USB
        # reconnect or a slow camera driver startup.
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise RuntimeError(
                    f"No message received from {topic} within {timeout:.1f}s. "
                    "Check the active luxi_adapter profile and hardware driver."
                )
            try:
                result = subprocess.run(
                    [
                        "ros2",
                        "topic",
                        "echo",
                        "--once",
                        "--no-daemon",
                        "--field",
                        "header",
                        "--qos-profile",
                        "sensor_data",
                        topic,
                    ],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=min(2.0, remaining),
                    check=False,
                )
            except subprocess.TimeoutExpired:
                result = None
            if result is not None and result.returncode == 0:
                break
            time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))

    return [LogInfo(msg="Canonical RGB-D input topics are ready.")]


def _processes() -> dict[int, tuple[int, str]]:
    result = subprocess.run(
        ["ps", "-eo", "pid=,ppid=,args="],
        check=True,
        capture_output=True,
        text=True,
    )
    processes: dict[int, tuple[int, str]] = {}
    for line in result.stdout.splitlines():
        fields = line.strip().split(maxsplit=2)
        if len(fields) != 3:
            continue
        pid, parent_pid, command = fields
        processes[int(pid)] = (int(parent_pid), command)
    return processes


def _terminate_processes(_: object) -> list[LogInfo]:
    processes = _processes()
    target_pids = {
        pid
        for pid, (_, command) in processes.items()
        if any(marker in command for marker in TEST_PROCESS_MARKERS)
    }
    if not target_pids:
        return [LogInfo(msg="No residual RGB-D mapping test process found.")]

    changed = True
    while changed:
        changed = False
        for pid, (parent_pid, _) in processes.items():
            if parent_pid in target_pids and pid not in target_pids:
                target_pids.add(pid)
                changed = True

    for pid in sorted(target_pids, reverse=True):
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        alive = []
        for pid in target_pids:
            try:
                os.kill(pid, 0)
                alive.append(pid)
            except ProcessLookupError:
                pass
        if not alive:
            return [LogInfo(msg=f"Stopped residual mapping test processes: {sorted(target_pids)}")]
        time.sleep(0.1)

    for pid in alive:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    return [LogInfo(msg=f"Stopped residual mapping test processes: {sorted(target_pids)}")]


def generate_launch_description() -> LaunchDescription:
    rtabmap_launch = PathJoinSubstitution(
        [FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py"]
    )
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

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments={
            "stereo": "false",
            "depth": "false",
            "localization": LaunchConfiguration("localization"),
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "rviz": "false",
            "frame_id": LaunchConfiguration("base_frame"),
            "map_frame_id": LaunchConfiguration("map_frame"),
            "odom_frame_id": "",
            "database_path": LaunchConfiguration("database_path"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "rgbd_sync": LaunchConfiguration("rgbd_sync"),
            "subscribe_rgbd": LaunchConfiguration("subscribe_rgbd"),
            "rgbd_topic": LaunchConfiguration("rgbd_topic"),
            "approx_rgbd_sync": "true",
            "approx_sync": "true",
            "approx_sync_max_interval": LaunchConfiguration("approx_sync_max_interval"),
            "visual_odometry": LaunchConfiguration("visual_odometry"),
            "icp_odometry": LaunchConfiguration("icp_odometry"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "qos": LaunchConfiguration("qos"),
            "qos_image": LaunchConfiguration("qos"),
            "qos_camera_info": LaunchConfiguration("qos"),
            "wait_for_transform": LaunchConfiguration("wait_for_transform"),
            "wait_imu_to_init": LaunchConfiguration("use_imu"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "rtabmap_args": effective_rtabmap_args,
            "odom_args": LaunchConfiguration("odom_args"),
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="luxi_mapping_rviz",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    learned_frontend = Node(
        package="luxi_visual_frontend",
        executable="visual_odometry_node",
        name="luxi_visual_frontend",
        output="screen",
        parameters=[LaunchConfiguration("visual_frontend_config")],
        condition=IfCondition(LaunchConfiguration("learned_frontend")),
    )

    # The adapter owns sensor TF and is started before this algorithm launch.
    # Scope upstream arguments so its internal ``rviz=false`` does not overwrite
    # this launch file's RViz option.
    delayed_rtabmap = TimerAction(
        period=0.5,
        actions=[GroupAction(actions=[rtabmap], scoped=True)],
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
            DeclareLaunchArgument("visual_odometry", default_value="true"),
            DeclareLaunchArgument("icp_odometry", default_value="false"),
            DeclareLaunchArgument("odom_topic", default_value="odom"),
            DeclareLaunchArgument("rgbd_sync", default_value="true"),
            DeclareLaunchArgument("subscribe_rgbd", default_value="false"),
            DeclareLaunchArgument("rgbd_topic", default_value="rgbd_image"),
            DeclareLaunchArgument("qos", default_value="2"),
            DeclareLaunchArgument("approx_sync_max_interval", default_value="0.05"),
            DeclareLaunchArgument("wait_for_transform", default_value="0.5"),
            DeclareLaunchArgument("use_imu", default_value="true"),
            DeclareLaunchArgument("imu_topic", default_value="/sensors/imu/data"),
            DeclareLaunchArgument("wait_for_camera", default_value="true"),
            DeclareLaunchArgument(
                "camera_wait_timeout",
                default_value="60.0",
                description="Maximum time to wait for canonical sensor messages after hardware startup or reconnect.",
            ),
            DeclareLaunchArgument("new_map", default_value="false"),
            DeclareLaunchArgument("load_saved_map", default_value="true"),
            DeclareLaunchArgument(
                "rtabmap_args",
                default_value=(
                    "--Rtabmap/DetectionRate 1 "
                    "--Rtabmap/LoopThr 0.2 "
                    "--RGBD/LinearUpdate 0.1 "
                    "--RGBD/AngularUpdate 0.1 "
                    "--Kp/MinDepth 0.2 "
                    "--Kp/MaxDepth 4.5"
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
            OpaqueFunction(function=_terminate_processes),
            OpaqueFunction(function=_prepare_database_path),
            OpaqueFunction(function=_wait_for_camera_inputs),
            learned_frontend,
            delayed_rtabmap,
            delayed_rviz,
            publish_map,
        ]
    )
