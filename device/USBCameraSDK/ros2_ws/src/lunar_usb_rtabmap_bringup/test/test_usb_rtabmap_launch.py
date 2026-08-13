"""Static contract tests for the device-owned USB mapping entry."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "usb_rtabmap.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location(
        "usb_rtabmap_launch", LAUNCH_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_one_entry_owns_the_single_vpi_luxi_profile():
    text = LAUNCH_PATH.read_text(encoding="utf-8")
    bringup_text = (
        PACKAGE_ROOT.parent / "usb_camera_bringup" / "launch" /
        "stereo_rgbd.launch.py"
    ).read_text(encoding="utf-8")

    assert '"use_imu",\n            default_value="true"' in text
    assert 'DeclareLaunchArgument("mode"' not in text
    assert '"learned_frontend": "true"' in text
    assert 'DeclareLaunchArgument(\n            "depth_backend"' not in bringup_text
    assert '"depth_backend":' not in bringup_text
    depth_source = (
        PACKAGE_ROOT.parent / "usb_camera_driver" / "src" /
        "stereo_depth_node.cpp"
    ).read_text(encoding="utf-8")
    assert "depth_backend" not in depth_source
    assert "StereoSGBM" not in depth_source
    assert "cv::cuda" not in depth_source


def test_camera_driver_auto_discovers_replugged_usb_pair():
    driver_root = PACKAGE_ROOT.parent / "usb_camera_driver"
    source = (driver_root / "src" / "stereo_node.cpp").read_text(
        encoding="utf-8"
    )
    config = yaml.safe_load(
        (driver_root / "config" / "stereo_camera.yaml").read_text(
            encoding="utf-8"
        )
    )["stereo_node"]["ros__parameters"]

    assert config["left_device"] == "auto"
    assert config["right_device"] == "auto"
    assert config["auto_discover_devices"] is True
    assert config["auto_exposure"] is False
    assert config["exposure_absolute"] == 100
    assert config["gain"] == 50
    assert 'kCameraModalias[] = "usb:v32E4p2234"' in source
    assert 'readTextFile(entry.path() / "index") != "0"' in source


def test_camera_exit_stops_mapping_chain_for_web_status():
    bringup_launch = (
        PACKAGE_ROOT.parent / "usb_camera_bringup" / "launch" /
        "stereo_rgbd.launch.py"
    ).read_text(encoding="utf-8")

    assert "on_exit=[" in bringup_launch
    assert 'Shutdown(reason="USB stereo capture exited")' in bringup_launch


def test_single_profile_keeps_validated_odometry_postprocessor():
    module = _load_launch_module()
    arguments = module._mapping_arguments()

    assert arguments["odom_output_topic"] == "/rtabmap/odom_raw"
    assert arguments["publish_odom_tf"] == "false"
    assert "--OdomF2M/BundleAdjustmentMaxFrames 5" in arguments["odom_args"]
    assert "--Vis/FeatureType 8" in arguments["odom_args"]
    assert "--Vis/MinInliersDistribution 0.003" in arguments["odom_args"]
    assert "--Vis/MaxDepth 3.0" in arguments["odom_args"]
    assert "--Grid/RangeMax 3.0" in arguments["rtabmap_args"]
    assert "--Grid/NoiseFilteringRadius 0.12" in arguments["rtabmap_args"]


def test_mapping_and_odometry_return_to_validated_near_depth():
    module = _load_launch_module()

    arguments = module._mapping_arguments()

    assert "--Grid/RangeMax 3.0" in arguments["rtabmap_args"]
    assert "--Vis/MaxDepth 3.0" in arguments["odom_args"]
    assert "--Kp/MaxDepth 3.0" in arguments["rtabmap_args"]
    assert arguments["visual_frontend_maximum_depth"] == "3.0"


def test_vpi_luxi_chain_uses_pre_extension_f2m_odometry():
    module = _load_launch_module()
    arguments = module._mapping_arguments()

    assert arguments["learned_frontend"] == "true"
    assert arguments["visual_odometry"] == "true"
    assert arguments["odom_topic"] == "/rtabmap/odom"
    assert arguments["odom_rgbd_topic"] == "/sensors/rgbd/rgbd_image"
    assert arguments["subscribe_odom_info"] == "true"
    assert arguments["odom_output_topic"] == "/rtabmap/odom_raw"
    assert arguments["publish_odom_tf"] == "false"
    assert arguments["visual_frontend_publish_tf"] == "false"
    assert arguments["visual_frontend_rgbd_features_rate"] == "1.0"
    assert arguments["visual_frontend_depth_sampling_radius"] == "2"
    assert (
        arguments["visual_frontend_use_depth_translation_refinement"]
        == "true"
    )
    assert "--OdomF2M/BundleAdjustment 1" in arguments["odom_args"]
    assert "--Vis/MinInliersDistribution 0.003" in arguments["odom_args"]


def test_vpi_profile_fuses_f2m_translation_with_h30_orientation():
    module = _load_launch_module()
    action = module._imu_fused_odometry()

    assert action.node_executable == "imu_fused_odometry"
    normalized = action._Node__parameters[0]
    parameters = {
        "".join(part.text for part in key): (
            yaml.safe_load("".join(part.text for part in value))
            if isinstance(value, tuple)
            else value
        )
        for key, value in normalized.items()
    }
    assert parameters["input_topic"] == "/rtabmap/odom_raw"
    assert parameters["output_topic"] == "/rtabmap/odom"
    assert parameters["imu_topic"] == "/sensors/imu/data"
    assert parameters["maximum_raw_translation"] == 0.30
    assert parameters["maximum_raw_rotation_deg"] == 60.0
    assert parameters["maximum_orientation_disagreement_deg"] == 15.0
    assert parameters["maximum_pose_variance"] == 0.1
    assert parameters["publish_tf"] is True


def test_vpi_sensor_actions_accept_runtime_imu_parameter():
    module = _load_launch_module()

    actions = module._sensor_actions()

    assert len(actions) == 3


def test_removed_mapping_modes_are_rejected():
    module = _load_launch_module()
    context = LaunchContext()
    context.launch_configurations["mode"] = "stable"

    with pytest.raises(RuntimeError, match="fixed to VPI/Luxi"):
        module._launch_profile(context)


def test_usb_adapter_publishes_the_canonical_sensor_contract():
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "usb_adapter.yaml").read_text(
            encoding="utf-8"
        )
    )["luxi_adapter"]["ros__parameters"]

    assert parameters["rgbd_output_topic"] == "/sensors/rgbd/rgbd_image"
    assert parameters["color_output_topic"] == "/sensors/rgbd/color/image_raw"
    assert parameters["depth_output_topic"] == "/sensors/rgbd/depth/image_raw"
    assert parameters["imu_output_topic"] == "/sensors/imu/data_raw"


def test_usb_driver_restores_pre_extension_depth_limits():
    driver_root = PACKAGE_ROOT.parent / "usb_camera_driver"
    source = (driver_root / "src" / "stereo_depth_node.cpp").read_text(
        encoding="utf-8"
    )
    parameters = yaml.safe_load(
        (driver_root / "config" / "stereo_depth.yaml").read_text(
            encoding="utf-8"
        )
    )["usb_stereo_depth_node"]["ros__parameters"]

    assert parameters["max_depth_m"] == 6.0
    assert parameters["point_cloud_max_depth_m"] == 3.0
    assert "depth_far_sparse_pixel_step_" not in source


def test_h30_driver_reconnects_and_fails_fast_when_serial_is_silent():
    workspace_root = PACKAGE_ROOT.parents[1]
    repo_root = PACKAGE_ROOT.parents[4]
    h30_source = (
        workspace_root / "src" / "third_party" / "yesense_ros2" /
        "yesense_std_ros2" / "src" / "yesense_node.cpp"
    ).read_text(encoding="utf-8")
    h30_config = yaml.safe_load(
        (PACKAGE_ROOT.parent / "usb_camera_bringup" / "config" /
         "h30_imu.yaml").read_text(encoding="utf-8")
    )["yesense_pub"]["ros__parameters"]
    sync_source = (
        repo_root / "project" / "luxi_RTAB_Map" / "src" /
        "sensor_sync_check.cpp"
    ).read_text(encoding="utf-8")

    assert h30_config["serial_port"] == "/dev/imu-H30"
    assert h30_config["baud_rate"] == 921600
    assert h30_config["reconnect_interval_sec"] == 1.0
    assert h30_config["no_data_warn_sec"] == 2.0
    assert h30_config["no_data_reopen_sec"] == 5.0
    assert "open_ros_serial" in h30_source
    assert "H30 NO DATA" in h30_source
    assert "quaternion_norm" in h30_source
    assert "IMU NO DATA" in sync_source
    assert '"imu_no_data_timeout_sec", 8.0' in sync_source
