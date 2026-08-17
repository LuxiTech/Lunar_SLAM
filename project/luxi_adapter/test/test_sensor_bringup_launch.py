"""Public hardware-profile selection contract for sensor_bringup.launch.py."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.utilities import perform_substitutions


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "sensor_bringup.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("sensor_bringup_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _load_python_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_hik_argument_selects_hik_driver_and_complete_config():
    module = _load_launch_module()

    selection = module.resolve_hardware_selection(
        hardware="HIK",
        config_path="",
        package_share=PACKAGE_ROOT,
    )

    assert selection.profile == "hik"
    assert selection.launch_path == PACKAGE_ROOT / "launch" / "hik.launch.py"
    assert selection.config_path == PACKAGE_ROOT / "config" / "hik_sensor_bringup.yaml"


def test_d435i_argument_selects_d435i_driver_and_complete_config():
    module = _load_launch_module()

    selection = module.resolve_hardware_selection(
        hardware="D435i",
        config_path="",
        package_share=PACKAGE_ROOT,
    )

    assert selection.profile == "d435i"
    assert selection.launch_path == PACKAGE_ROOT / "launch" / "d435i.launch.py"
    assert selection.config_path == PACKAGE_ROOT / "config" / "sensor_bringup.yaml"


def test_d455_argument_selects_d455_driver_and_complete_config():
    module = _load_launch_module()

    selection = module.resolve_hardware_selection(
        hardware="D455",
        config_path="",
        package_share=PACKAGE_ROOT,
    )

    assert selection.profile == "d455"
    assert selection.launch_path == PACKAGE_ROOT / "launch" / "d455.launch.py"
    assert selection.config_path == (
        PACKAGE_ROOT / "config" / "d455_sensor_bringup.yaml"
    )


def test_sensor_bringup_detects_residual_components(tmp_path):
    module = _load_launch_module()
    current = tmp_path / str(1234)
    unrelated = tmp_path / str(5678)
    current.mkdir()
    unrelated.mkdir()
    (current / "cmdline").write_bytes(
        b"/opt/ros/humble/lib/imu_filter_madgwick/imu_filter_madgwick_node\0"
    )
    (unrelated / "cmdline").write_bytes(b"/usr/bin/python3\0worker.py\0")

    assert module.find_residual_sensor_processes(tmp_path) == [
        (1234, "/opt/ros/humble/lib/imu_filter_madgwick/imu_filter_madgwick_node")
    ]


def test_d435i_uses_project_domain_42():
    yaml = pytest.importorskip("yaml")
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "sensor_bringup.yaml").read_text(
            encoding="utf-8"
        )
    )["luxi_adapter"]["ros__parameters"]

    assert parameters["ros_domain_id"] == 42


def test_d435i_estimated_mount_uses_fourteen_centimeter_midpoint_and_auto_level():
    yaml = pytest.importorskip("yaml")
    parameters = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "sensor_bringup.yaml").read_text(
            encoding="utf-8"
        )
    )["luxi_adapter"]["ros__parameters"]

    assert parameters["camera_x"] == pytest.approx(0.14)
    assert parameters["camera_y"] == pytest.approx(0.0)
    assert parameters["camera_yaw"] == pytest.approx(0.0)
    assert parameters["auto_level_imu"] is True
    assert parameters["auto_level_start_automatically"] is False
    assert parameters["imu_level_calibration_service"] == "/sensors/imu/calibrate_level"


def test_explicit_hardware_rejects_mismatched_config():
    module = _load_launch_module()

    with pytest.raises(RuntimeError, match="selects 'd435i'.*requested 'hik'"):
        module.resolve_hardware_selection(
            hardware="hik",
            config_path=str(PACKAGE_ROOT / "config" / "sensor_bringup.yaml"),
            package_share=PACKAGE_ROOT,
        )


def test_future_profile_uses_naming_convention_without_dispatcher_changes(tmp_path):
    module = _load_launch_module()
    (tmp_path / "launch").mkdir()
    (tmp_path / "config").mkdir()
    (tmp_path / "launch" / "future_depth.launch.py").touch()
    (tmp_path / "config" / "future_depth_sensor_bringup.yaml").write_text(
        "luxi_adapter:\n"
        "  ros__parameters:\n"
        "    hardware_profile: future_depth\n",
        encoding="utf-8",
    )

    selection = module.resolve_hardware_selection(
        hardware="future_depth",
        config_path="",
        package_share=tmp_path,
    )

    assert selection.profile == "future_depth"
    assert selection.launch_path == tmp_path / "launch" / "future_depth.launch.py"
    assert selection.config_path == (
        tmp_path / "config" / "future_depth_sensor_bringup.yaml"
    )


def test_mapping_rviz_uses_only_canonical_adapter_images():
    rviz_path = PACKAGE_ROOT.parent / "luxi_RTAB_Map" / "rviz" / "rgbd_mapping.rviz"
    rviz = pytest.importorskip("yaml").safe_load(rviz_path.read_text(encoding="utf-8"))
    displays = rviz["Visualization Manager"]["Displays"]
    topics = {
        display["Name"]: display.get("Topic", {}).get("Value")
        for display in displays
    }

    assert topics["RGBImage"] == "/sensors/rgbd/color/image_raw"
    assert topics["DepthPreview"] == "/sensors/rgbd/depth/image_raw"


def test_all_sensor_configs_share_the_algorithm_facing_topics():
    yaml = pytest.importorskip("yaml")
    d435i = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "sensor_bringup.yaml").read_text(encoding="utf-8")
    )["luxi_adapter"]["ros__parameters"]
    hik = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "hik_sensor_bringup.yaml").read_text(
            encoding="utf-8"
        )
    )["luxi_adapter"]["ros__parameters"]
    d455 = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "d455_sensor_bringup.yaml").read_text(
            encoding="utf-8"
        )
    )["luxi_adapter"]["ros__parameters"]
    public_topics = (
        "rgbd_output_topic",
        "color_output_topic",
        "depth_output_topic",
        "camera_info_output_topic",
        "imu_output_topic",
        "compressed_color_output_topic",
    )

    for topic_name in public_topics:
        assert hik[topic_name] == d435i[topic_name]
        assert d455[topic_name] == d435i[topic_name]


def test_d455_uses_wide_matched_profiles_and_aligned_depth():
    yaml = pytest.importorskip("yaml")
    adapter = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "d455_sensor_bringup.yaml").read_text(
            encoding="utf-8"
        )
    )["luxi_adapter"]["ros__parameters"]
    device_config = (
        PACKAGE_ROOT.parents[1]
        / "device"
        / "D455"
        / "ros2_ws"
        / "src"
        / "lunar_d455_bringup"
        / "config"
        / "d455.yaml"
    )
    device = yaml.safe_load(device_config.read_text(encoding="utf-8"))[
        "/camera/camera"
    ]["ros__parameters"]

    assert adapter["depth_input_topic"] == (
        "/camera/camera/aligned_depth_to_color/image_raw"
    )
    assert adapter["d455_color_profile"] == "848,480,30"
    assert adapter["d455_depth_profile"] == "848,480,30"
    assert device["device_type"] == "d455"
    assert device["align_depth.enable"] is True
    assert device["rgb_camera.color_profile"] == "848,480,30"
    assert device["depth_module.depth_profile"] == "848,480,30"


def test_hik_empty_stereo_override_uses_hik_profile_default(tmp_path):
    module = _load_launch_module()
    expected = tmp_path / "config" / "stereo_proc.yaml"

    assert module.resolve_hik_stereo_proc_params("", tmp_path) == expected


def test_generic_mapping_depth_decimation_supports_both_camera_resolutions():
    launch_path = (
        PACKAGE_ROOT.parent
        / "luxi_RTAB_Map"
        / "launch"
        / "rgbd_mapping_learned.launch.py"
    )
    module = _load_python_module(launch_path, "rgbd_mapping_learned_launch")
    declaration = next(
        action
        for action in module.generate_launch_description().entities
        if getattr(action, "name", None) == "grid_depth_decimation"
    )

    assert perform_substitutions(LaunchContext(), declaration.default_value) == "2"
