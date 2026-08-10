"""Static contract tests for the device-owned USB mapping entry."""

import importlib.util
from pathlib import Path

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


def test_one_entry_owns_both_supported_profiles():
    text = LAUNCH_PATH.read_text(encoding="utf-8")

    assert 'default_value="vpi_learned"' in text
    assert '"use_imu",\n            default_value="true"' in text
    assert '{"stable", "vpi_learned"}' in text
    assert '"opencv_cuda_sgm" if mode == "stable"' in text
    assert 'else "vpi_ofa_pva_vic"' in text


def test_stable_profile_keeps_validated_odometry_postprocessor():
    module = _load_launch_module()
    arguments = module._mapping_arguments("stable")

    assert arguments["odom_output_topic"] == "/rtabmap/odom_raw"
    assert arguments["publish_odom_tf"] == "false"
    assert "--OdomF2M/BundleAdjustmentMaxFrames 5" in arguments["odom_args"]
    assert "--Vis/FeatureType 8" in arguments["odom_args"]


def test_vpi_profile_keeps_depth_refinement_and_external_odometry():
    module = _load_launch_module()
    arguments = module._mapping_arguments("vpi_learned")

    assert arguments["learned_frontend"] == "true"
    assert arguments["visual_odometry"] == "true"
    assert arguments["odom_topic"] == "/rtabmap/odom"
    assert arguments["odom_rgbd_topic"] == "/sensors/rgbd/rgbd_image"
    assert arguments["publish_odom_tf"] == "true"
    assert arguments["visual_frontend_publish_tf"] == "false"
    assert arguments["visual_frontend_rgbd_features_rate"] == "1.0"
    assert arguments["visual_frontend_depth_sampling_radius"] == "2"
    assert (
        arguments["visual_frontend_use_depth_translation_refinement"]
        == "true"
    )
    assert "--OdomF2M/BundleAdjustment 1" in arguments["odom_args"]


def test_vpi_sensor_actions_accept_runtime_imu_parameter():
    module = _load_launch_module()

    actions = module._sensor_actions("vpi_learned")

    assert len(actions) == 3


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


def test_unknown_mode_is_rejected():
    module = _load_launch_module()

    class Mode:
        def perform(self, _context):
            return "unknown"

    original = module.LaunchConfiguration
    module.LaunchConfiguration = lambda _name: Mode()
    try:
        with pytest.raises(RuntimeError, match="Unsupported USB mapping mode"):
            module._launch_profile(object())
    finally:
        module.LaunchConfiguration = original
