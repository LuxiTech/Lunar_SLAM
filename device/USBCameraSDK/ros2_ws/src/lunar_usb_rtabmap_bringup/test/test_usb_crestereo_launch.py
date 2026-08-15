"""Static contracts for the independent pre-trained CREStereo experiment."""

import importlib.util
from pathlib import Path

import numpy as np
import onnx
from launch import LaunchContext
from std_msgs.msg import Header
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_ROOT.parent
DRIVER_ROOT = WORKSPACE_SRC / "usb_camera_driver"
BRINGUP_ROOT = WORKSPACE_SRC / "usb_camera_bringup"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_pretrained_model_is_valid_fixed_shape_onnx():
    path = DRIVER_ROOT / "models" / "crestereo_init_iter2_180x320_fp16.onnx"
    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    dimensions = [item.dim_value for item in model.graph.input[0].type.tensor_type.shape.dim]
    assert dimensions == [1, 3, 180, 320]
    assert len(model.graph.input) == 2


def test_primary_refinement_model_is_valid_four_input_cascade():
    path = DRIVER_ROOT / "models" / "crestereo_combined_iter2_360x640.onnx"
    model = onnx.load(str(path))
    onnx.checker.check_model(model)
    dimensions = [
        [item.dim_value for item in value.type.tensor_type.shape.dim]
        for value in model.graph.input
    ]
    assert dimensions == [
        [1, 3, 180, 320],
        [1, 3, 180, 320],
        [1, 3, 360, 640],
        [1, 3, 360, 640],
    ]


def test_crestereo_config_is_learned_only_and_nx_bounded():
    parameters = yaml.safe_load(
        (DRIVER_ROOT / "config" / "crestereo_depth.yaml").read_text(encoding="utf-8")
    )["usb_crestereo_depth_node"]["ros__parameters"]
    source = (DRIVER_ROOT / "scripts" / "crestereo_depth_node.py").read_text(
        encoding="utf-8"
    )
    assert parameters["execution_provider"] == "cuda"
    assert parameters["enable_cuda_graph"] is True
    assert parameters["use_cuda_io_binding"] is True
    assert parameters["output_width"] == 480
    assert parameters["output_height"] == 270
    assert parameters["target_rate"] == 6.0
    assert parameters["max_depth_m"] == 10.0
    assert parameters["ground_plane_prior_enabled"] is False
    assert parameters["ground_plane_camera_height_m"] == 0.35
    assert "StereoSGBM" not in source
    assert "StereoBM" not in source
    assert "pending_pair" in source
    assert "TensorrtExecutionProvider" in source
    assert "run_with_iobinding" in source


def test_experimental_launch_does_not_change_production_vpi_entry():
    production = (BRINGUP_ROOT / "launch" / "stereo_rgbd.launch.py").read_text(
        encoding="utf-8"
    )
    experiment = (
        BRINGUP_ROOT / "launch" / "stereo_crestereo_rgbd.launch.py"
    ).read_text(encoding="utf-8")
    assert 'executable="stereo_depth_node"' in production
    assert "crestereo" not in production.lower()
    assert 'executable="crestereo_depth_node"' in experiment


def test_preview_cloud_keeps_dense_boundary_and_sparse_far_lattice():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node",
    )
    depth = np.full((7, 7), 5000, dtype=np.uint16)
    depth[1:6, 1:6] = 4000
    color = np.zeros((7, 7, 3), dtype=np.uint8)
    projection = np.array(
        [[100.0, 0.0, 3.0, 0.0], [0.0, 100.0, 3.0, 0.0], [0.0, 0.0, 1.0, 0.0]],
        dtype=np.float64,
    )

    cloud = module._point_cloud_message(
        Header(), color, depth, projection, 10.0, 4.0, 6
    )

    # All 25 boundary pixels are dense, plus four far lattice corners not
    # already inside that block: (0,0), (0,6), (6,0), (6,6).
    assert cloud.width == 29


def test_preview_cloud_uses_medium_and_far_lattices():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_tiers",
    )
    depth = np.full((9, 9), 7000, dtype=np.uint16)
    depth[:, :4] = 5000
    color = np.zeros((9, 9, 3), dtype=np.uint8)
    projection = np.array(
        [[100.0, 0.0, 4.0, 0.0], [0.0, 100.0, 4.0, 0.0], [0.0, 0.0, 1.0, 0.0]],
        dtype=np.float64,
    )

    cloud = module._point_cloud_message(
        Header(), color, depth, projection, 10.0, 4.0, 8, 6.0, 4
    )

    # Medium: three 4 px lattice cells in the first four columns. Far: the
    # remaining two points on the 8 px lattice.
    assert cloud.width == 5


def test_ground_plane_prior_repairs_only_physically_below_ground_depth():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_ground_prior",
    )
    depth = np.zeros((6, 5), dtype=np.uint16)
    # With fy=5, cy=1 and camera height 0.5 m, row 5 intersects level ground
    # at 0.625 m. A 2 m return is below ground, while 0.5 m is a real nearer
    # obstacle and zero remains unknown.
    depth[5, 1] = 2000
    depth[5, 2] = 500
    depth[1, 1] = 2000
    projection = np.array(
        [[5.0, 0.0, 2.0, 0.0], [0.0, 5.0, 1.0, 0.0], [0.0, 0.0, 1.0, 0.0]],
        dtype=np.float64,
    )

    corrected, mask = module._apply_ground_plane_prior(
        depth, projection, 0.5, 0.1, 0.5, 0.4, 10.0
    )

    assert corrected[5, 1] == 625
    assert mask[5, 1]
    assert corrected[5, 2] == 500
    assert not mask[5, 2]
    assert corrected[1, 1] == 2000
    assert corrected[5, 3] == 0


def test_stereo_photometric_normalization_reduces_sensor_response_mismatch():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_photometric_normalization",
    )
    ramp = np.tile(np.linspace(20, 200, 64, dtype=np.uint8), (32, 1))
    left = np.repeat(ramp[:, :, None], 3, axis=2)
    right = np.clip(
        left.astype(np.float32) * 0.80 + 15.0, 0.0, 255.0
    ).astype(np.uint8)

    normalized, gain, offset = module._normalize_stereo_photometry(
        left, right, np.ones(ramp.shape, dtype=bool), 0.75, 1.35, 40.0
    )

    before = np.mean(np.abs(left.astype(np.float32) - right.astype(np.float32)))
    after = np.mean(
        np.abs(left.astype(np.float32) - normalized.astype(np.float32))
    )
    assert 1.15 < gain < 1.35
    assert -30.0 < offset < -5.0
    assert after < before * 0.20


def test_subpixel_patch_consistency_rejects_local_reflection():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_photometric_patch",
    )
    generator = np.random.default_rng(7)
    left_gray = generator.integers(20, 220, size=(24, 40), dtype=np.uint8)
    right_gray = np.zeros_like(left_gray)
    # A two-pixel rectified disparity means right(x - 2) == left(x).
    right_gray[:, :-2] = left_gray[:, 2:]
    right_gray[8:16, 12:18] = 255 - right_gray[8:16, 12:18]
    left = np.repeat(left_gray[:, :, None], 3, axis=2)
    right = np.repeat(right_gray[:, :, None], 3, axis=2)
    disparity = np.full(left_gray.shape, 2.0, dtype=np.float32)
    valid = np.zeros(left_gray.shape, dtype=bool)
    valid[:, 3:-2] = True

    consistent, _, _ = module._photometric_consistency_mask(
        left, right, disparity, valid, 30.0, 3, 18.0
    )

    # The altered right patch projects to left columns 14:20 and must not be
    # accepted as ordinary texture, while most unchanged support survives.
    assert np.count_nonzero(consistent[8:16, 14:20]) == 0
    assert np.count_nonzero(consistent) > np.count_nonzero(valid) * 0.70


def test_left_right_consistency_rejects_geometrically_one_sided_depth():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_left_right_consistency",
    )
    left = np.full((8, 20), 3.0, dtype=np.float32)
    # The helper receives the prediction in flipped-right coordinates. A
    # constant field survives the unflip and validates ordinary pixels.
    reverse_flipped = np.full_like(left, 3.0)
    # Original right columns 6:9 disagree. Their flipped positions are 11:14.
    reverse_flipped[:, 11:14] = 7.0
    valid = np.ones(left.shape, dtype=bool)

    consistent, difference = module._left_right_consistency_mask(
        left, reverse_flipped, valid, 1.0
    )

    # A left pixel samples right at x-3, so bad right columns 6:9 invalidate
    # left columns 9:12 while unchanged bidirectional support remains valid.
    assert np.count_nonzero(consistent[:, 9:12]) == 0
    assert np.all(difference[:, 9:12] > 1.0)
    assert np.all(consistent[:, 3:9])


def test_guided_disparity_upsampling_preserves_holes_and_removes_blocks():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_guided_upsampling",
    )
    disparity = np.array(
        [[4.0, 4.0, 8.0], [4.0, 0.0, 8.0]], dtype=np.float32
    )
    valid = disparity > 0.0
    guide = np.zeros((4, 6, 3), dtype=np.uint8)
    guide[:, 4:] = 255

    output = module._edge_aware_upsample_disparity(
        disparity, valid, guide, (6, 4), 2.0, 2, 10.0
    )

    assert output.shape == (4, 6)
    assert output.dtype == np.float32
    # The invalid model cell expands to a 2x2 hole and is never synthesized.
    assert np.count_nonzero(output[2:4, 2:4]) == 0
    # Linear/guided interpolation creates sub-pixel values instead of three
    # exact nearest-neighbor disparity plateaus.
    assert len(np.unique(np.round(output[output > 0.0], 3))) > 3


def test_crestereo_rtab_launch_uses_atomic_learned_odometry_and_features():
    module = _load(
        PACKAGE_ROOT / "launch" / "usb_crestereo_rtabmap.launch.py",
        "usb_crestereo_rtabmap_launch",
    )
    actions = module._sensor_actions()
    assert len(actions) == 3
    arguments = module._mapping_arguments()
    assert arguments["learned_frontend"] == "true"
    assert arguments["visual_odometry"] == "false"
    assert arguments["odom_topic"] == "/luxi_visual_frontend/odom"
    assert arguments["rgbd_topic"] == "/luxi_visual_frontend/rgbd_image"
    assert arguments["subscribe_odom_info"] == "false"
    assert arguments["visual_frontend_publish_tf"] == "true"
    assert arguments["visual_frontend_target_rate"].describe() == (
        "LaunchConfig('crestereo_frontend_target_rate')"
    )
    assert arguments["visual_frontend_upstream_rate_limited"] == "true"
    assert arguments["visual_frontend_keyframe_max_age"] == "3.0"
    assert arguments["visual_frontend_depth_sampling_radius"] == "3"
    assert arguments["visual_frontend_use_depth_translation_refinement"] == "false"
    assert arguments["visual_frontend_stationary_maximum_median_pixel_motion"] == "0.75"
    assert arguments["visual_frontend_stationary_maximum_rotation_deg"] == "0.50"
    assert arguments["visual_frontend_motion_hint_topic"] == "/cmd_vel"
    assert (
        arguments["visual_frontend_stationary_hint_maximum_median_pixel_motion"]
        == "2.0"
    )
    assert arguments["visual_frontend_stationary_hint_maximum_translation"] == "0.05"
    assert arguments["visual_frontend_constrain_vertical_translation"] == "true"
    assert arguments["visual_frontend_maximum_depth"] == "5.0"
    assert arguments["visual_frontend_mapping_depth_dense_maximum"] == "4.0"
    assert arguments["visual_frontend_mapping_depth_medium_maximum"] == "6.0"
    assert arguments["visual_frontend_mapping_depth_medium_sparse_pixel_step"] == "4"
    assert arguments["visual_frontend_mapping_depth_far_maximum"] == "10.0"
    assert arguments["visual_frontend_mapping_depth_far_sparse_pixel_step"] == "8"
    assert arguments["visual_frontend_mapping_ground_prior_enabled"] == "true"
    assert arguments["visual_frontend_mapping_ground_surface_tolerance"] == "0.0"
    assert arguments["visual_frontend_mapping_ground_maximum_correction"] == "1.2"
    assert arguments["visual_frontend_mapping_ground_minimum_up_alignment"] == "0.25"
    assert arguments["visual_frontend_mapping_ground_minimum_row_ratio"] == "0.52"
    assert (
        arguments["visual_frontend_mapping_ground_reject_unverified_below_plane"]
        == "true"
    )
    assert (
        arguments["visual_frontend_mapping_ground_repair_all_below_plane"]
        == "true"
    )
    assert "--Rtabmap/LoopThr 0.11" in arguments["rtabmap_args"]
    assert "--RGBD/LinearUpdate 0.06" in arguments["rtabmap_args"]
    assert "--RGBD/AngularUpdate 0.05" in arguments["rtabmap_args"]
    assert arguments["visual_frontend_mapping_ground_camera_height"] == "0.35"
    assert arguments["visual_frontend_mapping_ground_below_tolerance"] == "0.05"
    assert arguments["visual_frontend_mapping_ground_surface_tolerance"] == "0.0"
    assert "--Kp/MaxDepth 4.0" in arguments["rtabmap_args"]
    assert "--Grid/RangeMax 4.0" in arguments["rtabmap_args"]
    assert "--Grid/MaxGroundAngle 25.0" in arguments["rtabmap_args"]
    assert "--Grid/MapFrameProjection true" in arguments["rtabmap_args"]
    assert "--Grid/RayTracing true" in arguments["rtabmap_args"]
    assert "--Grid/MaxObstacleHeight 0.8" in arguments["rtabmap_args"]
    assert "--Grid/NoiseFilteringRadius 0.12" in arguments["rtabmap_args"]
    assert "--Grid/NoiseFilteringMinNeighbors 8" in arguments["rtabmap_args"]


def test_crestereo_profile_uses_h30_by_default():
    text = (
        PACKAGE_ROOT / "launch" / "usb_crestereo_rtabmap.launch.py"
    ).read_text(encoding="utf-8")

    assert "*_BASE._common_launch_arguments()" in text
    assert '"start_imu": use_imu' in text
    assert '"enable_imu": ParameterValue(use_imu' in text
    assert '"camera_params": LaunchConfiguration("camera_params")' in text
    assert '"calibration_file": LaunchConfiguration("calibration_file")' in text


def test_web_mount_calibration_launch_owns_only_h30_and_level_calibrator():
    text = (
        PACKAGE_ROOT / "launch" / "usb_imu_level_calibration.launch.py"
    ).read_text(encoding="utf-8")
    assert 'executable="yesense_node_publisher"' in text
    assert 'executable="imu_level_calibrator_node"' in text
    assert '"nominal_roll": -math.pi / 2.0' in text
    assert 'executable="stereo_node"' not in text
    assert 'executable="crestereo_depth_node"' not in text


def test_saved_map_navigation_owns_usb_crestereo_h30_sensor():
    launch = (
        PACKAGE_ROOT
        / "launch"
        / "usb_crestereo_saved_map_navigation.launch.py"
    ).read_text(encoding="utf-8")

    assert "stereo_crestereo_rgbd.launch.py" in launch
    assert 'executable="sensor_adapter_node"' in launch
    assert '"enable_imu": ParameterValue(use_imu' in launch
    assert "saved_map_navigation.launch.py" in launch
    assert '"hloc_map_directory": LaunchConfiguration(' in launch


def test_max_performance_profile_is_explicit_and_keeps_native_camera_input():
    parameters = yaml.safe_load(
        (
            DRIVER_ROOT / "config" / "crestereo_depth_max_performance.yaml"
        ).read_text(encoding="utf-8")
    )["usb_crestereo_depth_node"]["ros__parameters"]
    launch = (
        PACKAGE_ROOT / "launch" / "usb_crestereo_max_performance_rtabmap.launch.py"
    ).read_text(encoding="utf-8")

    assert parameters["target_rate"] == 10.0
    assert parameters["output_width"] == 960
    assert parameters["output_height"] == 540
    assert parameters["preprocess_threads"] == 3
    assert parameters["pipeline_preprocessing"] is True
    assert parameters["use_fixed_point_remap"] is True
    assert parameters["ground_plane_prior_enabled"] is False
    assert parameters["ground_plane_camera_height_m"] == 0.35
    assert "crestereo_frontend_target_rate\": \"10.0" in launch
    assert "crestereo_frontend_max_keypoints\": \"640" in launch
    assert "crestereo_lightglue_cuda_graph_keypoints\": \"640" in launch
    assert "crestereo_mapping_rate\": \"2.0" in launch


def test_primary_mapping_profile_uses_native_640x360_geometry():
    parameters = yaml.safe_load(
        (DRIVER_ROOT / "config" / "crestereo_depth_mapping_quality.yaml").read_text(
            encoding="utf-8"
        )
    )["usb_crestereo_depth_node"]["ros__parameters"]
    launch = (
        PACKAGE_ROOT / "launch" / "usb_crestereo_rtabmap.launch.py"
    ).read_text(encoding="utf-8")

    assert parameters["output_width"] == 640
    assert parameters["output_height"] == 360
    assert parameters["target_rate"] == 3.0
    assert parameters["guided_upsampling_radius"] == 0
    assert parameters["ground_plane_prior_enabled"] is False
    assert parameters["left_right_consistency"] is True
    assert parameters["left_right_maximum_difference_px"] == 1.0
    assert "crestereo_combined_iter2_360x640.onnx" in launch
    assert "crestereo_init_iter2_180x320_fp16.onnx" in launch
    assert "crestereo_depth_mapping_quality.yaml" in launch
    assert 'default_value="3.0"' in launch


def test_crestereo_model_preflight_accepts_models_and_reports_missing(tmp_path):
    module = _load(
        PACKAGE_ROOT / "launch" / "usb_crestereo_rtabmap.launch.py",
        "usb_crestereo_rtabmap_model_preflight",
    )
    context = LaunchContext()
    context.launch_configurations["crestereo_model_path"] = str(
        DRIVER_ROOT / "models" / "crestereo_combined_iter2_360x640.onnx"
    )
    context.launch_configurations["crestereo_left_right_model_path"] = str(
        DRIVER_ROOT / "models" / "crestereo_init_iter2_180x320_fp16.onnx"
    )

    assert module._validate_crestereo_models(context) == []

    context.launch_configurations["crestereo_model_path"] = str(
        tmp_path / "missing.onnx"
    )
    with np.testing.assert_raises_regex(
        RuntimeError, "CREStereo model preflight failed"
    ):
        module._validate_crestereo_models(context)


def test_crestereo_shutdown_joins_native_workers_before_session_teardown():
    source = (DRIVER_ROOT / "scripts" / "crestereo_depth_node.py").read_text(
        encoding="utf-8"
    )

    assert source.count("daemon=False") >= 2
    assert "preprocess_pool.shutdown(wait=True, cancel_futures=True)" in source
