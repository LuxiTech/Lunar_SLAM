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


def test_low_light_enhancement_uses_one_curve_for_both_cameras():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_low_light",
    )
    ramp = np.tile(np.arange(8, 40, dtype=np.uint8), (32, 2))
    left = np.repeat(ramp[:, :, None], 3, axis=2)
    right = np.clip(left.astype(np.int16) + 3, 0, 255).astype(np.uint8)

    bright_left, bright_right, gamma, luma = module._enhance_stereo_low_light(
        left, right, np.ones(ramp.shape, dtype=bool), 36.0, 52.0, 0.55
    )

    assert luma < 36.0
    assert 0.55 <= gamma < 1.0
    assert np.median(bright_left) > np.median(left)
    # A shared monotonic LUT preserves the binocular ordering and cannot
    # manufacture independent CLAHE features.
    assert np.all(bright_right >= bright_left)


def test_fast_foundation_stereo_tensor_uses_imagenet_rgb_and_symmetric_padding():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_ffs_tensor",
    )
    image = np.empty((180, 320, 3), dtype=np.uint8)
    image[:] = (10, 20, 30)  # BGR input becomes RGB tensor 30,20,10.

    tensor, pad_top = module._fast_foundation_stereo_tensor(image)

    assert tensor.shape == (1, 3, 192, 320)
    assert tensor.dtype == np.float32
    assert pad_top == 6
    expected = (
        np.array([30.0, 20.0, 10.0], dtype=np.float32) / 255.0
        - np.array([0.485, 0.456, 0.406], dtype=np.float32)
    ) / np.array([0.229, 0.224, 0.225], dtype=np.float32)
    assert np.allclose(tensor[0, :, 6, 0], expected, atol=1e-6)
    # Fast-FoundationStereo's InputPadder replicates boundary rows; zeros in
    # normalized space would instead create an artificial gray horizon.
    assert np.all(tensor[:, :, :6] == tensor[:, :, 6:7])
    assert np.all(tensor[:, :, 186:] == tensor[:, :, 185:186])


def test_fast_foundation_stereo_output_crop_restores_320x180_content():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_ffs_crop",
    )
    output = np.broadcast_to(
        np.arange(192, dtype=np.float32)[None, None, :, None],
        (1, 1, 192, 320),
    ).copy()

    cropped = module._crop_fast_foundation_stereo_output(
        output, (320, 180), 6
    )

    assert cropped.shape == (1, 180, 320)
    assert np.all(cropped[:, 0] == 6.0)
    assert np.all(cropped[:, -1] == 185.0)


def test_model_family_switch_keeps_crestereo_tensor_contract_as_default():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_model_family",
    )
    image = np.array([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8)

    tensor = module.CREStereoDepthNode._tensor(image)

    assert tensor.shape == (1, 3, 1, 2)
    assert tensor.dtype == np.float32
    assert np.array_equal(tensor[0, :, 0, 0], [30.0, 20.0, 10.0])
    source = (DRIVER_ROOT / "scripts" / "crestereo_depth_node.py").read_text(
        encoding="utf-8"
    )
    assert '"model_family", "crestereo"' in source
    assert 'FAST_FOUNDATION_STEREO_REVISION = "ffs_trt_half_v1"' in source
    assert "TensorRT-direct" in source
    assert (
        'self.low_light_enhancement and self.model_family == "crestereo"'
        in source
    )
    assert "self.model_remap_size = (640, 360)" in source
    assert "content_size, interpolation=cv2.INTER_AREA" in source
    assert (
        'self.direct_engine.input_names != ["left_image", "right_image"]'
        in source
    )
    assert 'self.direct_engine.output_names != ["disparity"]' in source
    assert module.MODEL_FAMILY_REVISIONS["crestereo"] == "confidence_v2"
    assert (
        module.MODEL_FAMILY_REVISIONS["fast_foundation_stereo"]
        == "ffs_trt_half_v1"
    )


def test_local_disparity_gate_rejects_spike_but_keeps_coherent_pit():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_local_disparity",
    )
    disparity = np.full((15, 15), 12.0, dtype=np.float32)
    disparity[7, 7] = 2.0
    disparity[2:7, 2:7] = 4.0
    valid = np.ones(disparity.shape, dtype=bool)

    filtered, _, _ = module._local_disparity_consistency(
        disparity, valid, 5, 0.75, 0.08
    )

    assert not filtered[7, 7]
    assert np.all(filtered[3:6, 3:6])


def test_temporal_warp_is_identity_for_static_frame():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_temporal_warp",
    )
    gray = np.tile(np.arange(64, dtype=np.uint8), (32, 1))
    disparity = np.full(gray.shape, 9.0, dtype=np.float32)
    valid = np.ones(gray.shape, dtype=bool)

    warped, warped_valid, error = module._warp_previous_disparity(
        gray, gray, disparity, valid
    )

    assert np.mean(warped_valid) > 0.95
    assert np.median(np.abs(warped[warped_valid] - 9.0)) < 0.05
    assert np.median(error[warped_valid]) < 1.0


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


def test_radiometric_confidence_separates_texture_from_view_dependent_highlight():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_radiometric_confidence",
    )
    generator = np.random.default_rng(19)
    left_gray = generator.integers(20, 180, size=(36, 64), dtype=np.uint8)
    right_gray = np.zeros_like(left_gray)
    right_gray[:, :-2] = left_gray[:, 2:]
    left = np.repeat(left_gray[:, :, None], 3, axis=2)
    right = np.repeat(right_gray[:, :, None], 3, axis=2)
    disparity = np.full(left_gray.shape, 2.0, dtype=np.float32)
    valid = np.zeros(left_gray.shape, dtype=bool)
    valid[:, 4:-2] = True

    texture, reflection = module._radiometric_stereo_confidence(
        left, right, disparity, valid, 7, 3.0, 2.0, 12.0, 12.0
    )

    assert np.median(texture[valid]) > 0.90
    assert np.median(reflection[valid]) < 0.10

    reflected_right = right.copy()
    # Right columns 22:30 project into left columns 24:32.
    reflected_right[12:24, 22:30] = 255
    _, reflected = module._radiometric_stereo_confidence(
        left, reflected_right, disparity, valid, 7, 3.0, 2.0, 12.0, 12.0
    )
    assert np.median(reflected[12:24, 24:32]) > 0.55

    diffuse = np.full_like(left, 40)
    texture, reflection = module._radiometric_stereo_confidence(
        diffuse, diffuse, disparity, valid, 7, 3.0, 2.0, 12.0, 12.0
    )
    # Low texture is uncertainty but is not mislabeled as reflection.
    assert np.median(texture[valid]) < 0.05
    assert np.median(reflection[valid]) < 0.10


def test_radiometric_texture_requires_horizontal_epipolar_support():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_radiometric_epipolar_texture",
    )
    horizontal_bands = np.zeros((40, 64), dtype=np.uint8)
    horizontal_bands[::4] = 180
    horizontal_bands[1::4] = 180
    vertical_bands = np.zeros_like(horizontal_bands)
    vertical_bands[:, ::4] = 180
    vertical_bands[:, 1::4] = 180
    disparity = np.zeros(horizontal_bands.shape, dtype=np.float32)
    valid = np.zeros(horizontal_bands.shape, dtype=bool)
    valid[4:-4, 4:-4] = True

    def score(gray):
        image = np.repeat(gray[:, :, None], 3, axis=2)
        texture, _ = module._radiometric_stereo_confidence(
            image, image, disparity, valid, 7, 3.0, 2.0, 12.0, 12.0
        )
        return float(np.median(texture[valid]))

    # Contrast without x-gradient is not observable by rectified stereo.
    assert score(horizontal_bands) < 0.05
    assert score(vertical_bands) > 0.90


def test_confidence_v2_keeps_verified_low_texture_and_uses_joint_vetoes():
    module = _load(
        DRIVER_ROOT / "scripts" / "crestereo_depth_node.py",
        "crestereo_depth_node_confidence_v2",
    )
    lr = np.array([[1.0, 0.70, 1.0, 1.0]], dtype=np.float32)
    photo = np.array([[1.0, 0.70, 1.0, 1.0]], dtype=np.float32)
    local = np.array([[1.0, 1.0, 0.70, 1.0]], dtype=np.float32)
    temporal = np.ones_like(lr)
    temporal_visible = np.zeros_like(lr, dtype=bool)
    texture = np.array([[0.0, 0.0, 1.0, 1.0]], dtype=np.float32)
    reflection = np.array([[0.0, 0.0, 0.80, 0.80]], dtype=np.float32)

    confidence, low_texture_risk, reflection_risk = (
        module._combine_depth_confidence(
            lr,
            photo,
            local,
            temporal,
            temporal_visible,
            texture,
            reflection,
            0.25,
            0.55,
        )
    )

    # Texture alone no longer pushes a geometrically verified diffuse pixel
    # below the output threshold.
    assert confidence[0, 0] > 0.95
    assert not low_texture_risk[0, 0]
    # Low texture needs two weak independent geometry checks.
    assert low_texture_risk[0, 1]
    # Reflection remains a hard veto when one geometry check also disagrees,
    # but a bright region with fully consistent geometry is not deleted.
    assert reflection_risk[0, 2]
    assert not reflection_risk[0, 3]


def test_confidence_v2_revision_and_prethreshold_temporal_history_are_visible():
    source = (DRIVER_ROOT / "scripts" / "crestereo_depth_node.py").read_text(
        encoding="utf-8"
    )
    assert 'ALGORITHM_REVISION = "confidence_v2"' in source
    assert 'FAST_FOUNDATION_STEREO_REVISION = "ffs_trt_half_v1"' in source
    assert "algorithm_revision={self.algorithm_revision}" in source
    assert "MODEL_FAMILY_REVISIONS[self.model_family]" in source
    history = source.index("temporal_history_valid = valid.copy()")
    threshold = source.index("valid &= confidence >= self.minimum_depth_confidence")
    assignment = source.index(
        "self.previous_temporal_valid = temporal_history_valid"
    )
    assert history < threshold < assignment
    assert "self.previous_temporal_valid = valid.copy()" not in source
    assert 'self.model_family == "fast_foundation_stereo"' in source
    assert "base_valid\n                & radiometric_valid" in source


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

    # A confidence hole beside constant disparity must not dilute the last
    # valid column into a false-far boundary. Guided numerator/support
    # normalization keeps 10 px at the model grid equal to 20 px after 2x
    # output scaling, while NN validity still preserves the hole.
    constant = np.full((2, 4), 10.0, dtype=np.float32)
    half_valid = np.zeros(constant.shape, dtype=bool)
    half_valid[:, :2] = True
    flat_guide = np.zeros((4, 8, 3), dtype=np.uint8)
    boundary = module._edge_aware_upsample_disparity(
        constant, half_valid, flat_guide, (8, 4), 2.0, 2, 10.0
    )
    assert np.allclose(boundary[:, :4], 20.0, atol=0.05)
    assert np.count_nonzero(boundary[:, 4:]) == 0


def test_crestereo_rtab_launch_uses_atomic_learned_odometry_and_features():
    module = _load(
        PACKAGE_ROOT / "launch" / "usb_crestereo_rtabmap.launch.py",
        "usb_crestereo_rtabmap_launch",
    )
    actions = module._sensor_actions()
    assert len(actions) == 3
    sensor_arguments = dict(actions[0].launch_arguments)
    assert sensor_arguments["point_cloud_max_depth_m"] == "10.0"
    assert sensor_arguments["point_cloud_far_sparse_start_m"] == "4.0"
    assert sensor_arguments["point_cloud_medium_max_depth_m"] == "6.0"
    assert sensor_arguments["point_cloud_medium_sparse_pixel_step"] == "4"
    assert sensor_arguments["point_cloud_far_sparse_pixel_step"] == "8"
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
    assert arguments["visual_frontend_motion_hint_topic"] == "/d1/cmd_vel_standard"
    assert (
        arguments["visual_frontend_stationary_hint_maximum_median_pixel_motion"]
        == "2.0"
    )
    assert arguments["visual_frontend_stationary_hint_maximum_translation"] == "0.0"
    assert arguments["visual_frontend_constrain_vertical_translation"] == "true"
    assert arguments["visual_frontend_maximum_depth"] == "5.0"
    assert arguments["visual_frontend_mapping_depth_dense_maximum"] == "4.0"
    assert arguments["visual_frontend_mapping_depth_medium_maximum"] == "6.0"
    assert arguments["visual_frontend_mapping_depth_medium_sparse_pixel_step"] == "4"
    assert arguments["visual_frontend_mapping_depth_far_maximum"] == "10.0"
    assert arguments["visual_frontend_mapping_depth_far_sparse_pixel_step"] == "8"
    assert arguments["visual_frontend_mapping_ground_prior_enabled"] == "true"
    assert arguments["visual_frontend_mapping_ground_surface_tolerance"] == "0.04"
    assert arguments["visual_frontend_mapping_ground_maximum_correction"] == "0.12"
    assert (
        arguments["visual_frontend_mapping_ground_angular_uncertainty_degrees"]
        == "2.0"
    )
    assert arguments["visual_frontend_mapping_ground_minimum_up_alignment"] == "0.55"
    assert arguments["visual_frontend_mapping_ground_minimum_row_ratio"] == "0.52"
    assert (
        arguments["visual_frontend_mapping_ground_reject_unverified_below_plane"]
        == "true"
    )
    assert (
        arguments["visual_frontend_mapping_ground_repair_all_below_plane"]
        == "false"
    )
    assert arguments["visual_frontend_mapping_ground_hole_fill_radius"] == "0"
    assert arguments["visual_frontend_mapping_ground_hole_fill_minimum_support"] == "0"
    assert arguments["visual_frontend_mapping_ground_hole_fill_maximum_depth"] == "0.0"
    assert arguments["visual_frontend_mapping_ground_filter_only"] == "true"
    assert "--Rtabmap/LoopThr 0.25" in arguments["rtabmap_args"]
    assert "--RGBD/OptimizeMaxError 1.0" in arguments["rtabmap_args"]
    assert "--Vis/MinInliers 30" in arguments["rtabmap_args"]
    assert "--Optimizer/Robust true" in arguments["rtabmap_args"]
    assert "--RGBD/LinearUpdate 0.06" in arguments["rtabmap_args"]
    assert "--RGBD/AngularUpdate 0.05" in arguments["rtabmap_args"]
    assert arguments["visual_frontend_mapping_ground_camera_height"] == "0.35"
    assert arguments["visual_frontend_mapping_ground_below_tolerance"] == "0.05"
    assert arguments["visual_frontend_mapping_ground_surface_tolerance"] == "0.04"
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
    assert '"body_imu_topic": LaunchConfiguration(' in launch
    assert '"model_family": LaunchConfiguration("model_family")' in launch
    assert '"left_right_model_path": LaunchConfiguration(' in launch
    assert 'DeclareLaunchArgument("wait_for_camera", default_value="true")' in launch
    assert "function=_launch_navigation_after_sensor_health" in launch
    assert "on_exit=_on_sensor_health_exit" in launch
    assert "TimerAction(" in launch
    assert "period=3.0" in launch
    assert "subprocess.run" not in launch
    assert "_wait_for_sensor_inputs" not in launch
    assert 'DeclareLaunchArgument("camera_x", default_value="0.20")' in launch
    assert 'DeclareLaunchArgument("camera_y", default_value="0.044982")' in launch
    assert 'DeclareLaunchArgument("camera_z", default_value="0.20")' in launch


def test_saved_map_navigation_health_gate_requires_h30_without_blocking_launch():
    module = _load(
        PACKAGE_ROOT
        / "launch"
        / "usb_crestereo_saved_map_navigation.launch.py",
        "usb_crestereo_saved_map_navigation_health_gate",
    )
    context = LaunchContext()
    context.launch_configurations.update({
        "wait_for_camera": "true",
        "camera_wait_timeout": "12.0",
        "use_imu": "true",
    })
    actions = module._launch_navigation_after_sensor_health(context)

    assert [type(action).__name__ for action in actions] == [
        "RegisterEventHandler",
        "ExecuteProcess",
    ]
    health_check = actions[1]
    command = [
        "".join(part.perform(context) for part in item)
        for item in health_check.process_description.cmd
    ]
    assert "rgbd_topic:=/sensors/rgbd/rgbd_image" in command
    assert "imu_topic:=/sensors/imu/data" in command
    assert "require_imu:=true" in command
    assert "timeout_sec:=12.0" in command

    class FailedHealth:
        returncode = 1

    with np.testing.assert_raises_regex(
        RuntimeError, "localization and navigation were not started"
    ):
        module._on_sensor_health_exit(FailedHealth(), context)

    class PassedHealth:
        returncode = 0

    success_actions = module._on_sensor_health_exit(PassedHealth(), context)
    assert [type(action).__name__ for action in success_actions] == [
        "LogInfo",
        "IncludeLaunchDescription",
    ]


def test_fast_foundation_saved_map_navigation_reuses_one_engine(tmp_path):
    module = _load(
        PACKAGE_ROOT
        / "launch"
        / "usb_fast_foundation_stereo_saved_map_navigation.launch.py",
        "usb_fast_foundation_stereo_saved_map_navigation_launch",
    )
    arguments = module._forwarded_arguments()

    assert module.DEFAULT_ENGINE_PATH == (
        "~/.cache/luxi/fast_foundation_stereo/ffs_320x192_i4_d96.engine"
    )
    assert arguments["model_family"] == "fast_foundation_stereo"
    assert arguments["crestereo_model_path"].describe() == (
        "LaunchConfig('model_path')"
    )
    assert arguments["crestereo_left_right_model_path"] == ""
    assert arguments["execution_provider"] == "tensorrt"
    assert arguments["database_path"].describe() == (
        "LaunchConfig('database_path')"
    )
    assert arguments["body_imu_topic"].describe() == (
        "LaunchConfig('body_imu_topic')"
    )

    context = LaunchContext()
    engine = tmp_path / "ffs.engine"
    engine.write_bytes(b"tensorrt")
    context.launch_configurations["model_path"] = str(engine)
    assert module._validate_fast_foundation_engine(context) == []

    empty_engine = tmp_path / "empty.engine"
    empty_engine.touch()
    context.launch_configurations["model_path"] = str(empty_engine)
    with np.testing.assert_raises_regex(RuntimeError, "engine file is empty"):
        module._validate_fast_foundation_engine(context)

    wrong_suffix = tmp_path / "model.onnx"
    wrong_suffix.write_bytes(b"not-a-tensorrt-engine")
    context.launch_configurations["model_path"] = str(wrong_suffix)
    with np.testing.assert_raises_regex(RuntimeError, "end in .engine"):
        module._validate_fast_foundation_engine(context)

    context.launch_configurations["model_path"] = str(tmp_path / "missing.engine")
    with np.testing.assert_raises_regex(
        RuntimeError, "install_fast_foundation_stereo_engine.py"
    ):
        module._validate_fast_foundation_engine(context)


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
    assert parameters["low_light_enhancement"] is True
    assert parameters["radiometric_confidence"] is True
    assert parameters["radiometric_analysis_width"] == 320
    assert parameters["minimum_depth_confidence"] == 0.54
    assert parameters["temporal_consistency"] is True
    assert parameters["output_median_filter_size"] == 1
    assert parameters["confidence_output_topic"] == "/usb_stereo/depth_confidence"
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


def test_fast_foundation_launch_reuses_learned_mapping_and_one_engine(tmp_path):
    module = _load(
        PACKAGE_ROOT / "launch" / "usb_fast_foundation_stereo_rtabmap.launch.py",
        "usb_fast_foundation_stereo_rtabmap_launch",
    )
    arguments = module._forwarded_arguments()

    assert module.DEFAULT_ENGINE_PATH == (
        "~/.cache/luxi/fast_foundation_stereo/ffs_320x192_i4_d96.engine"
    )
    assert arguments["model_family"] == "fast_foundation_stereo"
    assert arguments["crestereo_model_path"].describe() == (
        "LaunchConfig('model_path')"
    )
    assert arguments["crestereo_left_right_model_path"] == ""
    assert arguments["execution_provider"] == "tensorrt"
    assert arguments["crestereo_frontend_target_rate"] == "4.0"
    assert arguments["crestereo_mapping_rate"] == "2.0"
    assert arguments["crestereo_sensor_rgbd_topic"] == (
        "/sensors/rgbd/rgbd_image"
    )

    context = LaunchContext()
    engine = tmp_path / "ffs.engine"
    engine.write_bytes(b"tensorrt")
    context.launch_configurations["model_path"] = str(engine)
    assert module._validate_fast_foundation_engine(context) == []

    empty_engine = tmp_path / "empty.engine"
    empty_engine.touch()
    context.launch_configurations["model_path"] = str(empty_engine)
    with np.testing.assert_raises_regex(RuntimeError, "engine file is empty"):
        module._validate_fast_foundation_engine(context)

    context.launch_configurations["model_path"] = str(tmp_path / "missing.engine")
    with np.testing.assert_raises_regex(
        RuntimeError, "install_fast_foundation_stereo_engine.py"
    ):
        module._validate_fast_foundation_engine(context)


def test_fast_foundation_profile_keeps_raw_input_and_guided_disparity():
    parameters = yaml.safe_load(
        (
            DRIVER_ROOT
            / "config"
            / "fast_foundation_stereo_depth_mapping_quality.yaml"
        ).read_text(encoding="utf-8")
    )["usb_crestereo_depth_node"]["ros__parameters"]

    assert parameters["model_family"] == "fast_foundation_stereo"
    assert parameters["execution_provider"] == "tensorrt"
    assert parameters["output_width"] == 640
    assert parameters["output_height"] == 360
    assert parameters["target_rate"] == 4.0
    assert parameters["stereo_photometric_normalization"] is False
    assert parameters["low_light_enhancement"] is False
    assert parameters["left_right_consistency"] is True
    assert parameters["left_right_maximum_difference_px"] == 1.5
    assert parameters["left_right_model_path"] == ""
    assert parameters["radiometric_analysis_width"] == 320
    assert parameters["guided_upsampling_radius"] == 4
    assert parameters["guided_upsampling_epsilon"] == 100.0
    assert parameters["guided_upsampling_width"] == 640


def test_shared_camera_launch_forwards_model_family_to_depth_node():
    launch = (
        BRINGUP_ROOT / "launch" / "stereo_crestereo_rgbd.launch.py"
    ).read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("model_family", default_value="crestereo")' in launch
    assert '"model_family": LaunchConfiguration("model_family")' in launch


def test_base_preflight_accepts_one_fast_foundation_engine(tmp_path):
    module = _load(
        PACKAGE_ROOT / "launch" / "usb_crestereo_rtabmap.launch.py",
        "usb_crestereo_rtabmap_fast_foundation_preflight",
    )
    engine = tmp_path / "ffs.engine"
    engine.write_bytes(b"tensorrt")
    context = LaunchContext()
    context.launch_configurations["model_family"] = "fast_foundation_stereo"
    context.launch_configurations["crestereo_model_path"] = str(engine)
    # No left_right_model_path is required: the swapped pass reuses this
    # serialized session instead of loading a second engine into NX memory.
    assert module._validate_crestereo_models(context) == []

    context.launch_configurations["crestereo_model_path"] = str(
        tmp_path / "empty.engine"
    )
    Path(context.launch_configurations["crestereo_model_path"]).touch()
    with np.testing.assert_raises_regex(RuntimeError, "engine file is empty"):
        module._validate_crestereo_models(context)


def test_crestereo_shutdown_joins_native_workers_before_session_teardown():
    source = (DRIVER_ROOT / "scripts" / "crestereo_depth_node.py").read_text(
        encoding="utf-8"
    )

    assert source.count("daemon=False") >= 2
    assert "preprocess_pool.shutdown(wait=True, cancel_futures=True)" in source
