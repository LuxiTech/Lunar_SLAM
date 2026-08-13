"""Static contracts for the independent pre-trained CREStereo experiment."""

import importlib.util
from pathlib import Path

import numpy as np
import onnx
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
    assert arguments["visual_frontend_target_rate"] == "6.0"
    assert arguments["visual_frontend_upstream_rate_limited"] == "true"
    assert arguments["visual_frontend_depth_sampling_radius"] == "2"
    assert arguments["visual_frontend_use_depth_translation_refinement"] == "true"
    assert arguments["visual_frontend_maximum_depth"] == "4.0"
    assert arguments["visual_frontend_mapping_depth_dense_maximum"] == "4.0"
    assert arguments["visual_frontend_mapping_depth_medium_maximum"] == "6.0"
    assert arguments["visual_frontend_mapping_depth_medium_sparse_pixel_step"] == "4"
    assert arguments["visual_frontend_mapping_depth_far_maximum"] == "10.0"
    assert arguments["visual_frontend_mapping_depth_far_sparse_pixel_step"] == "8"
    assert "--Kp/MaxDepth 4.0" in arguments["rtabmap_args"]
    assert "--Grid/RangeMax 10.0" in arguments["rtabmap_args"]
    assert "--Grid/NoiseFilteringRadius 0.35" in arguments["rtabmap_args"]
    assert "--Grid/NoiseFilteringMinNeighbors 3" in arguments["rtabmap_args"]


def test_crestereo_profile_uses_h30_by_default():
    text = (
        PACKAGE_ROOT / "launch" / "usb_crestereo_rtabmap.launch.py"
    ).read_text(encoding="utf-8")

    assert "*_BASE._common_launch_arguments()" in text
    assert '"start_imu": use_imu' in text
    assert '"enable_imu": ParameterValue(use_imu' in text
