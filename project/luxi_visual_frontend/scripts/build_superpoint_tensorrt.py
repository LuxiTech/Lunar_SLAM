#!/usr/bin/env python3
"""Build a fixed-shape TensorRT engine for the bundled SuperPoint model."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile


WORKSPACE = Path(__file__).resolve().parents[3]
sys.path[:0] = [
    str(WORKSPACE / "3parts/hloc_gpu_python"),
    str(WORKSPACE / "3parts/hloc_python"),
    str(WORKSPACE / "3parts/hloc"),
    str(WORKSPACE / "3parts/hloc/third_party"),
    str(WORKSPACE / "project/luxi_visual_frontend"),
]
os.environ.setdefault("TORCH_HOME", str(WORKSPACE / "3parts/hloc_models"))

import torch  # noqa: E402
from hloc import extractors  # noqa: E402
from hloc.utils.base_model import dynamic_load  # noqa: E402

from luxi_visual_frontend.tensorrt_superpoint import engine_filename  # noqa: E402


class DenseSuperPoint(torch.nn.Module):
    """Export the static convolution, score and descriptor portion of SuperPoint."""

    def __init__(self, network: torch.nn.Module) -> None:
        """Wrap the weight-loaded PyTorch SuperPoint network for ONNX export."""
        super().__init__()
        self.network = network

    def forward(self, image: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
        """Return dense scores before NMS and L2-normalized dense descriptors."""
        network = self.network
        features = network.relu(network.conv1a(image))
        features = network.relu(network.conv1b(features))
        features = network.pool(features)
        features = network.relu(network.conv2a(features))
        features = network.relu(network.conv2b(features))
        features = network.pool(features)
        features = network.relu(network.conv3a(features))
        features = network.relu(network.conv3b(features))
        features = network.pool(features)
        features = network.relu(network.conv4a(features))
        features = network.relu(network.conv4b(features))

        scores = network.convPb(network.relu(network.convPa(features)))
        scores = torch.nn.functional.softmax(scores, 1)[:, :-1]
        batch, _, height, width = scores.shape
        scores = scores.permute(0, 2, 3, 1).reshape(batch, height, width, 8, 8)
        scores = scores.permute(0, 1, 3, 2, 4).reshape(
            batch, height * 8, width * 8
        )

        descriptors = network.convDb(network.relu(network.convDa(features)))
        descriptors = torch.nn.functional.normalize(descriptors, p=2, dim=1)
        return scores, descriptors


def parse_args() -> argparse.Namespace:
    """Parse the fixed input shape and destination."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--height", type=int, required=True)
    parser.add_argument("--width", type=int, required=True)
    parser.add_argument("--workspace-mib", type=int, default=256)
    parser.add_argument(
        "--precision", choices=("fp32", "fp16", "int8"), default="fp32"
    )
    parser.add_argument(
        "--calibration-data",
        type=Path,
        help="NPZ containing float32 NCHW 'images'; required for INT8",
    )
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def build_int8_engine(
    onnx_path: Path,
    output: Path,
    calibration_path: Path,
    workspace_mib: int,
    expected_shape: tuple[int, int, int, int],
) -> None:
    """Build a calibrated TensorRT INT8 engine with FP32 network I/O."""
    import tensorrt as trt
    from cuda import cudart
    import numpy as np

    calibration_archive = np.load(calibration_path)
    if "images" not in calibration_archive:
        raise ValueError("calibration NPZ must contain an 'images' array")
    images = np.ascontiguousarray(calibration_archive["images"], dtype=np.float32)
    if images.ndim != 4 or tuple(images.shape[1:]) != expected_shape[1:]:
        raise ValueError(
            f"calibration shape must be Nx{expected_shape[1:]}, got {images.shape}"
        )
    if len(images) < 16:
        raise ValueError("INT8 calibration requires at least 16 images")
    if not np.isfinite(images).all() or images.min() < 0.0 or images.max() > 1.0:
        raise ValueError("calibration images must be finite and normalized to [0, 1]")

    cache_path = output.with_suffix(".calibration.cache")

    class ImageCalibrator(trt.IInt8EntropyCalibrator2):
        """Feed normalized HIK images to TensorRT's entropy calibrator."""

        def __init__(self) -> None:
            """Allocate one reusable CUDA calibration input."""
            super().__init__()
            self.index = 0
            error, pointer = cudart.cudaMalloc(images[0].nbytes)
            if error != cudart.cudaError_t.cudaSuccess:
                raise RuntimeError(f"cudaMalloc failed: {error}")
            self.pointer = pointer

        def get_batch_size(self) -> int:
            """Calibrate one image per batch to minimize device memory."""
            return 1

        def get_batch(self, names: list[str]) -> list[int] | None:
            """Copy the next representative image to the calibration input."""
            if names != ["image"]:
                raise RuntimeError(f"unexpected calibration inputs: {names}")
            if self.index >= len(images):
                return None
            image = images[self.index]
            self.index += 1
            error = cudart.cudaMemcpy(
                self.pointer,
                image.ctypes.data,
                image.nbytes,
                cudart.cudaMemcpyKind.cudaMemcpyHostToDevice,
            )[0]
            if error != cudart.cudaError_t.cudaSuccess:
                raise RuntimeError(f"cudaMemcpy failed: {error}")
            return [int(self.pointer)]

        def read_calibration_cache(self) -> bytes | None:
            """Reuse an existing cache generated from the same destination."""
            return cache_path.read_bytes() if cache_path.is_file() else None

        def write_calibration_cache(self, cache: bytes) -> None:
            """Persist activation scales next to the hardware-specific engine."""
            cache_path.write_bytes(cache)

        def close(self) -> None:
            """Release the reusable CUDA input."""
            if self.pointer:
                cudart.cudaFree(self.pointer)
                self.pointer = 0

    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, logger)
    if not parser.parse(onnx_path.read_bytes()):
        errors = [str(parser.get_error(index)) for index in range(parser.num_errors)]
        raise RuntimeError("failed to parse ONNX:\n" + "\n".join(errors))
    config = builder.create_builder_config()
    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE, workspace_mib * 1024 * 1024
    )
    config.set_flag(trt.BuilderFlag.INT8)
    calibrator = ImageCalibrator()
    config.int8_calibrator = calibrator
    try:
        serialized = builder.build_serialized_network(network, config)
        if serialized is None:
            raise RuntimeError("TensorRT INT8 engine build failed")
        output.write_bytes(serialized)
    finally:
        calibrator.close()


def main() -> int:
    """Export ONNX and build the selected precision for this Jetson."""
    args = parse_args()
    filename = engine_filename(args.height, args.width, args.precision)
    output = args.output or Path(os.environ["TORCH_HOME"]) / "tensorrt" / filename
    output.parent.mkdir(parents=True, exist_ok=True)

    local_type = dynamic_load(extractors, "superpoint")
    model = local_type(
        {"name": "superpoint", "nms_radius": 3, "max_keypoints": 2048}
    ).eval().cuda()
    dense_model = DenseSuperPoint(model.net).eval().cuda()
    sample = torch.zeros((1, 1, args.height, args.width), device="cuda")

    trtexec = Path("/usr/src/tensorrt/bin/trtexec")
    if not trtexec.is_file():
        raise RuntimeError(f"TensorRT builder not found: {trtexec}")
    with tempfile.TemporaryDirectory(prefix="luxi_superpoint_") as directory:
        onnx_path = Path(directory) / "superpoint.onnx"
        with torch.inference_mode():
            torch.onnx.export(
                dense_model,
                sample,
                onnx_path,
                input_names=["image"],
                output_names=["raw_scores", "descriptors"],
                opset_version=17,
                do_constant_folding=True,
            )
        if args.precision == "int8":
            if args.calibration_data is None:
                raise ValueError("--calibration-data is required for INT8")
            build_int8_engine(
                onnx_path,
                output,
                args.calibration_data,
                args.workspace_mib,
                (1, 1, args.height, args.width),
            )
        else:
            command = [
                str(trtexec),
                f"--onnx={onnx_path}",
                f"--saveEngine={output}",
                f"--memPoolSize=workspace:{args.workspace_mib}",
                "--skipInference",
            ]
            command.append("--noTF32" if args.precision == "fp32" else "--fp16")
            subprocess.run(command, check=True)
    print(f"Built {args.precision.upper()} TensorRT engine: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
