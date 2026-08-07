"""GPU SuperPoint extraction and LightGlue matching."""

from __future__ import annotations

from dataclasses import dataclass
import copy
import os
from pathlib import Path
import time
from typing import Any
import warnings

import cv2
import numpy as np

from .tensorrt_superpoint import (
    TensorRTDenseSuperPoint,
    engine_filename,
    engine_profile,
)


@dataclass(frozen=True)
class NeuralFeatures:
    """SuperPoint features in original image coordinates."""

    keypoints: np.ndarray
    descriptors: np.ndarray | None
    scores: np.ndarray
    image_size: np.ndarray
    device_data: dict[str, Any] | None = None

    def descriptor_array(self) -> np.ndarray:
        """Materialize RTAB descriptors only when a message is published."""
        if self.descriptors is not None:
            return np.asarray(self.descriptors, dtype=np.float32)
        if self.device_data is None:
            raise RuntimeError("features contain no descriptors")
        return (
            self.device_data["descriptors"].transpose(0, 1).detach().cpu().numpy()
            .astype(np.float32, copy=False)
        )


class SuperPointLightGlueBackend:
    """Own SuperPoint and LightGlue on one configured Torch device."""

    def __init__(
        self,
        device: str,
        resize_max: int,
        max_keypoints: int,
        nms_radius: int,
        depth_confidence: float,
        width_confidence: float,
        cpu_threads: int,
        superpoint_backend: str = "auto",
        superpoint_engine_directory: str = "",
        superpoint_precision: str = "fp32",
        lightglue_mixed_precision: bool = False,
        superpoint_cuda_graph: bool = True,
        lightglue_cuda_graph: bool = True,
        lightglue_cuda_graph_keypoints: int = 512,
        lightglue_cuda_graph_layers: int = 3,
    ) -> None:
        """Load SuperPoint and LightGlue on the requested Torch device."""
        import torch
        from hloc import extractors, matchers
        from hloc.utils.base_model import dynamic_load

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("device=cuda requested but CUDA PyTorch is unavailable")
        if device not in ("cpu", "cuda"):
            raise ValueError("device must be auto, cpu or cuda")
        if superpoint_backend not in ("auto", "pytorch", "tensorrt"):
            raise ValueError("superpoint_backend must be auto, pytorch or tensorrt")
        if superpoint_backend == "tensorrt" and device != "cuda":
            raise ValueError("superpoint_backend=tensorrt requires device=cuda")
        if superpoint_precision not in ("fp32", "fp16", "int8"):
            raise ValueError("superpoint_precision must be fp32, fp16 or int8")
        if resize_max <= 0 or max_keypoints <= 0:
            raise ValueError("resize_max and max_keypoints must be positive")
        if lightglue_cuda_graph_keypoints <= 0:
            raise ValueError("lightglue_cuda_graph_keypoints must be positive")
        if lightglue_cuda_graph_layers <= 0:
            raise ValueError("lightglue_cuda_graph_layers must be positive")
        if device == "cpu":
            torch.set_num_threads(max(1, cpu_threads))

        local_configuration = {
            "model": {
                "name": "superpoint",
                "nms_radius": nms_radius,
                "max_keypoints": max_keypoints,
            }
        }
        matcher_configuration = {
            "model": {
                "name": "lightglue",
                "features": "superpoint",
                "depth_confidence": depth_confidence,
                "width_confidence": width_confidence,
                "mp": bool(lightglue_mixed_precision and device == "cuda"),
            }
        }
        local_type = dynamic_load(extractors, local_configuration["model"]["name"])
        matcher_type = dynamic_load(matchers, matcher_configuration["model"]["name"])
        self.local_model = local_type(local_configuration["model"]).eval().to(device)
        self.matcher_model = matcher_type(matcher_configuration["model"]).eval().to(device)
        self.torch = torch
        self.device = device
        self.resize_max = resize_max
        self.superpoint_backend = superpoint_backend
        self.superpoint_precision = superpoint_precision
        self.lightglue_mixed_precision = bool(
            lightglue_mixed_precision and device == "cuda"
        )
        self.superpoint_cuda_graph = bool(superpoint_cuda_graph and device == "cuda")
        self.lightglue_cuda_graph = bool(
            lightglue_cuda_graph and self.lightglue_mixed_precision
        )
        self.lightglue_cuda_graph_keypoints = lightglue_cuda_graph_keypoints
        self.lightglue_cuda_graph_layers = lightglue_cuda_graph_layers
        if superpoint_engine_directory:
            self.superpoint_engine_directory = Path(superpoint_engine_directory)
        else:
            torch_home = os.environ.get("TORCH_HOME")
            self.superpoint_engine_directory = (
                Path(torch_home) / "tensorrt" if torch_home else None
            )
        self._tensorrt_runners: dict[
            tuple[int, int, str], TensorRTDenseSuperPoint
        ] = {}
        self._unavailable_tensorrt_profiles: set[tuple[int, int, str]] = set()
        self.active_superpoint_backend = "pytorch"
        self.active_lightglue_backend = (
            "pytorch_amp_fp16" if self.lightglue_mixed_precision else "pytorch_fp32"
        )
        self.last_extract_seconds = 0.0
        self.last_match_seconds = 0.0
        self.last_match_layers = 0
        self._matcher_placeholder = torch.empty((1, 1, 1, 1), device=self.device)
        if self.lightglue_cuda_graph:
            self._enable_lightglue_cuda_graph()

    def _enable_lightglue_cuda_graph(self) -> None:
        """Graph the common small temporal-matching path with eager fallback."""
        torch = self.torch
        network = self.matcher_model.net
        layer_count = min(self.lightglue_cuda_graph_layers, len(network.transformers))
        point_count = self.lightglue_cuda_graph_keypoints
        descriptor_dim = int(network.conf.descriptor_dim)
        head_count = int(network.conf.num_heads)
        head_dim = descriptor_dim // head_count

        class CudaGraphDispatch(torch.nn.Module):
            """Use a fixed-shape graph only when LightGlue selected its bucket."""

            def __init__(self, eager: Any, graphed: Any, static_points: int) -> None:
                super().__init__()
                self.eager = eager
                self.graphed = graphed
                self.static_points = static_points

            def forward(
                self,
                desc0: Any,
                desc1: Any,
                encoding0: Any,
                encoding1: Any,
                mask0: Any = None,
                mask1: Any = None,
            ) -> Any:
                arguments = (desc0, desc1, encoding0, encoding1, mask0, mask1)
                if (
                    mask0 is not None
                    and desc0.shape[-2] == self.static_points
                    and desc1.shape[-2] == self.static_points
                ):
                    return self.graphed(*arguments)
                return self.eager(*arguments)

        sample = (
            torch.zeros(
                1, point_count, descriptor_dim, device=self.device, dtype=torch.float16
            ),
            torch.zeros(
                1, point_count, descriptor_dim, device=self.device, dtype=torch.float16
            ),
            torch.zeros(
                2,
                1,
                head_count,
                point_count,
                head_dim,
                device=self.device,
                dtype=torch.float16,
            ),
            torch.zeros(
                2,
                1,
                head_count,
                point_count,
                head_dim,
                device=self.device,
                dtype=torch.float16,
            ),
            torch.ones(1, point_count, 1, device=self.device, dtype=torch.bool),
            torch.ones(1, point_count, 1, device=self.device, dtype=torch.bool),
        )
        replacements = list(network.transformers)
        try:
            pool = torch.cuda.graph_pool_handle()
            with torch.inference_mode(), torch.autocast(
                device_type="cuda", enabled=True, cache_enabled=False
            ):
                for index in range(layer_count):
                    eager = network.transformers[index]
                    graphed = torch.cuda.make_graphed_callables(
                        copy.deepcopy(eager).eval(),
                        sample,
                        num_warmup_iters=3,
                        pool=pool,
                    )
                    replacements[index] = CudaGraphDispatch(
                        eager, graphed, point_count
                    )
            network.transformers = torch.nn.ModuleList(replacements)
            network.static_lengths = [point_count]
            self.active_lightglue_backend = (
                f"pytorch_amp_fp16_cudagraph_{point_count}x{layer_count}"
            )
        except Exception as error:
            warnings.warn(
                f"LightGlue CUDA Graph unavailable; using eager execution: {error}",
                RuntimeWarning,
                stacklevel=2,
            )

    def _tensorrt_runner(self, height: int, width: int) -> TensorRTDenseSuperPoint | None:
        """Load the fixed-shape engine once, or select the configured fallback."""
        profile = engine_profile(height, width, self.superpoint_precision)
        if (
            self.superpoint_backend == "pytorch"
            or profile in self._unavailable_tensorrt_profiles
        ):
            return None
        runner = self._tensorrt_runners.get(profile)
        if runner is not None:
            return runner
        directory = self.superpoint_engine_directory
        engine_path = (
            directory / engine_filename(height, width, self.superpoint_precision)
            if directory
            else None
        )
        if engine_path is None or not engine_path.is_file():
            if self.superpoint_backend == "tensorrt":
                raise RuntimeError(f"TensorRT SuperPoint engine not found: {engine_path}")
            self._unavailable_tensorrt_profiles.add(profile)
            return None
        try:
            runner = TensorRTDenseSuperPoint(
                engine_path, self.torch, self.superpoint_cuda_graph
            )
        except Exception:
            if self.superpoint_backend == "tensorrt":
                raise
            self._unavailable_tensorrt_profiles.add(profile)
            return None
        self._tensorrt_runners[profile] = runner
        return runner

    def _postprocess_superpoint(
        self, raw_scores: Any, dense_descriptors: Any
    ) -> dict[str, Any]:
        """Apply the original FP32 SuperPoint NMS and descriptor sampling."""
        from SuperGluePretrainedNetwork.models.superpoint import (
            remove_borders,
            sample_descriptors,
            simple_nms,
            top_k_keypoints,
        )

        network = self.local_model.net
        scores_map = simple_nms(raw_scores, network.config["nms_radius"])
        _, _, height, width = dense_descriptors.shape
        keypoints = [
            self.torch.nonzero(scores > network.config["keypoint_threshold"])
            for scores in scores_map
        ]
        scores = [
            score_map[tuple(points.t())]
            for score_map, points in zip(scores_map, keypoints)
        ]
        keypoints, scores = list(
            zip(
                *[
                    remove_borders(
                        points,
                        point_scores,
                        network.config["remove_borders"],
                        height * 8,
                        width * 8,
                    )
                    for points, point_scores in zip(keypoints, scores)
                ]
            )
        )
        if network.config["max_keypoints"] >= 0:
            keypoints, scores = list(
                zip(
                    *[
                        top_k_keypoints(
                            points, point_scores, network.config["max_keypoints"]
                        )
                        for points, point_scores in zip(keypoints, scores)
                    ]
                )
            )
        keypoints = [self.torch.flip(points, [1]).float() for points in keypoints]
        descriptors = [
            sample_descriptors(points[None], dense[None], 8)[0]
            for points, dense in zip(keypoints, dense_descriptors)
        ]
        return {"keypoints": keypoints, "scores": scores, "descriptors": descriptors}

    def extract(self, rgb: np.ndarray) -> NeuralFeatures:
        """Extract features and scale their coordinates back to the input image."""
        started = time.perf_counter()
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("RGB image must be uint8 HxWx3")
        original_size = np.array(image.shape[:2][::-1], dtype=np.float32)
        if max(original_size) > self.resize_max:
            scale = self.resize_max / float(max(original_size))
            new_size = tuple(int(round(value * scale)) for value in original_size)
            image = cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)
        grayscale = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)[None]
        tensor = self.torch.from_numpy(
            np.ascontiguousarray(grayscale.astype(np.float32) / 255.0)
        ).unsqueeze(0).to(self.device)
        with self.torch.inference_mode():
            runner = self._tensorrt_runner(int(tensor.shape[-2]), int(tensor.shape[-1]))
            if runner is None:
                prediction = self.local_model({"image": tensor})
                self.active_superpoint_backend = "pytorch"
            else:
                prediction = self._postprocess_superpoint(*runner(tensor))
                suffix = "_cudagraph" if runner.active_cuda_graph else ""
                self.active_superpoint_backend = (
                    f"tensorrt_{self.superpoint_precision}{suffix}"
                )
        processed_size = np.array(tensor.shape[-2:][::-1], dtype=np.float32)
        scales = original_size / processed_size
        keypoints_device = prediction["keypoints"][0].float()
        scale_device = self.torch.as_tensor(scales, device=self.device)
        keypoints_device = (keypoints_device + 0.5) * scale_device - 0.5
        descriptors_device = prediction["descriptors"][0].float()
        scores_device = prediction.get("scores", prediction.get("keypoint_scores"))[0].float()
        keypoints = keypoints_device.cpu().numpy()
        scores = scores_device.cpu().numpy()
        if descriptors_device.shape != (256, len(keypoints)):
            raise RuntimeError(
                f"unexpected SuperPoint descriptor shape: {tuple(descriptors_device.shape)}"
            )
        features = NeuralFeatures(
            keypoints.astype(np.float32),
            None,
            np.asarray(scores, dtype=np.float32),
            original_size,
            {
                "keypoints": keypoints_device,
                "descriptors": descriptors_device,
                "scores": scores_device,
            },
        )
        self.last_extract_seconds = time.perf_counter() - started
        return features

    def match(self, first: NeuralFeatures, second: NeuralFeatures) -> np.ndarray:
        """Return the second-feature index for every first feature, or -1."""
        started = time.perf_counter()
        torch = self.torch

        def tensor(value: np.ndarray) -> Any:
            return torch.from_numpy(np.asarray(value)).float().unsqueeze(0).to(self.device)

        def device_features(features: NeuralFeatures) -> tuple[Any, Any, Any]:
            if features.device_data is not None:
                return (
                    features.device_data["keypoints"].unsqueeze(0),
                    features.device_data["descriptors"].unsqueeze(0),
                    features.device_data["scores"].unsqueeze(0),
                )
            descriptors = features.descriptor_array()
            return tensor(features.keypoints), tensor(descriptors.T), tensor(features.scores)

        keypoints0, descriptors0, scores0 = device_features(first)
        keypoints1, descriptors1, scores1 = device_features(second)
        data = {
            "keypoints0": keypoints0,
            "descriptors0": descriptors0,
            "keypoint_scores0": scores0,
            # The bundled LightGlue only checks for this field; it normalizes
            # from keypoint extents and never reads image pixels or shape.
            "image0": self._matcher_placeholder,
            "keypoints1": keypoints1,
            "descriptors1": descriptors1,
            "keypoint_scores1": scores1,
            "image1": self._matcher_placeholder,
        }
        with torch.inference_mode():
            prediction = self.matcher_model(data)
        matches = prediction["matches0"][0].cpu().numpy().astype(np.int64)
        self.last_match_layers = int(prediction.get("stop", 0))
        self.last_match_seconds = time.perf_counter() - started
        return matches
