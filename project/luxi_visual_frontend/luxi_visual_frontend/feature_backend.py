"""GPU SuperPoint extraction and LightGlue matching."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


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
        if resize_max <= 0 or max_keypoints <= 0:
            raise ValueError("resize_max and max_keypoints must be positive")
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
            }
        }
        local_type = dynamic_load(extractors, local_configuration["model"]["name"])
        matcher_type = dynamic_load(matchers, matcher_configuration["model"]["name"])
        self.local_model = local_type(local_configuration["model"]).eval().to(device)
        self.matcher_model = matcher_type(matcher_configuration["model"]).eval().to(device)
        self.torch = torch
        self.device = device
        self.resize_max = resize_max

    def extract(self, rgb: np.ndarray) -> NeuralFeatures:
        """Extract features and scale their coordinates back to the input image."""
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
            prediction = self.local_model({"image": tensor})
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
        return NeuralFeatures(
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

    def match(self, first: NeuralFeatures, second: NeuralFeatures) -> np.ndarray:
        """Return the second-feature index for every first feature, or -1."""
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

        height0, width0 = int(first.image_size[1]), int(first.image_size[0])
        height1, width1 = int(second.image_size[1]), int(second.image_size[0])
        keypoints0, descriptors0, scores0 = device_features(first)
        keypoints1, descriptors1, scores1 = device_features(second)
        data = {
            "keypoints0": keypoints0,
            "descriptors0": descriptors0,
            "keypoint_scores0": scores0,
            "image0": torch.empty((1, 1, height0, width0), device=self.device),
            "keypoints1": keypoints1,
            "descriptors1": descriptors1,
            "keypoint_scores1": scores1,
            "image1": torch.empty((1, 1, height1, width1), device=self.device),
        }
        with torch.inference_mode():
            prediction = self.matcher_model(data)
        return prediction["matches0"][0].cpu().numpy().astype(np.int64)
