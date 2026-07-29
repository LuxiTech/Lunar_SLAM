"""HLoc model loading, retrieval, matching and coarse pose estimation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any

import cv2
import h5py
import numpy as np

from .geometry import camera_matrix, invert_transform
from .map_io import read_frames, read_metadata
from .pose_estimator import CandidatePose, estimate_candidate


@dataclass(frozen=True)
class LocalizationOutput:
    accepted: bool
    reason: str
    map_from_camera: np.ndarray | None
    reference_name: str
    retrieval_score: float
    candidate: CandidatePose | None
    elapsed_seconds: float
    candidates_tested: int


class HlocFeatureBackend:
    """Own the three neural networks and run them on one configured device."""

    def __init__(
        self,
        device: str,
        resize_max: int,
        max_keypoints: int,
        cpu_threads: int,
    ) -> None:
        import torch
        from hloc import extractors, matchers
        from hloc.utils.base_model import dynamic_load

        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "cpu"
        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("device=cuda requested but CUDA PyTorch is unavailable")
        if device not in ("cpu", "cuda"):
            raise ValueError("device must be auto, cpu or cuda")
        if device == "cpu":
            torch.set_num_threads(max(1, cpu_threads))

        retrieval_conf = {"model": {"name": "netvlad"}}
        local_conf = {
            "model": {
                "name": "superpoint",
                "nms_radius": 3,
                "max_keypoints": max_keypoints,
            }
        }
        matcher_conf = {
            "model": {
                "name": "lightglue",
                "features": "superpoint",
                "depth_confidence": 0.90,
                "width_confidence": 0.95,
            }
        }

        retrieval_type = dynamic_load(extractors, retrieval_conf["model"]["name"])
        local_type = dynamic_load(extractors, local_conf["model"]["name"])
        matcher_type = dynamic_load(matchers, matcher_conf["model"]["name"])
        self.retrieval_model = retrieval_type(retrieval_conf["model"]).eval().to(device)
        self.local_model = local_type(local_conf["model"]).eval().to(device)
        self.matcher_model = matcher_type(matcher_conf["model"]).eval().to(device)
        self.torch = torch
        self.device = device
        self.resize_max = resize_max

    def _image_tensor(self, rgb: np.ndarray, grayscale: bool) -> tuple[Any, np.ndarray]:
        image = np.asarray(rgb)
        if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
            raise ValueError("RGB image must be uint8 HxWx3")
        original_size = np.array(image.shape[:2][::-1], dtype=np.float32)
        if max(original_size) > self.resize_max:
            scale = self.resize_max / float(max(original_size))
            new_size = tuple(int(round(value * scale)) for value in original_size)
            image = cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)
        if grayscale:
            image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)[None]
        else:
            image = image.transpose(2, 0, 1)
        tensor = self.torch.from_numpy(
            np.ascontiguousarray(image.astype(np.float32) / 255.0)
        ).unsqueeze(0)
        return tensor.to(self.device), original_size

    def extract(self, rgb: np.ndarray) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        with self.torch.inference_mode():
            retrieval_image, _ = self._image_tensor(rgb, False)
            retrieval = self.retrieval_model({"image": retrieval_image})
            local_image, original_size = self._image_tensor(rgb, True)
            local = self.local_model({"image": local_image})

        descriptor = retrieval["global_descriptor"][0].float().cpu().numpy()
        prediction = {name: value[0].float().cpu().numpy() for name, value in local.items()}
        processed_size = np.array(local_image.shape[-2:][::-1], dtype=np.float32)
        scales = original_size / processed_size
        prediction["keypoints"] = (prediction["keypoints"] + 0.5) * scales - 0.5
        prediction["image_size"] = original_size
        return descriptor, prediction

    def match(self, query: dict[str, np.ndarray], reference: dict[str, np.ndarray]) -> np.ndarray:
        torch = self.torch

        def feature_tensor(name: str, value: np.ndarray) -> Any:
            array = torch.from_numpy(np.asarray(value)).float().unsqueeze(0)
            return array.to(self.device) if not name.startswith("image") else array

        height0, width0 = int(query["image_size"][1]), int(query["image_size"][0])
        height1, width1 = int(reference["image_size"][1]), int(reference["image_size"][0])
        data = {
            "keypoints0": feature_tensor("keypoints0", query["keypoints"]),
            "descriptors0": feature_tensor("descriptors0", query["descriptors"]),
            "keypoint_scores0": feature_tensor(
                "keypoint_scores0", query.get("scores", query.get("keypoint_scores"))
            ),
            "image0": torch.empty((1, 1, height0, width0)),
            "keypoints1": feature_tensor("keypoints1", reference["keypoints"]),
            "descriptors1": feature_tensor("descriptors1", reference["descriptors"]),
            "keypoint_scores1": feature_tensor(
                "keypoint_scores1",
                reference.get("scores", reference.get("keypoint_scores")),
            ),
            "image1": torch.empty((1, 1, height1, width1)),
        }
        with torch.inference_mode():
            prediction = self.matcher_model(data)
        return prediction["matches0"][0].cpu().numpy().astype(np.int64)


class HlocLocalizer:
    """Retrieve map images, geometrically verify them, and return ``T_map_camera``."""

    def __init__(self, map_directory: Path, parameters: dict[str, Any]) -> None:
        self.map_directory = Path(map_directory)
        self.metadata = read_metadata(self.map_directory)
        self.frames = read_frames(self.map_directory / "frames.csv")
        self.reference_names = list(self.frames)
        self.backend = HlocFeatureBackend(
            parameters["device"],
            parameters["resize_max"],
            parameters["max_keypoints"],
            parameters["cpu_threads"],
        )
        self.parameters = parameters
        with h5py.File(self.map_directory / self.metadata["retrieval_features"], "r") as handle:
            self.reference_descriptors = np.stack(
                [
                    np.asarray(handle[name]["global_descriptor"], dtype=np.float32)
                    for name in self.reference_names
                ]
            )
        norms = np.linalg.norm(self.reference_descriptors, axis=1, keepdims=True)
        self.reference_descriptors /= np.maximum(norms, 1e-12)

    @property
    def device(self) -> str:
        return self.backend.device

    @staticmethod
    def _read_feature(group: h5py.Group) -> dict[str, np.ndarray]:
        return {name: np.asarray(value, dtype=np.float32) for name, value in group.items()}

    def localize(
        self,
        rgb: np.ndarray,
        depth: np.ndarray | None,
        query_intrinsics: np.ndarray,
        query_depth_scale: float,
        excluded_references: set[str] | None = None,
    ) -> LocalizationOutput:
        started = time.perf_counter()
        query_descriptor, query_features = self.backend.extract(rgb)
        query_descriptor /= max(float(np.linalg.norm(query_descriptor)), 1e-12)
        scores = self.reference_descriptors @ query_descriptor
        for name in excluded_references or ():
            if name in self.frames:
                scores[self.reference_names.index(name)] = float("-inf")
        top_k = min(self.parameters["top_k"], len(scores))
        candidate_indices = np.argsort(scores)[-top_k:][::-1]
        best_reason = "NO_CANDIDATE"
        best_name = ""
        best_score = float("-inf")
        best_candidate: CandidatePose | None = None
        tested_count = 0

        local_path = self.map_directory / self.metadata["local_features"]
        landmark_path = self.map_directory / self.metadata["landmarks"]
        with h5py.File(local_path, "r") as local_file, h5py.File(landmark_path, "r") as landmarks:
            for tested, candidate_index in enumerate(candidate_indices, start=1):
                tested_count = tested
                name = self.reference_names[int(candidate_index)]
                score = float(scores[candidate_index])
                if score < self.parameters["minimum_retrieval_score"]:
                    best_reason = "RETRIEVAL_SCORE_LOW"
                    break
                reference_features = self._read_feature(local_file[name])
                matches = self.backend.match(query_features, reference_features)
                query_indices = np.flatnonzero(matches >= 0)
                reference_indices = matches[query_indices]
                landmark_group = landmarks[name]
                points3d = np.asarray(landmark_group["points3d"], dtype=np.float64)
                valid_landmarks = np.asarray(landmark_group["valid"], dtype=np.bool_)
                valid_matches = valid_landmarks[reference_indices]
                query_indices = query_indices[valid_matches]
                reference_indices = reference_indices[valid_matches]
                map_points = points3d[reference_indices]
                query_points = query_features["keypoints"][query_indices].astype(np.float64)
                result = estimate_candidate(
                    map_points,
                    query_points,
                    query_intrinsics,
                    depth,
                    query_depth_scale,
                    match_count=int(np.count_nonzero(matches >= 0)),
                    minimum_matches=self.parameters["minimum_matches"],
                    minimum_landmarks=self.parameters["minimum_landmarks"],
                    minimum_inliers=self.parameters["minimum_inliers"],
                    minimum_inlier_ratio=self.parameters["minimum_inlier_ratio"],
                    maximum_reprojection_rmse=self.parameters["maximum_reprojection_rmse"],
                    ransac_reprojection_error=self.parameters["ransac_reprojection_error"],
                    ransac_iterations=self.parameters["ransac_iterations"],
                    ransac_confidence=self.parameters["ransac_confidence"],
                    require_depth_verification=self.parameters["require_depth_verification"],
                    minimum_depth_verified=self.parameters["minimum_depth_verified"],
                    maximum_median_depth_residual=self.parameters[
                        "maximum_median_depth_residual"
                    ],
                    minimum_depth=self.parameters["minimum_depth"],
                    maximum_depth=self.parameters["maximum_depth"],
                )
                if score > best_score:
                    best_name = name
                    best_score = score
                    best_reason = result.reason
                    best_candidate = result
                if result.accepted and result.pnp is not None:
                    return LocalizationOutput(
                        True,
                        "ACCEPTED",
                        invert_transform(result.pnp.camera_from_map),
                        name,
                        score,
                        result,
                        time.perf_counter() - started,
                        tested,
                    )
        return LocalizationOutput(
            False,
            best_reason,
            None,
            best_name,
            best_score,
            best_candidate,
            time.perf_counter() - started,
            tested_count,
        )


def intrinsics_from_values(fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    return camera_matrix(fx, fy, cx, cy)
