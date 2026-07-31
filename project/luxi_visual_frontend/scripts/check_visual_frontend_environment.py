#!/usr/bin/env python3
"""Validate model files and the selected Torch runtime."""

from pathlib import Path
import json
import os
import sys


WORKSPACE = Path(__file__).resolve().parents[3]
sys.path[:0] = [
    str(WORKSPACE / "3parts/hloc_gpu_python"),
    str(WORKSPACE / "3parts/hloc_python"),
]
os.environ.setdefault("TORCH_HOME", str(WORKSPACE / "3parts/hloc_models"))

import torch  # noqa: E402


def main() -> int:
    """Report CUDA and model availability, returning nonzero when incomplete."""
    models = {
        "superpoint": WORKSPACE
        / "3parts/hloc/third_party/SuperGluePretrainedNetwork/models/weights/superpoint_v1.pth",
        "lightglue": WORKSPACE
        / "3parts/hloc_models/hub/checkpoints/superpoint_lightglue_v0-1_arxiv.pth",
    }
    report = {
        "torch": torch.__version__,
        "cuda_available": torch.cuda.is_available(),
        "cuda_device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
        "torch_home": os.environ["TORCH_HOME"],
        "models": {
            name: {"path": str(path), "exists": path.is_file(), "bytes": path.stat().st_size if path.is_file() else 0}
            for name, path in models.items()
        },
    }
    print(json.dumps(report, indent=2))
    return 0 if report["cuda_available"] and all(item["exists"] for item in report["models"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
