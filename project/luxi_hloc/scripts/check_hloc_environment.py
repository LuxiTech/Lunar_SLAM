#!/usr/bin/env python3
"""Report whether the isolated HLoc runtime is CPU- or CUDA-capable."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys


WORKSPACE = Path(__file__).resolve().parents[3]
RUNTIME = os.environ.get("LUXI_HLOC_RUNTIME", "auto").lower()
if RUNTIME not in {"auto", "gpu", "cpu"}:
    raise RuntimeError("LUXI_HLOC_RUNTIME must be one of: auto, gpu, cpu")
dependencies = [
    WORKSPACE / "3parts/hloc_python",
    WORKSPACE / "3parts/hloc",
    WORKSPACE / "3parts/lightglue",
]
gpu_runtime = WORKSPACE / "3parts/hloc_gpu_python"
if RUNTIME != "cpu" and gpu_runtime.is_dir():
    dependencies.insert(0, gpu_runtime)
elif RUNTIME == "gpu":
    raise RuntimeError(f"GPU runtime not found: {gpu_runtime}")
sys.path[:0] = [str(dependency) for dependency in dependencies]
os.environ.setdefault("TORCH_HOME", str(WORKSPACE / "3parts/hloc_models"))

import cv2  # noqa: E402
import h5py  # noqa: E402
import torch  # noqa: E402
from hloc import __version__ as hloc_version  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--require-cuda", action="store_true")
    arguments = parser.parse_args()
    cuda_available = torch.cuda.is_available()
    report = {
        "python": sys.version.split()[0],
        "architecture": os.uname().machine,
        "torch": torch.__version__,
        "torch_cuda_build": torch.version.cuda,
        "cuda_available": cuda_available,
        "cuda_device": torch.cuda.get_device_name(0) if cuda_available else None,
        "hloc": hloc_version,
        "opencv": cv2.__version__,
        "h5py": h5py.__version__,
        "torch_home": os.environ["TORCH_HOME"],
        "runtime_selection": RUNTIME,
        "torch_location": torch.__file__,
    }
    print(json.dumps(report, indent=2))
    return 1 if arguments.require_cuda and not cuda_available else 0


if __name__ == "__main__":
    raise SystemExit(main())
