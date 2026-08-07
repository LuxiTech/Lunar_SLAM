"""TensorRT execution support for the dense SuperPoint network."""

from __future__ import annotations

from pathlib import Path
from typing import Any


def engine_filename(height: int, width: int, precision: str = "fp32") -> str:
    """Return the hardware-specific engine name for a shape and precision."""
    if height <= 0 or width <= 0:
        raise ValueError("TensorRT input dimensions must be positive")
    if precision not in ("fp32", "fp16", "int8"):
        raise ValueError("TensorRT precision must be fp32, fp16 or int8")
    return f"superpoint_{height}x{width}_{precision}.engine"


def engine_profile(height: int, width: int, precision: str) -> tuple[int, int, str]:
    """Return the cache key after validating shape and precision."""
    engine_filename(height, width, precision)
    return height, width, precision


class TensorRTDenseSuperPoint:
    """Execute one fixed-shape TensorRT engine using Torch CUDA buffers."""

    def __init__(
        self, engine_path: Path, torch_module: Any, enable_cuda_graph: bool = False
    ) -> None:
        """Deserialize an engine and validate its FP32 input/output contract."""
        import tensorrt as trt

        self.torch = torch_module
        self.logger = trt.Logger(trt.Logger.ERROR)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.runtime.deserialize_cuda_engine(engine_path.read_bytes())
        if self.engine is None:
            raise RuntimeError(f"failed to deserialize TensorRT engine: {engine_path}")
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError(f"failed to create TensorRT context: {engine_path}")

        names = {
            self.engine.get_tensor_name(index)
            for index in range(self.engine.num_io_tensors)
        }
        expected = {"image", "raw_scores", "descriptors"}
        if names != expected:
            raise RuntimeError(
                f"unexpected TensorRT tensors: expected {sorted(expected)}, got {sorted(names)}"
            )
        for name in expected:
            if self.engine.get_tensor_dtype(name) != trt.float32:
                raise RuntimeError(f"TensorRT tensor {name} is not FP32")

        self.input_shape = tuple(self.engine.get_tensor_shape("image"))
        if len(self.input_shape) != 4 or any(dimension <= 0 for dimension in self.input_shape):
            raise RuntimeError(f"TensorRT engine must have a fixed input shape: {self.input_shape}")
        self.output_shapes = {
            name: tuple(self.engine.get_tensor_shape(name))
            for name in ("raw_scores", "descriptors")
        }
        self.outputs: dict[str, Any] | None = None
        self.enable_cuda_graph = enable_cuda_graph
        self.active_cuda_graph = False
        self._graph_disabled = False
        self._graph: Any | None = None
        self._graph_input: Any | None = None

    def _execute(self, image: Any) -> tuple[Any, Any]:
        """Enqueue TensorRT once on the current Torch CUDA stream."""
        if self.outputs is None:
            self.outputs = {
                name: self.torch.empty(
                    shape, dtype=self.torch.float32, device=image.device
                )
                for name, shape in self.output_shapes.items()
            }
            for name, output in self.outputs.items():
                self.context.set_tensor_address(name, output.data_ptr())
        self.context.set_tensor_address("image", image.data_ptr())
        stream = self.torch.cuda.current_stream(image.device).cuda_stream
        if not self.context.execute_async_v3(stream_handle=stream):
            raise RuntimeError("TensorRT SuperPoint inference failed")
        return self.outputs["raw_scores"], self.outputs["descriptors"]

    def _capture_cuda_graph(self, image: Any) -> None:
        """Capture the fixed-shape TensorRT enqueue after a safe warmup."""
        self._graph_input = self.torch.empty_like(image)
        self._graph_input.copy_(image)
        current = self.torch.cuda.current_stream(image.device)
        warmup = self.torch.cuda.Stream(device=image.device)
        warmup.wait_stream(current)
        with self.torch.cuda.stream(warmup):
            for _ in range(3):
                self._execute(self._graph_input)
        current.wait_stream(warmup)
        self.torch.cuda.synchronize(image.device)
        graph = self.torch.cuda.CUDAGraph()
        with self.torch.cuda.graph(graph):
            self._execute(self._graph_input)
        self._graph = graph
        self.active_cuda_graph = True

    def __call__(self, image: Any) -> tuple[Any, Any]:
        """Run inference without host/device copies and return Torch tensors."""
        if tuple(image.shape) != self.input_shape:
            raise ValueError(
                "TensorRT input shape mismatch: "
                f"expected {self.input_shape}, got {tuple(image.shape)}"
            )
        if image.device.type != "cuda" or image.dtype != self.torch.float32:
            raise ValueError("TensorRT SuperPoint input must be a CUDA FP32 tensor")
        image = image.contiguous()
        if self.enable_cuda_graph and not self._graph_disabled and self._graph is None:
            try:
                self._capture_cuda_graph(image)
            except Exception:
                self._graph_disabled = True
                self.active_cuda_graph = False
                self._graph = None
                self._graph_input = None
        if self._graph is not None:
            self._graph_input.copy_(image)
            self._graph.replay()
            assert self.outputs is not None
            return self.outputs["raw_scores"], self.outputs["descriptors"]
        return self._execute(image)
