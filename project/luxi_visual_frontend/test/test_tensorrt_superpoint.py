"""Tests for TensorRT SuperPoint engine selection."""

import pytest

from luxi_visual_frontend.tensorrt_superpoint import engine_filename, engine_profile


def test_engine_filename_includes_fixed_shape_and_precision() -> None:
    assert engine_filename(586, 800) == "superpoint_586x800_fp32.engine"
    assert engine_filename(586, 800, "fp16") == "superpoint_586x800_fp16.engine"
    assert engine_filename(586, 800, "int8") == "superpoint_586x800_int8.engine"


@pytest.mark.parametrize("height,width", [(0, 800), (586, 0), (-1, 800)])
def test_engine_filename_rejects_invalid_shape(height: int, width: int) -> None:
    with pytest.raises(ValueError, match="positive"):
        engine_filename(height, width)


def test_engine_filename_rejects_unknown_precision() -> None:
    with pytest.raises(ValueError, match="precision"):
        engine_filename(586, 800, "fp8")


def test_engine_profile_keeps_precision_in_cache_key() -> None:
    assert engine_profile(586, 800, "fp32") != engine_profile(586, 800, "fp16")
