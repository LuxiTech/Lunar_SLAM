"""Encode OpenCV matrices using RTAB-Map's ``compressData`` wire format."""

from __future__ import annotations

import struct
import zlib

import numpy as np


_OPENCV_TYPES = {
    np.dtype(np.uint8): 0,   # CV_8UC1
    np.dtype(np.float32): 5,  # CV_32FC1
}


def compress_descriptor_matrix(descriptors: np.ndarray) -> bytes:
    """Return zlib data followed by native int rows, columns and cv::Mat type."""
    matrix = np.asarray(descriptors)
    if matrix.ndim != 2:
        raise ValueError("descriptors must be a two-dimensional matrix")
    if matrix.dtype not in _OPENCV_TYPES:
        raise ValueError("descriptors must use uint8 or float32")
    continuous = np.ascontiguousarray(matrix)
    header = struct.pack(
        "=iii",
        int(continuous.shape[0]),
        int(continuous.shape[1]),
        _OPENCV_TYPES[continuous.dtype],
    )
    # Level 1 preserves the exact descriptor bytes while avoiding expensive
    # default-level compression on every RTAB feature publication.
    return zlib.compress(continuous.tobytes(order="C"), level=1) + header


def decode_descriptor_matrix(payload: bytes) -> np.ndarray:
    """Decode the format for unit tests and diagnostics."""
    if len(payload) < 12:
        raise ValueError("compressed descriptor payload is too short")
    rows, columns, matrix_type = struct.unpack("=iii", payload[-12:])
    dtypes = {value: key for key, value in _OPENCV_TYPES.items()}
    if rows < 0 or columns < 0 or matrix_type not in dtypes:
        raise ValueError("invalid RTAB descriptor header")
    raw = zlib.decompress(payload[:-12])
    expected = rows * columns * dtypes[matrix_type].itemsize
    if len(raw) != expected:
        raise ValueError("RTAB descriptor data size does not match its header")
    return np.frombuffer(raw, dtype=dtypes[matrix_type]).reshape(rows, columns).copy()
