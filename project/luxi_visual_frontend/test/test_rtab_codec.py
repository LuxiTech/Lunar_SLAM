import numpy as np
import pytest

from luxi_visual_frontend.rtab_codec import (
    compress_descriptor_matrix,
    decode_descriptor_matrix,
)


@pytest.mark.parametrize("dtype", [np.float32, np.uint8])
def test_rtab_descriptor_round_trip(dtype):
    descriptors = np.arange(8 * 256, dtype=dtype).reshape(8, 256)
    payload = compress_descriptor_matrix(descriptors)
    decoded = decode_descriptor_matrix(payload)
    assert decoded.dtype == descriptors.dtype
    np.testing.assert_array_equal(decoded, descriptors)


def test_rtab_descriptor_codec_rejects_unsupported_layout():
    with pytest.raises(ValueError, match="two-dimensional"):
        compress_descriptor_matrix(np.zeros((2, 3, 4), dtype=np.float32))
    with pytest.raises(ValueError, match="uint8 or float32"):
        compress_descriptor_matrix(np.zeros((2, 3), dtype=np.float64))
