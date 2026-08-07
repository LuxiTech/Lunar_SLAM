import pytest

from luxi_visual_frontend.rate_limit import processing_is_due, publication_is_due


def test_publication_rate_limiter_accepts_first_period_and_clock_reset():
    assert publication_is_due(None, 10.0, 1.0)
    assert not publication_is_due(10.0, 10.9, 1.0)
    assert publication_is_due(10.0, 11.0, 1.0)
    assert publication_is_due(10.0, 9.0, 1.0)


def test_publication_rate_limiter_rejects_invalid_rate():
    with pytest.raises(ValueError, match="positive"):
        publication_is_due(None, 1.0, 0.0)


def test_processing_rate_limiter_tolerates_sensor_jitter():
    assert processing_is_due(-1.0, 10.0, 10.0)
    assert processing_is_due(10.0, 10.09, 10.0)
    assert not processing_is_due(10.0, 10.07, 10.0)
    assert processing_is_due(10.0, 9.0, 10.0)


def test_processing_rate_limiter_rejects_invalid_rate():
    with pytest.raises(ValueError, match="positive"):
        processing_is_due(-1.0, 1.0, 0.0)
