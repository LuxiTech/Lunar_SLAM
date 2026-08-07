"""Small timestamp rate limiter used at ROS publication boundaries."""

from __future__ import annotations


def publication_is_due(
    last_stamp: float | None, current_stamp: float, rate_hz: float
) -> bool:
    """Return true for the first sample, clock resets, or an elapsed period."""
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")
    if last_stamp is None or current_stamp <= last_stamp:
        return True
    return current_stamp - last_stamp >= 1.0 / rate_hz


def processing_is_due(last_stamp: float, current_stamp: float, rate_hz: float) -> bool:
    """Rate-limit incoming frames while tolerating normal sensor timestamp jitter."""
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")
    if last_stamp < 0.0 or current_stamp <= last_stamp:
        return True
    return current_stamp - last_stamp >= 0.8 / rate_hz
