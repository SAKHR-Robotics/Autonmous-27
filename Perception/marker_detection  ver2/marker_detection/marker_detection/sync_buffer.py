"""Time-buffered depth-frame matching, decoupled from ROS and from RGB
detection (Part 2 of the ERC downstream-interface hardening pass).

Previously, RGB and depth were fused with a message_filters
ApproximateTimeSynchronizer *before* either stream reached the detection
callback, so any depth stall (dropped frames, driver hiccup) silently
stopped 2D ArUco detection too -- not just 3D estimation.

This buffer removes that coupling: `marker_detection_node` now subscribes to
RGB and depth independently. Every RGB frame runs 2D detection unconditionally;
this buffer is asked, best-effort, for the closest-in-time depth frame
already received. If one exists within `max_delta_s` it is used for 3D
association/PnP fusion as before; if not, 2D detection output is still
published and only the depth-dependent stages are skipped for that frame
(see marker_detection_node._rgb_callback).

Kept free of rclpy/sensor_msgs so the matching logic itself is unit-testable
without a ROS environment (see test/test_sync_buffer.py).
"""
from __future__ import annotations
from collections import deque
from dataclasses import dataclass
from typing import Generic, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class DepthMatch(Generic[T]):
    payload: T
    timestamp: float
    delta_s: float


class DepthSyncBuffer(Generic[T]):
    """Keeps the most recent depth frames and finds the closest match to an RGB timestamp.

    `payload` is left generic (rather than typed to sensor_msgs.msg.Image) so
    this module has no ROS dependency at all.
    """

    def __init__(self, max_size: int, max_delta_s: float) -> None:
        if max_size < 1:
            raise ValueError("max_size must be >= 1.")
        if max_delta_s <= 0:
            raise ValueError("max_delta_s must be > 0.")
        self.max_delta_s = max_delta_s
        self._buffer: deque[tuple[float, T]] = deque(maxlen=max_size)

    def push(self, timestamp: float, payload: T) -> None:
        self._buffer.append((timestamp, payload))

    def match(self, timestamp: float) -> DepthMatch[T] | None:
        """Closest-in-time buffered frame within max_delta_s, or None if the
        buffer is empty or every buffered frame is too far away in time."""
        if not self._buffer:
            return None
        best_ts, best_payload = min(self._buffer, key=lambda item: abs(item[0] - timestamp))
        delta = abs(best_ts - timestamp)
        if delta > self.max_delta_s:
            return None
        return DepthMatch(best_payload, best_ts, delta)

    @property
    def latest_timestamp(self) -> float | None:
        return self._buffer[-1][0] if self._buffer else None

    def __len__(self) -> int:
        return len(self._buffer)
