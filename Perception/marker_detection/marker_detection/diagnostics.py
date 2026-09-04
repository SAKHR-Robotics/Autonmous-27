"""Pure-Python performance/health statistics (Improvements #14 diagnostics, #15 performance).

Kept free of ROS/diagnostic_msgs imports so the percentile/threshold logic is
unit-testable without a ROS environment; the node wraps this module's output
in diagnostic_msgs.msg.DiagnosticArray/DiagnosticStatus for publishing, and
throttles publication itself (see marker_detection_node.py).
"""
from __future__ import annotations
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class LatencyStats:
    count: int
    avg_ms: float
    p50_ms: float
    p95_ms: float
    p99_ms: float
    max_ms: float
    fps: float


def _percentile(sorted_values: list[float], fraction: float) -> float:
    if not sorted_values:
        return float("nan")
    index = min(len(sorted_values) - 1, max(0, math.ceil(fraction * len(sorted_values)) - 1))
    return sorted_values[index]


def latency_stats(samples_s: list[float], window_seconds: float) -> LatencyStats:
    """samples_s: per-frame processing durations in seconds, over the reporting window."""
    if not samples_s:
        return LatencyStats(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    ms = sorted(value * 1000.0 for value in samples_s)
    fps = len(samples_s) / window_seconds if window_seconds > 0 else 0.0
    return LatencyStats(len(samples_s), sum(ms) / len(ms), _percentile(ms, 0.50), _percentile(ms, 0.95),
                        _percentile(ms, 0.99), max(ms), fps)


@dataclass(frozen=True)
class SyncStats:
    """RGB-depth timestamp-delta distribution (Part 8), for matched pairs only."""
    count: int
    avg_ms: float
    p50_ms: float
    p95_ms: float
    max_ms: float


def sync_delta_stats(deltas_s: list[float]) -> SyncStats:
    """deltas_s: |rgb_stamp - depth_stamp| in seconds, one per matched pair,
    over the reporting window. Reuses _percentile so P50/P95 agree with
    latency_stats' definition."""
    if not deltas_s:
        return SyncStats(0, 0.0, 0.0, 0.0, 0.0)
    ms = sorted(value * 1000.0 for value in deltas_s)
    return SyncStats(len(ms), sum(ms) / len(ms), _percentile(ms, 0.50), _percentile(ms, 0.95), max(ms))


class HealthLevel:
    """Mirrors diagnostic_msgs/DiagnosticStatus levels (0/1/2) without importing the message type."""
    OK = 0
    WARN = 1
    ERROR = 2


def synchronization_level(seconds_since_last_pair: float, stale_threshold_s: float) -> int:
    if seconds_since_last_pair > stale_threshold_s * 3.0:
        return HealthLevel.ERROR
    if seconds_since_last_pair > stale_threshold_s:
        return HealthLevel.WARN
    return HealthLevel.OK


def latency_level(stats: LatencyStats, warn_p95_ms: float, error_p95_ms: float) -> int:
    if stats.count == 0:
        return HealthLevel.WARN
    if stats.p95_ms > error_p95_ms:
        return HealthLevel.ERROR
    if stats.p95_ms > warn_p95_ms:
        return HealthLevel.WARN
    return HealthLevel.OK


def overall_level(*levels: int) -> int:
    return max(levels) if levels else HealthLevel.OK
