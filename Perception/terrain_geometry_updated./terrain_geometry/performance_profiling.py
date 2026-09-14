#!/usr/bin/env python3
"""
performance_profiling.py

Lightweight per-stage timing helper for the terrain_geometry pipeline.

This module implements ONLY timing/bookkeeping -- it has no knowledge of
ROS, point clouds, or any specific pipeline stage's math, so it stays
reusable and independently unit-testable, consistent with the rest of
this package's "ROS-free computational core" modules.

DESIGN:
    `PipelineProfiler` is a simple stopwatch with checkpoints. Call
    `start()` once at the top of the callback, then `mark(name)` after
    each stage completes; `mark()` records the elapsed time since the
    previous mark (or since `start()`, for the first mark) under that
    stage's name. `total_ms()` returns the elapsed time since `start()`.

    Uses `time.perf_counter()`, a monotonic high-resolution timer
    unaffected by system clock adjustments -- appropriate for measuring
    short in-process durations.

    Recording a mark is a handful of Python-level float operations
    (subtraction, a dict/list append) -- negligible next to any of the
    NumPy/SciPy/scikit-learn stage work being measured, so this can stay
    active every frame without materially affecting the timing it
    reports. Only the *logging* of a summary is throttled by the caller
    (`enable_performance_profiling` + `profiling_interval` in
    terrain_node.py), not the measurement itself.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class PipelineProfiler:
    """Stopwatch-with-checkpoints for per-stage pipeline timing.

    Attributes:
        stage_ms: Ordered mapping of stage name -> elapsed milliseconds
            for that stage, populated by `mark()` calls since the last
            `start()`.
    """

    stage_ms: "dict[str, float]" = field(default_factory=dict)
    _t_start: float = 0.0
    _t_last: float = 0.0

    def start(self) -> None:
        """Resets the stopwatch and clears any previously recorded stages."""
        now = time.perf_counter()
        self._t_start = now
        self._t_last = now
        self.stage_ms.clear()

    def mark(self, stage_name: str) -> float:
        """Records the elapsed time since the previous mark (or start).

        Args:
            stage_name: Label for this stage (e.g. "ground_removal").

        Returns:
            The elapsed time for this stage, in milliseconds.
        """
        now = time.perf_counter()
        elapsed_ms = (now - self._t_last) * 1000.0
        self.stage_ms[stage_name] = elapsed_ms
        self._t_last = now
        return elapsed_ms

    def total_ms(self) -> float:
        """Returns the total elapsed time since `start()`, in milliseconds."""
        return (time.perf_counter() - self._t_start) * 1000.0

    def format_summary(self, counts: "dict[str, int]", total_ms: float) -> str:
        """Builds a compact, human-readable multi-line profiling summary.

        Args:
            counts: Ordered mapping of point/cluster count labels (e.g.
                "input", "roi", "non_ground", "voxel", "filtered",
                "clusters") to their integer values for this frame.
            total_ms: Total frame processing time, in milliseconds.

        Returns:
            A multi-line string in the format:

                TerrainGeometry:
                input=307200
                roi=120000
                non_ground=28000
                voxel=8500
                filtered=7200
                clusters=14
                total=58.4 ms
                fps=17.1
        """
        fps = (1000.0 / total_ms) if total_ms > 0.0 else 0.0
        lines = ["TerrainGeometry:"]
        for name, value in counts.items():
            lines.append(f"{name}={value}")
        lines.append(f"total={total_ms:.1f} ms")
        lines.append(f"fps={fps:.1f}")
        return "\n".join(lines)

    def format_stage_breakdown(self) -> str:
        """Builds a one-line, per-stage millisecond breakdown for detailed logs.

        Returns:
            A single line like:
                "decode=1.2ms tf=0.1ms roi=0.8ms ground=4.5ms voxel=2.1ms ..."
        """
        return " ".join(f"{name}={ms:.2f}ms" for name, ms in self.stage_ms.items())
