#!/usr/bin/env python3
"""
perception_health.py

Explicit perception validity states (Part 3).

THE SAFETY BUG THIS FIXES
    Before this module, a perception failure (empty cloud, missing TF,
    ground-segmentation exception, etc.) resulted in `_publish_empty()`
    being called in terrain_node.py -- which, depending on
    `use_unknown_space`, could publish a grid with the "actively
    sensed" region marked FREE even though nothing was actually
    successfully observed this frame. A downstream planner reading
    that grid has no way to distinguish "I looked and it's clear" from
    "perception silently failed" -- exactly the unacceptable failure
    chain described in Part 3.

WHAT THIS MODULE DOES
    Defines three explicit states -- VALID, DEGRADED, INVALID -- and a
    `PerceptionHealthMonitor` that classifies each frame based on
    concrete, checkable conditions (empty cloud, stale cloud, invalid/
    missing TF, too few surviving points, ground-estimation failure,
    excessive per-frame latency). `terrain_node.py` uses the resulting
    `PerceptionHealth` to decide whether it's safe to stamp the known
    region FREE this frame, or whether it must leave it UNKNOWN
    instead -- see `_publish_empty`/`_publish_degraded` in
    terrain_node.py.

    This module has zero ROS/rclpy dependency (a timestamp is passed
    in as a plain float, not a ROS message) so it's independently unit
    testable; terrain_node.py is responsible for extracting those
    plain values from ROS messages/clocks.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum


class PerceptionState(str, Enum):
    """Overall per-frame perception validity.

    VALID: Every check passed; downstream consumers may trust that
        "no obstacle reported" means "observed and clear".
    DEGRADED: Perception produced *some* usable output this frame, but
        under a condition that reduces trust in it (e.g. a stale
        cloud, or the ground fit had unusually few points). The
        pipeline keeps running, but the known-free region should be
        treated with more caution (e.g. inflated more aggressively, or
        simply logged) -- see `should_report_free`.
    INVALID: Perception could not produce a trustworthy result this
        frame at all (empty cloud, missing TF, ground segmentation
        failed, processing timed out). `should_report_free` is always
        False -- the caller must not claim any region is known-free.
    """

    VALID = "VALID"
    DEGRADED = "DEGRADED"
    INVALID = "INVALID"


@dataclass
class PerceptionHealth:
    """Result of one frame's health check.

    Attributes:
        state: Overall `PerceptionState` for this frame.
        reasons: Human-readable list of every condition that
            contributed to a non-VALID state (empty for VALID frames).
        should_report_free: Whether it is safe for the occupancy grid
            to mark the actively-sensed region FREE this frame. False
            for INVALID; True for VALID; configurable for DEGRADED
            (see `PerceptionHealthMonitor.degraded_may_report_free`).
    """

    state: PerceptionState
    reasons: list[str] = field(default_factory=list)
    should_report_free: bool = True

    @property
    def is_valid(self) -> bool:
        return self.state == PerceptionState.VALID

    @property
    def is_invalid(self) -> bool:
        return self.state == PerceptionState.INVALID


class PerceptionHealthMonitor:
    """Evaluates concrete per-frame conditions into a `PerceptionHealth`.

    Every threshold is configurable (Part 25). Holds no ROS state --
    callers pass plain values already extracted from ROS
    messages/clocks/TF lookups.
    """

    def __init__(
        self,
        max_cloud_age_sec: float = 0.5,
        min_valid_points: int = 20,
        min_ground_points: int = 50,
        max_processing_latency_sec: float = 0.5,
        degraded_may_report_free: bool = True,
    ) -> None:
        """
        Args:
            max_cloud_age_sec: A cloud older than this (now - cloud
                stamp) is considered stale -> DEGRADED (or INVALID if
                far past it, see `check`).
            min_valid_points: Fewer finite points than this after the
                initial PointCloud2 decode -> INVALID (too little data
                to say anything at all).
            min_ground_points: Fewer ground-classified points than
                this -> DEGRADED (the local terrain model will have
                poor coverage this frame, so its output should be
                trusted less, but the frame isn't necessarily unusable).
            max_processing_latency_sec: If the previous frame took
                longer than this to process, flags DEGRADED (falling
                behind real-time is itself a safety-relevant signal).
            degraded_may_report_free: Whether a DEGRADED frame is still
                allowed to mark its known region FREE. Defaults True
                (DEGRADED is "trust a bit less", not "don't trust at
                all") -- set False to be more conservative.
        """
        if max_cloud_age_sec <= 0.0:
            raise ValueError("max_cloud_age_sec must be > 0")
        if min_valid_points < 0:
            raise ValueError("min_valid_points must be >= 0")
        if min_ground_points < 0:
            raise ValueError("min_ground_points must be >= 0")
        if max_processing_latency_sec <= 0.0:
            raise ValueError("max_processing_latency_sec must be > 0")

        self.max_cloud_age_sec = float(max_cloud_age_sec)
        self.min_valid_points = int(min_valid_points)
        self.min_ground_points = int(min_ground_points)
        self.max_processing_latency_sec = float(max_processing_latency_sec)
        self.degraded_may_report_free = bool(degraded_may_report_free)

    def check(
        self,
        *,
        now_sec: float,
        cloud_stamp_sec: float | None,
        num_valid_points: int | None,
        tf_available: bool,
        ground_segmentation_ok: bool,
        num_ground_points: int | None = None,
        last_frame_latency_sec: float | None = None,
    ) -> PerceptionHealth:
        """Classifies one frame.

        Args:
            now_sec: Current time (seconds, any monotonic/wall clock
                consistent with `cloud_stamp_sec`).
            cloud_stamp_sec: The incoming cloud's header stamp
                (seconds), or None if no cloud was received at all
                this evaluation (e.g. a periodic watchdog tick).
            num_valid_points: Number of finite XYZ points after the
                initial PointCloud2 decode, or None if decoding itself
                failed/was skipped.
            tf_available: Whether the required sensor->target TF
                transform was available this frame.
            ground_segmentation_ok: Whether ground_removal.segment()
                completed without raising.
            num_ground_points: Number of points classified as ground
                this frame, if segmentation succeeded.
            last_frame_latency_sec: Wall-clock time the previous frame
                took to process end-to-end, if measured.

        Returns:
            A `PerceptionHealth` summarizing this frame.
        """
        reasons: list[str] = []
        invalid = False
        degraded = False

        if cloud_stamp_sec is None:
            invalid = True
            reasons.append("no point cloud received")
        else:
            age = now_sec - cloud_stamp_sec
            if age < 0:
                # Clock skew / cloud stamped in the future -- treat as
                # a timing anomaly, degrade rather than crash.
                degraded = True
                reasons.append(f"cloud timestamp is {abs(age):.3f}s in the future")
            elif age > self.max_cloud_age_sec * 4.0:
                invalid = True
                reasons.append(
                    f"point cloud is severely stale ({age:.3f}s > "
                    f"{self.max_cloud_age_sec * 4.0:.3f}s)"
                )
            elif age > self.max_cloud_age_sec:
                degraded = True
                reasons.append(
                    f"point cloud is stale ({age:.3f}s > {self.max_cloud_age_sec:.3f}s)"
                )

        if not tf_available:
            invalid = True
            reasons.append("required TF transform unavailable")

        if num_valid_points is None:
            invalid = True
            reasons.append("point cloud could not be decoded")
        elif num_valid_points < self.min_valid_points:
            invalid = True
            reasons.append(
                f"too few valid points ({num_valid_points} < {self.min_valid_points})"
            )

        if not ground_segmentation_ok:
            invalid = True
            reasons.append("ground segmentation failed")
        elif num_ground_points is not None and num_ground_points < self.min_ground_points:
            degraded = True
            reasons.append(
                f"too few ground points for reliable terrain modeling "
                f"({num_ground_points} < {self.min_ground_points})"
            )

        if (
            last_frame_latency_sec is not None
            and last_frame_latency_sec > self.max_processing_latency_sec
        ):
            degraded = True
            reasons.append(
                f"previous frame latency {last_frame_latency_sec:.3f}s exceeded "
                f"{self.max_processing_latency_sec:.3f}s"
            )

        if invalid:
            state = PerceptionState.INVALID
            should_report_free = False
        elif degraded:
            state = PerceptionState.DEGRADED
            should_report_free = self.degraded_may_report_free
        else:
            state = PerceptionState.VALID
            should_report_free = True

        return PerceptionHealth(
            state=state, reasons=reasons, should_report_free=should_report_free
        )
