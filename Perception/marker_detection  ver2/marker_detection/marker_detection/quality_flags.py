"""Bitmask quality flags surfaced to downstream consumers (Improvement #22).

A single scalar confidence cannot tell a navigation/mission layer *why* an
observation is untrustworthy. Each flag names one specific, independently
checkable reason. Flags are informational annotations; hard accept/reject
decisions are made by the thresholds documented in each producing module
(image_quality.py, pose_estimator.py, tracker_manager.py, depth_processor.py).

Wire representation: a single uint32 bitmask field in the ROS messages
(MarkerDetection.quality_flags, MarkerPose.quality_flags,
MarkerTrackedPose.quality_flags). Use `names()` to render it for logging.
"""
from __future__ import annotations
from enum import IntFlag


class QualityFlag(IntFlag):
    VALID = 0
    LOW_DEPTH_QUALITY = 1 << 0        # Too few / too dispersed valid depth samples inside the marker.
    LOW_IMAGE_QUALITY = 1 << 1        # Blurred or low-contrast marker ROI.
    LOW_MARKER_RESOLUTION = 1 << 2    # Marker's apparent pixel size below the pose-quality threshold.
    HIGH_VIEWING_ANGLE = 1 << 3       # Marker normal far from the camera's viewing axis (foreshortened).
    HIGH_REPROJECTION_ERROR = 1 << 4  # Reprojection error elevated but still below the hard reject gate.
    DEPTH_MISMATCH = 1 << 5           # PnP-derived and depth-derived positions disagree.
    TEMPORAL_OUTLIER = 1 << 6         # Measurement rejected by the tracker's innovation gate.
    POSE_AMBIGUOUS = 1 << 7           # Multiple IPPE candidates were close in score (planar-pose ambiguity).
    STALE_DATA = 1 << 8               # RGB/depth pair or transform is older than the freshness threshold.
    TRACK_DEGRADED = 1 << 9           # Track has not been reconfirmed within confirmation_window_seconds.
    TRACK_LOST = 1 << 10              # Track has exceeded lost_timeout_seconds with no detection.
    TF_UNAVAILABLE = 1 << 11          # A required TF lookup failed or timed out.
    MULTIPLE_INSTANCES_SAME_ID = 1 << 12  # >1 physical detection shared this ID in one frame; one was
                                           # deterministically selected, the rest were NOT averaged in.


def names(flags: int) -> list[str]:
    """Human-readable flag names set in a bitmask, in declaration order."""
    return [flag.name for flag in QualityFlag if flag != QualityFlag.VALID and flag & flags]
