import pytest
from marker_detection.sync_buffer import DepthSyncBuffer


def test_match_returns_closest_within_tolerance():
    buffer: DepthSyncBuffer[str] = DepthSyncBuffer(max_size=5, max_delta_s=0.05)
    buffer.push(1.00, "a")
    buffer.push(1.05, "b")
    buffer.push(1.09, "c")
    match = buffer.match(1.08)
    assert match is not None
    assert match.payload == "c"
    assert match.delta_s == pytest.approx(0.01, abs=1e-9)


def test_match_returns_none_when_buffer_is_empty():
    buffer: DepthSyncBuffer[str] = DepthSyncBuffer(max_size=3, max_delta_s=0.05)
    assert buffer.match(1.0) is None


def test_match_returns_none_when_closest_frame_is_outside_tolerance():
    """This is the Part 2 regression case: a depth stall (no recent frame within
    slop) must not be silently treated as a match -- the caller falls back to
    2D-only output instead of fabricating a stale 3D association."""
    buffer: DepthSyncBuffer[str] = DepthSyncBuffer(max_size=5, max_delta_s=0.05)
    buffer.push(1.0, "stale")
    assert buffer.match(5.0) is None  # depth stream stalled for 4 seconds


def test_buffer_evicts_oldest_beyond_max_size():
    buffer: DepthSyncBuffer[int] = DepthSyncBuffer(max_size=2, max_delta_s=1.0)
    buffer.push(1.0, 1)
    buffer.push(2.0, 2)
    buffer.push(3.0, 3)  # evicts timestamp 1.0
    assert len(buffer) == 2
    match = buffer.match(1.0)
    assert match is not None
    assert match.payload == 2  # closest surviving entry, not the evicted one


def test_latest_timestamp_tracks_the_most_recently_pushed_frame():
    buffer: DepthSyncBuffer[int] = DepthSyncBuffer(max_size=3, max_delta_s=1.0)
    assert buffer.latest_timestamp is None
    buffer.push(1.0, 1)
    buffer.push(2.5, 2)
    assert buffer.latest_timestamp == 2.5


def test_depth_recovers_after_gap_without_special_handling():
    """Part 2: once a fresh depth frame arrives again, matching just works --
    no explicit 'resume' step is needed anywhere in the pipeline."""
    buffer: DepthSyncBuffer[str] = DepthSyncBuffer(max_size=5, max_delta_s=0.05)
    buffer.push(1.0, "before_gap")
    assert buffer.match(3.0) is None  # gap: depth stalled
    buffer.push(3.01, "after_gap")
    match = buffer.match(3.0)
    assert match is not None and match.payload == "after_gap"


def test_invalid_construction_arguments_are_rejected():
    with pytest.raises(ValueError):
        DepthSyncBuffer(max_size=0, max_delta_s=0.05)
    with pytest.raises(ValueError):
        DepthSyncBuffer(max_size=1, max_delta_s=0.0)
