"""
test_confidence.py

Part 23/24 coverage: confidence must be derived from measurable
statistics (never a fixed constant) and must behave monotonically in
the expected directions.
"""

import numpy as np
import pytest

from terrain_geometry.confidence import compute_confidence, ConfidenceWeights


def _dense_cluster(n=200, spread=0.05, center=(2.0, 0.0, 0.3), seed=0):
    rng = np.random.default_rng(seed)
    pts = rng.normal(loc=center, scale=spread, size=(n, 3))
    return pts


def test_confidence_is_bounded():
    pts = _dense_cluster()
    score = compute_confidence(
        pts, distance=2.0, height_above_terrain=0.2, terrain_confidence=0.9
    )
    assert 0.0 <= score <= 1.0


def test_more_points_increase_confidence():
    few = _dense_cluster(n=3, seed=1)
    many = _dense_cluster(n=200, seed=2)
    score_few = compute_confidence(few, 2.0, 0.2, 0.9)
    score_many = compute_confidence(many, 2.0, 0.2, 0.9)
    assert score_many > score_few


def test_farther_distance_decreases_confidence():
    pts = _dense_cluster()
    near = compute_confidence(pts, distance=1.0, height_above_terrain=0.2, terrain_confidence=0.9)
    far = compute_confidence(pts, distance=8.0, height_above_terrain=0.2, terrain_confidence=0.9)
    assert near > far


def test_noisier_depth_decreases_confidence():
    rng = np.random.default_rng(3)
    tight = rng.normal(loc=(2.0, 0.0, 0.3), scale=(0.005, 0.05, 0.05), size=(150, 3))
    noisy = rng.normal(loc=(2.0, 0.0, 0.3), scale=(0.2, 0.05, 0.05), size=(150, 3))
    score_tight = compute_confidence(tight, 2.0, 0.2, 0.9)
    score_noisy = compute_confidence(noisy, 2.0, 0.2, 0.9)
    assert score_tight > score_noisy


def test_greater_terrain_separation_increases_confidence():
    pts = _dense_cluster()
    barely_above = compute_confidence(pts, 2.0, height_above_terrain=0.01, terrain_confidence=0.9)
    clearly_above = compute_confidence(pts, 2.0, height_above_terrain=0.3, terrain_confidence=0.9)
    assert clearly_above > barely_above


def test_low_terrain_model_confidence_reduces_overall_confidence():
    pts = _dense_cluster()
    high_terrain_conf = compute_confidence(pts, 2.0, 0.2, terrain_confidence=1.0)
    low_terrain_conf = compute_confidence(pts, 2.0, 0.2, terrain_confidence=0.0)
    assert high_terrain_conf > low_terrain_conf


def test_never_returns_fixed_constant_across_varied_inputs():
    """Part 24: 'do not create a fake confidence value that is always
    1.0' -- sample a handful of varied inputs and check they aren't
    all identical."""
    scores = set()
    for i in range(5):
        pts = _dense_cluster(n=10 * (i + 1), spread=0.02 * (i + 1), seed=i)
        scores.add(round(compute_confidence(pts, float(i + 1), 0.1 * (i + 1), 0.8), 6))
    assert len(scores) > 1


def test_weights_must_be_positive_sum():
    with pytest.raises(ValueError):
        ConfidenceWeights(0, 0, 0, 0, 0).normalized()
