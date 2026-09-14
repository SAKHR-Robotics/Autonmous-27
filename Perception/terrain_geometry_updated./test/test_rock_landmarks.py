"""
test_rock_landmarks.py

Parts 10-19 coverage, including the mandatory Part 31 synthetic
sequence: a rock seen, then missing for two frames, then seen again
must keep the SAME persistent ID throughout -- never rock_1/rock_2/rock_3.
"""

import numpy as np
import pytest

from terrain_geometry.rock_landmarks import (
    RockLandmarkDatabase,
    RockLandmarkConfigError,
    RockObservation,
)


def _obs(x, y, z=0.0, width=0.3, depth=0.3, height=0.2, confidence=0.8, track_id=0):
    return RockObservation(
        position_map=np.array([x, y, z]),
        width=width,
        depth=depth,
        height=height,
        classification="ROCK",
        confidence=confidence,
        track_id=track_id,
        num_points=100,
        distance=2.0,
    )


class TestConfigValidation:
    def test_rejects_bad_gate(self):
        with pytest.raises(RockLandmarkConfigError):
            RockLandmarkDatabase(association_distance_gate=0.0)

    def test_rejects_bad_smoothing(self):
        with pytest.raises(RockLandmarkConfigError):
            RockLandmarkDatabase(position_smoothing=1.5)


class TestPart31PersistentIdSequence:
    def test_disappearance_and_reappearance_keeps_same_id(self):
        """Part 31's exact mandatory scenario."""
        db = RockLandmarkDatabase(
            association_distance_gate=0.3,
            dimension_similarity_tolerance=0.6,
            position_smoothing=0.5,
            min_confidence_to_create=0.3,
        )

        # Frame 1: rock at (4.0, 1.0)
        visible = db.update([_obs(4.0, 1.0, track_id=0)], now_sec=0.0)
        assert len(visible) == 1
        rock_id = visible[0].persistent_id

        # Frame 2: rock at (4.02, 1.01) -- tiny sensor noise, same rock
        visible = db.update([_obs(4.02, 1.01, track_id=0)], now_sec=0.1)
        assert len(visible) == 1
        assert visible[0].persistent_id == rock_id

        # Frame 3: not visible
        visible = db.update([], now_sec=0.2)
        assert visible == []
        lm = db.get(rock_id)
        assert lm is not None
        assert lm.currently_visible is False

        # Frame 4: still not visible
        visible = db.update([], now_sec=0.3)
        assert visible == []
        assert db.get(rock_id).currently_visible is False

        # Frame 5: visible again at (4.05, 1.02) -- must re-identify,
        # NOT spawn a new rock_2/rock_3.
        visible = db.update([_obs(4.05, 1.02, track_id=5)], now_sec=0.4)
        assert len(visible) == 1
        assert visible[0].persistent_id == rock_id
        assert len(db) == 1  # never more than one physical rock in the DB


class TestAssociation:
    def test_two_far_apart_rocks_get_different_ids(self):
        db = RockLandmarkDatabase(association_distance_gate=0.3)
        visible = db.update(
            [_obs(1.0, 0.0, track_id=0), _obs(5.0, 0.0, track_id=1)], now_sec=0.0
        )
        assert len(visible) == 2
        assert visible[0].persistent_id != visible[1].persistent_id

    def test_same_position_but_very_different_size_is_not_reidentified(self):
        """Part 14: association must use dimension similarity, not
        distance alone."""
        db = RockLandmarkDatabase(
            association_distance_gate=0.5, dimension_similarity_tolerance=0.3
        )
        db.update([_obs(2.0, 0.0, width=0.2, depth=0.2, height=0.15, track_id=0)], now_sec=0.0)
        # A much bigger object shows up at nearly the same spot next frame.
        visible = db.update(
            [_obs(2.05, 0.0, width=1.5, depth=1.5, height=1.2, track_id=1)], now_sec=0.1
        )
        assert len(db) == 2  # treated as a distinct object, not a re-ID

    def test_rover_motion_does_not_spawn_duplicate_when_map_frame_used(self):
        """Part 16: since observations are already given in map frame
        (ego-motion compensation happens upstream via TF), a rock that
        stays at the same map position across frames re-identifies
        correctly regardless of how much the rover itself moved."""
        db = RockLandmarkDatabase(association_distance_gate=0.3)
        db.update([_obs(3.0, -1.0, track_id=0)], now_sec=0.0)
        visible = db.update([_obs(3.01, -0.99, track_id=1)], now_sec=1.0)
        assert len(db) == 1
        assert visible[0].observation_count == 2


class TestPositionSmoothing:
    def test_position_is_smoothed_not_overwritten(self):
        """Part 15: repeated observations should stabilize the estimate,
        and a single noisy reading should not overwrite it outright."""
        db = RockLandmarkDatabase(association_distance_gate=0.5, position_smoothing=0.2)
        db.update([_obs(2.0, 0.0, track_id=0)], now_sec=0.0)
        # One noisy outlier reading 0.3m off.
        visible = db.update([_obs(2.3, 0.0, track_id=1)], now_sec=0.1)
        lm = db.get(visible[0].persistent_id)
        # With alpha=0.2, the estimate should move only partway toward
        # the noisy reading, not land exactly on it.
        assert 2.0 < lm.position_map[0] < 2.3
        assert lm.position_map[0] < 2.0 + 0.2 * 0.3 + 1e-6


class TestVisibilityAndLowConfidence:
    def test_low_confidence_observation_does_not_spawn_new_landmark(self):
        db = RockLandmarkDatabase(min_confidence_to_create=0.5)
        visible = db.update([_obs(1.0, 1.0, confidence=0.1, track_id=0)], now_sec=0.0)
        assert visible == []
        assert len(db) == 0

    def test_low_confidence_can_still_update_existing_landmark(self):
        db = RockLandmarkDatabase(association_distance_gate=0.3, min_confidence_to_create=0.5)
        db.update([_obs(1.0, 1.0, confidence=0.8, track_id=0)], now_sec=0.0)
        visible = db.update([_obs(1.02, 1.0, confidence=0.1, track_id=1)], now_sec=0.1)
        assert len(visible) == 1
        assert len(db) == 1


class TestTimeout:
    def test_landmark_expires_after_timeout(self):
        db = RockLandmarkDatabase(association_distance_gate=0.3, landmark_timeout_sec=1.0)
        db.update([_obs(1.0, 1.0, track_id=0)], now_sec=0.0)
        assert len(db) == 1
        db.update([], now_sec=2.5)  # 2.5s since last_seen > timeout
        assert len(db) == 0

    def test_no_timeout_persists_indefinitely(self):
        db = RockLandmarkDatabase(association_distance_gate=0.3, landmark_timeout_sec=None)
        db.update([_obs(1.0, 1.0, track_id=0)], now_sec=0.0)
        db.update([], now_sec=10_000.0)
        assert len(db) == 1


class TestPersistence:
    def test_save_and_load_roundtrip(self, tmp_path):
        db = RockLandmarkDatabase(association_distance_gate=0.3)
        db.update([_obs(1.0, 2.0, track_id=0), _obs(5.0, -2.0, track_id=1)], now_sec=0.0)
        path = tmp_path / "landmarks.json"
        db.save_to_file(str(path))

        db2 = RockLandmarkDatabase(association_distance_gate=0.3)
        db2.load_from_file(str(path))
        assert len(db2) == 2
        ids_before = {lm.persistent_id for lm in db.all_landmarks()}
        ids_after = {lm.persistent_id for lm in db2.all_landmarks()}
        assert ids_before == ids_after
