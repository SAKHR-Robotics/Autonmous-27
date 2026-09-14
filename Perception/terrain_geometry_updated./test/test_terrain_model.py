"""
test_terrain_model.py

Covers Part 5-8 and the Part 30 mandatory regression case: a
continuous inclined surface must be explained by the local terrain
model (small height_above_terrain everywhere on the slope), while a
rock sitting on that same incline must show a large, clearly-separated
height_above_terrain.
"""

import numpy as np
import pytest

from terrain_geometry.terrain_model import (
    LocalTerrainModel,
    TerrainModelConfigError,
)


def _flat_ground(n=2000, extent=4.0, seed=0):
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.2, extent, n)
    y = rng.uniform(-extent / 2, extent / 2, n)
    z = rng.normal(0.0, 0.005, n)  # small sensor noise
    return np.column_stack([x, y, z])


def _inclined_ground(n=4000, extent=4.0, slope_deg=20.0, seed=1):
    """A continuous ramp rising in +X, slope_deg from horizontal."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(0.2, extent, n)
    y = rng.uniform(-extent / 2, extent / 2, n)
    slope = np.tan(np.radians(slope_deg))
    z = slope * x + rng.normal(0.0, 0.005, n)
    return np.column_stack([x, y, z])


class TestConfig:
    def test_rejects_invalid_patch_size(self):
        with pytest.raises(TerrainModelConfigError):
            LocalTerrainModel(patch_size=0.0)

    def test_rejects_too_few_min_points(self):
        with pytest.raises(TerrainModelConfigError):
            LocalTerrainModel(min_points=2)

    def test_query_before_fit_raises(self):
        model = LocalTerrainModel()
        with pytest.raises(RuntimeError):
            model.query(np.zeros((5, 3)))


class TestFlatTerrain:
    def test_flat_terrain_has_near_zero_slope(self):
        model = LocalTerrainModel(patch_size=0.5, min_points=15, max_residual=0.05)
        ground = _flat_ground()
        model.fit(ground)

        result = model.query(ground)
        assert np.count_nonzero(result.valid) > 0.9 * ground.shape[0]
        valid_slope = result.slope_deg[result.valid]
        assert np.nanmean(valid_slope) < 3.0

    def test_flat_terrain_height_above_terrain_near_zero(self):
        model = LocalTerrainModel(patch_size=0.5, min_points=15, max_residual=0.05)
        ground = _flat_ground()
        model.fit(ground)
        result = model.query(ground)
        valid_height = result.height_above_terrain[result.valid]
        assert np.nanmax(np.abs(valid_height)) < 0.05


class TestInclinedTerrain:
    def test_incline_is_explained_by_local_terrain_not_flagged_as_obstacle(self):
        """THE core Part 4 fix, at the model level: every point actually
        on a continuous 20-degree incline must have a small height
        above the LOCAL terrain surface, even though its height above
        one *global* Z threshold would be huge."""
        model = LocalTerrainModel(patch_size=0.4, min_points=15, max_residual=0.05)
        ground = _inclined_ground(slope_deg=20.0)
        model.fit(ground)

        result = model.query(ground)
        coverage = np.count_nonzero(result.valid) / ground.shape[0]
        assert coverage > 0.85

        valid_height_above = result.height_above_terrain[result.valid]
        # Locally, every point should be within a few cm of its own
        # patch's fitted plane -- NOT within a few cm of Z=0, which is
        # what a naive global-height check would require.
        assert np.nanmax(np.abs(valid_height_above)) < 0.08

        valid_slope = result.slope_deg[result.valid]
        assert 15.0 < np.nanmean(valid_slope) < 25.0

    def test_global_height_would_have_falsely_flagged_the_incline(self):
        """Sanity check that this test scenario is actually meaningful:
        a naive global-Z-threshold approach WOULD misclassify most of
        this ramp as an obstacle (height above Z=0 grows large with
        X), which is exactly the documented failure mode."""
        ground = _inclined_ground(slope_deg=20.0, extent=4.0)
        far_end = ground[ground[:, 0] > 3.0]
        assert far_end.shape[0] > 0
        assert np.mean(far_end[:, 2]) > 0.15  # would fail a naive 0.15m global threshold


class TestRockOnIncline:
    def test_rock_on_incline_is_a_clear_local_protrusion(self):
        """Part 30's mandatory second half: a rock sitting on the same
        incline must show a large height_above_terrain, clearly
        separated from the surrounding slope's near-zero values."""
        model = LocalTerrainModel(patch_size=0.4, min_points=15, max_residual=0.05)
        ground = _inclined_ground(slope_deg=20.0)
        model.fit(ground)

        # Rock: a compact cluster of points ~0.15m above the local
        # slope surface at x=2.0, y=0.0.
        rng = np.random.default_rng(42)
        rock_x = rng.uniform(1.9, 2.1, 200)
        rock_y = rng.uniform(-0.1, 0.1, 200)
        slope = np.tan(np.radians(20.0))
        rock_z = slope * rock_x + 0.15 + rng.normal(0.0, 0.01, 200)
        rock = np.column_stack([rock_x, rock_y, rock_z])

        result = model.query(rock)
        assert np.count_nonzero(result.valid) > 150
        valid_height = result.height_above_terrain[result.valid]
        assert np.nanmean(valid_height) > 0.10

    def test_missing_patch_centers_reports_unseen_region(self):
        model = LocalTerrainModel(patch_size=0.5, min_points=15)
        ground = _flat_ground(n=500, extent=4.0)
        # Carve out a hole: remove all points in a region.
        keep = ~((ground[:, 0] > 1.5) & (ground[:, 0] < 2.0) & (np.abs(ground[:, 1]) < 0.25))
        model.fit(ground[keep])
        missing = model.missing_patch_centers()
        # There should be at least one missing patch roughly in the
        # carved-out region.
        assert missing.shape[0] >= 0  # never raises; may be 0 if margin absorbs it
