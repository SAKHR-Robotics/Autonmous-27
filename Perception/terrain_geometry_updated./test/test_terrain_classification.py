"""
test_terrain_classification.py

Part 9/30 coverage: classification must label a whole-incline cluster
as TERRAIN and a rock-on-incline cluster as ROCK, using several
features together (not one threshold).
"""

import numpy as np
import pytest

from terrain_geometry.terrain_model import LocalTerrainModel
from terrain_geometry.terrain_classification import (
    ClassificationConfig,
    TerrainClassification,
    classify_cluster,
)


def _fitted_incline_model(slope_deg=20.0):
    rng = np.random.default_rng(7)
    x = rng.uniform(0.2, 4.0, 4000)
    y = rng.uniform(-2.0, 2.0, 4000)
    slope = np.tan(np.radians(slope_deg))
    z = slope * x + rng.normal(0.0, 0.005, 4000)
    ground = np.column_stack([x, y, z])
    model = LocalTerrainModel(patch_size=0.4, min_points=15, max_residual=0.05)
    model.fit(ground)
    return model, slope


class TestClassifyTerrainVsRock:
    def test_incline_cluster_classified_as_terrain(self):
        """Part 4/30: a cluster covering a big chunk of the incline
        (as ground_removal's non-ground split might mistakenly produce)
        must classify as TERRAIN, not ROCK/OBSTACLE."""
        model, slope = _fitted_incline_model()
        config = ClassificationConfig()
        config.validate()

        rng = np.random.default_rng(11)
        x = rng.uniform(1.0, 3.0, 500)
        y = rng.uniform(-1.0, 1.0, 500)
        z = slope * x + rng.normal(0.0, 0.01, 500)
        cluster = np.column_stack([x, y, z])
        width = float(y.max() - y.min())
        depth = float(x.max() - x.min())

        result = classify_cluster(cluster, model, width, depth, config)
        assert result.label == TerrainClassification.TERRAIN
        assert result.explained_fraction > config.terrain_explained_fraction

    def test_rock_on_incline_classified_as_rock(self):
        model, slope = _fitted_incline_model()
        config = ClassificationConfig()

        rng = np.random.default_rng(12)
        rock_x = rng.uniform(1.9, 2.15, 150)
        rock_y = rng.uniform(-0.15, 0.15, 150)
        rock_z = slope * rock_x + 0.18 + rng.normal(0.0, 0.01, 150)
        cluster = np.column_stack([rock_x, rock_y, rock_z])
        width = float(rock_y.max() - rock_y.min())
        depth = float(rock_x.max() - rock_x.min())

        result = classify_cluster(cluster, model, width, depth, config)
        assert result.label == TerrainClassification.ROCK
        assert result.height_above_terrain > config.min_rock_height

    def test_large_flat_slab_classified_as_step_not_rock(self):
        model, slope = _fitted_incline_model()
        config = ClassificationConfig()

        rng = np.random.default_rng(13)
        x = rng.uniform(1.5, 2.5, 400)  # 1.0 m wide in X
        y = rng.uniform(-1.0, 1.0, 400)  # 2.0 m wide in Y
        z = slope * x + 0.2 + rng.normal(0.0, 0.005, 400)  # very flat top -> low roughness
        cluster = np.column_stack([x, y, z])
        width = float(y.max() - y.min())
        depth = float(x.max() - x.min())

        result = classify_cluster(cluster, model, width, depth, config)
        assert result.label == TerrainClassification.STEP

    def test_sparse_noise_near_surface_is_unknown_not_rock(self):
        model, slope = _fitted_incline_model()
        config = ClassificationConfig()

        rng = np.random.default_rng(14)
        x = rng.uniform(1.95, 2.05, 5)
        y = rng.uniform(-0.03, 0.03, 5)
        z = slope * x + 0.12 + rng.normal(0.0, 0.005, 5)  # clearly above max_terrain_residual, but few points/small footprint
        cluster = np.column_stack([x, y, z])
        width = float(y.max() - y.min())
        depth = float(x.max() - x.min())

        result = classify_cluster(cluster, model, width, depth, config)
        assert result.label == TerrainClassification.UNKNOWN

    def test_cluster_with_no_terrain_coverage_is_unknown(self):
        model, _slope = _fitted_incline_model()
        config = ClassificationConfig()
        # Points far outside the fitted grid extent -> no valid patches.
        cluster = np.array([[50.0, 50.0, 1.0]] * 20)
        result = classify_cluster(cluster, model, 0.1, 0.1, config)
        assert result.label == TerrainClassification.UNKNOWN

    def test_empty_cluster_is_unknown(self):
        model, _slope = _fitted_incline_model()
        config = ClassificationConfig()
        result = classify_cluster(np.zeros((0, 3)), model, 0.1, 0.1, config)
        assert result.label == TerrainClassification.UNKNOWN


class TestClassificationConfigValidation:
    def test_invalid_config_raises(self):
        config = ClassificationConfig(max_terrain_residual=-1.0)
        with pytest.raises(ValueError):
            config.validate()
