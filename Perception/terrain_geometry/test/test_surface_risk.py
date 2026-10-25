import numpy as np
import pytest
from terrain_geometry.surface_risk import assess_surface


def surface(slope):
    x, y = np.meshgrid(np.linspace(.01,.2,8), np.linspace(.01,.2,8))
    return np.column_stack((x.ravel(),y.ravel(),slope*x.ravel()))


def evaluate(points):
    return assess_surface(points,[0,0,1],cell_size=.25,max_slope_deg=20,max_step_m=.1,max_roughness_m=.02)


def test_flat_ground_not_hazardous_and_slope_is():
    assert evaluate(surface(0)) == []
    assert len(evaluate(surface(1))) == 1


def test_no_samples_not_claimed_safe():
    assert evaluate(np.empty((0,3))) == []


def test_missing_measured_limits_rejected():
    with pytest.raises(ValueError):
        assess_surface(surface(0),[0,0,1],cell_size=.25,max_slope_deg=0,max_step_m=0,max_roughness_m=0)
