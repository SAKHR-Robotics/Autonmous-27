import numpy as np
from marker_detection.position_kalman import PositionKalman


def test_kalman_reduces_noisy_stationary_measurements_with_variable_dt():
    filter_ = PositionKalman(np.array([0.0, 0.0, 2.0]), 0.1, 0.5, 0.01)
    measurements = [2.00, 2.05, 1.96, 2.04, 1.98]
    filtered = []
    for dt, value in zip([0.02, 0.04, 0.033, 0.02, 0.04], measurements):
        filter_.predict(dt)
        filtered.append(filter_.update(np.array([0.0, 0.0, value]))[2])
    assert np.std(filtered) < np.std(measurements)


def test_mahalanobis_innovation_grows_for_sudden_outlier():
    filter_ = PositionKalman(np.array([0.0, 0.0, 2.0]), 0.1, 0.5, 0.01)
    filter_.predict(0.033)
    _, _, normal = filter_.innovation(np.array([0.0, 0.0, 2.01]))
    _, _, outlier = filter_.innovation(np.array([0.0, 0.0, 8.5]))
    assert outlier > normal


def test_update_axis_fuses_an_independent_z_measurement_without_touching_xy():
    filter_ = PositionKalman(np.array([1.0, -0.5, 2.10]), 0.1, 0.5, 0.01)
    filter_.predict(0.02)
    fused = filter_.update_axis(2, 2.00, 1e-6)
    assert abs(fused[2] - 2.00) < 0.02          # Tight-variance depth measurement dominates the Z fusion.
    assert abs(fused[0] - 1.0) < 1e-6            # X is untouched by a Z-only partial update.
    assert abs(fused[1] - (-0.5)) < 1e-6         # Y is untouched by a Z-only partial update.


def test_update_axis_covariance_shrinks_only_along_the_updated_axis():
    filter_ = PositionKalman(np.array([0.0, 0.0, 2.0]), 0.5, 0.5, 0.01)
    filter_.predict(0.02)
    before = filter_.covariance[:3, :3].diagonal().copy()
    filter_.update_axis(2, 2.0, 1e-4)
    after = filter_.covariance[:3, :3].diagonal()
    assert after[2] < before[2]
    assert abs(after[0] - before[0]) < 1e-9
    assert abs(after[1] - before[1]) < 1e-9


def test_per_measurement_noise_overrides_the_constant_default():
    tight = PositionKalman(np.array([0.0, 0.0, 2.0]), 0.5, 0.5, 0.01)
    loose = PositionKalman(np.array([0.0, 0.0, 2.0]), 0.5, 0.5, 0.01)
    tight.predict(0.02)
    loose.predict(0.02)
    tight.update(np.array([0.0, 0.0, 2.5]), noise=np.eye(3) * 1e-6)
    loose.update(np.array([0.0, 0.0, 2.5]), noise=np.eye(3) * 10.0)
    # A tight-noise measurement should pull the state much closer to it than a loose-noise one.
    assert abs(tight.state[2] - 2.5) < abs(loose.state[2] - 2.5)
