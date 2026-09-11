import numpy as np
from marker_detection.quaternion_filter import normalize


def test_tf_quaternion_input_can_be_normalized_without_axis_reordering():
    quaternion = normalize(np.array([0.0, 0.0, 0.0, 2.0]))
    assert np.allclose(quaternion, [0.0, 0.0, 0.0, 1.0])


def test_invalid_tf_quaternion_is_rejected():
    assert normalize(np.array([0.0, 0.0, 0.0, 0.0])) is None
