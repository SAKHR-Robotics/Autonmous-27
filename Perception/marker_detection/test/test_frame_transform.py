import numpy as np
from marker_detection.frame_transform import transform_to_base_link
from marker_detection.quaternion_filter import to_rotation_matrix


def test_identity_transform_leaves_position_and_orientation_unchanged():
    identity_quat = np.array([0.0, 0.0, 0.0, 1.0])
    camera_position = np.array([1.0, 2.0, 3.0])
    resolved = transform_to_base_link(
        transform_translation=np.zeros(3), transform_rotation=identity_quat,
        camera_position=camera_position, camera_orientation=identity_quat,
        camera_position_covariance=None)
    assert resolved is not None
    assert np.allclose(resolved.position, camera_position)
    assert np.allclose(resolved.orientation, identity_quat)
    assert resolved.position_covariance is None


def test_pure_translation_offsets_position_only():
    identity_quat = np.array([0.0, 0.0, 0.0, 1.0])
    translation = np.array([0.5, -0.2, 1.0])
    resolved = transform_to_base_link(
        transform_translation=translation, transform_rotation=identity_quat,
        camera_position=np.array([1.0, 0.0, 0.0]), camera_orientation=identity_quat,
        camera_position_covariance=None)
    assert resolved is not None
    assert np.allclose(resolved.position, np.array([1.5, -0.2, 1.0]))
    assert np.allclose(resolved.orientation, identity_quat)


def test_90_degree_yaw_rotates_position_and_composes_orientation():
    # 90 deg rotation about +Z: (x, y, z, w) = (0, 0, sin(45deg), cos(45deg)).
    half = np.deg2rad(45.0)
    rotation_quat = np.array([0.0, 0.0, np.sin(half), np.cos(half)])
    camera_position = np.array([1.0, 0.0, 0.0])
    resolved = transform_to_base_link(
        transform_translation=np.zeros(3), transform_rotation=rotation_quat,
        camera_position=camera_position, camera_orientation=np.array([0.0, 0.0, 0.0, 1.0]),
        camera_position_covariance=None)
    assert resolved is not None
    # +X in camera frame becomes +Y in base_link after a +90deg yaw.
    assert np.allclose(resolved.position, np.array([0.0, 1.0, 0.0]), atol=1e-9)
    # Composed orientation must equal the transform's own rotation (camera
    # orientation was identity), and R(composed) must match direct R@R.
    assert np.allclose(np.abs(resolved.orientation), np.abs(rotation_quat), atol=1e-9)


def test_composed_orientation_matches_direct_matrix_multiplication():
    """Cross-check the quaternion composition convention against rotation matrices
    directly, since a sign/order bug here would silently give the wrong pose."""
    rng = np.random.default_rng(7)
    for _ in range(20):
        transform_rotation = _random_unit_quaternion(rng)
        camera_orientation = _random_unit_quaternion(rng)
        resolved = transform_to_base_link(
            transform_translation=np.zeros(3), transform_rotation=transform_rotation,
            camera_position=np.zeros(3), camera_orientation=camera_orientation,
            camera_position_covariance=None)
        assert resolved is not None
        expected = to_rotation_matrix(transform_rotation) @ to_rotation_matrix(camera_orientation)
        actual = to_rotation_matrix(resolved.orientation)
        assert np.allclose(expected, actual, atol=1e-8)


def test_covariance_is_rotated_not_just_copied():
    half = np.deg2rad(45.0)
    rotation_quat = np.array([0.0, 0.0, np.sin(half), np.cos(half)])  # +90deg about Z
    identity_quat = np.array([0.0, 0.0, 0.0, 1.0])
    covariance = np.diag([1.0, 4.0, 9.0])  # anisotropic: X and Y variances differ.
    resolved = transform_to_base_link(
        transform_translation=np.zeros(3), transform_rotation=rotation_quat,
        camera_position=np.zeros(3), camera_orientation=identity_quat,
        camera_position_covariance=covariance)
    assert resolved is not None
    assert resolved.position_covariance is not None
    # After a +90deg yaw, camera-X variance (1.0) should now sit on base_link-Y,
    # and camera-Y variance (4.0) should now sit on base_link-X.
    assert np.isclose(resolved.position_covariance[0, 0], 4.0, atol=1e-8)
    assert np.isclose(resolved.position_covariance[1, 1], 1.0, atol=1e-8)
    assert np.isclose(resolved.position_covariance[2, 2], 9.0, atol=1e-8)
    # Trace (total uncertainty "volume") is invariant under a pure rotation.
    assert np.isclose(np.trace(resolved.position_covariance), np.trace(covariance))


def test_invalid_inputs_never_fabricate_a_pose():
    identity_quat = np.array([0.0, 0.0, 0.0, 1.0])
    assert transform_to_base_link(
        transform_translation=np.zeros(3), transform_rotation=np.zeros(4),  # invalid (zero-norm) quaternion
        camera_position=np.zeros(3), camera_orientation=identity_quat,
        camera_position_covariance=None) is None
    assert transform_to_base_link(
        transform_translation=np.zeros(3), transform_rotation=identity_quat,
        camera_position=np.array([np.nan, 0.0, 0.0]), camera_orientation=identity_quat,
        camera_position_covariance=None) is None


def _random_unit_quaternion(rng: np.random.Generator) -> np.ndarray:
    value = rng.normal(size=4)
    return value / np.linalg.norm(value)
