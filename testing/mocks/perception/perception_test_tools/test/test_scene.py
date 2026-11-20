import cv2
import numpy as np
import pytest
from perception_test_tools.scene import make_scene, scene_ground_truth, OPTICAL_TO_BASE, MOUNT


def test_deterministic_calibrated_inputs():
    a, image, depth, _ = make_scene()
    b, _, _, _ = make_scene()
    np.testing.assert_array_equal(a, b)
    assert image.shape == (480, 640, 3)
    assert depth.dtype == np.float32
    assert depth[240, 320] == 2.
    base = a @ OPTICAL_TO_BASE.T + MOUNT
    assert np.max(base[:, 2]) > .4


def test_ground_invalid_and_occlusion_cases():
    cloud, _, _, visible = make_scene('ground_only')
    assert not visible
    np.testing.assert_allclose((cloud @ OPTICAL_TO_BASE.T + MOUNT)[:, 2], 0., atol=1e-6)
    assert not np.isfinite(make_scene('invalid_cloud')[0]).any()
    assert not make_scene('turn_occlusion', tick=25)[3]


def test_ground_z_offset_and_ground_truth_metadata():
    cloud, _, _, visible = make_scene('ground_only', ground_z_offset=-0.255)
    assert not visible
    np.testing.assert_allclose((cloud @ OPTICAL_TO_BASE.T + MOUNT)[:, 2], -0.255, atol=1e-6)
    gt = scene_ground_truth('clear_aruco_nav', ground_z_offset=-0.255)
    assert gt['obstacles'][0]['center'][2] == pytest.approx(-0.005)
    assert gt['markers'][0]['expected_ambiguous'] is False


def test_clear_aruco_nav_renders_tilted_detectable_marker():
    _, image, depth, visible = make_scene('clear_aruco_nav')
    assert visible
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary).detectMarkers(image)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
    assert ids is not None and 0 in ids.flatten()
    pts = corners[0].reshape(4, 2)
    # Tilted marker creates distinct side lengths/foreshortening and finite planar depth
    center = np.mean(pts, axis=0).astype(int)
    assert np.isfinite(depth[center[1], center[0]])
    assert 1.2 <= float(depth[center[1], center[0]]) <= 1.8
