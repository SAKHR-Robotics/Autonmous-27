import numpy as np
from marker_detection.image_quality import assess, corner_geometry, full_frame_laplacian


def square_corners(size: float = 40.0, cx: float = 100.0, cy: float = 100.0) -> np.ndarray:
    half = size / 2.0
    return np.array([[cx - half, cy - half], [cx + half, cy - half],
                     [cx + half, cy + half], [cx - half, cy + half]], dtype=np.float32)


def test_convex_square_has_zero_side_length_cv():
    convex, cv_value = corner_geometry(square_corners())
    assert convex
    assert cv_value < 1e-6


def test_self_intersecting_quad_is_not_convex():
    bowtie = np.array([[0.0, 0.0], [40.0, 40.0], [40.0, 0.0], [0.0, 40.0]], dtype=np.float32)
    convex, _ = corner_geometry(bowtie)
    assert not convex


def test_lopsided_quad_has_higher_side_length_cv_than_a_square():
    _, square_cv = corner_geometry(square_corners())
    lopsided = np.array([[0.0, 0.0], [60.0, 0.0], [65.0, 5.0], [0.0, 40.0]], dtype=np.float32)
    _, lopsided_cv = corner_geometry(lopsided)
    assert lopsided_cv > square_cv


def test_sharp_high_contrast_marker_scores_higher_blur_than_flat_grey():
    corners = square_corners()
    flat = np.full((200, 200), 128, dtype=np.uint8)
    checkerboard = flat.copy()
    checkerboard[80:120:4, 80:120] = 255
    checkerboard[80:120, 80:120:4] = 0
    flat_quality = assess(flat, corners, full_frame_laplacian(flat))
    sharp_quality = assess(checkerboard, corners, full_frame_laplacian(checkerboard))
    assert sharp_quality.blur_score > flat_quality.blur_score
    assert sharp_quality.contrast_score > flat_quality.contrast_score


def test_degenerate_zero_area_marker_returns_zero_quality_not_a_crash():
    corners = np.array([[10.0, 10.0], [10.0, 10.0], [10.0, 10.0], [10.0, 10.0]], dtype=np.float32)
    gray = np.zeros((200, 200), dtype=np.uint8)
    quality = assess(gray, corners, full_frame_laplacian(gray))
    assert quality.blur_score == 0.0 and quality.contrast_score == 0.0
