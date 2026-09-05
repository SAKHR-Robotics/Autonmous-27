import numpy as np
from marker_detection.image_processor import bgr_to_gray


def test_bgr_to_gray_preserves_shape():
    image = np.zeros((20, 30, 3), dtype=np.uint8)
    assert bgr_to_gray(image).shape == (20, 30)


def test_empty_image_is_rejected():
    try:
        bgr_to_gray(np.array([], dtype=np.uint8))
    except ValueError:
        return
    assert False, "empty input must be rejected"
