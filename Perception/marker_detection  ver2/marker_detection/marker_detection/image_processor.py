"""Small, intentionally conservative image-preprocessing functions."""
from __future__ import annotations
import cv2
import numpy as np


def bgr_to_gray(image: np.ndarray, use_clahe: bool = False,
                clahe_clip_limit: float = 2.0, clahe_tile_grid_size: int = 8) -> np.ndarray:
    """Convert BGR image to grayscale and optionally apply mild CLAHE."""
    if image is None or image.size == 0:
        raise ValueError("Image is empty.")
    if image.ndim == 2:
        gray = image
    elif image.ndim == 3 and image.shape[2] == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        raise ValueError(f"Expected mono or BGR image, got shape {image.shape}.")
    if use_clahe:
        gray = cv2.createCLAHE(clipLimit=clahe_clip_limit,
                               tileGridSize=(clahe_tile_grid_size, clahe_tile_grid_size)).apply(gray)
    return gray
