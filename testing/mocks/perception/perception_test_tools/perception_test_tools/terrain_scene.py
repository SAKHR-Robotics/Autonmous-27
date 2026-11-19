"""Offline synthetic XYZ fixtures; no ROS publishers or physical calibration."""
from typing import Iterator

import numpy as np


def synthetic_frames(num_frames: int, seed: int = 42) -> Iterator[np.ndarray]:
    """Yields synthetic point clouds: a ground plane plus 1-3 moving boxes.

    Not a substitute for real sensor data -- useful for a smoke-test /
    relative-performance benchmark without needing a recorded bag.
    """
    rng = np.random.default_rng(seed)
    for i in range(num_frames):
        ground = np.column_stack(
            [
                rng.uniform(-2, 10, 6000),
                rng.uniform(-6, 6, 6000),
                rng.uniform(-0.05, 0.05, 6000),
            ]
        ).astype(np.float32)

        boxes = []
        num_boxes = 1 + (i % 3)
        for b in range(num_boxes):
            cx = 2.0 + 0.05 * i + b * 1.5
            cy = -1.0 + b * 1.0
            box = rng.uniform(
                low=[cx - 0.2, cy - 0.2, 0.0],
                high=[cx + 0.2, cy + 0.2, 0.5],
                size=(200, 3),
            ).astype(np.float32)
            boxes.append(box)

        yield np.vstack([ground] + boxes)
