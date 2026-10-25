"""Conservative observed-surface tests. Missing samples are never declared safe."""
import numpy as np


def assess_surface(points, up, *, cell_size, max_slope_deg, max_step_m, max_roughness_m, min_samples=6):
    if not (cell_size > 0 and 0 < max_slope_deg < 90 and max_step_m > 0 and max_roughness_m > 0):
        raise ValueError("Measured positive terrain limits are required")
    points = np.asarray(points, dtype=float)
    points = points[np.isfinite(points).all(axis=1)]
    up = np.asarray(up, dtype=float)
    if up.shape != (3,) or not np.isfinite(up).all() or np.linalg.norm(up) < 1e-9:
        raise ValueError("Valid gravity reference required")
    up = up / np.linalg.norm(up)
    cells = np.floor(points[:, :2] / cell_size).astype(int)
    hazards = []
    for key in np.unique(cells, axis=0):
        patch = points[np.all(cells == key, axis=1)]
        if len(patch) < min_samples:
            continue  # unobserved/undersampled, not a free-space decision
        design = np.column_stack((patch[:, :2], np.ones(len(patch))))
        coefficients, _, rank, _ = np.linalg.lstsq(design, patch[:, 2], rcond=None)
        if rank < 3:
            continue
        normal = np.array([-coefficients[0], -coefficients[1], 1.])
        normal /= np.linalg.norm(normal)
        slope = np.degrees(np.arccos(np.clip(abs(normal @ up), 0., 1.)))
        residuals = patch[:, 2] - design @ coefficients
        roughness = np.sqrt(np.mean(residuals ** 2))
        step = np.ptp(residuals)
        if slope > max_slope_deg or step > max_step_m or roughness > max_roughness_m:
            low, high = patch.min(axis=0), patch.max(axis=0)
            hazards.append(((low + high) / 2, np.maximum(high - low, [cell_size, cell_size, .05])))
    return hazards
