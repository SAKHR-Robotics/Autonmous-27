"""Per-marker constant-velocity Kalman filter with Mahalanobis innovation gate.

Improvement #2 (depth+PnP fusion) is implemented here as two sequential,
independently-weighted measurement updates rather than a hand-rolled average:
first the full 3D PnP position (see `update`), then, if independent depth is
available for this frame, a partial Z-only update (see `update_axis`) using
the depth sensor's own measurement noise. Standard Kalman theory guarantees
this sequential application is equivalent to a single joint update when the
measurement noises are independent, which RGB reprojection noise and RealSense
depth-sensor noise are.
"""
from __future__ import annotations
import numpy as np


class PositionKalman:
    def __init__(self, position: np.ndarray, initial_covariance: float,
                 process_noise: float, measurement_noise: float) -> None:
        self.state = np.zeros(6, dtype=np.float64)
        self.state[:3] = position
        self.covariance = np.eye(6, dtype=np.float64) * initial_covariance
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

    def predict(self, dt: float) -> np.ndarray:
        dt = max(0.0, dt)
        transition = np.eye(6)
        transition[:3, 3:] = np.eye(3) * dt
        # Discrete white-acceleration process noise for [position, velocity].
        q = self.process_noise
        process = np.zeros((6, 6))
        process[:3, :3] = np.eye(3) * (dt ** 4 / 4.0) * q
        process[:3, 3:] = process[3:, :3] = np.eye(3) * (dt ** 3 / 2.0) * q
        process[3:, 3:] = np.eye(3) * (dt ** 2) * q
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process
        return self.state[:3].copy()

    def innovation(self, measurement: np.ndarray, noise: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, float]:
        """noise: optional 3x3 measurement covariance for this specific observation;
        falls back to the constant self.measurement_noise (Improvement #3: per-observation
        covariance derived from measured quality, not a fixed constant, when provided)."""
        residual = measurement - self.state[:3]
        measurement_covariance = noise if noise is not None else np.eye(3) * self.measurement_noise
        covariance = self.covariance[:3, :3] + measurement_covariance
        distance = float(residual.T @ np.linalg.solve(covariance, residual))
        return residual, covariance, distance

    def update(self, measurement: np.ndarray, noise: np.ndarray | None = None) -> np.ndarray:
        """Full 3D position measurement update; `noise` overrides the constant default."""
        observation = np.zeros((3, 6))
        observation[:, :3] = np.eye(3)
        measurement_covariance = noise if noise is not None else np.eye(3) * self.measurement_noise
        return self._update(observation, measurement, measurement_covariance)

    def update_axis(self, axis: int, value: float, variance: float) -> np.ndarray:
        """Partial 1-D update of a single position axis (0=X, 1=Y, 2=Z).

        Used to fuse an independent depth-derived Z into the position estimate
        after the full PnP update, without discarding the PnP-derived X/Y or
        double-counting the depth measurement across all three axes.
        """
        if axis not in (0, 1, 2):
            raise ValueError("axis must be 0 (X), 1 (Y), or 2 (Z).")
        observation = np.zeros((1, 6))
        observation[0, axis] = 1.0
        return self._update(observation, np.array([value]), np.array([[max(variance, 1e-9)]]))

    def _update(self, observation: np.ndarray, measurement: np.ndarray, noise: np.ndarray) -> np.ndarray:
        gain = self.covariance @ observation.T @ np.linalg.inv(observation @ self.covariance @ observation.T + noise)
        self.state += gain @ (measurement - observation @ self.state)
        identity = np.eye(6)
        # Joseph form is numerically more stable than (I-KH)P.
        residual = identity - gain @ observation
        self.covariance = residual @ self.covariance @ residual.T + gain @ noise @ gain.T
        return self.state[:3].copy()
