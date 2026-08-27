"""Minimal RGB optical-camera pinhole model for aligned depth deprojection."""
from __future__ import annotations
from dataclasses import dataclass
from sensor_msgs.msg import CameraInfo


@dataclass(frozen=True)
class RgbCameraModel:
    """Intrinsics from the RGB CameraInfo, never from the depth camera."""
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_camera_info(cls, info: CameraInfo) -> "RgbCameraModel | None":
        if len(info.k) != 9 or info.width <= 0 or info.height <= 0:
            return None
        fx, fy, cx, cy = float(info.k[0]), float(info.k[4]), float(info.k[2]), float(info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        return cls(fx, fy, cx, cy)

    def deproject(self, u: float, v: float, depth_m: float) -> tuple[float, float, float]:
        """Return X-right, Y-down, Z-forward in the RGB optical frame."""
        return ((u - self.cx) * depth_m / self.fx,
                (v - self.cy) * depth_m / self.fy,
                depth_m)
