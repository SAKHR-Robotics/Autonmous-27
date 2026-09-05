from marker_detection.camera_model import RgbCameraModel


def test_pinhole_deprojection_uses_rgb_intrinsics():
    model = RgbCameraModel(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
    assert model.deproject(380.0, 300.0, 2.0) == (0.2, 0.2, 2.0)
