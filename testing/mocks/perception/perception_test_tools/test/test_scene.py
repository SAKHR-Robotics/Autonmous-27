import numpy as np
from perception_test_tools.scene import make_scene,OPTICAL_TO_BASE,MOUNT


def test_deterministic_calibrated_inputs():
    a,image,depth,_=make_scene()
    b,_,_,_=make_scene()
    np.testing.assert_array_equal(a,b)
    assert image.shape==(480,640,3)
    assert depth.dtype==np.float32
    assert depth[240,320]==2.
    base=a@OPTICAL_TO_BASE.T+MOUNT
    assert np.max(base[:,2])>.4


def test_ground_invalid_and_occlusion_cases():
    cloud,_,_,visible=make_scene('ground_only')
    assert not visible
    np.testing.assert_allclose((cloud@OPTICAL_TO_BASE.T+MOUNT)[:,2],0.,atol=1e-6)
    assert not np.isfinite(make_scene('invalid_cloud')[0]).any()
    assert not make_scene('turn_occlusion',tick=25)[3]
