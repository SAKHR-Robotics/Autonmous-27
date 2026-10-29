import math
import pytest
from rover_slam.heuristic_slip_checker import (SlipCheckerCore,inflate_covariance,
    rotate_vector,normalize_quaternion,quaternion_product)

def test_yaw_threshold_is_effective():
    args=dict(v_wheel=.1,w_wheel=.6,w_imu=.0,a_imu_x=0.,current_time_sec=1.)
    assert SlipCheckerCore(slip_rot_threshold=.1).evaluate(**args)[0]
    assert not SlipCheckerCore(slip_rot_threshold=1.).evaluate(**args)[0]

def test_clock_reversal_clears_slip_hold():
    core=SlipCheckerCore()
    core.evaluate(0.,1.,0.,0.,10.)
    assert not core.evaluate(0.,0.,0.,0.,1.)[0]

def test_nonfinite_input_is_rejected():
    with pytest.raises(ValueError): SlipCheckerCore().evaluate(float('nan'),0.,0.,0.,1.)

def test_covariance_preserves_cross_terms_and_never_reduces_input_variance():
    original=[0.]*36; original[0]=4.; original[35]=5.; original[5]=original[30]=.1
    result=inflate_covariance(original,100.,.02,.05)
    assert result[0]==400. and result[35]==500. and result[5]==10.
    assert original[0]==4.
    with pytest.raises(ValueError): inflate_covariance([float('nan')]*36,100.,.02,.05)

def test_mount_rotation_and_orientation_composition():
    q=(0.,0.,math.sin(math.pi/4),math.cos(math.pi/4))
    assert rotate_vector(q,(1.,0.,0.)) == pytest.approx((0.,1.,0.))
    assert quaternion_product(q,(-q[0],-q[1],-q[2],q[3])) == pytest.approx((0.,0.,0.,1.))
    with pytest.raises(ValueError): normalize_quaternion((0.,0.,0.,0.))
