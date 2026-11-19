"""Core regressions; runnable without ROS from the repository root."""
import math
import pytest
from rover_slam.wheel_odometry import WheelOdometry

NAMES = ['left_front','right_front','left_rear','right_rear']

def sample(value): return {n:value for n in NAMES}
def core(): return WheelOdometry(NAMES, .001, .5)

def test_sensor_interval_not_publication_interval():
    wheel = core()
    assert wheel.observe(sample(0), 1.) is None
    result = wheel.observe(sample(100), 1.1)
    assert result[0] == pytest.approx(1.)
    assert wheel.x == pytest.approx(.1)
    assert wheel.observe(sample(100), 1.1) is None
    assert wheel.x == pytest.approx(.1)

def test_gap_does_not_integrate_unobserved_motion():
    wheel = core(); wheel.observe(sample(0), 1.)
    assert wheel.observe(sample(500), 2.) is None
    assert wheel.x == 0.
    assert wheel.observe(sample(510), 2.1)[0] == pytest.approx(.1)

def test_counter_jump_rebaselines_and_recovers():
    wheel = core(); wheel.observe(sample(10000), 1.)
    assert wheel.observe(sample(0), 1.1) is None
    assert wheel.x == 0.
    assert wheel.observe(sample(10), 1.2)[0] == pytest.approx(.1)

def test_missing_or_invalid_wheel_is_not_zero_motion():
    wheel = core(); wheel.observe(sample(0), 1.)
    assert wheel.observe({'left_front':1}, 1.1) is None
    bad=sample(1); bad['right_front']=float('nan')
    assert wheel.observe(bad, 1.1) is None
    assert wheel.stamp == 1.

def test_backward_samples_do_not_corrupt_baseline():
    wheel = core(); wheel.observe(sample(100), 2.)
    assert wheel.observe(sample(0), 1.) is None
    assert wheel.observe(sample(110), 2.1)[0] == pytest.approx(.1)
    wheel.reset_measurements()
    assert wheel.observe(sample(0), .1) is None
    assert wheel.observe(sample(10), .2)[0] == pytest.approx(.1)

def test_turn_and_invalid_calibration():
    wheel=core(); wheel.observe(sample(0),1.)
    counts={n:10 if 'right' in n else 0 for n in NAMES}
    wheel.observe(counts,1.1)
    assert wheel.yaw == pytest.approx(.02)
    assert wheel.y > 0.
    with pytest.raises(ValueError): WheelOdometry(NAMES,.001,0.)
