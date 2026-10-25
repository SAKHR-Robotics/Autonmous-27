import numpy as np
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
from vision_msgs.msg import Detection3D
from terrain_geometry.obstacle_memory import ObstacleMemory,transform_detection


def box(x=2.):
    b=Detection3D()
    b.bbox.center.position.x=x
    b.bbox.center.position.z=.2
    b.bbox.center.orientation.w=1.
    b.bbox.size.x=b.bbox.size.y=b.bbox.size.z=.4
    return b


def test_memory_expires_associates_and_resets():
    memory=ObstacleMemory(ttl=2)
    memory.update([box()],1.)
    memory.update([box(2.1)],1.5)
    assert len(memory.snapshot(2.)) == 1
    assert memory.snapshot(4.) == []
    memory.update([box()],5.)
    assert memory.snapshot(1.) == []


def test_turn_changes_coordinates_in_stable_frame():
    tf=TransformStamped()
    tf.transform.rotation.z=tf.transform.rotation.w=2**-.5
    moved=transform_detection(box(),tf,'odom',Time(sec=1))
    np.testing.assert_allclose([moved.bbox.center.position.x,moved.bbox.center.position.y],[0.,2.],atol=1e-8)
    assert moved.header.frame_id=='odom'


def test_only_observed_ground_clears_memory():
    memory=ObstacleMemory()
    memory.update([box()],1.)
    memory.clear_observed_ground(np.array([[5.,0.,0.]]*3))
    assert len(memory.snapshot(1.1))==1
    memory.clear_observed_ground(np.array([[2.,0.,0.]]*3))
    assert memory.snapshot(1.2)==[]
