import numpy as np
import pytest
import rclpy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from rclpy.parameter import Parameter
from terrain_geometry.occupancy_grid import OccupancyGridGenerator
from terrain_geometry.terrain_node import TerrainGeometryNode
from terrain_geometry.tf_transform import CloudFrameTransformer


def grid():
    return OccupancyGridGenerator(0.1, 10, 10, 0., 0., 0., 0., use_unknown_space=True,
        known_region_min_x=0., known_region_max_x=1., known_region_min_y=0., known_region_max_y=1.)


def test_unobserved_rectangle_is_not_free():
    msg, _ = grid().generate([], Header())
    assert set(msg.data) == {-1}


def test_only_finite_observed_ground_cells_are_free():
    msg, _ = grid().generate([], Header(), np.array([[0.25, 0.35, 0.], [np.nan, 0., 0.], [2., 2., 0.]]))
    assert list(msg.data).count(0) == 1
    assert msg.data[3 * 10 + 2] == 0


def test_legacy_free_space_requires_explicit_choice():
    generator = grid()
    generator.use_unknown_space = False
    assert set(generator.generate([], Header())[0].data) == {0}


@pytest.fixture
def node():
    rclpy.init()
    value = TerrainGeometryNode()
    yield value
    value.destroy_node()
    rclpy.shutdown()


def test_invalid_cloud_does_not_publish_valid_empty(node):
    observations = []
    node._publish_empty = lambda *args: observations.append(args)
    cloud = PointCloud2()
    cloud.header.stamp = node.get_clock().now().to_msg()
    node._cloud_callback(cloud)
    assert observations == []
    assert node._health_ok is False


def test_failure_grid_uses_target_frame_and_unknown(node):
    grids = []
    node._costmap_pub = type('Capture', (), {'publish': lambda self, msg: grids.append(msg)})()
    node._enable_costmap = True
    cloud = PointCloud2()
    cloud.header.frame_id = 'camera_optical'
    cloud.header.stamp = node.get_clock().now().to_msg()
    node._unavailable(cloud, 'invalid test input')
    assert grids[0].header.frame_id == 'base_link'
    assert set(grids[0].data) == {-1}
