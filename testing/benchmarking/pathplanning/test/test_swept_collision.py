"""Independent geometry examples, including obstacles between path samples."""
import math
import numpy as np
from PIL import Image
import pytest
import yaml
from global_path_benchmarking.navigation_probe import swept_collision


def truth(tmp_path, blocked=(), angle=0.0):
    image = np.full((40, 40), 255, dtype=np.uint8)
    for x, y in blocked:
        image[39-y, x] = 0
    Image.fromarray(image).save(tmp_path/'map.pgm')
    spec = {'image': 'map.pgm', 'resolution': 0.1, 'origin': [0., 0., angle],
            'negate': 0, 'free_thresh': 0.196, 'occupied_thresh': 0.65}
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(spec))
    return str(tmp_path/'map.yaml')


def test_free_route(tmp_path):
    assert swept_collision([(1., 1., 0.), (3., 1., 0.)], truth(tmp_path)) is False


def test_obstacle_between_path_vertices(tmp_path):
    assert swept_collision([(1., 1., 0.), (3., 1., 0.)], truth(tmp_path, [(20, 10)])) is True


def test_body_collision_away_from_centerline(tmp_path):
    assert swept_collision([(1., 1., 0.), (3., 1., 0.)], truth(tmp_path, [(20, 13)])) is True


def test_rotated_map_and_outside_bounds(tmp_path):
    path = [(-1., 1., math.pi/2), (-1., 3., math.pi/2)]
    assert swept_collision(path, truth(tmp_path, angle=math.pi/2)) is False
    assert swept_collision([(0., 0., 0.)], truth(tmp_path)) is True


@pytest.mark.parametrize('path', [[], [(float('nan'), 1., 0.)]])
def test_missing_or_nonfinite_geometry_is_unavailable(tmp_path, path):
    assert swept_collision(path, truth(tmp_path)) is None
