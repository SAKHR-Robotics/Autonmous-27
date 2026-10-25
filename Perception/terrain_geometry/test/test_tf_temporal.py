import numpy as np
from terrain_geometry.tf_transform import CloudFrameTransformer


def test_dynamic_transform_not_reused_without_opt_in():
    transformer = CloudFrameTransformer.__new__(CloudFrameTransformer)
    transformer.cache_static_transform = False
    transformer.transform_matrix = np.eye(4)
    transformer._cached_source_frame = 'camera'
    transformer._cached_target_frame = 'base_link'
    calls = []
    transformer.lookup_transform = lambda *args: calls.append(args)
    assert transformer._get_transform_matrix('base_link', 'camera', None) is None
    assert len(calls) == 1
    transformer.cache_static_transform = True
    assert transformer._get_transform_matrix('base_link', 'camera', None) is transformer.transform_matrix


