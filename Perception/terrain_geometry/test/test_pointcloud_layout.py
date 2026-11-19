"""PointCloud2 wire layouts must preserve geometry, padding and source headers."""
import struct
import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField
from terrain_geometry.tf_transform import CloudFrameTransformer, pointcloud2_to_xyz_array


def cloud(big_endian=False):
    order = '>' if big_endian else '<'
    fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z'))]
    fields.append(PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1))
    rows = [struct.pack(order + '3fI', *point, tag) + b'padding!'
            for point, tag in [((1., 2., 3.), 123), ((4., 5., 6.), 456)]]
    result = PointCloud2(height=2, width=1, fields=fields, point_step=16, row_step=24,
                        is_bigendian=big_endian, data=b''.join(rows))
    result.header.frame_id = 'camera'
    result.header.stamp.sec = 1
    return result


@pytest.mark.parametrize('big_endian', [False, True])
def test_decode_organized_padded_cloud(big_endian):
    np.testing.assert_allclose(pointcloud2_to_xyz_array(cloud(big_endian)), [[1., 2., 3.], [4., 5., 6.]])


@pytest.mark.parametrize('big_endian', [False, True])
def test_transform_preserves_padding_extra_fields_and_input_header(big_endian):
    source = cloud(big_endian)
    transformer = CloudFrameTransformer.__new__(CloudFrameTransformer)
    transformer._dtype_cache = {}
    matrix = np.eye(4)
    matrix[0, 3] = 10.
    result = transformer._apply_matrix(source, matrix, 'base_link')
    np.testing.assert_allclose(pointcloud2_to_xyz_array(result), [[11., 2., 3.], [14., 5., 6.]])
    assert source.header.frame_id == 'camera'
    assert result.header.frame_id == 'base_link'
    assert result.row_step == source.row_step and result.is_bigendian == big_endian
    for offset in (0, 24):
        assert bytes(result.data[offset + 12:offset + 24]) == bytes(source.data[offset + 12:offset + 24])


@pytest.mark.parametrize('defect', ['datatype', 'missing_z', 'array_x', 'overflow', 'short_data', 'row_step', 'duplicate'])
def test_malformed_layout_is_rejected_with_value_error(defect):
    value = cloud()
    if defect == 'datatype':
        value.fields[0].datatype = 99
    elif defect == 'missing_z':
        value.fields = [f for f in value.fields if f.name != 'z']
    elif defect == 'array_x':
        value.fields[0].count = 2
    elif defect == 'overflow':
        value.fields[0].offset = 16
    elif defect == 'short_data':
        value.data = bytes(value.data[:-1])
    elif defect == 'row_step':
        value.row_step = 8
    else:
        value.fields[1].name = 'x'
    with pytest.raises(ValueError):
        pointcloud2_to_xyz_array(value)
