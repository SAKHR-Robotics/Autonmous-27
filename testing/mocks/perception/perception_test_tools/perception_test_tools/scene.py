"""Deterministic geometric fixtures, not photorealistic terrain simulation."""
import cv2
import numpy as np
from scipy.spatial.transform import Rotation

SCENARIOS = (
    'rocks_and_marker',
    'ground_only',
    'missing_depth',
    'invalid_cloud',
    'dropout',
    'turn_occlusion',
    'terrain_hazard',
    'clear_aruco_nav',
)
# Optical X right, Y down, Z forward -> base X forward, Y left, Z up.
OPTICAL_TO_BASE = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
MOUNT = np.array([.2, 0., .4])
MARKER_SIZE_M = 0.21

_CLEAR_ARUCO_T = np.array([0.15, -0.05, 1.50], dtype=float)
_CLEAR_ARUCO_R = (
    Rotation.from_euler('yx', [32.0, 12.0], degrees=True).as_matrix()
    @ np.diag([1.0, -1.0, -1.0])
)


def _generate_aruco_tile(marker_id=0, side_px=140, pad_px=30):
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    marker = (
        cv2.aruco.generateImageMarker(dictionary, marker_id, side_px)
        if hasattr(cv2.aruco, 'generateImageMarker')
        else cv2.aruco.drawMarker(dictionary, marker_id, side_px)
    )
    tile_size = side_px + 2 * pad_px
    tile = np.full((tile_size, tile_size, 3), 255, dtype=np.uint8)
    tile[pad_px:pad_px + side_px, pad_px:pad_px + side_px] = marker[:, :, None]
    src_pts = np.array(
        [
            [pad_px - 0.5, pad_px - 0.5],
            [pad_px + side_px - 0.5, pad_px - 0.5],
            [pad_px + side_px - 0.5, pad_px + side_px - 0.5],
            [pad_px - 0.5, pad_px + side_px - 0.5],
        ],
        dtype=np.float32,
    )
    return tile, src_pts


def _render_tilted_marker(image, depth):
    tile, src_pts = _generate_aruco_tile(marker_id=0, side_px=140, pad_px=30)
    half = MARKER_SIZE_M / 2.0
    obj_pts = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=float,
    )
    cam_pts = (_CLEAR_ARUCO_R @ obj_pts.T).T + _CLEAR_ARUCO_T
    dst_pts = np.column_stack(
        [
            600.0 * cam_pts[:, 0] / cam_pts[:, 2] + 320.0,
            600.0 * cam_pts[:, 1] / cam_pts[:, 2] + 240.0,
        ]
    ).astype(np.float32)

    homography = cv2.getPerspectiveTransform(src_pts, dst_pts)
    warped = cv2.warpPerspective(
        tile,
        homography,
        (640, 480),
        flags=cv2.INTER_LINEAR,
        borderValue=(255, 255, 255),
    )
    image[:] = warped

    mask = np.zeros((480, 640), dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.rint(dst_pts).astype(np.int32), 1)
    vs, us = np.where(mask == 1)
    normal = _CLEAR_ARUCO_R[:, 2]
    plane_d = float(np.dot(normal, _CLEAR_ARUCO_T))
    rays_x = (us.astype(float) - 320.0) / 600.0
    rays_y = (vs.astype(float) - 240.0) / 600.0
    denom = normal[0] * rays_x + normal[1] * rays_y + normal[2]
    valid = np.abs(denom) > 1e-6
    depth[vs[valid], us[valid]] = (plane_d / denom[valid]).astype(np.float32)


def scene_ground_truth(scenario='rocks_and_marker', tick=0, ground_z_offset=0.0):
    """Return exact geometric truth for obstacles (base_link) and markers (optical)."""
    if scenario not in SCENARIOS:
        raise ValueError('Unknown scenario')
    gz = float(ground_z_offset)
    visible = (
        scenario not in ('ground_only', 'invalid_cloud')
        and not (scenario == 'turn_occlusion' and tick % 40 >= 20)
    )

    obstacles = []
    if visible:
        if scenario == 'clear_aruco_nav':
            obstacles.append({
                'id': 0,
                'frame_id': 'base_link',
                'center': [2.2, 0.7, gz + 0.25],
                'size': [0.4, 0.4, 0.4],
            })
        else:
            obstacles.append({
                'id': 0,
                'frame_id': 'base_link',
                'center': [2.0, 0.6, gz + 0.25],
                'size': [0.4, 0.4, 0.4],
            })

    markers = []
    if visible:
        if scenario == 'clear_aruco_nav':
            quat = Rotation.from_matrix(_CLEAR_ARUCO_R).as_quat().tolist()
            markers.append({
                'id': 0,
                'frame_id': 'test_camera_optical_frame',
                'position': _CLEAR_ARUCO_T.tolist(),
                'quaternion_xyzw': [float(v) for v in quat],
                'size_m': MARKER_SIZE_M,
                'expected_ambiguous': False,
            })
        else:
            quat = Rotation.from_matrix(np.diag([1.0, -1.0, -1.0])).as_quat().tolist()
            markers.append({
                'id': 0,
                'frame_id': 'test_camera_optical_frame',
                'position': [0.0, 0.0, 2.0],
                'quaternion_xyzw': [float(v) for v in quat],
                'size_m': MARKER_SIZE_M,
                'expected_ambiguous': True,
            })

    return {
        'scenario': scenario,
        'tick': int(tick),
        'ground_z_offset': gz,
        'visible': bool(visible),
        'obstacles': obstacles,
        'markers': markers,
    }


def make_scene(scenario='rocks_and_marker', seed=0, tick=0, ground_z_offset=0.0):
    if scenario not in SCENARIOS:
        raise ValueError('Unknown scenario')
    rng = np.random.default_rng(seed)
    gz = float(ground_z_offset)
    x, y = np.meshgrid(np.arange(.3, 6., .05), np.arange(-2., 2., .05))
    z = np.full_like(x, gz)
    if scenario == 'terrain_hazard':
        z = np.where((x > 2) & (x < 3), gz + .5 * (x - 2), gz)
    ground = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
    visible = scenario != 'ground_only' and not (scenario == 'turn_occlusion' and tick % 40 >= 20)
    points = ground
    if visible:
        if scenario == 'clear_aruco_nav':
            rock = rng.uniform([2.0, .5, gz + .05], [2.4, .9, gz + .45], (1800, 3))
        else:
            rock = rng.uniform([1.8, .4, gz + .05], [2.2, .8, gz + .45], (1800, 3))
        points = np.vstack((ground, rock))
    optical = (points - MOUNT) @ OPTICAL_TO_BASE
    if scenario == 'invalid_cloud':
        optical[:] = np.nan
    image = np.full((480, 640, 3), 255, dtype=np.uint8)
    depth = np.full((480, 640), np.nan, dtype=np.float32)
    finite = np.isfinite(optical).all(axis=1) & (optical[:, 2] > .1)
    valid = optical[finite]
    u = np.rint(600 * valid[:, 0] / valid[:, 2] + 320).astype(int)
    v = np.rint(600 * valid[:, 1] / valid[:, 2] + 240).astype(int)
    inside = (u >= 0) & (u < 640) & (v >= 0) & (v < 480)
    # Far-to-near assignment provides a basic depth buffer for projected geometry.
    order = np.argsort(valid[inside, 2])[::-1]
    depth[v[inside][order], u[inside][order]] = valid[inside, 2][order]
    if visible:
        if scenario == 'clear_aruco_nav':
            _render_tilted_marker(image, depth)
        else:
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            marker = (
                cv2.aruco.generateImageMarker(dictionary, 0, 63)
                if hasattr(cv2.aruco, 'generateImageMarker')
                else cv2.aruco.drawMarker(dictionary, 0, 63)
            )
            image[208:271, 288:351] = marker[:, :, None]
            depth[208:271, 288:351] = 2.
    return optical.astype(np.float32), image, depth, visible
