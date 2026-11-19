"""Deterministic geometric fixtures, not photorealistic terrain simulation."""
import cv2
import numpy as np

SCENARIOS=('rocks_and_marker','ground_only','missing_depth','invalid_cloud','dropout','turn_occlusion','terrain_hazard')
# Optical X right, Y down, Z forward -> base X forward, Y left, Z up.
OPTICAL_TO_BASE=np.array([[0,0,1],[-1,0,0],[0,-1,0]],dtype=float)
MOUNT=np.array([.2,0.,.4])


def make_scene(scenario='rocks_and_marker',seed=0,tick=0):
    if scenario not in SCENARIOS:
        raise ValueError('Unknown scenario')
    rng=np.random.default_rng(seed)
    x,y=np.meshgrid(np.arange(.3,6.,.05),np.arange(-2.,2.,.05))
    z=np.zeros_like(x)
    if scenario=='terrain_hazard':
        z=np.where((x>2)&(x<3),.5*(x-2),0.)
    ground=np.column_stack((x.ravel(),y.ravel(),z.ravel()))
    visible=scenario!='ground_only' and not(scenario=='turn_occlusion' and tick%40>=20)
    points=ground
    if visible:
        rock=rng.uniform([1.8,.4,.05],[2.2,.8,.45],(1800,3))
        points=np.vstack((ground,rock))
    optical=(points-MOUNT)@OPTICAL_TO_BASE
    if scenario=='invalid_cloud':
        optical[:]=np.nan
    image=np.full((480,640,3),255,dtype=np.uint8)
    depth=np.full((480,640),np.nan,dtype=np.float32)
    finite=np.isfinite(optical).all(axis=1)&(optical[:,2]>.1)
    valid=optical[finite]
    u=np.rint(600*valid[:,0]/valid[:,2]+320).astype(int)
    v=np.rint(600*valid[:,1]/valid[:,2]+240).astype(int)
    inside=(u>=0)&(u<640)&(v>=0)&(v<480)
    # Far-to-near assignment provides a basic depth buffer for projected geometry.
    order=np.argsort(valid[inside,2])[::-1]
    depth[v[inside][order],u[inside][order]]=valid[inside,2][order]
    if visible:
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        marker=(cv2.aruco.generateImageMarker(dictionary,0,63) if hasattr(cv2.aruco,'generateImageMarker')
                else cv2.aruco.drawMarker(dictionary,0,63))
        image[208:271,288:351]=marker[:,:,None]
        depth[208:271,288:351]=2.
    return optical.astype(np.float32),image,depth,visible
