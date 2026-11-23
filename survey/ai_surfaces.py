"""Metric reconstruction from rectified, relative inverse-depth predictions."""
import base64
from functools import lru_cache
from pathlib import Path
import re
import tempfile

import cv2
import numpy as np
import pycolmap as sfm

from cloud_planes import build_planes
from fusion import load_observations, STREAMS

METHOD = "depth-anything-v2-v1"
LABEL = "AI depth planes"
DISTANCE_LABEL = "Maximum landmark gap"
W, H = 96, 72
F = 64.  # virtual pinhole focal length in depth-map pixels
MAX_VIEWS = 400  # upper bound of the operator's "Views to infer" setting


def index_path(run, index):
    if not isinstance(index,str) or not re.fullmatch(r"features-[0-9a-f]{32}\.npz",index):
        raise ValueError("Rebuild the camera map to save its lens parameters.")
    return run / index


def bundle_path(run,index):
    return run / ("depth-v1-" + index_path(run,index).name)


@lru_cache(maxsize=4)
def rectification(index):
    with np.load(index) as data:
        camera = sfm.Camera(model=str(data['camera_model']),width=640,height=480,params=data['params'])
    y,x = np.indices((H*6,W*6))
    rays = np.stack(((x+.5-W*3)/(F*6),(y+.5-H*3)/(F*6),np.ones_like(x)), axis=-1)
    return (camera.img_from_cam(rays.reshape(-1,3)).reshape(*x.shape,2)-.5).astype('f')


def rectified_jpeg(run,index,frame):
    image = cv2.imread(str(run/"frames"/frame))
    if image is None:
        raise ValueError("Recorded image unavailable.")
    rect = cv2.remap(image,rectification(index_path(run,index)),None,cv2.INTER_LINEAR)
    return cv2.imencode(".jpg",rect)[1].tobytes()


def save_predictions(run,model,predictions):
    if not isinstance(predictions,dict) or not 2 <= len(predictions) <= MAX_VIEWS:
        raise ValueError("Select at least two mapped depth views.")
    frames = {p['frame'] for p in model['poses'] if 'rotation' in p}
    arrays = {}
    for frame,encoded in predictions.items():
        if frame not in frames or not re.fullmatch(r'[0-9]{7}\.jpg',frame) or not isinstance(encoded,str):
            raise ValueError("Unknown depth frame.")
        raw = base64.b64decode(encoded,validate=True)
        if len(raw) != W*H:
            raise ValueError("Invalid prediction dimensions.")
        arrays[frame] = np.frombuffer(raw,np.uint8).reshape(H,W)
    dest = bundle_path(run,model['feature_index'])
    with tempfile.NamedTemporaryFile(dir=run,suffix=".npz",delete=False) as tmp:
        stage = Path(tmp.name)
    try:
        np.savez_compressed(stage,**arrays)
        stage.replace(dest)
    finally:
        stage.unlink(missing_ok=True)


def cached_frames(run,index):
    path = bundle_path(run,index)
    if not path.exists():
        return []
    with np.load(path) as bundle:
        return sorted(bundle.files)


def preview_png(run,index,frame):
    with np.load(bundle_path(run,index)) as bundle:
        return cv2.imencode(".png",bundle[frame])[1].tobytes()


def projection(world,pose):
    camera = (world - pose['xyz']) @ np.asarray(pose['rotation'])
    z = camera[:,2]
    uv = F*camera[:,:2] / np.maximum(z[:,None],1e-6) + [W/2,H/2]
    valid = (z>.15)&(z<8)&np.all((uv>1)&(uv<[W-1,H-1]),axis=1)
    pixels = np.clip(uv,[0,0],[W-1,H-1]).astype(int)
    return pixels,z,valid


def metric_fit(pred,z):
    """Robust affine disparity fit with spatially distributed held-out landmarks."""
    if len(z)<15:
        return None
    hold = np.arange(len(z))%4 == 0
    x,y = pred[~hold],1/z[~hold]
    if np.ptp(np.percentile(x,[10,90])) < 5:
        return None
    rng = np.random.default_rng(24)
    a,b = rng.integers(len(x),size=(2,220))
    valid = abs(x[b]-x[a])>4
    slopes = (y[b[valid]]-y[a[valid]])/(x[b[valid]]-x[a[valid]])
    intercepts = y[a[valid]]-slopes*x[a[valid]]
    errors = abs(slopes[:,None]*x+intercepts[:,None]-y)/y
    if not np.any(slopes>0):
        return None
    best = np.argmin(np.where(slopes>0,np.median(errors,axis=1),np.inf))
    inliers = errors[best]<.18
    if inliers.sum()<8:
        return None
    slope,offset = np.polyfit(x[inliers],y[inliers],1)
    disparity = slope*pred[hold]+offset
    if not np.isfinite([slope,offset]).all() or slope<=0 or np.median(abs(disparity*z[hold]-1))>.15 or np.mean(abs(disparity*z[hold]-1)<.25)<.65:
        return None
    return slope,offset


def build_ai(run,model,span):
    index = model.get("feature_index")
    blank = dict(method=METHOD,units=model["units"],growth_distance_m=span,patches=[],retired=[])
    if model['units']!='m' or not index or not bundle_path(run,index).exists():
        return dict(blank,status="needs_predictions")
    observations = load_observations(run,model.get("streams",STREAMS))
    xyz = np.asarray(model['points'],float)[:,:3]
    poses = {p['frame']:p for p in model['poses']}
    views = []
    tof_rejected = 0
    verdicts = {}
    with np.load(bundle_path(run,index)) as predictions:
        frames = sorted(predictions.files)
        for frame in frames:
            verdicts[frame] = 'Insufficient landmark support'
            pose = poses.get(frame)
            if not pose or 'rotation' not in pose:
                continue
            ids = model.get('landmark_views',{}).get(frame,[])
            pixels,z,valid = projection(xyz[ids],pose)
            x,y = pixels[valid].T
            # Sort by location before splitting holdouts and count each image cell once.
            _,idx = np.unique(y*W+x,return_index=True)
            data = predictions[frame].astype(float)
            samples = data[y[idx],x[idx]]
            if len(samples)<15 or np.ptp(x[idx])<W*.15 or np.ptp(y[idx])<H*.15:
                continue
            fit = metric_fit(samples,z[valid][idx])
            if fit is None:
                verdicts[frame] = "Depth fit rejected by landmarks"
                continue
            inv = data*fit[0]+fit[1]
            depth = np.divide(1.,inv,out=np.zeros_like(inv),where=inv>.125)
            # Limit extrapolation into disparity ranges that the landmarks never sampled.
            lo,hi = np.percentile(samples,[2,98])
            margin = .2*(hi-lo)
            depth[(data < lo-margin) | (data > hi+margin)] = 0
            r = observations.get(frame,{}).get('range')
            if r is not None and r>=.25 and abs(np.median(depth[H//2-1:H//2+2,W//2-1:W//2+2])-r)>max(.20,.25*r):
                verdicts[frame] = "Rejected by center-beam range check"
                tof_rejected += 1
                continue
            verdicts[frame] = "Depth aligned to map"
            views.append((frame,pose,depth))
    # Sample each aligned image, then test its 3D predictions in other measured poses.
    # A shorter return there can be an occluder. A longer return disputes the point.
    yy,xx = np.mgrid[2:H:4,2:W:4]
    rays = np.stack(((xx+.5-W/2)/F,(yy+.5-H/2)/F,np.ones_like(xx)),axis=-1).reshape(-1,3)
    voxels = {}
    for frame,pose,depth in views:
        z = depth[yy,xx].ravel()
        world = (rays*z[:,None]) @ np.asarray(pose['rotation']).T + pose['xyz']
        corroborated = np.zeros(len(z),bool)
        disputed = np.zeros(len(z),int)
        confirmed_at = np.full(len(z),"",dtype="<U16")
        for other,eye,other_depth in views:
            baseline = np.linalg.norm(np.subtract(eye['xyz'],pose['xyz']))
            if not .08 <= baseline <= 2.5:
                continue
            uv,expected,inside = projection(world,eye)
            measured = other_depth[uv[:,1],uv[:,0]]
            ok = inside & (measured>.15)
            limit = .06+.065*expected
            match = ok & (abs(measured-expected)<limit)
            confirmed_at[match & ~corroborated] = max(frame,other)
            corroborated |= match
            disputed += ok & (measured > expected+2*limit)
        for p,t in zip(world[corroborated & (disputed==0) & (z>.15)],confirmed_at[corroborated & (disputed==0) & (z>.15)]):
            cell = tuple(np.floor(p/.09).astype(int))
            if cell not in voxels or t<voxels[cell][1]:
                voxels[cell] = (p.tolist(),str(t))
    stats = dict(frames=frames,verdicts=verdicts,aligned_views=len(views),tof_rejected=tof_rejected,confirmed_points=len(voxels))
    if len(voxels)<16:
        return dict(blank,status="insufficient_support",ai=stats)
    # Fit plates from corroborated model predictions. Keep synthetic sample IDs
    # private to this fit; exported landmark IDs address the recorded visual map.
    seen = {}
    points = []
    for p,t in voxels.values():
        seen.setdefault(t,[]).append(len(points))
        points.append(p)
    mesh = build_planes(dict(model,points=points,landmark_views=seen),span,observations)
    for face in mesh["patches"]:
        face["depth_samples"] = len(face["landmarks"])
        face["landmarks"] = []
    mesh.update(method=METHOD,ai=stats)
    return mesh
