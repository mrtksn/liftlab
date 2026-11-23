"""Estimate replay camera poses from depth-image correspondences to the saved map."""
from functools import lru_cache
from pathlib import Path
import time

import cv2
import numpy as np

from gaussian_map import WIDTH, HEIGHT, K


def estimate(run, model, folder, method):
    from depth_cloud import align_depth
    start = time.perf_counter()
    names = sorted(p.name for p in (run/'frames').glob('[0-9]'*7+'.jpg'))
    anchors = {p['frame']:p for p in model['poses'] if 'rotation' in p}
    xyz = np.asarray(model['points'],float)[:,:3]
    tracks = model.get('landmark_views',{})
    indices = np.array([i for i,name in enumerate(names) if name in anchors and len(tracks.get(name,[]))>=20])
    if not len(indices):
        raise ValueError('This walk has too few mapped landmark tracks for depth path regeneration.')

    @lru_cache(maxsize=20)
    def image(frame):
        raw = np.load(folder/Path(frame).with_suffix('.npy')).astype(np.float32)
        valid = np.isfinite(raw)
        if method!='anything':
            valid &= raw>0
            raw = np.divide(1.,raw,out=np.zeros_like(raw),where=valid)
        pixels = np.zeros(raw.shape,np.uint8)
        if valid.any():
            lo,hi = np.percentile(raw[valid],[2,98])
            if hi>lo:
                pixels[valid] = np.clip((raw[valid]-lo)*(255/(hi-lo)),0,255).astype(np.uint8)
        return cv2.resize(pixels,(WIDTH,HEIGHT),interpolation=cv2.INTER_LINEAR)

    @lru_cache(maxsize=16)
    def reference(frame):
        pose = anchors[frame]
        world = xyz[tracks[frame]]
        cam = (world-pose['xyz'])@pose['rotation']
        positive = cam[:,2]>1e-6
        world,cam = world[positive],cam[positive]
        uv = cam[:,:2]/cam[:,2,None]*K[0,0]+K[:2,2]
        valid = ((uv>[5,5]) & (uv<[WIDTH-6,HEIGHT-6])).all(axis=1)
        world_features,pixels,depths = world[valid],uv[valid],cam[valid,2]
        raw = np.load(folder/Path(frame).with_suffix('.npy')).astype(np.float32)
        depth = align_depth(raw,method,model['units'],cam) if len(cam) else None
        corners = cv2.goodFeaturesToTrack(image(frame),400,.007,5)
        if depth is not None and corners is not None:
            xy = corners.reshape(-1,2)
            h,w = depth.shape
            x,y = ((xy+.5)*[w/WIDTH,h/HEIGHT]).astype(int).T
            z = depth[y.clip(0,h-1),x.clip(0,w-1)]
            good = np.isfinite(z)&(z>0)&(z<np.percentile(cam[:,2],95)*2)
            xy,z = xy[good],z[good]
            local = np.column_stack(((xy-K[:2,2])/K[0,0]*z[:,None],z))
            world_features = np.vstack((world_features,local@np.asarray(pose['rotation']).T+pose['xyz']))
            pixels,depths = np.vstack((pixels,xy)),np.r_[depths,z]
        return world_features,np.ascontiguousarray(pixels,np.float32),depths

    poses = []
    for i,frame in enumerate(names):
        # Never use the target's own saved pose to claim a successful estimate.
        candidates = indices[(indices!=i)&(np.abs(indices-i)<=10)]
        for j in sorted(candidates,key=lambda j:abs(i-j))[:4]:
            anchor = names[j]
            world,uv,z = reference(anchor)
            if len(uv)<20:
                continue
            source,target = image(anchor),image(frame)
            flow,ok,_ = cv2.calcOpticalFlowPyrLK(source,target,uv,None)
            if flow is None:
                continue
            back,reverse,_ = cv2.calcOpticalFlowPyrLK(target,source,flow,None)
            if back is None:
                continue
            good = ok.ravel().astype(bool)&reverse.ravel().astype(bool)&(np.linalg.norm(back-uv,axis=1)<.8)
            if good.sum()<20:
                continue
            obj,pix = world[good],flow[good]
            # Hold out every fifth match for a separate reprojection check.
            hold = np.arange(len(obj))%5==0
            ok,r,t,inliers = cv2.solvePnPRansac(obj[~hold],pix[~hold],K,None,iterationsCount=80,
                                               reprojectionError=2.,flags=cv2.SOLVEPNP_EPNP)
            if not ok or len(inliers)<12 or len(inliers)<.65*(~hold).sum():
                continue
            fitted = pix[~hold][inliers.ravel()]
            if np.any(np.ptp(fitted,axis=0)<[WIDTH*.18,HEIGHT*.18]):
                continue
            R = cv2.Rodrigues(r)[0].T
            eye = (-R@t).ravel()
            check = (obj[hold]-eye)@R
            projection = check[:,:2]/check[:,2,None]*K[0,0]+K[:2,2]
            if (np.any(check[:,2]<=0) or np.median(np.linalg.norm(projection-pix[hold],axis=1))>2.5
                    or np.linalg.norm(eye-anchors[anchor]['xyz'])>.3*np.median(z)
                    or np.trace(R.T@anchors[anchor]['rotation'])<2.4):
                continue
            poses.append(dict(frame=frame,xyz=eye.tolist(),rotation=R.tolist(),forward=R[:,2].tolist(),
                              method='depth_track',anchor=anchor))
            break
    if len(poses)<3:
        raise ValueError('Fewer than three depth-image poses passed geometric verification.')
    return dict(poses=poses,frames=len(names),seconds=time.perf_counter()-start)
