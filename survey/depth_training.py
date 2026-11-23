"""Walk-specific robust depth calibration before backprojection."""
import numpy as np
from scipy.optimize import least_squares


def features(u, v, z):
    return np.stack([np.ones_like(z), u, v, u*v, u*u, v*v, np.log(z)], axis=-1)


def train(model, folder):
    from depth_cloud import align_depth, F, W
    from pathlib import Path
    xyz = np.asarray(model['points'])[:, :3]
    rows, targets = [], []
    for pose in model['poses']:
        if 'rotation' not in pose or pose.get('method'):
            continue
        frame = pose['frame']
        ids = model.get('landmark_views', {}).get(frame, [])
        if len(ids) < 15:
            continue
        camera = (xyz[ids] - pose['xyz']) @ pose['rotation']
        camera = camera[camera[:, 2] > 0]
        raw = np.load(folder / (Path(frame).stem+'.npy')).astype('f')
        depth = align_depth(raw, 'anything', model['units'], camera)
        if depth is None:
            continue
        h,w = raw.shape
        uv = np.floor(camera[:, :2]/camera[:, 2:3]*(F*w/W)+[w/2,h/2]).astype(int)
        inside = np.all((uv >= 0) & (uv < [w,h]), axis=1)
        u,v = uv[inside].T
        z = depth[v,u]
        valid = np.isfinite(z) & (z > 0)
        if valid.any():
            rows.append(features((u[valid]+.5)/w-.5,(v[valid]+.5)/h-.5,z[valid]))
            targets.append(np.log(camera[inside,2][valid]/z[valid]))
    if len(rows) < 3:
        raise ValueError('Depth training needs at least three aligned camera views.')
    X,y = np.concatenate(rows),np.concatenate(targets)
    fit = least_squares(lambda b: X@b-y,np.zeros(X.shape[1]),loss='soft_l1',f_scale=.05)
    return dict(coefficients=fit.x.tolist(),frames=len(rows),observations=len(y))


def correct(z, x, y, width, height, training):
    values = features((x+.5)/width-.5,(y+.5)/height-.5,z)
    return z*np.exp(np.clip(values@training['coefficients'],-.3,.3))
