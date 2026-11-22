"""Reconstruct geometry from replay depths in the saved camera coordinate system."""
import fcntl
import json
import time
import tempfile
from pathlib import Path

import cv2
import numpy as np

from ai_surfaces import F, W, metric_fit
from gaussian_map import rectified_frames
from replay_depth import output


def align_depth(raw, method, units, camera):
    if method == 'gaussian' or (method == 'apple' and units == 'm'):
        return raw
    h, w = raw.shape
    camera = camera[camera[:, 2] > 1e-5]
    z = camera[:, 2]
    xy = camera[:, :2] / z[:, None]
    uv = np.floor(xy * (F*w/W) + [w/2, h/2]).astype(int)
    keep = np.all((uv >= 0) & (uv < [w,h]), axis=1)
    u,v = uv[keep].T
    _, ix = np.unique(v*w+u, return_index=True)
    measured, z = raw[v[ix],u[ix]], z[keep][ix]
    good = np.isfinite(measured)
    if method != 'anything':
        good &= measured > 0
    measured,z = measured[good],z[good]
    if len(z) < 15:
        return None
    if method == 'anything':
        lo, hi = np.percentile(measured,[5,95])
        if hi <= lo:
            return None
        normalized = (raw-lo)*255/(hi-lo)
        fit = metric_fit((measured-lo)*255/(hi-lo),z)
        if fit is None:
            return None
        inv = normalized*fit[0]+fit[1]
        return np.divide(1, inv, out=np.zeros_like(raw), where=inv>0)
    ratios = z/measured
    hold = np.arange(len(z))%4 == 0
    scale = np.median(ratios[~hold])
    if np.median(np.abs(scale/ratios[hold]-1)) > .2:
        return None
    return raw*scale


def reconstruct(run, index, method, regenerate=False, saved_only=False, trained=False):
    if trained:
        if method != 'anything':
            raise ValueError('Depth calibration is available for Depth Anything V2.')
        base = reconstruct(run,index,method,regenerate,saved_only)
        folder = output(run,index,method)
        base_file = folder/('point-cloud-path-v1.json' if 'path' in base else 'point-cloud-v1.json')
        cache = base_file.with_suffix('.trained.json')
        with (folder/'.points.lock').open('a') as lock:
            fcntl.flock(lock,fcntl.LOCK_EX)
            # Reload the current path under lock in case another build finished.
            base = json.loads(base_file.read_bytes())
            inputs = [f.stat().st_mtime_ns for f in (base_file,run/'survey.json',folder/'complete.json')]
            if cache.is_file():
                result = json.loads(cache.read_bytes())
                if result.get('trainingInputs') == inputs:
                    return result
            model = json.loads((run/'survey.json').read_bytes())
            if model.get('feature_index') != index or inputs[0] < max(inputs[1:]):
                raise ValueError('The map changed. Reopen the walk and try again.')
            from depth_training import train
            start = time.perf_counter()
            training = train(model,folder)
            training['seconds'] = time.perf_counter()-start
            if 'path' in base:
                model['poses'] = base['path']['poses']
            result = fuse(run,model,folder,method,training)
            if 'path' in base:
                result['path'] = base['path']
            result.update(training=training,trainingInputs=inputs)
            result['stages']['Train depth correction'] = training['seconds']
            result['seconds'] += training['seconds']
            if inputs != [f.stat().st_mtime_ns for f in (base_file,run/'survey.json',folder/'complete.json')]:
                raise ValueError('The map changed during training. Try again.')
            with tempfile.NamedTemporaryFile(dir=folder, mode='w',delete=False) as tmp:
                json.dump(result,tmp,separators=(',',':'),allow_nan=False)
            Path(tmp.name).replace(cache)
            return result
    folder = output(run,index,method)
    if not (folder/'complete.json').is_file():
        raise ValueError('Finish generating this depth set first.')
    model = json.loads((run/'survey.json').read_bytes())
    if model.get('feature_index') != index:
        raise ValueError('Reopen the current walk.')
    cache = folder/'point-cloud-v1.json'
    with (folder/'.points.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        inputs = [(run/'survey.json').stat().st_mtime_ns,(folder/'complete.json').stat().st_mtime_ns]
        variant = folder/'point-cloud-path-v1.json'
        if not regenerate:
            if not saved_only and variant.is_file():
                saved = json.loads(variant.read_bytes())
                if saved.get('pathInputs')==inputs:
                    return saved
            if cache.is_file() and cache.stat().st_mtime_ns >= max(inputs):
                return json.loads(cache.read_bytes())
        if regenerate:
            from depth_path import estimate
            path = estimate(run,model,folder,method)
            result = fuse(run,dict(model,poses=path['poses']),folder,method)
            result.update(path=path,pathInputs=inputs)
            cache = variant
        else:
            result = fuse(run,model,folder,method)
        if inputs != [(run/'survey.json').stat().st_mtime_ns,(folder/'complete.json').stat().st_mtime_ns]:
            raise ValueError('The map or depth set changed. Reopen the walk and try again.')
        with tempfile.NamedTemporaryFile(dir=folder,mode='w',delete=False) as tmp:
            json.dump(result,tmp,separators=(',',':'),allow_nan=False)
        Path(tmp.name).replace(cache)
        return result


def fuse(run,model,folder,method,training=None):
    start = time.perf_counter()
    xyz = np.array(model['points'])[:,:3]
    scale = np.linalg.norm(np.ptp(np.percentile(xyz,[5,95],axis=0),axis=0))
    voxel = max(scale/140,1e-6)
    load_time = projection_time = 0.
    sampled, colors, used = [], [], 0
    for pose in model['poses']:
        if 'rotation' not in pose:
            continue
        frame = pose['frame']
        tick = time.perf_counter()
        raw = np.load(folder/(Path(frame).stem+'.npy')).astype('f')
        _, image = next(rectified_frames(run, model, [frame]))
        load_time += time.perf_counter()-tick
        tick = time.perf_counter()
        rot = np.asarray(pose['rotation'])
        ids = model.get('landmark_views',{}).get(frame,[])
        camera = (xyz[ids]-pose['xyz']) @ rot
        depth = align_depth(raw,method,model['units'],camera)
        if depth is not None:
            h,w = depth.shape
            y,x = np.mgrid[3:h:7,3:w:7]
            z = depth[y,x]
            keep = np.isfinite(z)&(z>0)&(z<scale*2)
            x,y,z = x[keep],y[keep],z[keep]
            if training and len(z):
                from depth_training import correct
                z = correct(z,x,y,w,h,training)
            if len(z):
                rays = np.stack(((x+.5-w/2)/(F*w/W),(y+.5-h/2)/(F*w/W),np.ones_like(z)),axis=1)
                points = (rays*z[:,None]) @ rot.T + pose['xyz']
                cells = np.rint(points/voxel).astype(int)
                keys,first = np.unique(cells,axis=0,return_index=True)
                sampled.append(keys)
                rgb = cv2.resize(image,(w,h),interpolation=cv2.INTER_AREA)[y,x,::-1]
                colors.append(rgb[first])
                used += 1
        projection_time += time.perf_counter()-tick
    if not sampled:
        raise ValueError('No saved camera views passed depth alignment.')
    tick = time.perf_counter()
    cells,ids,counts = np.unique(np.concatenate(sampled),axis=0,return_inverse=True,return_counts=True)
    rgb = np.concatenate(colors)
    rgb = np.stack([np.bincount(ids,weights=c)/counts for c in rgb.T],axis=1)
    supported = np.flatnonzero(counts>=2)
    if not len(supported):
        raise ValueError('No depth voxels had support from two camera views.')
    selected = supported[np.linspace(0,len(supported)-1,min(30000,len(supported)),dtype=int)]
    cloud = [list(p)+list(c) for p,c in zip((cells[selected]*voxel).round(5).tolist(),
                                           rgb[selected].round().astype(int).tolist())]
    return dict(points=cloud,used=used,poses=sum('rotation' in p for p in model['poses']),
                supported=len(supported),units=model['units'],method=method,
                stages={'Read depth and rectify colour':load_time,'Align and backproject':projection_time,
                        'Fuse and sample voxels':time.perf_counter()-tick},seconds=time.perf_counter()-start)
