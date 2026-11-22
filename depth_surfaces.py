"""Selectable surface fits for the replay depth point clouds."""
import fcntl
import json
import tempfile
import time
from pathlib import Path

import fast_simplification
import numpy as np
from scipy.ndimage import gaussian_filter
from scipy.spatial import cKDTree
from skimage.measure import marching_cubes

import cloud_planes
import depth_cloud
from balloon import build_balloon
from replay_depth import output


def planes(xyz, cell, units):
    # All distances are in the source cloud's units, including relative maps.
    mesh = cloud_planes.build_planes(dict(points=xyz, units=units), cell*4,
                                     relative_fit=True)
    return mesh['patches']


def voxel_skin(xyz, cell, units):
    origin = xyz.min(axis=0)-3*cell
    indices = np.rint((xyz-origin)/cell).astype(int)
    density = np.zeros(tuple(indices.max(axis=0)+4), np.float32)
    density[tuple(indices.T)] = 1
    density = gaussian_filter(density,.85)
    if density.max() <= .11:
        return []
    vertices, faces, _, _ = marching_cubes(density,.11,allow_degenerate=False)
    if len(faces)>4500:
        vertices, faces = fast_simplification.simplify(vertices,faces,target_count=4500)
    world = vertices*cell+origin
    return [dict(vertices=tri.round(6).tolist(), landmarks=[]) for tri in world[faces]]


METHODS = {'planes': ('Planar regions', planes), 'voxel': ('Voxel envelope', voxel_skin),
           'balloon': ('Path balloon', build_balloon)}


def range_points(run, model, xyz, cell):
    from gaussian_mesh import paired_range_rays
    poses = [p for p in model['poses'] if 'rotation' in p]
    check, paired, starts, rays, ranges = paired_range_rays(run, model, poses)
    endpoints = np.array([eye+ray*r for eye,ray,r in zip(starts,rays,ranges)]).reshape(-1,3)
    if len(endpoints):
        endpoints = endpoints[cKDTree(xyz).query(endpoints)[0]<=2*cell]
    return endpoints, dict(paired=len(paired),used=len(endpoints),mount=check['beam'])


def build(run,index,source,method,use_tof,detail=None,reach=None,saved_only=False,trained=False):
    if method not in METHODS or type(use_tof) is not bool:
        raise ValueError('Choose a surface method and a valid ToF setting.')
    cloud = depth_cloud.reconstruct(run,index,source,saved_only=saved_only,**({"trained":True} if trained else {}))
    model = json.loads((run/'survey.json').read_bytes())
    if 'path' in cloud:
        model['poses'] = cloud['path']['poses']
    units = cloud['units']
    if use_tof and units!='m':
        raise ValueError('ToF surface support requires a metric walk.')
    detail_key = ''
    if method=='balloon':
        if any(type(v) not in (int,float) or not np.isfinite(v)
               for v in (detail,reach)) or not 0 < detail < reach:
            raise ValueError('Choose a positive mesh detail smaller than the inflation limit.')
        detail_key = f'-{float(detail)}-{float(reach)}'
    folder = output(run,index,source)
    if 'path' in cloud:
        detail_key += '-regenerated-path'
    if trained:
        detail_key += '-trained'
    cache = folder/f'surface-v1-{method}-{"tof" if use_tof else "visual"}{detail_key}.json'
    inputs = [folder/('point-cloud-path-v1.json' if 'path' in cloud else 'point-cloud-v1.json')]
    if trained:
        inputs[0] = inputs[0].with_suffix('.trained.json')
    if method=='balloon':
        inputs.append(run/'survey.json')
    if use_tof:
        inputs += [run/name for name in ('tof-extrinsics.json','timestamps.csv','range.csv')]
    signature = [p.stat().st_mtime_ns if p.exists() else None for p in inputs]
    with cache.with_suffix('.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        if cache.is_file():
            existing = json.loads(cache.read_bytes())
            if existing.get('inputs')==signature:
                return existing
        start = time.perf_counter()
        xyz = np.asarray(cloud['points'],float)[:,:3]
        lo, hi = np.percentile(xyz,[.5,99.5],axis=0)
        span = max(hi-lo)
        if not np.isfinite(span) or span<=0:
            raise ValueError('Depth cloud has no spatial extent.')
        cell = span/80
        keep = ((xyz>=lo-4*cell)&(xyz<=hi+4*cell)).all(axis=1)
        xyz = xyz[keep]
        reading = None
        if use_tof:
            endpoints,reading = range_points(run,model,xyz,cell)
            xyz = np.vstack((xyz,endpoints))
        balloon = None
        if method=='balloon':
            # UI distances are centimetres for metric walks, percent of span otherwise.
            factor = .01 if units=='m' else span*.01
            if detail*factor < span*1e-7:
                raise ValueError('Minimum detail is too small for this walk.')
            patches,balloon = build_balloon(xyz,model['poses'],detail*factor,reach*factor)
        else:
            _,groups = np.unique(np.floor((xyz-lo)/cell).astype(int),axis=0,return_inverse=True)
            counts = np.bincount(groups)
            xyz = np.stack([np.bincount(groups,weights=v)/counts for v in xyz.T],axis=1)
            patches = METHODS[method][1](xyz,cell,units)
        for patch in patches:
            # Fitting indices refer to downsampled cells, not displayed depth points.
            patch['landmarks'] = []
        result = dict(method=method,source=source,units=units,patches=patches,tof=reading,
                      points=int(keep.sum()),seconds=time.perf_counter()-start,inputs=signature)
        if balloon:
            result['balloon'] = balloon
        with tempfile.NamedTemporaryFile(dir=folder,mode='w',delete=False) as tmp:
            json.dump(result,tmp,separators=(',',':'),allow_nan=False)
        Path(tmp.name).replace(cache)
        return result
