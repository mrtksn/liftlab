"""Offline body-clearance replay on a metric, three-state occupancy grid."""
import csv
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np
from numba import njit
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

from gaussian_mesh import CX, CY, FOCAL, confirm_depths, read_splats, render_depth, unproject
from node_params import pairing_ns, sensor_settings, tof_metres

UNKNOWN, FREE, OCCUPIED = 0, 1, 2
MAX_POSE_GAP_S = .55
# Settings of the Walk clearance operator; defaults are the values this module used to fix.
PARAMS = (
    dict(key="max_pose_gap_s", label="Longest pose gap to interpolate", kind="number", unit="s", default=MAX_POSE_GAP_S,
         min=.05, max=10.0),
    dict(key="views", label="Views rendered for splat depth", kind="int", default=120, min=3, max=2000),
    dict(key="min_cell_cm", label="Smallest grid cell", kind="number", unit="cm", default=4.0, min=.5, max=100.0),
    dict(key="cells_across", label="Grid cells across the walk", kind="int", default=220, min=20, max=2000),
    dict(key="margin_m", label="Margin around the walk", kind="number", unit="m", default=1.25, min=0.0, max=20.0),
)


def range_samples(run, frames):
    sensors = sensor_settings(run)
    window = pairing_ns(sensors)
    path = run/'range.csv'
    independent = path.is_file()
    rows = frames
    if independent:
        with path.open() as f:
            rows = list(csv.DictReader(f))
    samples = []
    for row in rows:
        try:
            raw = int(row['tof_raw'])
            if independent:
                start, end = int(row['pi_read_start_ns']), int(row['pi_read_end_ns'])
                if not 0 <= end-start <= window:
                    continue
                stamp = (start+end)//2
            else:
                stamp = int(row['pi_tof_read_end_ns'])
                if abs(int(row['pi_encoder_callback_ns'])-stamp)>window:
                    continue
            metres = tof_metres(raw, sensors)
            if metres is not None:
                samples.append((stamp, metres))
        except (ValueError, KeyError, TypeError):
            continue
    return sorted(samples), 'independent ToF' if independent else 'camera-paired ToF'


def sample_poses(times, positions, rotations, query, scale, max_gap=MAX_POSE_GAP_S):
    """Bracket only short intervals with bounded translation and rotation."""
    query = np.asarray(query)
    right = np.clip(np.searchsorted(times,query),0,len(times)-1)
    left = np.maximum(0,right-1)
    exact = times[right]==query
    dt = times[right]-times[left]
    frac = np.divide(query-times[left],dt,out=np.zeros(len(query)),where=dt>0)
    relative = rotations[left].inv()*rotations[right]
    valid = ((dt<=max_gap) & (frac>=0) & (frac<=1) & (query>=times[0]) & (query<=times[-1]) &
             (relative.magnitude()<np.deg2rad(45)) & (scale*np.linalg.norm(positions[right]-positions[left],axis=1)<.6)) | exact
    interp = np.clip(query,times[0],times[-1])
    xyz = positions[left]+np.clip(frac,0,1)[:,None]*(positions[right]-positions[left])
    return xyz, Slerp(times,rotations)(interp).as_matrix(), valid, exact


def select_scale(model, poses, depths, paired, mount_axis):
    if model['units']=='m':
        return 1., 'ToF-scaled map' if model.get('tof_scale') else 'metric map', None
    # A provisional scale can be estimated from registered splat depths alone.
    # Reserve alternating pairs as a check. Reject a fit with poor agreement.
    if mount_axis[2] <= 0:
        raise ValueError('Set a measured metric baseline for this walk.')
    x,y = np.rint([CX+FOCAL*mount_axis[0]/mount_axis[2], CY+FOCAL*mount_axis[1]/mount_axis[2]]).astype(int)
    pairs = []
    for pose,depth in zip(poses,depths):
        r = paired.get(pose['frame'])
        if r is None or r<.25 or not (2<=x<depth.shape[1]-2 and 2<=y<depth.shape[0]-2):
            continue
        window = depth[y-2:y+3,x-2:x+3]
        good = window[window>0]/mount_axis[2]
        if len(good)>=8 and np.ptp(np.percentile(good,[10,90])) < .15*np.median(good):
            pairs.append((r,np.median(good)))
    if len(pairs)>=12:
        train, val = np.array(pairs[::2]),np.array(pairs[1::2])
        scale = np.median(train[:,0]/train[:,1])
        error = np.abs(scale*val[:,1]-val[:,0])/val[:,0]
        if np.ptp(np.percentile(train[:,0],[10,90]))>=.30 and np.median(error)<.20:
            return float(scale),'provisional ToF scale',float(np.median(error))
    raise ValueError('Scale could not be checked against ToF. Mark a measured baseline and rebuild the map.')


@njit
def integrate(starts, ends, origin, voxel, occupied, votes, last, group_ids):
    # A narrow centerline can support cells close to it. Require two distinct
    # viewpoint groups before marking free. Endpoints always take precedence.
    nx,ny,nz = occupied.shape
    for i in range(len(ends)):
        start = starts[0 if len(starts)==1 else i]
        end = ends[i]
        dx,dy,dz = end-start
        length = np.sqrt(dx*dx+dy*dy+dz*dz)
        if length < 1.e-8:
            continue
        ix,iy,iz = np.floor((end-origin)/voxel).astype(np.int64)
        if 0<=ix<nx and 0<=iy<ny and 0<=iz<nz:
            occupied[ix,iy,iz] = True
        # Clip to grid before stepping, even for very distant splats.
        direction = np.array([dx,dy,dz])/length
        lo,hi = 0.,length-voxel
        for axis in range(3):
            low = origin[axis]-start[axis]
            high = low+occupied.shape[axis]*voxel
            if abs(direction[axis])<1.e-10:
                if low>0 or high<0:
                    hi=-1.
            else:
                a,b = low/direction[axis],high/direction[axis]
                lo,hi=max(lo,min(a,b)),min(hi,max(a,b))
        group = group_ids[0 if len(group_ids)==1 else i]
        step = voxel*.5
        for k in range(max(0,int(np.ceil((hi-lo)/step)))):
            t = lo+k*step
            x=(start[0]+t*direction[0]-origin[0])/voxel
            y=(start[1]+t*direction[1]-origin[1])/voxel
            z=(start[2]+t*direction[2]-origin[2])/voxel
            a,b,c=int(np.floor(x)),int(np.floor(y)),int(np.floor(z))
            if not (0<=a<nx and 0<=b<ny and 0<=c<nz):
                continue
            if (x-a-.5)**2+(y-b-.5)**2+(z-c-.5)**2>.22:
                continue
            if last[a,b,c]!=group:
                if votes[a,b,c]<2:
                    votes[a,b,c]+=1
                last[a,b,c]=group


def distance_to_cells(tree, point, voxel):
    if not tree.n:
        return None
    nearest = tree.query(point)[0]
    # Any closer cube has its centre within half a diagonal of the best
    # centre distance. Measure distance to its box, including face contact.
    ids = tree.query_ball_point(point, nearest+voxel*.867)
    delta = np.maximum(np.abs(tree.data[ids]-point)-voxel*.5,0)
    return float(np.linalg.norm(delta,axis=1).min())


def replay_distances(grid, origin, voxel, points, valid):
    def tree(mask):
        return cKDTree(origin+(np.argwhere(mask)+.5)*voxel)
    occupied = tree(grid==OCCUPIED)
    free = grid==FREE
    unknown_edge = tree((grid==UNKNOWN) & binary_dilation(free,structure=np.ones((3,3,3))))
    high = origin+np.asarray(grid.shape)*voxel
    result = []
    for point, ok in zip(points,valid):
        if not ok:
            result.append(None)
            continue
        idx = np.floor((point-origin)/voxel).astype(int)
        inside = np.all((idx>=0) & (idx<grid.shape))
        boundary = max(0.,float(min((high-point).min(),(point-origin).min())))
        if inside and free[tuple(idx)]:
            edge = distance_to_cells(unknown_edge,point,voxel)
            unseen = min(edge,boundary) if edge is not None else boundary
        else:
            unseen = 0.
        result.append(dict(obstacle_m=distance_to_cells(occupied,point,voxel), observed_m=unseen))
    return result


def artifact_dir(run, root, params=None):
    # A mount correction, replaced range log or other settings invalidate the derived grid.
    from node_params import defaults, validate
    digest = hashlib.sha256()
    params = validate(PARAMS, params)
    if params != defaults(PARAMS):
        digest.update(json.dumps(params, sort_keys=True).encode())
    for path in (run/'range.csv', run/'timestamps.csv', run/'tof-extrinsics.json', run/'sensors.json',
                 root/'gaussians.ply'):
        if path.exists():
            stat = path.stat()
            digest.update(f'{path.name}:{stat.st_size}:{stat.st_mtime_ns};'.encode())
    return root/('clearance-v1-' + digest.hexdigest()[:16])


def build(run, model, root, progress=lambda msg: None, params=None):
    from node_params import validate
    params = validate(PARAMS, params)
    dest = artifact_dir(run, root, params)
    if dest.exists():
        return json.loads((dest/'report.json').read_text())
    with (run/'timestamps.csv').open() as f:
        frames = list(csv.DictReader(f))
    by_name = {row['frame']:row for row in frames if row.get('pi_encoder_callback_ns')}
    poses = sorted((p for p in model['poses'] if p['frame'] in by_name and p.get('rotation') and not p.get('method')),
                   key=lambda p:int(by_name[p['frame']]['pi_encoder_callback_ns']))
    if len(poses)<3:
        raise ValueError('Clearance replay needs three or more registered, timestamped camera poses.')
    clocks = np.array([int(by_name[p['frame']]['pi_encoder_callback_ns']) for p in poses])
    if np.any(np.diff(clocks)<=0):
        raise ValueError('Camera timestamps must increase between registered poses.')
    seconds = lambda ns: (np.asarray(ns,dtype=np.int64)-clocks[0])*1.e-9
    times = seconds(clocks)
    xyz = np.array([p['xyz'] for p in poses])
    rot = Rotation.from_matrix(np.array([p['rotation'] for p in poses]))
    indices = np.linspace(0,len(poses)-1,min(params['views'],len(poses))).round().astype(int)
    eyes,mats = xyz[indices].astype('f'),rot.as_matrix()[indices].astype('f')
    splats = read_splats(root/'gaussians.ply')
    size = np.linalg.norm(np.ptp(np.percentile(splats[0],[10,90],axis=0),axis=0))
    depths = []
    for i,(eye,mat) in enumerate(zip(eyes,mats)):
        if i%20==0:
            progress(f'Clearance · rendering splat depth {i+1}/{len(eyes)}')
        depths.append(render_depth(splats,eye,mat,4*size))
    progress('Clearance · checking depth across views')
    accepted = confirm_depths(depths,eyes,mats,size)
    mount_path = run/'tof-extrinsics.json'
    mount = json.loads(mount_path.read_text()) if mount_path.exists() else {'origin_m':[0,0,0],'direction':[0,0,1]}
    axis,offset = (np.asarray(mount[k],float) for k in ('direction','origin_m'))
    if any(a.shape!=(3,) or not np.isfinite(a).all() for a in (axis,offset)) or np.linalg.norm(axis)<1.e-6:
        raise ValueError('Invalid ToF mount geometry.')
    axis /= np.linalg.norm(axis)
    # A nonzero physical offset requires a known scale before comparing depths.
    if model['units']!='m' and np.any(offset):
        raise ValueError('Set a measured baseline to apply the ToF offset to this relative map.')
    paired = {}
    sensors = sensor_settings(run)
    for name,row in by_name.items():
        if row.get('tof_raw') and row.get('pi_tof_read_end_ns') and abs(int(row['pi_tof_read_end_ns'])-int(row['pi_encoder_callback_ns']))<=pairing_ns(sensors):
            metres=tof_metres(row['tof_raw'],sensors)
            if metres is not None:
                paired[name]=metres
    scale,scale_source,scale_error = select_scale(model,[poses[i] for i in indices],accepted,paired,axis)
    progress('Clearance · integrating splat and ToF sightlines')
    # Restrict this replay grid to the neighborhood of the recorded walk.
    centers = xyz*scale
    origin,upper = centers.min(0)-params['margin_m'],centers.max(0)+params['margin_m']
    voxel = max(params['min_cell_cm']/100,float(np.max(upper-origin))/params['cells_across'])
    shape = np.ceil((upper-origin)/voxel).astype(int)
    occupied = np.zeros(shape,bool)
    votes = np.zeros(shape,'u1')
    last = np.full(shape,-1,np.int32)
    # Group coincident camera locations. Repeated register reads must not
    # independently certify free space from a stationary viewpoint.
    groups = {}
    def viewpoint(point):
        key = tuple(np.floor(point/(2*voxel)).astype(int))
        return groups.setdefault(key,len(groups))
    # Integrate all rays from each spatial viewpoint together for unique votes.
    batches = {}
    for depth,eye,mat in zip(accepted,eyes,mats):
        ends,_,_ = unproject(depth,eye,mat,2)
        start = eye.astype(float)*scale
        batches.setdefault(viewpoint(start),[]).append((start[None,:],ends*scale))
    samples,source = range_samples(run,frames)
    used = 0
    if samples:
        query = seconds([s[0] for s in samples])
        pos,rotations,keep,_ = sample_poses(times,xyz,rot,query,scale,params['max_pose_gap_s'])
        starts = pos[keep]*scale + rotations[keep] @ offset
        directions = rotations[keep] @ axis
        ends = starts+directions*np.array([s[1] for s in samples])[keep,None]
        grouped = {}
        for i,point in enumerate(starts):
            grouped.setdefault(viewpoint(point),[]).append(i)
        for group,ii in grouped.items():
            batches.setdefault(group,[]).append((starts[ii],ends[ii]))
        used = len(starts)
    for group,parts in batches.items():
        for starts,ends in parts:
            integrate(starts,ends,origin,voxel,occupied,votes,last,np.array([group]))
    grid = np.where(occupied,OCCUPIED,np.where(votes>=2,FREE,UNKNOWN)).astype('u1')
    progress('Clearance · measuring distance along the recorded walk')
    stamps = [int(row['pi_encoder_callback_ns']) if row.get('pi_encoder_callback_ns') else clocks[0]-10**9 for row in frames]
    path,_,valid,exact = sample_poses(times,xyz,rot,seconds(stamps),scale,params['max_pose_gap_s'])
    records = {}
    for i,dist in enumerate(replay_distances(grid,origin,voxel,path*scale,valid)):
        if dist is not None:
            records[frames[i]['frame']] = dict(**dist,xyz=path[i].tolist(),pose='registered' if exact[i] else 'interpolated')
    report = dict(frames=records, total_frames=len(frames), views=len(eyes), voxel_m=voxel,
                  scale=scale, scale_source=scale_source, scale_check_error=scale_error, range_source=source,
                  valid_ranges=len(samples), placed_ranges=used,
                  mount='measured mount' if mount_path.exists() else 'camera-axis approximation')
    with tempfile.TemporaryDirectory(dir=root,prefix='.clearance-') as tmp:
        ready = Path(tmp)/'ready'
        ready.mkdir()
        np.savez_compressed(ready/'grid.npz', cells=grid, origin_m=origin, voxel_m=voxel, metres_per_map_unit=scale)
        (ready/'report.json').write_text(json.dumps(report,allow_nan=False))
        ready.rename(dest)
    return report
