"""Extract an observed surface from a trained Brush map in its original coordinates."""
import csv
import itertools
import json
import math
from pathlib import Path
import tempfile

import fast_simplification
import numpy as np
from numba import njit
from scipy import ndimage
from scipy.spatial.transform import Rotation
from skimage.measure import marching_cubes

from ai_surfaces import W, H, F
from fusion import load_observations

WIDTH, HEIGHT, FOCAL = W * 2, H * 2, F * 2
CX, CY = (WIDTH - 1) / 2, (HEIGHT - 1) / 2


def read_splats(path):
    fields = []
    with path.open('rb') as file:
        if file.readline().strip() != b'ply' or file.readline().strip() != b'format binary_little_endian 1.0':
            raise ValueError('Expected a binary Brush Gaussian PLY')
        count = None
        for line in file:
            parts = line.decode('ascii').strip().split()
            if parts == ['end_header']:
                break
            if parts[:2] == ['element', 'vertex']:
                count = int(parts[2])
            elif parts[:2] == ['property', 'float']:
                fields.append((parts[2], '<f4'))
            elif parts[:1] in (['element'], ['property']):
                raise ValueError('Unsupported Gaussian PLY layout')
        if not count or not fields:
            raise ValueError('Empty Gaussian PLY')
        records = np.fromfile(file, dtype=fields, count=count)
        if len(records) != count:
            raise ValueError('Incomplete Gaussian PLY')
    xyz = np.column_stack([records[k] for k in 'xyz'])
    radii = np.exp(np.column_stack([records[f'scale_{i}'] for i in range(3)]))
    q = np.column_stack([records[f'rot_{i}'] for i in (1, 2, 3, 0)])
    opacity = 1 / (1 + np.exp(-np.clip(records['opacity'], -30, 30)))
    good = (opacity > .008) & (np.linalg.norm(q, axis=1) > 1e-6)
    good &= np.isfinite(np.column_stack((xyz, radii, q))).all(axis=1)
    axes = Rotation.from_quat(q[good]).as_matrix() * radii[good, None, :]
    return xyz[good], axes.astype('f'), opacity[good]


@njit
def composite(uv, z, cov, opacity, far):
    # Accumulate front-to-back visibility weights in log-depth bands. A translucent
    # foreground splat should not drag the estimated surface halfway to a rear wall.
    transmission = np.ones((HEIGHT, WIDTH), np.float32)
    bands = 128
    mass = np.zeros((HEIGHT, WIDTH, bands), np.float32)
    moment = np.zeros_like(mass)
    log_step = math.log(1000.) / bands
    for i in np.argsort(z):
        band = min(bands-1,max(0,int(math.log(z[i]/(far*.001))/log_step)))
        px, py = uv[i]
        a, b, c = cov[i]
        determinant = a*c - b*b
        if determinant <= 1e-7:
            continue
        rx, ry = 3*math.sqrt(a), 3*math.sqrt(c)
        for y in range(max(0, math.ceil(py-ry)), min(HEIGHT, math.floor(py+ry)+1)):
            for x in range(max(0, math.ceil(px-rx)), min(WIDTH, math.floor(px+rx)+1)):
                old = transmission[y, x]
                if old < .02:
                    continue
                dx, dy = x-px, y-py
                power = (c*dx*dx - 2*b*dx*dy + a*dy*dy)/determinant
                if power > 9:
                    continue
                alpha = min(.99, opacity[i]*math.exp(-.5*power))
                weight = old*alpha
                mass[y,x,band] += weight
                moment[y,x,band] += weight*z[i]
                transmission[y,x] = old-weight
    result = np.zeros((HEIGHT, WIDTH), np.float32)
    for y in range(HEIGHT):
        for x in range(WIDTH):
            total = 1-transmission[y,x]
            if total < .5:
                continue
            best, peak = 0., 1
            for k in range(1,bands-1):
                m = mass[y,x,k-1]+mass[y,x,k]+mass[y,x,k+1]
                if m > best:
                    best, peak = m, k
            if best >= .32*total:
                result[y,x] = (moment[y,x,peak-1]+moment[y,x,peak]+moment[y,x,peak+1])/best
    return result


def render_depth(splats, eye, rot, far):
    xyz, axes, opacity = splats
    camera = (xyz-eye) @ rot
    z = camera[:, 2]
    keep = (z > far*.001) & (z < far)
    keep &= (np.abs(camera[:, 0]) < z*1.15) & (np.abs(camera[:, 1]) < z*.95)
    x, y, z = camera[keep].T
    u, v = x/z, y/z
    local = rot.T @ axes[keep]
    scale = (FOCAL/z)[:, None]
    jx = scale*(local[:, 0] - u[:, None]*local[:, 2])
    jy = scale*(local[:, 1] - v[:, None]*local[:, 2])
    a, b, c = (jx*jx).sum(1)+.3, (jx*jy).sum(1), (jy*jy).sum(1)+.3
    # Very broad splats can explain color across a wall without locating its edge.
    valid = (a < (WIDTH*.25)**2) & (c < (HEIGHT*.25)**2)
    uv = np.column_stack((FOCAL*u+CX, FOCAL*v+CY))
    cov = np.column_stack((a, b, c))
    return composite(uv[valid], z[valid], cov[valid], opacity[keep][valid], far)


def unproject(depth, eye, rotation, stride=1):
    y, x = np.nonzero(depth[::stride, ::stride])
    y, x = y*stride, x*stride
    points = depth[y, x, None]*np.column_stack(((x-CX)/FOCAL, (y-CY)/FOCAL, np.ones(len(y))))
    return points @ rotation.T + eye, y, x


def sample_depth(points, eye, rotation, image):
    camera = (points-eye) @ rotation
    z = camera[:, 2]
    uv = camera[:, :2]*(FOCAL/np.maximum(z, 1e-8))[:, None] + [CX, CY]
    x, y = np.rint(uv).astype(np.int64).T
    inside = (z > 0) & (x >= 0) & (x < WIDTH) & (y >= 0) & (y < HEIGHT)
    sampled = np.zeros(len(z), 'f')
    sampled[inside] = image[y[inside], x[inside]]
    return z, sampled


def confirm_depths(images, eyes, rotations, size):
    result = []
    for i, image in enumerate(images):
        points, y, x = unproject(image, eyes[i], rotations[i])
        distances = np.linalg.norm(eyes-eyes[i], axis=1)
        candidates = np.flatnonzero((distances > size*.007) & (rotations[:, :, 2] @ rotations[i, :, 2] > .4))
        candidates = candidates[np.argsort(distances[candidates])]
        # Include short and medium baselines, even during a slow part of the walk.
        if len(candidates) > 16:
            candidates = candidates[np.linspace(0, min(40, len(candidates)-1), 16).astype(int)]
        votes = np.zeros(len(points), int)
        rays = points-eyes[i]
        for j in candidates:
            z, seen = sample_depth(points, eyes[j], rotations[j], images[j])
            other = points-eyes[j]
            parallax = np.linalg.norm(np.cross(rays, other), axis=1)
            enough = parallax > .018*np.linalg.norm(rays, axis=1)*np.linalg.norm(other, axis=1)
            votes += enough & (seen > 0) & (np.abs(seen-z) < .04*z)
        good = votes >= 2
        accepted = np.zeros_like(image)
        accepted[y[good], x[good]] = image[y[good], x[good]]
        result.append(accepted)
    return result


def flatten_planar_regions(vertices, faces, voxel):
    """Fit connected, locally planar mesh regions without adding edges across holes."""
    from scipy.spatial import cKDTree
    from scipy.sparse import coo_matrix

    # Local covariance proposes orientations in 3D; graph connectivity separates
    # nearby, disconnected sheets. Fix transition vertices shared with other regions.
    tree = cKDTree(vertices)
    dist, neighbors = tree.query(vertices, k=min(32, len(vertices)))
    local = vertices[neighbors]
    delta = local-local.mean(axis=1, keepdims=True)
    eigen, axes = np.linalg.eigh(delta.transpose(0,2,1) @ delta)
    roughness = eigen[:,0]/np.maximum(eigen[:,1],1e-12)
    normals = axes[:,:,0]
    edges = np.concatenate([faces[:,[0,1]],faces[:,[1,2]],faces[:,[2,0]]])
    graph = coo_matrix((np.ones(len(edges),bool), edges.T), shape=(len(vertices),len(vertices))).tocsr()
    graph = (graph+graph.T).tocsr()
    labels = np.full(len(vertices),-1)
    explored = np.zeros(len(vertices),int)
    fits = []
    for visit, seed in enumerate(np.argsort(roughness)[::6], 1):
        if labels[seed]>=0 or roughness[seed]>.07 or dist[seed,-1]>6*voxel:
            continue
        normal = normals[seed]
        stack = [seed]
        group = []
        explored[seed] = visit
        while stack:
            i = stack.pop()
            group.append(i)
            adjacent = graph.indices[graph.indptr[i]:graph.indptr[i+1]]
            adjacent = adjacent[(labels[adjacent]<0) & (explored[adjacent]!=visit)]
            explored[adjacent] = visit
            if len(adjacent):
                close = np.abs((vertices[adjacent]-vertices[seed]) @ normal) < voxel*.8
                aligned = np.abs(normals[adjacent] @ normal) > .86
                stack.extend(adjacent[close & aligned & (roughness[adjacent]<.18)])
        if len(group)<50:
            continue
        pts = vertices[group]
        centre = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts-centre, full_matrices=False)
        labels[group] = len(fits)
        fits.append((centre,vt[-1]))
    # Remove only the small normal displacement on each well-supported interior.
    boundary = labels[edges[:,0]]!=labels[edges[:,1]]
    interior = labels.copy()
    interior[np.unique(edges[boundary])] = -1
    for i, (centre, normal) in enumerate(fits):
        ids = np.flatnonzero(interior==i)
        residual = (vertices[ids]-centre) @ normal
        vertices[ids] -= np.clip(residual,-voxel,voxel)[:,None]*normal
    return len(fits)


def fuse(images, eyes, rotations):
    samples = np.concatenate([unproject(im, eye, rot, 3)[0] for im, eye, rot in zip(images, eyes, rotations)])
    if len(samples) < 50:
        raise ValueError('Too few consistent depth returns to build a mesh')
    lo, hi = np.percentile(samples, [.5, 99.5], axis=0)
    voxel = np.max(hi-lo)/180
    if not np.isfinite(voxel) or voxel <= 0:
        raise ValueError('Insufficient spatial extent')
    origin = lo - 5*voxel
    shape = np.ceil((hi-lo)/voxel).astype(int)+11
    indices = np.rint((samples-origin)/voxel).astype(int)
    inside = ((indices >= 0) & (indices < shape)).all(axis=1)
    band = np.zeros(shape, bool)
    band[tuple(indices[inside].T)] = True
    nodes = np.argwhere(ndimage.binary_dilation(band, iterations=4))
    points = origin + nodes*voxel
    weight = np.zeros(len(nodes), 'i')
    total = np.zeros(len(nodes), 'f')
    for im, eye, rot in zip(images, eyes, rotations):
        z, seen = sample_depth(points, eye, rot, im)
        sdf = (seen-z)/(3*voxel)
        good = (seen > 0) & (sdf >= -1)
        total[good] += np.minimum(sdf[good], 1)
        weight[good] += 1
    known = np.zeros(shape, bool)
    good = weight >= 2
    keys = tuple(nodes[good].T)
    known[keys] = True
    field = np.ones(shape, 'f')
    field[keys] = total[good]/weight[good]
    # Lewiner's mask indexes the upper corner of each evaluated cube.
    cells = np.zeros(shape, bool)
    cells[1:, 1:, 1:] = np.logical_and.reduce([
        known[a:a+shape[0]-1, b:b+shape[1]-1, c:c+shape[2]-1]
        for a, b, c in itertools.product((0, 1), repeat=3)])
    if not cells.any() or field.min() >= 0:
        raise ValueError('Insufficient overlapping depth for a surface')
    vertices, faces, _, _ = marching_cubes(field, level=0, mask=cells, allow_degenerate=False)
    vertices = origin + vertices*voxel
    original = len(faces)
    regions = flatten_planar_regions(vertices, faces, voxel)
    if original > 500:
        # Quadric edge collapse preferentially removes flat, coplanar triangles.
        vertices, faces = fast_simplification.simplify(vertices, faces, target_count=max(500, original//6), agg=4)
    return vertices, faces, voxel, original, regions


def ray_ranges(vertices, faces, eyes, directions):
    # First triangle hit along each physical beam; two-sided for open surfaces.
    a, b, c = vertices[faces].transpose(1, 0, 2)
    ab, ac = b-a, c-a
    result = []
    for eye, direction in zip(eyes, directions):
        cross = np.cross(direction, ac)
        det = np.einsum('ij,ij->i', ab, cross)
        inverse = np.divide(1., det, out=np.zeros_like(det), where=np.abs(det)>1e-12)
        delta = eye-a
        u = inverse*np.einsum('ij,ij->i', delta, cross)
        q = np.cross(delta, ab)
        v = inverse*(q @ direction)
        t = inverse*np.einsum('ij,ij->i', ac, q)
        hit = (u>=0) & (v>=0) & (u+v<=1) & (t>0)
        result.append(float(t[hit].min()) if hit.any() else np.nan)
    return np.array(result)


def range_evidence(run, model, vertices, faces, poses):
    observations = load_observations(run, ('tof',))
    cal = run/'tof-extrinsics.json'
    mount = json.loads(cal.read_text()) if cal.exists() else dict(direction=[0, 0, 1], origin_m=[0, 0, 0])
    direction, origin = (np.asarray(mount[k], float) for k in ('direction', 'origin_m'))
    if any(a.shape != (3,) or not np.isfinite(a).all() for a in (direction, origin)) or np.linalg.norm(direction)<1e-8:
        raise ValueError('Invalid ToF camera-frame extrinsics')
    check = dict(beam='measured mount' if cal.exists() else 'camera-axis approximation', units=model['units'])
    if model['units'] != 'm' and np.any(origin):
        check['reason'] = 'Metric scale required for measured sensor offset'
        return check, []
    valid = []
    timestamps = run/'timestamps.csv'
    if timestamps.exists():
        with timestamps.open() as stream:
            paired = set()
            for row in csv.DictReader(stream):
                camera, tof = row.get('pi_encoder_callback_ns'), row.get('pi_tof_read_end_ns')
                if camera and tof and abs(int(camera)-int(tof)) <= 80_000_000:
                    paired.add(row['frame'])
        valid = [p for p in poses if p['frame'] in paired and 'range' in observations.get(p['frame'], {})]
    # Reserve validation frames before intersecting rays, so misses count in coverage.
    starts, rays, truth = [], [], []
    axis = direction / np.linalg.norm(direction)
    for pose in valid:
        rot = np.asarray(pose['rotation'])
        starts.append(pose['xyz'] + rot @ origin)
        rays.append(rot @ axis)
        truth.append(observations[pose['frame']]['range'])
    measured = np.asarray(truth)
    predicted = ray_ranges(vertices, faces, starts, rays)
    test = predicted[1::2]
    found = np.isfinite(test)
    check.update(eligible=len(test), hits=int(found.sum()))
    scale = 1.
    if model['units'] != 'm':
        train, reference = predicted[::2], measured[::2]
        ok = np.isfinite(train)
        if ok.sum() < 8 or np.ptp(np.percentile(reference[ok], [10,90])) < .3:
            check['reason'] = 'Insufficient range variation or mesh hits to estimate scale'
            scale = None
        else:
            scale = float(np.median(reference[ok]/train[ok]))
            check['candidate_metres_per_unit'] = scale
    if scale is not None and found.any():
        errs = np.abs(test[found]*scale-measured[1::2][found])
        check.update(median_error_m=float(np.median(errs)), p80_error_m=float(np.percentile(errs,80)))
    readings = []
    for i, pose in enumerate(valid):
        hit = np.isfinite(predicted[i])
        readings.append(dict(frame=pose['frame'], validation=bool(i%2), origin=starts[i].tolist(),
            measured_m=truth[i], hit=bool(hit),
            endpoint=(starts[i]+rays[i]*truth[i]/scale).tolist() if scale is not None else None,
            intersection=(starts[i]+rays[i]*predicted[i]).tolist() if hit else None,
            mesh_m=float(predicted[i]*scale) if hit and scale is not None else None))
    return check, readings


def saved_range_evidence(run, model, mesh_path):
    # Read the indexed, packed triangle PLY produced by write_ply.
    with mesh_path.open('rb') as file:
        header = []
        while len(header) < 12:
            line = file.readline(256)
            if line == b'end_header\n':
                break
            header.append(line.decode('ascii').strip())
        if (len(header) != 8 or header[:2] != ['ply', 'format binary_little_endian 1.0']
                or header[3:6] != ['property float x', 'property float y', 'property float z']
                or header[7] != 'property list uchar int vertex_indices'
                or not header[2].startswith('element vertex ') or not header[6].startswith('element face ')):
            raise ValueError('Unsupported saved mesh format')
        nv, nf = int(header[2].split()[-1]), int(header[6].split()[-1])
        if min(nv, nf) < 1 or mesh_path.stat().st_size-file.tell() != 12*nv+13*nf:
            raise ValueError('Incomplete saved mesh')
        vertices = np.fromfile(file, '<f4', nv*3).reshape(nv,3).astype(float)
        triangles = np.fromfile(file, [('size','u1'),('indices','<i4',3)], nf)
    faces = triangles['indices']
    if np.any(triangles['size'] != 3) or faces.min()<0 or faces.max()>=nv:
        raise ValueError('Invalid saved triangles')
    poses = sorted((p for p in model['poses'] if 'rotation' in p and not p.get('method')), key=lambda p:p['frame'])
    summary, readings = range_evidence(run, model, vertices, faces, poses)
    return dict(summary=summary, readings=readings)


def write_ply(path, vertices, faces):
    header = (f'ply\nformat binary_little_endian 1.0\nelement vertex {len(vertices)}\n'
              f'property float x\nproperty float y\nproperty float z\nelement face {len(faces)}\n'
              'property list uchar int vertex_indices\nend_header\n')
    packed = np.empty(len(faces), dtype=[('size', 'u1'), ('indices', '<i4', 3)])
    packed['size'], packed['indices'] = 3, faces
    with path.open('wb') as file:
        file.write(header.encode())
        file.write(vertices.astype('<f4').tobytes())
        file.write(packed.tobytes())


def build(run, model, root, progress=lambda message: None):
    dest = root/'mesh-v1'
    report_path = dest/'report.json'
    if report_path.exists():
        return json.loads(report_path.read_text())
    poses = sorted((p for p in model['poses'] if 'rotation' in p and not p.get('method')), key=lambda p:p['frame'])
    if len(poses) < 3:
        raise ValueError('At least three registered poses are required')
    selected = [poses[i] for i in np.linspace(0,len(poses)-1,min(120,len(poses))).round().astype(int)]
    eyes = np.asarray([p['xyz'] for p in selected], 'f')
    rotations = np.asarray([p['rotation'] for p in selected], 'f')
    splats = read_splats(root/'gaussians.ply')
    size = np.linalg.norm(np.ptp(np.percentile(splats[0],[10,90],axis=0),axis=0))
    images = []
    for i, (eye, rot) in enumerate(zip(eyes, rotations)):
        if i%12 == 0:
            progress(f'Rendering Gaussian depth · {i+1}/{len(eyes)} views')
        images.append(render_depth(splats, eye, rot, 4*size))
    progress('Checking depths against neighboring registered views')
    accepted = confirm_depths(images, eyes, rotations, size)
    progress('Fusing depth and simplifying the surface')
    vertices, faces, voxel, raw, regions = fuse(accepted, eyes, rotations)
    progress('Comparing mesh intersections with recorded ToF ranges')
    report = dict(views=len(selected), triangles=len(faces), raw_triangles=raw, planar_regions=regions, vertices=len(vertices),
                  voxel_size=float(voxel), units=model['units'], accepted_pixels=int(sum(np.count_nonzero(im) for im in accepted)),
                  rendered_pixels=int(sum(np.count_nonzero(im) for im in images)),
                  tof=range_evidence(run, model, vertices, faces, poses)[0])
    with tempfile.TemporaryDirectory(dir=root, prefix='.mesh-') as tmp:
        ready = Path(tmp)/'ready'
        ready.mkdir()
        write_ply(ready/'mesh.ply', vertices, faces)
        (ready/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        ready.rename(dest)
    return report


if __name__ == '__main__':
    import sys
    from gaussian_map import artifact_dir
    run = Path(sys.argv[1])
    model = json.loads((run/'survey.json').read_bytes())
    print(build(run, model, artifact_dir(run, model['feature_index']), lambda msg: print(msg, flush=True)))
