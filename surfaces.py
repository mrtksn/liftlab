"""Fit bounded planar patches to sparse landmarks in survey coordinates."""
from collections import Counter

import numpy as np


def plane_inliers(points, tolerance, rng):
    # Score small batches to keep memory bounded for large landmark clouds.
    triples = points[rng.integers(len(points), size=(256, 3))]
    normals = np.cross(triples[:, 1]-triples[:, 0], triples[:, 2]-triples[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > tolerance**2
    if not valid.any():
        return None
    normals = normals[valid]/lengths[valid, None]
    offsets = np.sum(triples[valid, 0]*normals, axis=1)
    sizes = np.count_nonzero(np.abs(points @ normals.T-offsets) < tolerance, axis=0)
    best = sizes.argmax()
    mask = np.abs(points @ normals[best]-offsets[best]) < tolerance
    if mask.sum() < 30:
        return None
    centre = points[mask].mean(axis=0)
    _, spread, axes = np.linalg.svd(points[mask]-centre, full_matrices=False)
    if spread[1] < tolerance*3*np.sqrt(mask.sum()):
        return None
    u, v, normal = axes
    mask = np.abs((points-centre) @ normal) < tolerance
    return centre, u, v, normal, mask


def rectangles(occupied):
    """Merge only contiguous occupied cells, preserving holes."""
    cells = set(occupied)
    while cells:
        x, y = min(cells)
        right, top = x+1, y+1
        source = occupied[x,y]
        while (right,y) in cells and occupied[right,y] == source:
            right += 1
        while all((i,top) in cells and occupied[i,top] == source for i in range(x,right)):
            top += 1
        cells.difference_update((i,j) for i in range(x,right) for j in range(y,top))
        yield x,y,right,top,source


def close_small_holes(occupied, protected):
    """Fill single-cell cracks with coplanar support on four sides."""
    inferred = set()
    keys = set(occupied)
    for x,y in keys:
        hole = (x+1,y)
        if hole not in keys and hole not in protected and all(
            neighbour in keys for neighbour in ((x+2,y),(x+1,y-1),(x+1,y+1))
        ):
            inferred.add(hole)
    return inferred


def build_surfaces(model, observations=None):
    mesh = dict(method="bounded-planar-surfaces-v2", units=model["units"], patches=[])
    points = np.asarray([p[:3] for p in model["points"]], dtype=float).reshape(-1,3)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 30:
        return mesh
    lower, upper = np.percentile(points, [1,99], axis=0)
    span = np.linalg.norm(upper-lower)
    if span < 1e-5:
        return mesh
    points = points[((points >= lower) & (points <= upper)).all(axis=1)]
    cell = max(.22, span/85) if model["units"] == "m" else span/60
    tolerance = max(.025,cell*.16) if model["units"] == "m" else cell*.16
    mesh["cell_size"] = round(cell,6)

    if len(points) > 18000:
        points = points[np.linspace(0,len(points)-1,18000,dtype=int)]
    endpoints = []
    if model["units"] == "m" and observations and "tof" in model.get("streams", ()):
        for pose in model["poses"]:
            distance = observations.get(pose["frame"], {}).get("range")
            if distance is not None and distance >= .18 and "forward" in pose:
                # The mounted beam is approximately camera-forward. Use returns
                # only to corroborate existing visual plane evidence.
                endpoints.append(np.asarray(pose["xyz"])+distance*np.asarray(pose["forward"]))
    endpoints = np.asarray(endpoints, dtype=float).reshape(-1,3)
    path = np.asarray([p["xyz"] for p in model["poses"]],dtype=float).reshape(-1,3)
    rng = np.random.default_rng(790)
    for plane_id in range(12):
        if len(points) < 30:
            break
        plane = plane_inliers(points,tolerance,rng)
        if plane is None:
            break
        centre,u,v,normal,mask = plane
        axes = np.stack([u,v],axis=1)
        projected = (points[mask]-centre) @ axes / cell
        points = points[~mask]
        # Evidence spread into more than one sub-cell. Prevent tiny clusters
        # repeated by the camera from filling a whole patch on their own.
        cells = np.floor(projected).astype(int)
        subcells = np.floor(projected*3).astype(int)
        fine = Counter(tuple(p//3) for p in np.unique(subcells,axis=0))
        visual_counts = Counter(map(tuple,cells))
        near = endpoints[np.abs((endpoints-centre)@normal) < max(.06,tolerance)]
        ranged = Counter(map(tuple,np.floor((near-centre) @ axes / cell).astype(int)))
        occupied = {key: ranged[key] >= 2 for key in sorted(visual_counts)
                    if visual_counts[key] >= 3 and (fine[key] >= 3 or (fine[key] >= 2 and ranged[key] >= 2))}
        # Leave openings crossed by the camera and unbounded gaps alone.
        nearby_path = path[np.abs((path-centre)@normal) < cell*.7]
        protected = set(map(tuple,np.floor((nearby_path-centre) @ axes / cell).astype(int)))
        inferred = close_small_holes(occupied, protected)
        evidence = {key: ("tof" if support else "landmarks") for key,support in occupied.items()}
        evidence.update((key,"inferred") for key in inferred)
        for x,y,right,top,source in rectangles(evidence):
            vertices = centre + cell*(np.array([[x,y],[right,y],[right,top],[x,top]]) @ axes.T)
            mesh["patches"].append(dict(vertices=vertices.round(6).tolist(), plane=plane_id,
                                        tof_supported=source=="tof", inferred=source=="inferred"))
    return mesh
