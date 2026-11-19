"""Build surface triangles from the first supported returns around mapped camera poses."""
from functools import lru_cache

import numpy as np


METHOD = "first-return-cloth-v1"


@lru_cache(maxsize=1)
def ray_lattice():
    """Subdivide an icosahedron into a spherical cloth with no seam or poles."""
    phi = (1 + np.sqrt(5)) / 2
    nodes = np.array([(0, a, b*phi) for a in (-1, 1) for b in (-1, 1)] +
                     [(a, b*phi, 0) for a in (-1, 1) for b in (-1, 1)] +
                     [(b*phi, 0, a) for a in (-1, 1) for b in (-1, 1)])
    nodes /= np.linalg.norm(nodes[0])
    edges = np.linalg.norm(nodes[:, None] - nodes, axis=2) < 1.1
    faces = [(i,j,k) for i in range(12) for j in range(i) for k in range(j)
             if edges[i,j] and edges[i,k] and edges[j,k]]
    nodes = list(nodes)
    for _ in range(3):
        midpoints = {}

        def midpoint(i, j):
            edge = tuple(sorted((i, j)))
            if edge not in midpoints:
                direction = nodes[i] + nodes[j]
                midpoints[edge] = len(nodes)
                nodes.append(direction / np.linalg.norm(direction))
            return midpoints[edge]

        refined = []
        for a,b,c in faces:
            ab,bc,ca = midpoint(a,b),midpoint(b,c),midpoint(c,a)
            refined.extend(((a,ab,ca),(ab,b,bc),(ca,bc,c),(ab,bc,ca)))
        faces = refined
    return np.asarray(nodes), np.asarray(faces)


def compatible(cosine, r1, r2, span):
    # Distance squared splits into a radial jump and angular separation.
    separation2 = np.maximum(0, 2*r1*r2*(1 - np.clip(cosine, -1, 1)))
    jump = np.abs(r1-r2)
    return ((cosine >= np.cos(np.deg2rad(20))) &
            (jump**2 + separation2 <= span**2) &
            (jump <= .03 + 1.5*np.sqrt(separation2)))


def build_cloth(model, distance):
    if not np.isfinite(distance) or distance <= 0:
        raise ValueError("Enter a positive maximum span.")
    mesh = dict(method=METHOD, units=model["units"], growth_distance_m=distance, patches=[], retired=[])
    if model["units"] != "m":
        return dict(mesh, status="needs_scale")
    if not model.get("landmark_views"):
        return dict(mesh, status="needs_tracks")

    rays, lattice_faces = ray_lattice()
    xyz = np.asarray([p[:3] for p in model["points"]], dtype=float).reshape(-1, 3)
    eyes = {p["frame"]: np.asarray(p["xyz"]) for p in model["poses"]}
    first_eye = np.full_like(xyz, np.nan)
    confirmed = np.zeros(len(xyz), bool)
    supported = np.zeros(len(xyz), bool)
    proposals = {}
    for frame, observed in sorted(model["landmark_views"].items()):
        eye = eyes.get(frame)
        if eye is None or not np.isfinite(eye).all():
            continue
        ids = np.unique(np.asarray(observed, dtype=int))
        ids = ids[(ids >= 0) & (ids < len(xyz))]
        delta = xyz[ids] - eye
        ranges = np.linalg.norm(delta, axis=1)
        usable = np.isfinite(ranges) & (ranges > 1e-6)
        ids, ranges, delta = ids[usable], ranges[usable], delta[usable]
        new = np.isnan(first_eye[ids, 0])
        first_eye[ids[new]] = eye
        confirmed[ids] |= np.linalg.norm(first_eye[ids] - eye, axis=1) >= .01
        keep = confirmed[ids]
        ids, ranges = ids[keep], ranges[keep]
        if len(ids) < 3:
            continue
        directions = delta[keep] / ranges[:, None]
        # Test local support before selecting the first return. An isolated near
        # glitch must not win a direction and shadow the corroborated wall behind it.
        local_support = np.zeros(len(ids), bool)
        for start in range(0, len(ids), 128):
            block = slice(start, start+128)
            nearby = compatible(directions[block] @ directions.T, ranges[block,None], ranges, distance)
            # Each row includes the landmark itself and needs two other neighbours.
            local_support[block] = nearby.sum(axis=1) >= 3
        local = np.flatnonzero(local_support)
        supported[ids[local]] = True
        if len(local) < 3:
            continue
        sectors = (directions[local] @ rays.T).argmax(axis=1)
        contacts = np.full(len(rays), -1, dtype=int)
        # One catch per angular sector, closest supported landmark first.
        order = np.argsort(ranges[local], kind="stable")
        bins, first = np.unique(sectors[order], return_index=True)
        contacts[bins] = local[order[first]]
        faces = contacts[lattice_faces]
        faces = faces[np.all(faces >= 0, axis=1)]
        ends = np.roll(faces, -1, axis=1)
        cosine = np.sum(directions[faces]*directions[ends], axis=2)
        faces = faces[compatible(cosine, ranges[faces], ranges[ends], distance).all(axis=1)]
        for triangle in ids[faces]:
            key = tuple(sorted(triangle.tolist()))
            if key in proposals:
                proposals[key][0] += 1
                continue
            pa,pb,pc = xyz[triangle]
            normal = np.cross(pb-pa, pc-pa)
            area2 = np.linalg.norm(normal)
            sides2 = np.array([np.dot(v,v) for v in (pb-pa, pc-pb, pa-pc)])
            if area2 <= 1e-6 or area2 < .07*sides2.max():
                continue
            oriented = triangle if normal @ (eye-pa) >= 0 else triangle[::-1]
            proposals[key] = [1, frame, oriented.tolist(), area2/sides2.sum()]

    # Merge observations through shared landmark IDs. Prefer repeatedly
    # observed triangles, then well-shaped triangles, when views disagree.
    used_half_edges = set()
    for votes,first,ids,_ in sorted(proposals.values(), key=lambda p: (-p[0], -p[3], p[1], p[2])):
        directed = list(zip(ids, ids[1:] + ids[:1]))
        if any(edge in used_half_edges for edge in directed):
            continue
        used_half_edges.update(directed)
        mesh["patches"].append(dict(landmarks=ids, vertices=xyz[ids].tolist(), first=first))
    observed = np.isfinite(first_eye[:,0])
    return dict(mesh, status="ready", unconfirmed=int((observed & ~confirmed).sum()),
                isolated=int((confirmed & ~supported).sum()), ray_count=len(rays))
