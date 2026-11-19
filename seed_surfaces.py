"""Grow a metric mesh in camera observation order, testing visibility before insertion."""
from collections import defaultdict
from itertools import product

import numpy as np


MAX_EDGE = .20
NEIGHBOURS = tuple(product((-1, 0, 1), repeat=3))


def cell(point):
    return tuple(np.floor(point / MAX_EDGE).astype(int))


def unit(vector):
    return vector / max(np.linalg.norm(vector), 1e-15)


def edges(ids):
    for i, a in enumerate(ids):
        yield tuple(sorted((a, ids[(i + 1) % len(ids)])))


class GrowingMesh:
    def __init__(self, xyz):
        self.xyz = xyz
        self.accepted = set()
        self.nearby = defaultdict(list)
        self.faces = {}
        self.cells = defaultdict(set)
        self.incident = defaultdict(set)
        self.edge_faces = defaultdict(set)
        self.next_id = 0
        self.frame = None
        self.created = {}
        self.retired = []

    def face(self, ids, vertices, eye):
        if len(ids) < 3:
            key = self.next_id
            self.next_id += 1
            self.faces[key] = (ids, vertices, [])
            for i in ids:
                self.incident[i].add(key)
            return
        a, b, c = vertices
        normal = np.cross(b-a, c-a)
        if normal @ (eye-a) < 0:
            vertices = vertices[[0, 2, 1]]
            if len(ids) == 3:
                ids = (ids[0], ids[2], ids[1])
        lo, hi = cell(vertices.min(axis=0)), cell(vertices.max(axis=0))
        buckets = list(product(*(range(a, b+1) for a, b in zip(lo, hi))))
        key = self.next_id
        self.next_id += 1
        self.faces[key] = (ids, vertices, buckets)
        self.created[key] = self.frame
        for bucket in buckets:
            self.cells[bucket].add(key)
        for i in ids:
            self.incident[i].add(key)
        if len(ids) == 3:
            for edge in edges(ids):
                self.edge_faces[edge].add(key)

    def remove(self, key):
        ids, vertices, buckets = self.faces.pop(key)
        if len(ids) == 3:
            born = self.created.pop(key)
            if born != self.frame:
                self.retired.append(dict(landmarks=list(ids), vertices=vertices.tolist(),
                                         first=born, until=self.frame))
        for bucket in buckets:
            self.cells[bucket].remove(key)
            if not self.cells[bucket]:
                del self.cells[bucket]
        for i in ids:
            self.incident[i].remove(key)
        if len(ids) == 3:
            for edge in edges(ids):
                self.edge_faces[edge].remove(key)

    def ray_candidates(self, eye, point):
        # Traverse every 20 cm grid cell between the observing pose and landmark.
        start, end = np.array(cell(eye)), np.array(cell(point))
        steps = int(np.abs(end-start).sum())
        if steps > 512:
            return [k for k, (ids, _, _) in self.faces.items() if len(ids) == 3]
        result = set()
        direction = point-eye
        signs = np.sign(direction).astype(int)
        moving = signs != 0
        delta = np.full(3, np.inf)
        delta[moving] = MAX_EDGE / np.abs(direction[moving])
        boundary = (start + (signs > 0)) * MAX_EDGE
        t = np.full(3, np.inf)
        t[moving] = (boundary-eye)[moving] / direction[moving]
        for _ in range(steps+1):
            result.update(self.cells.get(tuple(start), ()))
            if np.array_equal(start, end):
                break
            axis = t.argmin()
            start[axis] += signs[axis]
            t[axis] += delta[axis]
        return result

    def occluded(self, eye, point):
        candidates = self.ray_candidates(eye, point)
        if not candidates:
            return False
        triangles = np.array([self.faces[k][1] for k in candidates])
        a = triangles[:, 0]
        u, v = triangles[:, 1]-a, triangles[:, 2]-a
        # Two-sided segment/triangle intersection. Touching the target itself
        # is allowed, including when refining the interior of an existing face.
        direction = point-eye
        h = np.cross(direction, v)
        det = np.sum(u*h, axis=1)
        valid = abs(det) > 1e-14
        inv = np.divide(1., det, out=np.zeros_like(det), where=valid)
        s = eye-a
        q = np.cross(s, u)
        x = np.sum(s*h, axis=1)*inv
        y = (q @ direction)*inv
        t = np.sum(v*q, axis=1)*inv
        return np.any(valid & (x >= -1e-9) & (y >= -1e-9) & (x+y <= 1+1e-9) &
                      (t > 1e-9) & (t < 1-1e-7/max(np.linalg.norm(direction), 1e-7)))

    def grow(self, i, eye):
        point = self.xyz[i]
        key = cell(point)
        neighbours = [j for dx, dy, dz in NEIGHBOURS
                      for j in self.nearby.get((key[0]+dx, key[1]+dy, key[2]+dz), ())
                      if np.linalg.norm(self.xyz[j]-point) <= MAX_EDGE]
        candidates = sorted({k for j in neighbours for k in self.incident[j]})
        close = set(neighbours)
        extensions = []
        refinements = []
        for k in candidates:
            ids, v, _ = self.faces[k]
            if len(ids) < 3:
                if close.issuperset(ids):
                    score = max(np.linalg.norm(point-self.xyz[j]) for j in ids)
                    if (len(ids) == 1 and score > 1e-8) or (len(ids) == 2 and self.facing((*ids, i), eye) is not None):
                        extensions.append((score, k, ids))
                continue
            a, b, c = v
            n = unit(np.cross(b-a, c-a))
            # Insert within a triangle only for a nearby observation on its
            # visible side. A foreground object well clear of it starts a seed.
            ray = point-eye
            denom = n @ ray
            if abs(denom) < 1e-12:
                continue
            hit = eye + ray * ((n @ (a-eye))/denom)
            sides = [np.cross(v[(j+1)%3]-v[j], hit-v[j]) @ n for j in range(3)]
            if min(sides) >= -1e-10:
                depth = np.linalg.norm(hit-point)
                if close.issuperset(ids) and depth <= .02 and min(sides) > 1e-10:
                    refinements.append((depth, k))
                continue
            for j, side in enumerate(sides):
                edge = (ids[j], ids[(j+1)%3])
                if side >= 0 or not close.issuperset(edge) or len(self.edge_faces[tuple(sorted(edge))]) != 1:
                    continue
                tri = (*edge, i)
                # Reject a fold across the existing sheet or a grazing sliver.
                new_normal = self.facing(tri, eye)
                if new_normal is None or abs(new_normal @ n) < .60:
                    continue
                score = max(np.linalg.norm(point-self.xyz[v]) for v in edge)
                extensions.append((score, k, edge))
        if refinements:
            _, k = min(refinements)
            ids = self.faces[k][0]
            self.remove(k)
            for edge in edges(ids):
                tri = (*edge, i)
                self.face(tri, self.xyz[list(tri)], eye)
        elif extensions:
            _, k, base = min(extensions)
            if len(self.faces[k][0]) < 3:
                self.remove(k)
            ids = (*base, i)
            self.face(ids, self.xyz[list(ids)], eye)
        else:
            self.face((i,), self.xyz[[i]], eye)
        self.accepted.add(i)
        self.nearby[key].append(i)

    def facing(self, tri, eye):
        a, b, c = self.xyz[list(tri)]
        normal = np.cross(b-a, c-a)
        length = np.linalg.norm(normal)
        if length < 1e-8 or abs(normal @ unit(eye-a)) < .10*length:
            return None
        return normal/length


def build_surfaces(model):
    mesh = dict(method="camera-ray-seeds-v1", units=model["units"], max_edge_m=MAX_EDGE, patches=[])
    if model["units"] != "m":
        return dict(mesh, status="needs_scale")
    views = model.get("landmark_views")
    if not views:
        return dict(mesh, status="needs_tracks")
    xyz = np.asarray([p[:3] for p in model["points"]], dtype=float).reshape(-1, 3)
    poses = {p["frame"]: np.asarray(p["xyz"]) for p in model["poses"]}
    growth = GrowingMesh(xyz)
    hidden = set()
    skips = 0
    for frame, ids in sorted(views.items()):
        growth.frame = frame
        eye = poses.get(frame)
        if eye is None or not np.isfinite(eye).all():
            continue
        pending = sorted(set(ids) - growth.accepted)
        if not pending:
            continue
        distances = np.linalg.norm(xyz[pending]-eye, axis=1)
        for j in np.argsort(distances, kind="stable"):
            if not np.isfinite(distances[j]) or distances[j] <= 1e-8:
                continue
            i = pending[j]
            if growth.occluded(eye, xyz[i]):
                hidden.add(i)
                skips += 1
                continue
            growth.grow(i, eye)
    for key, (ids, vertices, _) in growth.faces.items():
        if len(ids) == 3:
            mesh["patches"].append(dict(vertices=vertices.tolist(), landmarks=list(ids), first=growth.created[key]))
    mesh.update(status="ready", retired=growth.retired, accepted=len(growth.accepted), occluded_observations=skips,
                seeds=sum(len(ids) == 1 for ids, _, _ in growth.faces.values()),
                open_pairs=sum(len(ids) == 2 for ids, _, _ in growth.faces.values()),
                deferred_landmarks=sorted(hidden-growth.accepted))
    return mesh
