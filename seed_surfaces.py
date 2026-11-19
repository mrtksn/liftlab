"""Grow a metric mesh in camera observation order, testing visibility before insertion."""
from collections import defaultdict, deque
from itertools import product
from math import prod

import numpy as np


MAX_EDGE = .20
NEIGHBOURS = tuple(product((-1, 0, 1), repeat=3))


def unit(vector):
    return vector / max(np.linalg.norm(vector), 1e-15)


def edges(ids):
    for i, a in enumerate(ids):
        yield tuple(sorted((a, ids[(i + 1) % len(ids)])))


def segment_distance(point, start, end):
    edge = end-start
    fraction = min(1., max(0., (point-start) @ edge / (edge @ edge)))
    return np.linalg.norm(point-(start+fraction*edge))


class GrowingMesh:
    def __init__(self, xyz, reach):
        self.xyz = xyz
        self.reach = reach
        self.cell_size = max(reach, MAX_EDGE)
        self.accepted = set()
        self.orphans = {}
        self.orphan_cells = defaultdict(set)
        self.provisional = {}
        self.faces = {}
        self.bounds = {}
        self.planes = {}
        self.cells = defaultdict(set)
        self.large_faces = set()
        self.edge_faces = defaultdict(set)
        self.next_id = 0
        self.frame = None
        self.retired = []

    def cell(self, point):
        return tuple(np.floor(point / self.cell_size).astype(int))

    def around(self, cell):
        x, y, z = cell
        for dx, dy, dz in NEIGHBOURS:
            yield x+dx, y+dy, z+dz

    def face(self, ids, eye):
        vertices = self.xyz[list(ids)]
        key = self.next_id
        self.next_id += 1
        lower, upper = vertices.min(axis=0), vertices.max(axis=0)
        self.bounds[key] = lower-self.reach, upper+self.reach
        lo, hi = self.cell(lower), self.cell(upper)
        dimensions = [range(a,b+1) for a,b in zip(lo,hi)]
        buckets = tuple(product(*dimensions)) if prod(map(len, dimensions)) <= 512 else ()
        if buckets:
            for bucket in buckets:
                self.cells[bucket].add(key)
        else:
            self.large_faces.add(key)
        if len(ids) == 3:
            a, b, c = vertices
            normal = unit(np.cross(b-a, c-a))
            if normal @ (eye-a) < 0:
                normal = -normal
                ids = (ids[0], ids[2], ids[1])
                vertices = vertices[[0,2,1]]
            inward = np.cross(normal, np.roll(vertices, -1, axis=0)-vertices)
            self.planes[key] = normal, inward
            for edge in edges(ids):
                self.edge_faces[edge].add(key)
        else:
            for i in ids:
                self.provisional[i] = key
        self.faces[key] = (ids, vertices, buckets, self.frame)
        return key

    def remove(self, key):
        ids, vertices, buckets, born = self.faces.pop(key)
        del self.bounds[key]
        if len(ids) == 3:
            del self.planes[key]
            if born != self.frame:
                self.retired.append(dict(landmarks=list(ids), vertices=vertices.tolist(),
                                         first=born, until=self.frame))
            for edge in edges(ids):
                faces = self.edge_faces[edge]
                faces.remove(key)
                if not faces:
                    del self.edge_faces[edge]
        else:
            for i in ids:
                self.provisional.pop(i)
        for bucket in buckets:
            self.cells[bucket].remove(key)
            if not self.cells[bucket]:
                del self.cells[bucket]
        if not buckets:
            self.large_faces.remove(key)
        return ids

    def adopt(self, ids, eye):
        # Break old seed pairs when one end joins a triangle. The unused end
        # remains an independent seed eligible for the next extension.
        for key in sorted({self.provisional[i] for i in ids if i in self.provisional}):
            for i in self.remove(key):
                if i not in ids:
                    self.face((i,), eye)
        if len(ids) == 3:
            for i in ids:
                self.orphans.pop(i, None)
                bucket = self.cell(self.xyz[i])
                self.orphan_cells[bucket].discard(i)
                if not self.orphan_cells[bucket]:
                    del self.orphan_cells[bucket]
        return self.face(ids, eye)

    def ray_candidates(self, eye, point):
        # Traverse the grid cells between the observing pose and landmark.
        start, end = np.array(self.cell(eye)), np.array(self.cell(point))
        steps = int(np.abs(end-start).sum())
        if steps > 512:
            return [k for k, (ids, _, _, _) in self.faces.items() if len(ids) == 3]
        result = self.large_faces.copy()
        direction = point-eye
        signs = np.sign(direction).astype(int)
        moving = signs != 0
        delta = np.full(3, np.inf)
        delta[moving] = self.cell_size / np.abs(direction[moving])
        boundary = (start + (signs > 0)) * self.cell_size
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
        candidates = [k for k in self.ray_candidates(eye, point) if len(self.faces[k][0]) == 3]
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

    def attach(self, i, eye, *, pair, changed_face=None):
        point = self.xyz[i]
        candidates = [changed_face] if changed_face is not None else sorted(
            self.large_faces.union(*(self.cells.get(cell, ()) for cell in self.around(self.cell(point)))))
        extensions = []
        refinements = []
        pairs = []
        for key in candidates:
            ids, vertices, _, _ = self.faces[key]
            lower, upper = self.bounds[key]
            if i in ids or np.any(point < lower) or np.any(point > upper):
                continue
            if len(ids) == 1:
                distance = np.linalg.norm(point-vertices[0])
                if pair and 1e-8 < distance <= self.reach:
                    pairs.append((distance, ids))
                continue
            if len(ids) == 2:
                distance = segment_distance(point, *vertices)
                if distance <= self.reach and self.has_area((*ids, i)):
                    extensions.append((1, distance, ids))
                continue
            a = vertices[0]
            normal, inward = self.planes[key]
            ray = point-eye
            denominator = normal @ ray
            along_ray = (normal @ (a-eye))/denominator if abs(denominator) > 1e-12 else -1.
            plane_hit = eye + ray * along_ray if along_ray > 0 else point - normal * (normal @ (point-a))
            sides = np.sum((plane_hit-vertices)*inward, axis=1)
            if min(sides) >= -1e-10:
                depth = np.linalg.norm(plane_hit-point)
                if depth <= min(.02, self.reach) and min(sides) > 1e-10:
                    refinements.append((depth, key))
                continue
            for e, side in enumerate(sides):
                if side >= 0:
                    continue
                edge = (ids[e], ids[(e+1)%3])
                if len(self.edge_faces[tuple(sorted(edge))]) == 1:
                    distance = segment_distance(point, vertices[e], vertices[(e+1)%3])
                    if distance <= self.reach and self.has_area((*edge, i)):
                        extensions.append((0, distance, edge))
        if not (refinements or extensions or pairs):
            return []
        if changed_face is not None and self.occluded(eye, point):
            return []
        if refinements:
            _, key = min(refinements)
            ids = self.remove(key)
            return [self.adopt((*edge, i), eye) for edge in edges(ids)]
        if extensions:
            _, _, base = min(extensions)
            return [self.adopt((*base, i), eye)]
        if pairs:
            _, base = min(pairs)
            return [self.adopt((*base, i), eye)]
        return []

    def has_area(self, ids):
        a,b,c = self.xyz[list(ids)]
        return np.linalg.norm(np.cross(b-a, c-a)) > 1e-8

    def grow(self, i, eye):
        had_observation = i in self.accepted
        if not had_observation:
            self.accepted.add(i)
            self.orphan_cells[self.cell(self.xyz[i])].add(i)
        self.orphans[i] = eye
        changes = self.attach(i, eye, pair=not had_observation)
        if not changes and not had_observation:
            self.face((i,), eye)
        self.retry_seeds(changes)

    def retry_seeds(self, changes):
        pending = deque(changes)
        while pending:
            key = pending.popleft()
            face = self.faces.get(key)
            if not face:
                continue
            _, _, buckets, _ = face
            candidates = set(self.orphans) if not buckets else {
                i for bucket in buckets for cell in self.around(bucket) for i in self.orphan_cells.get(cell, ())
            }
            lower, upper = self.bounds[key]
            for i in sorted(candidates):
                point = self.xyz[i]
                if key not in self.faces:
                    break
                if i in self.orphans and np.all(point >= lower) and np.all(point <= upper):
                    pending.extend(self.attach(i, self.orphans[i], pair=False, changed_face=key))


def build_surfaces(model, growth_distance_m=MAX_EDGE):
    if not np.isfinite(growth_distance_m) or growth_distance_m <= 0:
        raise ValueError("Enter a positive growth distance.")
    mesh = dict(method="camera-ray-seeds-v1", units=model["units"], growth_distance_m=growth_distance_m, patches=[])
    if model["units"] != "m":
        return dict(mesh, status="needs_scale")
    views = model.get("landmark_views")
    if not views:
        return dict(mesh, status="needs_tracks")
    xyz = np.asarray([p[:3] for p in model["points"]], dtype=float).reshape(-1, 3)
    poses = {p["frame"]: np.asarray(p["xyz"]) for p in model["poses"]}
    growth = GrowingMesh(xyz, growth_distance_m)
    hidden = set()
    skips = 0
    for frame, ids in sorted(views.items()):
        growth.frame = frame
        eye = poses.get(frame)
        if eye is None or not np.isfinite(eye).all():
            continue
        # A later observation can give a seed a clearer sightline. Retry
        # other seeds when a face changes in their neighbourhood.
        pending = sorted(i for i in ids if i not in growth.accepted or i in growth.orphans)
        if not pending:
            continue
        distances = np.linalg.norm(xyz[pending]-eye, axis=1)
        for j in np.argsort(distances, kind="stable"):
            if not np.isfinite(distances[j]) or distances[j] <= 1e-8:
                continue
            i = pending[j]
            if i in growth.accepted and i not in growth.orphans:
                continue
            if growth.occluded(eye, xyz[i]):
                if i not in growth.accepted:
                    hidden.add(i)
                    skips += 1
                continue
            growth.grow(i, eye)
    for ids, vertices, _, born in growth.faces.values():
        if len(ids) == 3:
            mesh["patches"].append(dict(vertices=vertices.tolist(), landmarks=list(ids), first=born))
    mesh.update(status="ready", retired=growth.retired, accepted=len(growth.accepted), occluded_observations=skips,
                seeds=sum(len(ids) == 1 for ids, _, _, _ in growth.faces.values()),
                open_pairs=sum(len(ids) == 2 for ids, _, _, _ in growth.faces.values()),
                deferred_landmarks=sorted(hidden-growth.accepted))
    return mesh
