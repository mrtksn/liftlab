"""Reconstruct supported local planes in the survey's 3D coordinate frame."""
from collections import deque

import cv2
import numpy as np


def voxel_centres(points, spacing):
    cells = np.floor(points / spacing).astype(np.int64)
    _, index, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    return np.column_stack([np.bincount(index, weights=axis) / counts for axis in points.T])


def local_geometry(points):
    # Nearest neighbours after voxel averaging. The bounded neighbourhood also
    # prevents distant parallel sheets from contributing to the same normal.
    p = np.ascontiguousarray(points, dtype=np.float32)
    if len(p) <= 18000:
        distances, neighbours = cv2.batchDistance(p, p, cv2.CV_32F, K=min(20, len(p)))
    else:
        tree = cv2.flann_Index(p, dict(algorithm=1, trees=4))
        neighbours, squared = tree.knnSearch(p, 20, params=dict(checks=96))
        distances = np.sqrt(squared)
    nearby = distances <= 5.0
    offsets = points[neighbours] - points[:, None]
    # At corners or in noisy tracks, plain PCA mixes surfaces. Test planes
    # through the centre landmark and neighbourhood pairs before the PCA fit.
    k = offsets.shape[1]
    pairs = np.stack(np.triu_indices(k - 1, 1), axis=1) + 1
    pairs = pairs[np.linspace(0, len(pairs) - 1, min(48, len(pairs)), dtype=int)]
    support = np.zeros_like(nearby)
    for start in range(0, len(points), 512):
        block = offsets[start:start + 512]
        local = nearby[start:start + 512]
        normals = np.cross(block[:, pairs[:, 0]], block[:, pairs[:, 1]])
        lengths = np.linalg.norm(normals, axis=2)
        normals /= np.maximum(lengths[:, :, None], 1e-12)
        residual = np.abs(block @ normals.transpose(0, 2, 1))
        weights = np.maximum(0, 1 - (residual / .55)**2) * local[:, :, None]
        usable = local[:, pairs[:, 0]] & local[:, pairs[:, 1]] & (lengths > .25)
        best = np.argmax(weights.sum(axis=1) * usable, axis=1)
        residual = residual[np.arange(len(block)), :, best]
        support[start:start + len(block)] = local & (residual <= .55) & usable.any(axis=1)[:, None]
    offsets -= (offsets * support[:, :, None]).sum(axis=1)[:, None] / np.maximum(1, support.sum(axis=1))[:, None, None]
    offsets *= support[:, :, None]
    values, vectors = np.linalg.eigh(offsets.transpose(0, 2, 1) @ offsets)
    # Reject line-like features and volumetric scatter. Give priority to
    # seeds with well-supported area and a well-determined normal.
    spread = np.maximum(values[:, 1], 1e-12)
    confidence = (1 - values[:, 0] / spread) * np.sqrt(spread / np.maximum(values[:, 2], 1e-12))
    reliable = (support.sum(axis=1) >= 7) & (support.sum(axis=1) >= .50 * nearby.sum(axis=1)) & (values[:, 0] < .20 * spread) & (spread > .07 * values[:, 2])
    return neighbours, nearby, vectors[:, :, 0], confidence, reliable


def planar_regions(points):
    neighbours, nearby, normals, confidence, reliable = local_geometry(points)
    available = reliable.copy()
    for seed in np.argsort(-confidence, kind="stable"):
        if not available[seed]:
            continue
        queue = deque([seed])
        region = {seed}
        while queue:
            current = queue.popleft()
            candidates = neighbours[current, nearby[current]]
            candidates = candidates[available[candidates]]
            aligned = np.abs(normals[candidates] @ normals[seed]) >= .89
            close = np.abs((points[candidates] - points[seed]) @ normals[seed]) <= .85
            for index in candidates[aligned & close]:
                if index not in region:
                    region.add(index)
                    queue.append(index)
        if len(region) < 10:
            continue
        indices = np.array(sorted(region))
        centre = points[indices].mean(axis=0)
        _, spread, axes = np.linalg.svd(points[indices] - centre, full_matrices=False)
        if spread[1] < .2 * spread[0] or spread[2] > .30 * spread[1]:
            continue
        available[indices] = False
        yield centre, axes, points[indices]


def triangles_on_plane(uv, max_edge):
    # OpenCV triangulates in a 2D plane basis. Keep indices into the 3D
    # vertices so we can lift the mesh back into its fitted orientation.
    pixels = np.asarray((uv - uv.min(axis=0)) * 32 + 2, dtype=np.float32)
    _, ids = np.unique(pixels, axis=0, return_index=True)
    if len(ids) < 3:
        return
    width, height = np.ceil(pixels.max(axis=0)).astype(int) + 3
    subdivision = cv2.Subdiv2D((0, 0, int(width), int(height)))
    lookup = {tuple(pixels[i]): i for i in ids}
    subdivision.insert([tuple(map(float, pixels[i])) for i in ids])
    for triangle in np.asarray(subdivision.getTriangleList()).reshape(-1, 3, 2):
        indices = [lookup.get(tuple(vertex)) for vertex in triangle]
        if None in indices:
            continue
        corners = uv[indices]
        edges = np.roll(corners, -1, axis=0) - corners
        lengths = np.linalg.norm(edges, axis=1)
        area2 = abs(edges[0, 0] * edges[1, 1] - edges[0, 1] * edges[1, 0])
        if lengths.max() <= max_edge and area2 > .09 * lengths.max()**2:
            yield indices, area2 / 2


def inside_triangle(samples, triangle):
    """Indices of samples covered by a triangle in plane coordinates."""
    indices = np.flatnonzero(((samples >= triangle.min(axis=0)) &
                              (samples <= triangle.max(axis=0))).all(axis=1))
    if len(indices):
        edges = np.roll(triangle, -1, axis=0) - triangle
        offsets = samples[indices, None] - triangle
        cross = edges[:, 0] * offsets[:, :, 1] - edges[:, 1] * offsets[:, :, 0]
        indices = indices[(cross >= -1e-8).all(axis=1) | (cross <= 1e-8).all(axis=1)]
    return indices


def build_surfaces(model, observations=None):
    gravity = bool(model.get("gravity_aligned"))
    mesh = dict(method="local-normal-mesh-v4", units=model["units"], gravity_aligned=gravity, patches=[])
    xyz = np.asarray([p[:3] for p in model["points"]], dtype=float).reshape(-1, 3)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) < 10:
        return mesh

    # A single scale for all axes preserves angles, aspect ratio and pose alignment.
    origin = np.median(xyz, axis=0)
    radius = np.linalg.norm(xyz - origin, axis=1)
    extent = np.percentile(radius, 98)
    if extent <= 1e-9:
        return mesh
    spacing = extent / 80
    metric = model["units"] == "m"
    if metric:
        spacing = max(.05, spacing)
    points = voxel_centres((xyz[radius <= 2 * extent] - origin) / spacing, 1.)
    if len(points) < 10:
        return mesh

    endpoints = []
    if metric and observations and "tof" in model.get("streams", ()):
        for pose in model["poses"]:
            distance = observations.get(pose["frame"], {}).get("range")
            if distance is not None and np.isfinite(distance) and distance >= .18 and "forward" in pose:
                endpoints.append(np.asarray(pose["xyz"]) + distance * np.asarray(pose["forward"]))
    endpoints = (np.asarray(endpoints).reshape(-1, 3) - origin) / spacing
    path = (np.asarray([p["xyz"] for p in model["poses"]]).reshape(-1, 3) - origin) / spacing
    order = {name: i for i, name in enumerate(model.get("path_frames") or model.get("sampled_frames", []))}
    frames = [order.get(p.get("frame"), -2) for p in model["poses"]]
    connected = (np.diff(frames) == 1) & (np.array(frames[:-1]) >= 0)
    max_edge = min(4., .36 / spacing) if metric else 4.
    floor_candidates = []
    for plane, (centre, axes, region) in enumerate(planar_regions(points)):
        basis, normal = axes[:2], axes[2]
        # Decimate within this region only. Each vertex is the measured local
        # centroid projected onto the fitted plane.
        uv = voxel_centres((region - centre) @ basis.T, 2.)
        vertices = ((centre + uv @ basis) * spacing + origin).round(8).tolist()
        path_height = (path - centre) @ normal
        path_uv = (path - centre) @ basis.T
        protected = path_uv[np.abs(path_height) < .9]
        crossings = np.flatnonzero(connected & (path_height[:-1] * path_height[1:] < 0))
        if len(crossings):
            h = path_height[crossings]
            fraction = h / (h - path_height[crossings + 1])
            intersections = path_uv[crossings] + fraction[:, None] * (path_uv[crossings + 1] - path_uv[crossings])
            protected = np.vstack([protected, intersections])
        hits = endpoints[np.abs((endpoints - centre) @ normal) <= .85]
        hits_uv = (hits - centre) @ basis.T
        floor_possible = gravity and abs(normal[2]) > .966 and len(path) and np.mean(path_height * normal[2] > 1.) >= .75
        under_path = set()
        orientation = "level" if abs(normal[2]) >= np.cos(np.deg2rad(15)) else (
            "upright" if abs(normal[2]) <= np.sin(np.deg2rad(15)) else "sloped")
        area = 0.
        for ids, triangle_area in triangles_on_plane(uv, max_edge):
            tri = uv[ids]
            if len(protected) and len(inside_triangle(protected, tri)):
                continue
            supported = len(hits_uv) >= 2 and len(inside_triangle(hits_uv, tri)) >= 2
            face = dict(vertices=[vertices[i] for i in ids], plane=plane, tof_supported=bool(supported))
            if gravity:
                face["orientation"] = orientation
            mesh["patches"].append(face)
            area += triangle_area
            if floor_possible:
                under_path.update(i for i in inside_triangle(path_uv, tri) if path_height[i] * normal[2] > 1.)
        if floor_possible and len(under_path) >= max(3, .04 * len(path)):
            floor_candidates.append((centre[2], plane, area))

    if floor_candidates:
        min_area = .45 * max(area for _, _, area in floor_candidates)
        _, floor, _ = min(item for item in floor_candidates if item[2] >= min_area)
        mesh["floor_plane"] = floor
        for patch in mesh["patches"]:
            if patch["plane"] == floor:
                patch["kind"] = "floor"
    return mesh
