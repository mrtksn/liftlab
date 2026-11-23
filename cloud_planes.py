"""Fit bounded, arbitrarily oriented planar polygons to connected landmark clouds."""
import cv2
import numpy as np

METHOD = "cloud-planes-v1"
LABEL = "Cloud planes"
DISTANCE_LABEL = "Maximum landmark gap"
MIN_SUPPORT = 16


def footprints(uv, radius):
    """Trace connected support in plane coordinates, retaining interior boundaries."""
    origin = uv.min(axis=0)
    # Limit raster size on very thin regions and degenerate input.
    step = max(radius / 5, float(np.ptp(uv, axis=0).max()) / 1500, 1e-7)
    pixels = np.rint((uv-origin)/step).astype(np.int32) + 2
    width, height = pixels.max(axis=0) + 3
    mask = np.zeros((height, width), np.uint8)
    hull_mask = np.zeros_like(mask)
    cv2.fillConvexPoly(hull_mask, cv2.convexHull(pixels), 1)
    r = int(np.floor(radius/step))
    for x,y in pixels:
        cv2.circle(mask, (int(x),int(y)), r, 1, -1)
    mask &= hull_mask
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    membership = labels[pixels[:,1],pixels[:,0]]
    for label in range(1,count):
        ids = np.flatnonzero(membership == label)
        if len(ids) < MIN_SUPPORT:
            continue
        x,y,w,h,_ = stats[label]
        contours, tree = cv2.findContours((labels[y:y+h,x:x+w] == label).astype(np.uint8),
                                          cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        outer = next(i for i,entry in enumerate(tree[0]) if entry[3] < 0)
        rings = []
        for i in [outer] + [j for j,entry in enumerate(tree[0]) if entry[3] == outer]:
            contour = cv2.approxPolyDP(contours[i], max(1, .35*r), True).reshape(-1,2)
            if len(contour) >= 3:
                rings.append((contour + [x-2,y-2])*step + origin)
            elif i == outer:
                break
        if rings:
            yield ids, rings


def build_planes(model, span, observations=None, *, relative_fit=False):
    if not np.isfinite(span) or span <= 0:
        raise ValueError("Enter a positive maximum landmark gap.")
    tolerance = span*.12 if relative_fit and model["units"]!="m" else min(.04, span*.12)
    mesh = dict(method=METHOD, units=model["units"], growth_distance_m=span, tolerance_m=tolerance,
                status="ready", patches=[], retired=[], support_count=0)
    if model["units"] != "m" and not relative_fit:
        return dict(mesh, status="needs_scale")

    xyz = np.asarray([p[:3] for p in model["points"]], float).reshape(-1,3)
    first = {}
    for frame, ids in sorted(model.get("landmark_views", {}).items()):
        for i in ids:
            if 0 <= i < len(xyz) and i not in first:
                first[i] = frame
    finish = max(model.get("path_frames") or [p["frame"] for p in model.get("poses", [])] or list(first) or [""])
    original = np.flatnonzero(np.isfinite(xyz).all(axis=1))
    xyz = xyz[original]
    if len(xyz) < MIN_SUPPORT:
        return mesh

    # Normals and flatness from a local least-squares fit. Line-shaped or
    # volumetric neighborhoods cannot establish a reliable surface orientation.
    coords = np.ascontiguousarray(xyz - xyz.mean(axis=0), dtype=np.float32)
    distances, neighbours = cv2.batchDistance(coords, coords, cv2.CV_32F, normType=cv2.NORM_L2, K=12)
    local = xyz[neighbours]
    centered = local - local.mean(axis=1, keepdims=True)
    values, vectors = np.linalg.eigh(centered.transpose(0,2,1) @ centered)
    normals = vectors[:,:,0]
    flatness = np.maximum(0,values[:,0]) / np.maximum(values[:,1],1e-14)
    usable = (flatness < .22) & (values[:,1] > .035*values[:,2]) & (distances[:,5] <= span)
    agreement = np.cos(np.deg2rad(30))
    claimed = np.zeros(len(xyz), bool)
    for seed in np.argsort(flatness, kind="stable"):
        if claimed[seed] or not usable[seed]:
            continue
        normal = normals[seed]
        # Anchor the growth to the seed plane so a chain cannot bend around a corner.
        frontier = [seed]
        region = {seed}
        while frontier:
            i = frontier.pop()
            for j in neighbours[i, distances[i] <= span]:
                if j in region or claimed[j]:
                    continue
                if usable[j] and abs(normals[j] @ normal) < agreement:
                    continue
                if abs((xyz[j]-xyz[seed]) @ normal) <= tolerance:
                    region.add(j)
                    frontier.append(j)
        if len(region) < MIN_SUPPORT:
            continue
        members = np.asarray(sorted(region))
        points = xyz[members]
        center = points.mean(axis=0)
        _, _, axes = np.linalg.svd(points-center, full_matrices=False)
        inliers = np.abs((points-center) @ axes[2]) <= tolerance
        members, points = members[inliers], points[inliers]
        if len(points) < MIN_SUPPORT:
            continue
        center = points.mean(axis=0)
        _, sizes, axes = np.linalg.svd(points-center, full_matrices=False)
        if sizes[1] < .10*sizes[0] or sizes[1] <= 1e-8 or sizes[2] > .30*sizes[1]:
            continue
        uv = (points-center) @ axes[:2].T
        radius = min(span/2, float(np.median(distances[members,4]))*1.35)
        if radius <= 1e-7:
            continue
        for group, rings in footprints(uv, radius):
            ids = original[members[group]].tolist()
            loops = [(center + ring @ axes[:2]).tolist() for ring in rings]
            residual = (points[group]-center) @ axes[2]
            mesh["patches"].append(dict(vertices=loops[0], holes=loops[1:], normal=axes[2].tolist(),
                                        landmarks=ids, first=max(first.get(i, finish) for i in ids),
                                        rms_m=float(np.sqrt(np.mean(residual**2)))))
            claimed[members[group]] = True
    mesh["support_count"] = int(claimed.sum())
    return indoor_constraints(model, mesh, observations or {})


def indoor_constraints(model, mesh, observations):
    """Use room directions and synchronized measurements only where they agree with the cloud."""
    from fusion import gravity_alignment, up_alignment

    faces = mesh["patches"]
    if not faces:
        return mesh
    xyz = np.asarray([p[:3] for p in model["points"]])
    poses = model.get("poses", [])
    motion = [p for p in poses if 'rotation' in p and 'q' in observations.get(p['frame'],{})]
    up = None
    source = "unresolved"
    if model.get("gravity_aligned"):
        up, source = np.array([0.,0.,1.]), "inertial gravity"
    elif len(motion) >= 12:
        align = gravity_alignment([np.asarray(p['rotation']) for p in motion],
                                  [observations[p['frame']]['q'] for p in motion],
                                  [int(p['frame'].split('.')[0]) for p in motion])
        if align is not None:
            up, source = align[2], "inertial gravity"
    if up is None:
        levelled = [p for p in poses if 'rotation' in p and 'up' in observations.get(p['frame'],{})]
        fit = up_alignment([np.asarray(p['rotation']) for p in levelled],
                           [observations[p['frame']]['up'] for p in levelled])
        if fit is not None:
            up, source = fit["up"], "inertial up fit"

    normals = np.asarray([f["normal"] for f in faces])
    weights = np.array([len(f["landmarks"]) for f in faces])
    parallel = np.cos(np.deg2rad(8))
    votes = (abs(normals @ normals.T) >= parallel) @ weights
    best = np.argsort(-votes,kind='stable')
    # Height changes can disambiguate which observed plane family is horizontal.
    # Hold out alternating blocks of poses; compare against a constant-height baseline.
    heights = [p for p in poses if "height" in observations.get(p['frame'],{})]
    height_error = None
    if len(heights) >= 24:
        positions = np.array([p['xyz'] for p in heights])
        baro = np.array([observations[p['frame']]['height'] for p in heights])
        train = np.arange(len(heights))//8 % 2 == 0
        hold = ~train
        def height_residual(axis):
            residual = baro - positions @ axis
            return np.abs(residual - np.median(residual[train]))
        if up is None and np.ptp(np.percentile(baro[train],[10,90])) > .35:
            candidates = np.concatenate((normals[best[:12]], -normals[best[:12]]))
            candidate = min(candidates, key=lambda axis: np.median(height_residual(axis)[train]))
            residual = height_residual(candidate)
            baseline = np.abs(baro[hold]-np.median(baro[train]))
            if (np.median(residual[hold]) < .65*np.median(baseline)
                    and np.percentile(residual[hold],80) < .22
                    and np.ptp(np.percentile(positions[hold] @ candidate,[10,90])) > .30):
                up, source = candidate, "barometric height trend"
        if up is not None:
            height_error = float(np.median(height_residual(up)[hold]))

    axes = [up if up is not None else normals[best[0]]]
    for i in best:
        n = normals[i]
        if abs(n @ axes[0]) <= np.sin(np.deg2rad(8)):
            tangent = n - (n @ axes[0])*axes[0]
            axes.append(tangent / np.linalg.norm(tangent))
            axes.append(np.cross(*axes))
            break
    axes = np.array(axes)
    tolerance = mesh['tolerance_m']
    snapped = 0
    for face,normal in zip(faces,normals):
        points = xyz[face['landmarks']]
        center = points.mean(axis=0)
        alignment = axes @ normal
        idx = np.argmax(abs(alignment))
        target = axes[idx] * np.sign(alignment[idx])
        residual = (points-center) @ target
        rms = float(np.sqrt(np.mean(residual**2)))
        if (abs(alignment[idx]) >= parallel and np.max(abs(residual)) <= tolerance
                and rms <= face['rms_m'] + .20*tolerance):
            # Orthogonal projection leaves every boundary loop on the same plate.
            for loop in [face['vertices'], *face['holes']]:
                vertices = np.array(loop)
                loop[:] = (vertices - np.outer((vertices-center) @ target,target)).tolist()
            face.update(normal=target.tolist(), rms_m=rms)
            snapped += 1

    # Test the nearest bounded plate on each beam. A ray through a doorway
    # can reach a farther surface. Grazing or unsupported returns supply no offset.
    polygons = []
    for face in faces:
        n = np.asarray(face['normal'])
        keep = np.arange(3) != np.argmax(abs(n))
        rings = [np.ascontiguousarray(np.asarray(r)[:,keep], dtype=np.float32)
                 for r in [face['vertices'], *face['holes']]]
        polygons.append((n,np.asarray(face['vertices'][0]),keep,rings))
    offsets = [[] for f in faces]
    comparisons = [0]*len(faces)
    rays = compared = matches = 0
    for p in poses:
        obs = observations.get(p['frame'],{})
        distance = obs.get('range')
        if distance is None or distance < .25:
            continue
        if 'rotation' in p:
            direction = np.asarray(p['rotation'])[:,2]
        elif 'forward' in p:
            direction = np.asarray(p['forward'])
        else:
            continue
        eye = np.asarray(p['xyz'])
        direction = direction / np.linalg.norm(direction)
        rays += 1
        nearest = None
        for i,(n,origin,keep,rings) in enumerate(polygons):
            cosine = float(n @ direction)
            if abs(cosine) < .20:
                continue
            t = float(n @ (origin-eye) / cosine)
            if t <= 0 or (nearest and t >= nearest[0]):
                continue
            hit = tuple((eye+t*direction)[keep].tolist())
            if cv2.pointPolygonTest(rings[0],hit,False) >= 0 and all(
                    cv2.pointPolygonTest(h,hit,False)<0 for h in rings[1:]):
                nearest = t,i,cosine
        if nearest:
            compared += 1
            t,i,cosine = nearest
            comparisons[i] += 1
            if abs(distance-t) < max(.08,.10*distance):
                matches += 1
                offsets[i].append(((distance-t)*cosine,eye))
    adjusted = 0
    for face,samples,total in zip(faces,offsets,comparisons):
        face['tof_matches'] = len(samples)
        face['tof_compared'] = total
        if len(samples) < max(3, .67*total) or np.ptp([s[1] for s in samples],axis=0).max() < .08:
            continue
        n = np.asarray(face['normal'])
        # Give the visual fit at least as much weight as the median range offset.
        shift = float(np.clip(np.median([s[0] for s in samples])*.5,-tolerance/2,tolerance/2))
        residual = (xyz[face['landmarks']] - face['vertices'][0]) @ n - shift
        if np.max(abs(residual)) > tolerance:
            continue
        for loop in [face['vertices'],*face['holes']]:
            loop[:] = (np.array(loop) + shift*n).tolist()
        face.update(rms_m=float(np.sqrt(np.mean(residual**2))), tof_offset_m=shift)
        adjusted += 1
    mesh['indoor'] = dict(regularized=snapped, up_source=source, up=None if up is None else up.tolist(),
                          height_samples=len(heights), height_median_error_m=height_error,
                          attitude_samples=len(motion), range_rays=rays, intersected=compared,
                          range_matches=matches, range_adjusted=adjusted)
    return mesh
