"""Associate Pi-clock measurements and estimate an optional single-beam scale."""
import csv
from pathlib import Path

import numpy as np

from node_params import pairing_ns, sensor_settings, tof_metres


STREAMS = ("camera", "tof", "gyro", "accel", "barometer")
STREAM_LABELS = {"camera": "Camera", "tof": "ToF", "gyro": "Gyroscope", "accel": "Accelerometer",
                 "barometer": "Barometer"}


def gyro_attitudes(motion, times):
    """Integrate gyro-only relative rotations within uninterrupted intervals."""
    result = []
    q = np.array([1., 0., 0., 0.])
    segment = 0
    last = None
    for i, row in enumerate(motion):
        try:
            rate = np.array([float(row[key+"_rad_s"]) for key in ("gx","gy","gz")])
        except (KeyError, ValueError):
            rate = np.full(3,np.nan)
        dt = times[i]-times[i-1] if i else 0
        valid = np.isfinite(rate).all()
        if not valid or last is None or not 0 < dt < .05:
            q = np.array([1.,0.,0.,0.])
            segment += 1
        else:
            turn = (rate+last) * (.5*dt)
            angle = np.linalg.norm(turn)
            w, v = np.cos(angle/2), .5*np.sinc(angle/(2*np.pi))*turn
            q = np.r_[q[0]*w-q[1:]@v, q[0]*v+w*q[1:]+np.cross(q[1:],v)]
            q /= np.linalg.norm(q)
        result.append((q,segment) if valid else None)
        last = rate if valid else None
    return result


def load_observations(run, streams=STREAMS, sensors=None):
    sensors = sensors or sensor_settings(run)
    window = pairing_ns(sensors)*1e-9
    def rows(name):
        path = Path(run) / name
        if path.exists():
            with path.open() as stream:
                yield from csv.DictReader(stream)

    motion = list(rows("motion.csv")) if any(k in streams for k in ("gyro","accel","barometer")) else []
    times = np.array([float(r["pi_uart_monotonic_s"]) for r in motion])
    gyro = gyro_attitudes(motion,times) if "gyro" in streams and "accel" not in streams else None
    data = {}
    for row in rows("timestamps.csv"):
        obs = data[row["frame"]] = {}
        if "tof" in streams and row.get("tof_raw"):
            metres = tof_metres(row["tof_raw"], sensors)
            if metres is not None:
                obs["range"] = metres
        if not times.size or not row.get("pi_encoder_callback_ns"):
            continue
        t = float(row["pi_encoder_callback_ns"]) * 1e-9
        j = int(times.searchsorted(t))
        k = min((i for i in (j-1, j) if 0 <= i < len(times)), key=lambda i: abs(times[i]-t))
        if abs(times[k]-t) > window:
            continue
        r = motion[k]
        if "gyro" in streams and "accel" in streams:
            q = np.array([float(r[v]) for v in ("qw","qx","qy","qz")])
            if abs(np.linalg.norm(q)-1) < .02:
                obs.update(q=q / np.linalg.norm(q), q_segment=0)
                obs["up"] = body_up(obs["q"])
        elif gyro is not None and gyro[k] is not None:
            # Gyro-only attitude is relative to its segment start: no up direction.
            obs["q"], obs["q_segment"] = gyro[k]
        elif "accel" in streams:
            a = np.array([float(r[v]) for v in ("ax_m_s2","ay_m_s2","az_m_s2")])
            norm = np.linalg.norm(a)
            if abs(norm-9.81) < .5:
                # Near 1 g the specific force points away from the ground.
                obs["accel_direction"] = obs["up"] = a / norm
        if "barometer" in streams and float(r["height_valid"]) and np.isfinite(float(r["height_m"])):
            obs["height"] = float(r["height_m"])
    return data


def sensor_pairs(names, similarities, observations):
    # Add candidates to the visual-only shortlist. A bad sensor cue cannot
    # remove a visual candidate or bypass geometric verification.
    for i, name in enumerate(names):
        a = observations.get(name, {})
        candidates = []
        for j in range(i+9, len(names)):
            b = observations.get(names[j], {})
            penalties = []
            if "q" in a and "q" in b and a.get("q_segment")==b.get("q_segment"):
                penalties.append(1-abs(float(a["q"] @ b["q"])))
            if "accel_direction" in a and "accel_direction" in b:
                penalties.append(.5*(1-float(a["accel_direction"] @ b["accel_direction"])))
            for key in ("height", "range"):
                if key in a and key in b:
                    penalties.append(min(abs(a[key]-b[key]), 1))
            if penalties:
                candidates.append((similarities[i,j]-.35*np.mean(penalties), j))
        for _, j in sorted(candidates, reverse=True)[:5]:
            yield name, names[j]


def range_ratio(observation, camera, pixels, depths):
    distance = observation.get("range")
    if distance is None or distance < .25:
        return None
    centre = [camera.principal_point_x, camera.principal_point_y]
    near = np.linalg.norm(np.asarray(pixels).reshape(-1, 2) - centre, axis=1) < 32
    depths = np.asarray(depths)
    sample = depths[near & (depths > 0)]
    if len(sample) < 5:
        return None
    z = np.median(sample)
    if np.ptp(np.percentile(sample, [10, 90])) < .12*z:
        return (distance/z, distance, z)
    return None


def estimate_scale(model, images, observations, extra=None):
    ratios = {}
    for im in images:
        if not observations.get(im.name, {}).get("range"):
            continue
        points = [p for p in im.points2D if p.has_point3D()]
        transform = im.cam_from_world()
        ratio = range_ratio(observations[im.name], im.camera, [p.xy for p in points],
                            [(transform * model.point3D(p.point3D_id).xyz)[2] for p in points])
        if ratio:
            ratios[im.name] = ratio
    primary = checked_scale([ratios[f] for f in sorted(ratios)])
    if primary or not extra:
        return primary
    ratios.update(extra)
    return checked_scale([ratios[f] for f in sorted(ratios)])


def checked_scale(ratios):
    if len(ratios) < 12:
        return None
    # Check on withheld views and require a change in target distance.
    train, test = ratios[::2], ratios[1::2]
    scale = float(np.median([x[0] for x in train]))
    distances = [x[1] for x in ratios]
    err = [abs(z*scale-r)/r for _,r,z in test]
    if np.ptp(np.percentile(distances,[10,90])) < .30 or np.percentile(err,80) > .15:
        return None
    return dict(metres_per_unit=scale, views=len(ratios), validation_median_relative_error=float(np.median(err)))


def gravity_alignment(rotations, quaternions, frames=None):
    """Fit visual-world to inertial-world rotation using world-frame turns.

    The fixed camera-to-IMU mounting rotation cancels in R_j @ R_i.T.
    Require independent turn axes and agreement on held-out intervals.
    """
    import cv2

    if len(rotations) < 12:
        return None
    body = []
    for w, x, y, z in quaternions:
        v = np.array([x,y,z])
        skew = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        body.append((w*w-v@v)*np.eye(3) + 2*np.outer(v,v) + 2*w*skew)
    visual, inertial = [], []
    for j in range(3,len(body),3):
        i = j-3
        if frames is not None and frames[j]-frames[i] > 25:
            continue
        a = cv2.Rodrigues(rotations[j] @ rotations[i].T)[0].ravel()
        b = cv2.Rodrigues(body[j] @ body[i].T)[0].ravel()
        if .07 < np.linalg.norm(a) < 2.8 and abs(np.linalg.norm(a)-np.linalg.norm(b)) < .08:
            visual.append(a)
            inertial.append(b)
    if len(visual) < 9:
        return None
    a, b = np.asarray(visual), np.asarray(inertial)
    holdout = np.arange(len(a)) % 3 == 0
    def fit(mask):
        u, singular, vt = np.linalg.svd(b[mask].T @ a[mask])
        if singular[1] < .03 or singular[1] < .015*singular[0]:
            return None
        return u @ np.diag([1,1,np.linalg.det(u@vt)]) @ vt

    alignment = fit(~holdout)
    cross_check = fit(holdout)
    if alignment is None or cross_check is None:
        return None
    # The up direction must agree independently, including on walks that
    # mostly turn around vertical and offer less excitation in the other axes.
    if alignment[2] @ cross_check[2] < np.cos(np.deg2rad(8.0)):
        return None
    errors = np.linalg.norm(a[holdout] @ alignment.T - b[holdout],axis=1)
    if np.percentile(errors,80) > .08:
        return None
    return alignment


def body_up(q):
    """Up in the IMU body frame from a body-to-world quaternion (w, x, y, z) with world Z up."""
    w, x, y, z = q
    return np.array([2*(x*z-w*y), 2*(y*z+w*x), w*w-x*x-y*y+z*z])


def _sphere(n):
    i = np.arange(n)+.5
    polar, azimuth = np.arccos(1-2*i/n), np.pi*(1+5**.5)*i
    return np.c_[np.cos(azimuth)*np.sin(polar), np.sin(azimuth)*np.sin(polar), np.cos(polar)]


def _rings(up, step=.5, widest=30, around=24):
    """Directions on rings at step-degree intervals around up."""
    a = np.cross(up, [1, 0, 0] if abs(up[0]) < .9 else [0, 1, 0])
    a /= np.linalg.norm(a)
    b = np.cross(up, a)
    tilt = np.radians(np.arange(step, widest+step, step))[:, None]
    turn = np.linspace(0, 2*np.pi, around, endpoint=False)[None, :]
    side = np.cos(turn)[..., None]*a + np.sin(turn)[..., None]*b
    return (np.cos(tilt)[..., None]*up + np.sin(tilt)[..., None]*side).reshape(-1, 3)


def _kabsch(h):
    u, _, vt = np.linalg.svd(h)
    return u @ np.diag([1, 1, np.sign(np.linalg.det(u @ vt))]) @ vt


def _scores(rotations, ups, directions):
    h = np.einsum("abc,nc->nab", np.einsum("ia,icb->abc", ups, rotations), directions)
    u, s, vt = np.linalg.svd(h)
    return s[:, 0]+s[:, 1]+np.sign(np.linalg.det(u @ vt))*s[:, 2], h


def _fit_up(rotations, ups, grid):
    """Visual up u and camera-to-body mount M maximising sum(ups_i . M R_i^T u).

    For each candidate u the best mount is a Kabsch fit, so its score is
    linear algebra on H(u) = sum ups_i (R_i^T u)^T. A sphere search avoids
    local optima; alternating updates refine the best candidate.
    """
    scores, h = _scores(rotations, ups, grid)
    up = grid[scores.argmax()]
    mount = _kabsch(h[scores.argmax()])
    for _ in range(50):
        up = np.einsum("icb,ab,ia->c", rotations, mount, ups)
        up /= np.linalg.norm(up)
        mount = _kabsch(np.einsum("ia,icb,c->ab", ups, rotations, up))
    return up, mount, scores


def _errors_deg(rotations, ups, up, mount):
    predicted = np.einsum("ab,icb,c->ia", mount, rotations, up)
    return np.degrees(np.arccos(np.clip(np.sum(predicted*ups, axis=1), -1, 1)))


def up_alignment(rotations, ups, max_uncertainty=10):
    """Up direction of the visual world from IMU up vectors.

    rotations are world_from_camera for views with an up vector measured in
    the IMU body frame (attitude tilt or near-1 g specific force). The
    camera-to-IMU mount is unknown and fitted per walk, so a remounted board
    needs no setting. Unlike gravity_alignment this needs no turns about a
    second axis: handheld walks turn mostly about the vertical and tilt a
    little, and both constrain up. Reject fits that withheld views
    contradict, or whose up direction the data does not pin down.
    """
    rotations, ups = np.asarray(rotations, float), np.asarray(ups, float)
    if len(rotations) < 12:
        return None
    grid = _sphere(6000)
    up, mount, scores = _fit_up(rotations, ups, grid)
    errors = _errors_deg(rotations, ups, up, mount)
    best = float(np.sum(np.einsum("ab,icb,c->ia", mount, rotations, up)*ups))
    # Chi-square style: how far up can move before the fit is clearly worse.
    # The coarse sphere catches a distant second solution, the rings the local spread.
    noise = max(np.radians(1), np.sqrt(np.mean(np.radians(errors)**2)))**2
    directions = np.r_[grid, _rings(up)]
    scores = np.r_[scores, _scores(rotations, ups, directions[len(grid):])[0]]
    plausible = 2*(best-scores)/noise < 9
    angles = np.degrees(np.arccos(np.clip(directions @ up, -1, 1)))
    uncertainty = float(max(.5, angles[plausible].max() if plausible.any() else 0))
    test = np.arange(len(rotations)) % 3 == 0
    train_up, train_mount, _ = _fit_up(rotations[~test], ups[~test], grid)
    holdout = float(np.percentile(_errors_deg(rotations[test], ups[test], train_up, train_mount), 80))
    if uncertainty > max_uncertainty or holdout > 6:
        return None
    camera_up = mount.T @ ups.mean(axis=0)
    camera_up /= np.linalg.norm(camera_up)
    return dict(up=up, views=len(rotations), uncertainty_deg=round(uncertainty, 1),
                holdout_p80_deg=round(holdout, 2), fit_median_deg=round(float(np.median(errors)), 2),
                # Camera axes: x right, y down, z forward. Positive pitch looks down.
                camera_pitch_down_deg=round(float(np.degrees(np.arcsin(-camera_up[2]))), 1))


def levelling_rotation(up):
    """Smallest rotation taking the unit vector up onto +Z, so the heading barely changes."""
    import cv2

    up = np.asarray(up, float)/np.linalg.norm(up)
    axis = np.cross(up, [0, 0, 1])
    sine = np.linalg.norm(axis)
    if sine < 1e-9:
        return np.eye(3) if up[2] > 0 else np.diag([1., -1., -1.])
    return cv2.Rodrigues(axis/sine*np.arctan2(sine, up[2]))[0]
