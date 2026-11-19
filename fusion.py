"""Associate Pi-clock measurements and estimate an optional single-beam scale."""
import csv
from pathlib import Path

import numpy as np


STREAMS = ("camera", "tof", "gyro", "accel", "barometer")


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


def load_observations(run, streams=STREAMS):
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
            raw = int(row["tof_raw"])
            if 0 < raw < 250:
                obs["range"] = raw / 104.0  # wall checks: 52 counts at 0.50 m and 104 at 1.00 m
        if not times.size or not row.get("pi_encoder_callback_ns"):
            continue
        t = float(row["pi_encoder_callback_ns"]) * 1e-9
        j = int(times.searchsorted(t))
        k = min((i for i in (j-1, j) if 0 <= i < len(times)), key=lambda i: abs(times[i]-t))
        if abs(times[k]-t) > .08:
            continue
        r = motion[k]
        if "gyro" in streams and "accel" in streams:
            q = np.array([float(r[v]) for v in ("qw","qx","qy","qz")])
            if abs(np.linalg.norm(q)-1) < .02:
                obs.update(q=q / np.linalg.norm(q), q_segment=0)
        elif gyro is not None and gyro[k] is not None:
            obs["q"], obs["q_segment"] = gyro[k]
        elif "accel" in streams:
            a = np.array([float(r[v]) for v in ("ax_m_s2","ay_m_s2","az_m_s2")])
            norm = np.linalg.norm(a)
            if abs(norm-9.81) < .5:
                obs["accel_direction"] = a / norm
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


def estimate_scale(model, images, observations):
    # The TOF is mounted just below the lens, approximately parallel to its
    # optical axis. Restrict association to compact, depth-consistent patches.
    ratios = []
    for im in images:
        r = observations.get(im.name, {}).get("range")
        if r is None or r < .25:
            continue
        camera = im.camera
        centre = np.array([camera.principal_point_x, camera.principal_point_y])
        depths = []
        transform = im.cam_from_world()
        for pt in im.points2D:
            if pt.has_point3D() and np.linalg.norm(pt.xy-centre) < 32:
                z = (transform * model.point3D(pt.point3D_id).xyz)[2]
                if z > 0:
                    depths.append(z)
        if len(depths) < 5:
            continue
        z = np.median(depths)
        if np.percentile(depths, 90)-np.percentile(depths, 10) < .12*z:
            ratios.append((r/z,r,z))
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


def gravity_alignment(rotations, quaternions):
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
        a = cv2.Rodrigues(rotations[j] @ rotations[i].T)[0].ravel()
        b = cv2.Rodrigues(body[j] @ body[i].T)[0].ravel()
        if .07 < np.linalg.norm(a) < 2.8 and abs(np.linalg.norm(a)-np.linalg.norm(b)) < .10:
            visual.append(a)
            inertial.append(b)
    if len(visual) < 9:
        return None
    a, b = np.asarray(visual), np.asarray(inertial)
    holdout = np.arange(len(a)) % 3 == 0
    u, singular, vt = np.linalg.svd(b[~holdout].T @ a[~holdout])
    if singular[1] < .03 or singular[1] < .05*singular[0]:
        return None
    alignment = u @ np.diag([1,1,np.linalg.det(u@vt)]) @ vt
    errors = np.linalg.norm(a[holdout] @ alignment.T - b[holdout],axis=1)
    if np.percentile(errors,80) > .08:
        return None
    return alignment
