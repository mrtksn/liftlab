"""Live position fixes for the drone's navigation (docs/map-navigation.md): each picture from the drone's downward camera
located in a walk's map (localize.Locator), turned into where the drone's hub is in the map and how its heading there
relates to its flight core's, and sent as `mapfix T X Y Z PSI SIGMA` to dfb_pi's command port.

- Where: the camera's centre in the map (the camera sits at the hub, near enough: a few centimetres).
- Which way it points: the camera is turned on the drone (flowcal measures it: the design's flow camera mount, +90°
  on the toy drone), so the nose is a fixed direction in the camera's picture. In the camera's axes as the map's poses
  have them (x right, y down, looking along z), the body's nose is (cos θ, sin θ, 0) for a camera turned θ about its
  axis (the design's mount: x right, y up, looking along −z). Its direction in the map, flattened, is the map heading.
- The flight core's heading at the same moment: its attitude (the LTEL quaternion in the flight telemetry samples,
  stamped on the Pi's clock as the pictures are). PSI, the turn from the map's axes to the navigation's: the flight
  core's heading less the map's.
- T: the picture's time on the Pi's clock (dfb_pi takes a fix's time on its own clock).
The walk's map must be in metres (a ToF scale: survey.json units "m").
"""
import math
import socket

import numpy as np

from localize import Locator


def yaw_of(q):
    """Heading [rad] of a body→world quaternion (w, x, y, z): where its x axis points, flattened."""
    w, x, y, z = q
    return math.atan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))


def nose_in_camera(turn_deg):
    """The body's nose in the map's camera axes (x right, y down, along z) for a camera turned turn_deg on the drone."""
    t = math.radians(turn_deg)
    return np.array([math.cos(t), math.sin(t), 0.0])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Attitudes:
    """The flight core's attitude samples, by the Pi's clock: (seconds, (w, x, y, z))."""
    def __init__(self, keep=4000):
        self.t, self.q, self.keep = [], [], keep

    def add(self, t, q):
        self.t.append(t); self.q.append(q)
        if len(self.t) > self.keep:
            del self.t[:len(self.t) - self.keep], self.q[:len(self.q) - self.keep]

    def at(self, t, within=0.1):
        if not self.t:
            return None
        i = int(np.searchsorted(self.t, t))
        best = min((j for j in (i - 1, i) if 0 <= j < len(self.t)), key=lambda j: abs(self.t[j] - t))
        return self.q[best] if abs(self.t[best] - t) <= within else None


class Fixer:
    def __init__(self, run, model, turn_deg=90.0, minimum=12):
        if model.get("units") != "m":
            raise ValueError("The walk's map isn't in metres (no ToF scale): fixes need metres.")
        self.locator, self.nose, self.minimum = Locator(run, model), nose_in_camera(turn_deg), minimum
        self.n = self.located = 0

    def fix(self, jpeg, t_s, attitudes):
        """A fix for one picture taken at t_s (the Pi's clock), or why not: dict(line?, xyz, heading, psi, inliers, status)."""
        self.n += 1
        r = self.locator.locate(jpeg, self.minimum)
        if r.get("status") != "Located":
            return dict(status=r.get("status", "Searching"), matches=r.get("matches", 0))
        R = np.asarray(r["rotation"])                    # camera → map
        nose = R @ self.nose
        if math.hypot(nose[0], nose[1]) < 0.3:
            return dict(status="Nose straight up or down", inliers=r["inliers"])
        heading = math.atan2(nose[1], nose[0])
        q = attitudes.at(t_s)
        if q is None:
            return dict(status="No flight telemetry at that moment", inliers=r["inliers"], xyz=r["xyz"], heading=heading)
        psi = wrap(yaw_of(q) - heading)
        x, y, z = r["xyz"]
        sigma = max(0.03, 0.6 / math.sqrt(r["inliers"]))   # (fewer landmarks, less sure)
        self.located += 1
        return dict(status="Located", xyz=[x, y, z], heading=heading, psi=psi, inliers=r["inliers"], sigma=sigma,
                    line=f"mapfix {t_s:.3f} {x:.3f} {y:.3f} {z:.3f} {psi:.4f} {sigma:.2f}")


class Sender:
    """mapfix lines to the drone's command port (UDP; dfb_pi doesn't answer a fix)."""
    def __init__(self, host, port=14560):
        self.addr, self.sock, self.sent = (host, port), socket.socket(socket.AF_INET, socket.SOCK_DGRAM), 0

    def send(self, line):
        self.sock.sendto(line.encode(), self.addr); self.sent += 1


def replay(run, model, turn_deg=90.0, every=1):
    """Each recorded picture of a walk located again in its own map, as the drone's would be in flight: for checking.
    Returns rows (frame, t, status, xyz, heading, psi, inliers) and the walk's own poses by frame."""
    import csv
    att = Attitudes(keep=10 ** 7)
    with (run / "motion.csv").open() as f:
        for row in csv.DictReader(f):
            att.add(float(row["pi_uart_monotonic_s"]), tuple(float(row[k]) for k in ("qw", "qx", "qy", "qz")))
    fixer, rows = Fixer(run, model, turn_deg), []
    with (run / "timestamps.csv").open() as f:
        frames = list(csv.DictReader(f))
    for row in frames[::every]:
        t = int(row["pi_encoder_callback_ns"]) / 1e9
        r = fixer.fix((run / "frames" / row["frame"]).read_bytes(), t, att)
        rows.append(dict(frame=row["frame"], t=t, **r))
    poses = {p["frame"]: p for p in model["poses"]}
    return rows, poses
