"""Local Mac capture and reconstruction dashboard."""
import argparse
import base64
from collections import deque
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import threading
import time
import uuid
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, quote, urlsplit
from zipfile import ZipFile
import re

from replay import read_timeline
from fusion import STREAMS, load_observations
from seed_surfaces import MAX_EDGE, build_surfaces
import gaussian_map
import ai_surfaces
import depth_assets
from cloud_planes import METHOD as PLANE_METHOD, build_planes
from cloth_surfaces import METHOD as CLOTH_METHOD, build_cloth


import numpy as np
import pycolmap as sfm
from localize import Locator
from calibration import corners, solve
from reconstruct import atomic_json, build, recover_gaps

SURFACE_BUILDERS = {"camera-ray-seeds-v1": build_surfaces, CLOTH_METHOD: build_cloth, PLANE_METHOD: build_planes, ai_surfaces.METHOD: ai_surfaces.build_ai}
HERE = Path(__file__).resolve().parent
HEADER = struct.Struct(">4sQI")


MOTION = struct.Struct("<d17f")
RANGE = struct.Struct(">QQi")
SENSOR_BATCH = struct.Struct(">QHH")


def surface_mesh(run, model, method, distance):
    if method == ai_surfaces.METHOD:
        return ai_surfaces.build_ai(run, model, distance)
    if method == PLANE_METHOD:
        observations = load_observations(run, model.get("streams", STREAMS))
        return build_planes(model, distance, observations)
    return SURFACE_BUILDERS.get(method, build_surfaces)(model, distance)


class Survey:
    def __init__(self, root, pi):
        self.root, self.pi = root, pi
        root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.process = self.latest = self.run = self.log = None
        self.tof = None
        self.motion_log = self.range_log = None
        self.range_stats = deque()
        self.motion_arrival = 0
        self.connection = "Disconnected"
        self.message = "Ready"
        self.count = 0
        self.building = None
        self.build_queue = deque()
        self.keys = []
        self.locator_key = self.locator = None
        self.locator_lock = threading.Lock()
        self.samples = []
        self.stamp = self.arrival = self.rate = 0
        self.refs = {}
        self.version = 0
        self.next_build = 12
        self.calib = root / "ov5647-640x480.json"

    @property
    def busy(self):
        return self.building is not None

    def connect(self):
        if self.process:
            return
        self.connection = "Connecting"
        self.range_stats.clear()
        self.tof = None
        err = (self.root / "camera.log").open("wb")
        try:
            p = subprocess.Popen(["ssh", "-T", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6",
                                  "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2",
                                  self.pi, "python3 -u -"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=err)
        finally:
            err.close()
        self.process = p
        p.stdin.write((HERE / "pi_camera.py").read_bytes())
        p.stdin.close()
        threading.Thread(target=self.capture, args=(p,), daemon=True).start()

    def capture(self, p):
        reason = "Camera connection ended"
        try:
            while header := p.stdout.read(HEADER.size):
                magic, stamp, size = HEADER.unpack(header)
                if magic not in (b"ROOM", b"RNG1", b"IMU1", b"IMU2") or not 0 < size < 2_000_000:
                    raise ValueError("Invalid camera packet")
                ranging, motion = [], []
                tof = None
                if magic == b"IMU2":
                    callback_ns, motion_count, range_count = SENSOR_BATCH.unpack(p.stdout.read(SENSOR_BATCH.size))
                    if max(motion_count, range_count) > 1024:
                        raise ValueError("Invalid sensor packet")
                    motion = list(MOTION.iter_unpack(p.stdout.read(motion_count * MOTION.size)))
                    ranging = list(RANGE.iter_unpack(p.stdout.read(range_count * RANGE.size)))
                    prior = next((row for row in reversed(ranging) if row[1] <= callback_ns), None)
                    if prior is None and self.tof and self.tof[1]:
                        prior = (0, self.tof[1], self.tof[2])
                    if prior and 0 <= callback_ns-prior[1] <= 80_000_000:
                        tof = (callback_ns, prior[1], prior[2])
                    else:
                        tof = (callback_ns, "", -1)
                elif magic != b"ROOM":
                    tof = RANGE.unpack(p.stdout.read(RANGE.size))
                    if magic == b"IMU1":
                        count, = struct.unpack(">H", p.stdout.read(2))
                        if count > 1024:
                            raise ValueError("Invalid sensor packet")
                        motion = list(MOTION.iter_unpack(p.stdout.read(count * MOTION.size)))
                data = p.stdout.read(size)
                if len(data) != size or not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
                    raise ValueError("Incomplete JPEG")
                with self.lock:
                    if self.process is not p:
                        break
                    if self.stamp and stamp > self.stamp:
                        self.rate = 1e6 / (stamp - self.stamp)
                    self.latest, self.stamp, self.arrival = data, stamp, time.monotonic()
                    self.tof = tof
                    if ranging:
                        self.range_stats.extend((end, 0 < raw < 250) for _, end, raw in ranging)
                        cutoff = ranging[-1][1] - 2_000_000_000
                        while self.range_stats and self.range_stats[0][0] < cutoff:
                            self.range_stats.popleft()
                    if motion:
                        self.motion_arrival = self.arrival
                    self.connection = "Live"
                    if self.log:
                        if self.motion_log:
                            for row in motion:
                                self.motion_log.write(",".join(map(str, row)) + "\n")
                            self.motion_log.flush()
                        if self.range_log and ranging:
                            self.range_log.writelines(f"{start},{end},{raw if raw >= 0 else ''}\n"
                                                     for start,end,raw in ranging)
                            self.range_log.flush()
                        self.save_frame(data, stamp)
        except Exception as exc:
            reason = str(exc)
        finally:
            p.stdout.close()
            p.terminate()
            p.wait()
            with self.lock:
                if self.process is p:
                    self.process = self.latest = None
                    self.connection = reason + " · see camera.log"
                    if self.log:
                        self.stop()

    def save_frame(self, data, stamp):
        name = f"{self.count:07}.jpg"
        (self.run / "frames" / name).write_bytes(data)
        fields = list(self.tof) if self.tof else ["","",""]
        if fields[-1] == -1:
            fields[-1] = ""
        self.log.write(",".join(map(str,[name,stamp,*fields]))+"\n")
        self.log.flush()
        self.count += 1
        if not self.keys or stamp - self.last_key >= 500000:
            self.keys.append(name)
            self.last_key = stamp
            if len(self.keys) >= self.next_build and not self.busy:
                self.next_build = len(self.keys) + max(16, len(self.keys)//2)
                self.schedule()

    def start(self):
        if self.log:
            raise ValueError("Finish the current recording first.")
        if not self.latest or time.monotonic()-self.arrival > 3:
            raise ValueError("Wait for a fresh camera frame.")
        folder = datetime.now().strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:5]
        self.run = self.root / folder
        (self.run / "frames").mkdir(parents=True)
        if self.calib.exists():
            shutil.copyfile(self.calib, self.run / "calibration.json")
        self.log = (self.run / "timestamps.csv").open("w")
        self.log.write("frame,pi_camera_relative_us,pi_encoder_callback_ns,pi_tof_read_end_ns,tof_raw\n")
        self.motion_log = (self.run / "motion.csv").open("w")
        self.motion_log.write("pi_uart_monotonic_s,fc_time_mod_s,fc_state,flags,qw,qx,qy,qz,ax_m_s2,ay_m_s2,az_m_s2,gx_rad_s,gy_rad_s,gz_rad_s,battery_v,height_m,vz_m_s,height_valid\n")
        self.range_log = (self.run / "range.csv").open("w")
        self.range_log.write("pi_read_start_ns,pi_read_end_ns,tof_raw\n")
        self.keys, self.refs, self.count, self.next_build = [], {}, 0, 12
        self.message = "Collecting overlapping views"
        self.version += 1

    def mark(self, label, distance=None):
        if not self.log or not self.count or time.monotonic()-self.arrival > 2:
            raise ValueError("A reference requires an active recording with fresh frames.")
        if label == "B":
            if "A" not in self.refs or not distance or not math.isfinite(distance) or distance <= 0:
                raise ValueError("Mark A first and enter a positive measured distance in metres.")
            if self.refs["A"] == f"{self.count-1:07}.jpg":
                raise ValueError("Move to the second measured position before marking B.")
            self.refs["metres"] = distance
        else:
            self.refs = {}
        self.refs[label] = f"{self.count-1:07}.jpg"
        atomic_json(self.run / "reference.json", self.refs)

    def stop(self):
        if self.log:
            if self.motion_log:
                self.motion_log.close()
                self.motion_log = None
            if self.range_log:
                self.range_log.close()
                self.range_log = None
            self.log.close()
            self.log = None
            self.schedule()

    def schedule(self, streams=STREAMS):
        if len(self.keys) < 4:
            self.message = "Need at least four overlapping views from different positions."
            return
        if self.log:
            candidates, limit = self.keys, 160
        else:
            # Denser overlap for turns and doorways on the final reconstruction.
            frames = sorted(p.name for p in (self.run/"frames").glob("*.jpg"))
            candidates, limit = sorted(set(frames[::2] + frames[-1:])), 400
        selected = [candidates[i] for i in np.linspace(0,len(candidates)-1,min(len(candidates),limit),dtype=int)]
        names = sorted(set(selected + [self.refs[k] for k in ("A","B") if k in self.refs]))
        self.build_queue.append((self.run, names, self.refs.copy(), not self.log, streams))
        if self.busy:
            self.message = "Recording saved · reconstruction queued"
        else:
            self.start_next_build()

    def start_next_build(self):
        job = self.build_queue.popleft()
        self.building = job[0]
        threading.Thread(target=self.map, args=job, daemon=True).start()

    def map(self, run, names, refs, recover=False, streams=STREAMS):
        def report(message):
            with self.lock:
                if self.run == run:
                    self.message = message
        try:
            model = build(run, names, report, recover=recover, streams=streams)
            if recover:
                recover_gaps(run, model, report)
            positions = {p["frame"]:np.array(p["xyz"]) for p in model["poses"]}
            if all(refs.get(k) in positions for k in ("A","B")):
                baseline = np.linalg.norm(positions[refs["A"]] - positions[refs["B"]])
                if baseline > 1e-5:
                    scale = refs["metres"] / baseline
                    for p in model["poses"]:
                        p["xyz"] = (np.array(p["xyz"]) * scale).tolist()
                    for point in model["points"]:
                        point[:3] = (np.array(point[:3]) * scale).tolist()
                    model.update(units="m", reference=refs)
            path = run / "survey.json"
            if recover:
                report("Growing surfaces from observed landmarks")
                try:
                    previous = json.loads(path.read_bytes())
                except (FileNotFoundError, json.JSONDecodeError):
                    previous = {}
                mesh = previous.get("surfaces") or {}
                distance = mesh.get("growth_distance_m", mesh.get("max_edge_m", MAX_EDGE))
                model["surfaces"] = surface_mesh(run, model, mesh.get("method"), distance)
            model["run"] = run.name
            atomic_json(path, model)
            with self.lock:
                if self.run == run:
                    self.version += 1
            registered = model["registered_views"]
            recovered = len(model["poses"]) - registered
            report(f"Registered {registered} of {len(names)} sampled views · {recovered} additional poses recovered")
        except Exception as exc:
            report(str(exc))
        finally:
            self.finish_build()

    def finish_build(self):
        with self.lock:
            self.building = None
            if self.build_queue:
                self.start_next_build()

    def rebuild_surfaces(self, run, model, distance, method):
        try:
            mesh = surface_mesh(run, model, method, distance)
            model["surfaces"] = mesh
            atomic_json(run / "survey.json", model)
            with self.lock:
                if self.run == run:
                    self.version += 1
                    unit = "polygons" if method in (PLANE_METHOD, ai_surfaces.METHOD) else "triangles"
                    self.message = f"Surfaces rebuilt · {len(mesh['patches']):,} {unit}"
        except Exception as exc:
            with self.lock:
                if self.run == run:
                    self.message = f"Surface rebuild failed · {exc}"
        finally:
            self.finish_build()

    def train_gaussians(self, run, model, target):
        try:
            result = gaussian_map.build(run, model, target, os.getenv("ROOM_SURVEY_BRUSH"))
            message = f"Gaussian map ready · {result['test_views']} evaluation images"
        except Exception as exc:
            message = f"Gaussian mapping failed · {exc}"
        finally:
            with self.lock:
                if self.run == run:
                    self.message = message
                    self.version += 1
                self.finish_build()

    def train_lingbot(self, run, model, target):
        try:
            cache = Path.home() / "Library/Caches/RoomSurvey/lingbot"
            with tempfile.TemporaryDirectory(dir=run, prefix=".lingbot-") as tmp:
                stage = Path(tmp)
                gaussian_map.export_droid(run, model, stage/"rectified")
                proc = subprocess.run([str(cache/"env/bin/python"), str(HERE/"lingbot_apple.py"),
                    str(stage/"rectified"), str(cache/"lingbot-map.pt"), str(stage/"result"), "--stride", "8"],
                    env=dict(os.environ,PYTHONPATH=str(cache/"source"),PYTORCH_ENABLE_MPS_FALLBACK="1"),
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
                if proc.returncode:
                    raise ValueError(proc.stderr.strip().splitlines()[-1] if proc.stderr.strip() else "Inference exited early")
                (stage/"result").rename(target)
            message = "LingBot-Map reconstruction ready"
        except Exception as exc:
            message = f"LingBot-Map failed · {exc}"
        finally:
            with self.lock:
                if self.run == run:
                    self.message = message
                    self.version += 1
                self.finish_build()

    def runs(self):
        return sorted((p.name for p in self.root.iterdir()
                       if not p.name.startswith(".") and not p.is_symlink()
                       and (p/"timestamps.csv").is_file()), reverse=True)

    def finished_walk(self, name):
        if name not in self.runs():
            raise ValueError("Select a saved walk.")
        run = self.root / name
        if (run == self.run and self.log) or run == self.building or any(run == job[0] for job in self.build_queue):
            raise ValueError("Wait for this walk to finish.")
        return run

    def delete(self, name):
        run = self.finished_walk(name)
        trash = self.root / ".trash"
        trash.mkdir(exist_ok=True)
        run.rename(trash / f"{name}-{uuid.uuid4().hex[:8]}")
        if self.run == run:
            self.run = None
            self.count = 0
            self.keys = []
            self.refs = {}
            self.locator_key = None
        self.version += 1
        self.message = f"Moved {name} to survey trash"

    def load(self, name):
        if self.log or self.busy or name not in self.runs():
            raise ValueError("Select a saved walk after the current run finishes.")
        self.run = self.root / name
        self.keys = []
        self.count = 0
        last = -1000000
        for row in (self.run/"timestamps.csv").read_text().splitlines()[1:]:
            filename, stamp = row.split(",")[:2]
            if not (self.run/"frames"/filename).is_file():
                continue
            self.count += 1
            if int(stamp)-last >= 500000:
                self.keys.append(filename)
                last = int(stamp)
        ref = self.run/"reference.json"
        self.refs = json.loads(ref.read_text()) if ref.exists() else {}
        self.version += 1
        self.message = "Opened saved walk"

    def state(self):
        fresh = bool(self.latest and time.monotonic()-self.arrival < 3)
        elapsed = (self.range_stats[-1][0]-self.range_stats[0][0])*1e-9 if len(self.range_stats)>1 else 0
        return dict(connection=self.connection, fresh=fresh,
                    tofHz=round((len(self.range_stats)-1)/elapsed,1) if fresh and elapsed else None,
                    tofValidPercent=round(100*sum(ok for _,ok in self.range_stats)/len(self.range_stats))
                                    if fresh and self.range_stats else None,
                    motionFresh=bool(self.motion_arrival and time.monotonic()-self.motion_arrival < 1),
                    tofRaw=self.tof[-1] if self.tof and self.tof[-1] >= 0 else None,
                    fps=round(self.rate,1), recording=bool(self.log), run=self.run.name if self.run else None,
                    frames=self.count, keys=len(self.keys), busy=self.busy, message=self.message,
                    buildingWalks=([self.building.name] if self.building else []) +
                                  [job[0].name for job in self.build_queue],
                    version=self.version, refs=self.refs, samples=len(self.samples), calibrated=self.calib.exists(),
                    hasMap=bool(self.run and (self.run/"survey.json").exists()))

    def locate(self, body):
        with self.lock:
            run = self.run
            if not run or self.busy or self.log or not (run/"survey.json").exists():
                raise ValueError("Open a finished reference walk first.")
            if body.get("run") != run.name:
                raise ValueError("Reference walk changed.")
            version = self.version
            source = body.get("source")
            if source == "live":
                if not self.state()["fresh"]:
                    raise ValueError("Waiting for a fresh camera frame.")
                stamp, jpeg = self.stamp, self.latest
            elif source == "replay":
                frame = int(body["frame"])
                if not 0 <= frame < self.count:
                    raise ValueError("Frame outside this walk.")
                stamp = frame
                jpeg = (run/"frames"/f"{frame:07}.jpg").read_bytes()
            else:
                raise ValueError("Choose live camera or replay.")
        with self.locator_lock:
            key = (run,version)
            if self.locator_key != key:
                model = json.loads((run/"survey.json").read_text())
                if not model.get("feature_index"):
                    raise ValueError("Use Rebuild to index this older walk.")
                self.locator = Locator(run, model)
                self.locator_key = key
            result = self.locator.locate(jpeg,max(8,min(100,int(body.get("minimum",12)))))
        return dict(result,run=run.name,version=version,source=source,stamp=stamp,
                    image=base64.b64encode(jpeg).decode())

    def action(self, action, body):
        if action == "locate":
            return self.locate(body)
        if action == "range":
            address = int(body["address"])
            with self.lock:
                if address == 0x52 and self.state()["fresh"] and self.tof:
                    if self.tof[-1] < 0:
                        raise ValueError("TOF250 unavailable on the live stream")
                    return dict(raw=self.tof[-1], pi_read_ns=self.tof[1])
            if address not in (0x29,0x52,0x64):
                raise ValueError("Unsupported TOF250 address")
            try:
                proc = subprocess.run(["ssh","-T","-o","BatchMode=yes","-o","ConnectTimeout=4",
                                      self.pi,f"python3 - {address}"], input=(HERE/"tof_probe.py").read_bytes(),
                                      capture_output=True,timeout=15)
            except subprocess.TimeoutExpired as e:
                raise ValueError("Range probe timed out") from e
            if proc.returncode:
                raise ValueError(proc.stderr.decode(errors="replace").strip().splitlines()[-1] if proc.stderr else "Range probe failed")
            return json.loads(proc.stdout)
        with self.lock:
            if action == "connect":
                self.connect()
            elif action == "disconnect":
                if self.log:
                    self.stop()
                if self.process:
                    self.process.terminate()
                    self.process = None
                self.connection, self.latest, self.stamp = "Disconnected", None, 0
            elif action == "start":
                self.start()
            elif action == "stop":
                self.stop()
            elif action in ("A","B"):
                self.mark(action,float(body.get("metres") or 0))
            elif action == "load":
                self.load(body["name"])
            elif action == "delete":
                self.delete(body["name"])
            elif action == "replay-baseline":
                if self.busy or self.log or not self.run or self.run.name != body.get("run"):
                    raise ValueError("Select a finished walk first.")
                model = json.loads((self.run/"survey.json").read_bytes())
                positions = {p["frame"]: np.asarray(p["xyz"]) for p in model["poses"]}
                a, b = body.get("A"), body.get("B")
                distance = float(body.get("metres") or 0)
                if a not in positions or b not in positions or np.linalg.norm(positions[a]-positions[b]) < 1e-5:
                    raise ValueError("Choose two distinct mapped camera positions.")
                if not math.isfinite(distance) or distance <= 0:
                    raise ValueError("Enter the measured camera separation in metres.")
                self.refs = {"A": a, "B": b, "metres": distance}
                atomic_json(self.run/"reference.json", self.refs)
                self.schedule(model.get("streams", STREAMS))
            elif action in ("build-gaussians","build-lingbot"):
                if self.busy or self.log or not self.run or self.run.name != body.get("run"):
                    raise ValueError("Choose an idle, finished walk.")
                model = json.loads((self.run/"survey.json").read_bytes())
                if body.get("index") != model.get("feature_index"):
                    raise ValueError("The camera map changed. Reopen the walk.")
                target = gaussian_map.artifact_dir(self.run, model.get("feature_index"))
                lingbot = action == "build-lingbot"
                if lingbot:
                    target = self.run / ("lingbot-v1-" + ai_surfaces.index_path(self.run,model["feature_index"]).stem)
                if not target.is_dir():
                    self.building = self.run
                    self.message = "Triangulating points and training Gaussian splats on this Mac"
                    if lingbot:
                        self.message = "Running LingBot-Map on Apple Metal"
                    threading.Thread(target=self.train_lingbot if lingbot else self.train_gaussians,args=(self.run,model,target),daemon=True).start()
            elif action == "rebuild-surfaces":
                if self.busy or self.log or not self.run or self.run.name != body.get("run"):
                    raise ValueError("Choose a finished walk.")
                try:
                    centimetres = float(body.get("growth_distance_cm", body.get("max_edge_cm")))
                except (TypeError, ValueError):
                    raise ValueError("Enter a positive growth distance in centimetres.") from None
                distance = centimetres / 100
                if not math.isfinite(distance) or distance <= 0:
                    raise ValueError("Enter a positive growth distance in centimetres.")
                model = json.loads((self.run / "survey.json").read_bytes())
                method = body.get("method", (model.get("surfaces") or {}).get("method", "camera-ray-seeds-v1"))
                if not isinstance(method, str) or method not in SURFACE_BUILDERS:
                    raise ValueError("Choose a supported surface method.")
                if model["units"] != "m" or (method != PLANE_METHOD and not model.get("landmark_views")):
                    raise ValueError("Surface growth needs metric scale and seed methods need landmark tracks.")
                if method == ai_surfaces.METHOD:
                    if body.get("index") != model.get("feature_index") or body.get("version") != self.version:
                        raise ValueError("The map changed. Reopen the walk and retry depth inference.")
                    if "predictions" in body:
                        ai_surfaces.save_predictions(self.run,model,body["predictions"])
                    elif not ai_surfaces.cached_frames(self.run,model["feature_index"]):
                        raise ValueError("Depth predictions unavailable. Retry browser inference.")
                self.building = self.run
                self.message = "Rebuilding surfaces from saved landmarks"
                threading.Thread(target=self.rebuild_surfaces, args=(self.run, model, distance, method),
                                 daemon=True).start()
            elif action == "rebuild":
                if not self.run or self.busy or self.log:
                    raise ValueError("Choose a finished walk.")
                requested = body.get("streams", list(STREAMS))
                if not isinstance(requested,list) or "camera" not in requested or any(k not in STREAMS for k in requested):
                    raise ValueError("Select Camera for a 3D reconstruction.")
                streams = tuple(k for k in STREAMS if k in requested)
                if self.calib.exists():
                    shutil.copyfile(self.calib, self.run / "calibration.json")
                self.schedule(streams)
            elif action == "sample":
                if self.log or not self.state()["fresh"]:
                    raise ValueError("Use the live preview before starting a walk.")
                pixels = corners(self.latest)
                if any(np.mean(np.linalg.norm(pixels-s,axis=2)) < 12 for s in self.samples):
                    raise ValueError("Change the board position or tilt before adding another view.")
                self.samples.append(pixels)
            elif action == "reset-samples":
                self.samples.clear()
            elif action == "calibrate":
                if self.log:
                    raise ValueError("Finish the recording before changing calibration.")
                calibration = solve(self.samples)
                atomic_json(self.calib,calibration)
                self.message = f"Calibration saved · {calibration['rms_px']:.2f} px RMS"
            else:
                raise ValueError("Unknown command")
            return self.state()


class Handler(BaseHTTPRequestHandler):
    def send(self, data, mime, code=200):
        self.send_response(code)
        self.send_header("Content-Type",mime)
        self.send_header("Content-Length",str(len(data)))
        self.send_header("Cache-Control","no-store")
        self.send_header("Cross-Origin-Resource-Policy", "same-origin")
        self.send_header("X-Frame-Options", "SAMEORIGIN" if urlsplit(self.path).path == "/lingbot-artifact" else "DENY")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def local_host(self):
        return self.headers.get("Host") in (f"127.0.0.1:{self.server.server_port}",
                                           f"localhost:{self.server.server_port}")

    def download_walk(self, name):
        with tempfile.TemporaryFile() as output:
            try:
                with survey.lock:
                    run = survey.finished_walk(name)
                    with ZipFile(output, "w") as archive:
                        for folder, _, files in os.walk(run):
                            for filename in sorted(files):
                                path = Path(folder) / filename
                                if not path.is_symlink() and path.is_file():
                                    archive.write(path, path.relative_to(run.parent).as_posix())
            except (ValueError, OSError) as exc:
                return self.send(str(exc).encode(), "text/plain; charset=utf-8", 400)
            size = output.tell()
            output.seek(0)
            self.send_response(200)
            self.send_header("Content-Type", "application/zip")
            self.send_header("Content-Length", str(size))
            self.send_header("Content-Disposition", "attachment; filename*=UTF-8''" + quote(name + ".zip", safe=""))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                shutil.copyfileobj(output, self.wfile)
            except (BrokenPipeError, ConnectionResetError):
                pass

    def do_GET(self):
        if not self.local_host():
            return self.send(b"Local dashboard required", "text/plain", 403)
        url = urlsplit(self.path)
        path = url.path
        files = {"/":"index.html","/ui.js":"ui.js","/gaussian-view.js":"gaussian-view.js","/depth-worker.js":"depth-worker.js","/ui.css":"ui.css","/target":"target.html"}
        if path in files:
            p = HERE / files[path]
            return self.send(p.read_bytes(), {".html":"text/html; charset=utf-8",".css":"text/css",".js":"text/javascript"}[p.suffix])
        if path in ("/gaussian-artifact","/lingbot-artifact"):
            query = parse_qs(url.query)
            with survey.lock:
                run = survey.run
                if not run or survey.log or query.get("run") != [run.name]:
                    return self.send(b"Select a finished walk","text/plain",404)
            name = query.get("file",[""])[0]
            allowed = name in ("report.json","cloud.ply","gaussians.ply") or re.fullmatch(
                r"(reference/[0-9]{7}\.jpg|renders/[0-9]{7}\.png)",name)
            if path == "/lingbot-artifact":
                if name in ("view.html","cloud.ply","trajectory.json","predictions.npz"):
                    try:
                        index = ai_surfaces.index_path(run,query.get("index",[""])[0]).stem
                        file = run/("lingbot-v1-"+index)/name
                        return self.send(file.read_bytes(),{".html":"text/html",".json":"application/json"}.get(file.suffix,"application/octet-stream"))
                    except (ValueError,OSError):
                        pass
                return self.send(b"LingBot-Map result unavailable","text/plain",404)
            if allowed:
                try:
                    root = gaussian_map.artifact_dir(run,query.get("index",[""])[0])
                    file = root/name
                    mime = {".ply":"application/octet-stream",".json":"application/json",
                            ".jpg":"image/jpeg",".png":"image/png"}[file.suffix]
                    return self.send(file.read_bytes(),mime)
                except (ValueError,OSError):
                    pass
            return self.send(b"Gaussian map unavailable","text/plain",404)
        if path.startswith("/depth-assets/"):
            name = path.removeprefix("/depth-assets/")
            if name not in depth_assets.FILES:
                return self.send(b"Unknown asset","text/plain",404)
            try:
                asset = depth_assets.asset(name)
                mime = {".mjs":"text/javascript",".wasm":"application/wasm",".onnx":"application/octet-stream"}
                return self.send(asset.read_bytes(), mime[asset.suffix])
            except (OSError,ValueError,subprocess.SubprocessError) as exc:
                return self.send(str(exc).encode(),"text/plain",503)
        if path in ("/depth-cache","/depth-input.jpg","/depth-preview.png"):
            query = parse_qs(url.query)
            with survey.lock:
                run = survey.run
                if not run or run.name != query.get("run",[""])[0] or survey.log:
                    return self.send(b"Select a finished walk","text/plain",404)
            frame = query.get("frame",[""])[0]
            index = query.get("index",[""])[0]
            if path == "/depth-cache":
                try:
                    return self.send(json.dumps(ai_surfaces.cached_frames(run,index)).encode(),"application/json")
                except (ValueError,OSError):
                    return self.send(b"Invalid camera map","text/plain",400)
            if re.fullmatch(r"[0-9]{7}\.jpg",frame):
                try:
                    if path == "/depth-input.jpg":
                        return self.send(ai_surfaces.rectified_jpeg(run,index,frame),"image/jpeg")
                    return self.send(ai_surfaces.preview_png(run,index,frame),"image/png")
                except (ValueError,OSError,KeyError):
                    pass
            return self.send(b"Depth view unavailable","text/plain",404)
        if path == "/walk.zip":
            return self.download_walk(parse_qs(url.query).get("run", [""])[0])
        if path == "/timeline" or path == "/recorded.jpg":
            query = parse_qs(url.query)
            with survey.lock:
                run = survey.run
                if not run or run.name != query.get("run", [""])[0] or survey.log:
                    return self.send(b"Select a finished walk", "text/plain", 404)
            try:
                if path == "/timeline":
                    return self.send(json.dumps(read_timeline(run), allow_nan=False).encode(), "application/json")
                frame = query.get("frame", [""])[0]
                if re.fullmatch(r"[0-9]{7}\.jpg", frame):
                    return self.send((run / "frames" / frame).read_bytes(), "image/jpeg")
            except OSError:
                pass
            return self.send(b"Recording data unavailable", "text/plain", 404)
        data, mime, code = b"Unavailable", "text/plain", 404
        with survey.lock:
            if path == "/state":
                data, mime, code = json.dumps(survey.state()).encode(), "application/json", 200
            elif path == "/runs":
                data, mime, code = json.dumps(survey.runs()).encode(), "application/json", 200
            elif path == "/preview.jpg" and survey.latest:
                data, mime, code = survey.latest, "image/jpeg", 200
            elif path == "/survey.json" and survey.run and (survey.run/"survey.json").exists():
                model = json.loads((survey.run/"survey.json").read_bytes())
                model.pop("submaps", None)
                model.pop("components", None)
                data, mime, code = json.dumps(model).encode(), "application/json", 200
        self.send(data, mime, code)

    def do_POST(self):
        try:
            if not self.local_host():
                raise ValueError("Local dashboard required")
            if self.headers.get("Content-Type") != "application/json":
                raise ValueError("Expected JSON")
            origin = self.headers.get("Origin")
            if origin and origin != "http://" + self.headers["Host"]:
                raise ValueError("Local dashboard required")
            size = int(self.headers.get("Content-Length","0"))
            limit = 360_000 if self.path == "/rebuild-surfaces" else 2048
            if not 0 < size <= limit:
                raise ValueError("Invalid command")
            result = survey.action(self.path.removeprefix("/"),json.loads(self.rfile.read(size)))
            self.send(json.dumps(result).encode(),"application/json")
        except (ValueError, OSError, KeyError) as exc:
            self.send(json.dumps({"error":str(exc)}).encode(),"application/json",400)

    def log_message(self, *_):
        pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--pi", default="pi@192.168.2.127")
    ap.add_argument("--port", type=int, default=8779)
    ap.add_argument("--output", type=Path, default=HERE/"captures"/"room-surveys")
    config = ap.parse_args()
    sfm.logging.minloglevel = 2
    survey = Survey(config.output,config.pi)
    server = ThreadingHTTPServer(("127.0.0.1",config.port),Handler)
    print(f"Room survey at http://127.0.0.1:{config.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        if survey.process:
            survey.process.terminate()
        server.server_close()
