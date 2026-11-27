"""Processing steps the pipeline can run, described as data.

Each operator declares typed inputs and outputs and the parameters it accepts,
with the control the dashboard draws for each one. Choice options come from
the modules that implement them. The dashboard builds its node catalogue,
inspector and runner from this description and holds no method names.

A request names an operator, its parameter values and, for each input, either
a reference to the upstream operator ({"op", "params", "inputs"}) or data a
browser node produced ({"data"}). The response is either {"outputs": {...}},
{"pending": message} while a background job runs, or {"request": {...}} when
the operator needs the browser to compute something first.
"""
import hashlib
import json
import math
from pathlib import Path
import threading

import ai_surfaces
import cloth_surfaces
import cloud_planes
import depth_surfaces
import gaussian_map
import replay_depth
import seed_surfaces
from fusion import STREAMS, STREAM_LABELS
from node_params import CAMERA_MAP_PARAMS, SENSOR_PARAMS, defaults, validate
from replay import read_timeline

# Data types passed between nodes. `display` says how the dashboard draws or lists
# values of the type; `panel` names extra tools shown beside it.
TYPES = {
    "Recording tables": dict(display="tables",
        description="Object keyed by CSV filename. Each value is the complete array of rows in recording order. "
                        "sensors.json holds one row: the walk's sensor settings."),
    "Recorded photos": dict(display="frames",
        description="Array of {frame, ms}. await readFrame(ref) returns width, height and RGBA pixels."),
    "Telemetry": dict(display="table",
        description="Array of frame-paired sensor records, in recording order."),
    "Camera map": dict(display="map",
        description="{ poses, points, units }. Each point is [x, y, z, red, green, blue]."),
    "Camera path": dict(display="path",
        description="Array of poses {frame, xyz, rotation?} in map coordinates. A replay path's poses also say how "
                    "each was made: source mapped, interpolated, extrapolated or held (replay-path.js)."),
    "Depth frames": dict(display="frames",
        description="Array of {frame, ms, method}. await readFrame(ref) returns width, height and float depth pixels "
                    "in native model units."),
    "Registered depth points": dict(display="points",
        description="Array of [x, y, z, red, green, blue] rows in the map frame."),
    "Colored points": dict(display="points",
        description="Array of [x, y, z, red, green, blue] rows. RGB channels range from 0 to 255."),
    "Polygon mesh": dict(display="polygons", panel="mesh",
        description="Array of polygon records. Each has vertices [[x, y, z], …] and may have holes containing inner rings."),
    "Triangle list": dict(display="triangles",
        description="Array of triangles, each containing three [x, y, z] vertices in map coordinates."),
    "Point-fitted splats": dict(display="splats",
        description="Float32Array of 16 values per splat: xyz, opacity, RGB (0–1), padding, radii XYZ, padding, "
                           "quaternion WXYZ."),
    "Trained Gaussian map": dict(display="splats", panel="gaussian",
        description="Float32Array of 16 values per trained splat, in the same layout as point-fitted splats."),
    "LingBot trajectory and points": dict(display="report", panel="lingbot",
        description="Saved trajectory descriptors in LingBot coordinates."),
    "Clearance map": dict(display="report", panel="clearance",
        description="Clearance report: per-frame distance from the walked path to the nearest occupied cell."),
    "Navigation map": dict(display="report",
        description="The drone's map (map.dmp, runner/fc/map_core.h): {url, file, bytes, cells, cell_m, origin_m, "
                    "occupied, free, unknown, levelled, units}. Download it from url, or send it with Send to the drone."),
    "Pixel frames": dict(display="pixels",
        description="Array of {frame, width, height, format, pixels}. Use Float32Array for depth, Uint8ClampedArray "
                    "for rgba."),
    "JSON records": dict(display="table",
        description="Array of JSON objects."),
}
# Interchangeable shapes: a port that accepts the key also accepts these.
COMPATIBLE = {"Colored points": ["Registered depth points"], "Point-fitted splats": ["Trained Gaussian map"]}

SURFACE_MODULES = (seed_surfaces, cloth_surfaces, cloud_planes)
SURFACE_DEFAULTS = {cloud_planes.METHOD: {"min_support": cloud_planes.MIN_SUPPORT}}
SURFACE_BUILDERS = {seed_surfaces.METHOD: lambda model, distance, observations: seed_surfaces.build_surfaces(model, distance),
                    cloth_surfaces.METHOD: lambda model, distance, observations: cloth_surfaces.build_cloth(model, distance),
                    cloud_planes.METHOD: lambda model, distance, observations: cloud_planes.build_planes(model, distance, observations)}


def choice(key, label, options, default=None, **extra):
    options = [list(o) for o in options]
    return dict(key=key, label=label, kind="choice", options=options,
                default=default if default is not None else options[0][0], **extra)


def number(key, label, default, low, high, unit=None, kind="number", **extra):
    return dict(key=key, label=label, kind=kind, default=default, min=low, max=high,
                **({"unit": unit} if unit else {}), **extra)


def flag(key, label, default=False, **extra):
    return dict(key=key, label=label, kind="bool", default=default, **extra)


def port(name, *types, optional=False):
    return dict(name=name, types=list(types), **({"optional": True} if optional else {}))


class Pending(Exception):
    pass


class Context:
    def __init__(self, survey, operator, body):
        self.survey, self.operator = survey, operator
        self.run = survey.run
        if not self.run or survey.log or body.get("run") != self.run.name:
            raise ValueError("Open a finished walk.")
        self.params = validate(operator["params"], body.get("params"))
        self.inputs = body.get("inputs") or {}
        if not isinstance(self.inputs, dict):
            raise ValueError("Inputs must be an object.")
        self.body = body
        for p in operator["inputs"]:
            if not p.get("optional") and p["name"] not in self.inputs:
                raise ValueError(f"Connect {p['name']} ({' or '.join(p['types'])}).")
        path = self.run / "survey.json"
        self.model = json.loads(path.read_bytes()) if path.is_file() else None

    def ref(self, name):
        value = self.inputs.get(name)
        return value if isinstance(value, dict) and "op" in value else None

    def data(self, name):
        value = self.inputs.get(name)
        return value.get("data") if isinstance(value, dict) else None

    def map(self):
        if not self.model or not self.model.get("feature_index"):
            raise ValueError("Run the camera map first.")
        return self.model

    def key(self):
        return json.dumps([self.operator["id"], self.params, self.inputs], sort_keys=True, default=str)

    def start(self, message, target, *args):
        """Run a background job for this request, or report the result of the last one."""
        survey, key = self.survey, self.key()
        if self.body.get("peek"):
            raise Pending("Not built yet")
        with survey.lock:
            if survey.busy or survey.build_queue:
                raise Pending(survey.message)
            jobs = survey.operator_jobs
            if jobs.pop(key, None) == "started":
                # The job finished without producing this output.
                raise ValueError(survey.message or f"{self.operator['label']} failed.")
            jobs[key] = "started"
            survey.building = self.run
            survey.message = message
            threading.Thread(target=target, args=args, daemon=True).start()
        raise Pending(message)

    def schedule_map(self, streams, params):
        survey, key = self.survey, self.key()
        if self.body.get("peek"):
            raise Pending("Not built yet")
        with survey.lock:
            if survey.busy or survey.build_queue:
                raise Pending(survey.message)
            if survey.operator_jobs.pop(key, None) == "started":
                raise ValueError(survey.message or "Camera map failed.")
            survey.operator_jobs[key] = "started"
            survey.schedule(streams, params)
            if not survey.busy:
                survey.operator_jobs.pop(key, None)
                raise ValueError(survey.message)
        raise Pending(survey.message)


def variant(params, schema):
    """Short folder suffix for non-default settings; the default build keeps its original folder."""
    changed = {k: v for k, v in params.items() if v != defaults(schema)[k]}
    return hashlib.sha1(json.dumps(changed, sort_keys=True).encode()).hexdigest()[:8] if changed else ""


def artifact(path, run, index, file, **extra):
    from urllib.parse import urlencode
    return path + "?" + urlencode(dict(run=run.name, index=index, file=file, **extra))


def frame_refs(run, method=None):
    return [dict(frame=f["id"], ms=f["ms"], **({"method": method} if method else {})) for f in read_timeline(run)]


# Operators ---------------------------------------------------------------------------------------

def run_walk(ctx):
    query = f"?run={ctx.run.name}"
    return dict(tables=dict(url="/recording-tables" + query), photos=dict(data=frame_refs(ctx.run)),
                telemetry=dict(url="/timeline" + query))


STREAM_PARAMS = tuple(flag("stream_" + key, STREAM_LABELS[key], True, group="Input streams")
                      for key in STREAMS if key != "camera")
CAMERA_KEYS = tuple(p["key"] for p in CAMERA_MAP_PARAMS)


def run_camera_map(ctx):
    streams = ("camera",) + tuple(k for k in STREAMS if k != "camera" and ctx.params["stream_" + k])
    params = {k: ctx.params[k] for k in CAMERA_KEYS}
    model = ctx.model
    saved = {**defaults(CAMERA_MAP_PARAMS), **(model or {}).get("params", {})}
    if model and model.get("feature_index") and sorted(model.get("streams", STREAMS)) == sorted(streams) and saved == params:
        return dict(map=dict(index=model["feature_index"], url=f"/survey.json?run={ctx.run.name}"))
    ctx.schedule_map(streams, params)


DEPTH_MODELS = [(k, v) for k, v in replay_depth.LABELS.items() if k != "gaussian"]


def depth_frames(ctx, method):
    index = ctx.map()["feature_index"]
    status = replay_depth.inventory(ctx.run, index)[method]
    if status["complete"]:
        units = replay_depth.UNITS[method]
        return dict(depth=dict(data=frame_refs(ctx.run, method), index=index, method=method,
                               units=ctx.model.get("units", "map units") if units == "map" else units))
    if status["running"]:
        raise Pending(f"Generating {replay_depth.LABELS[method]} depth")
    if not status["available"]:
        raise ValueError(f"{replay_depth.LABELS[method]} depth needs its source map first.")
    ctx.start(f"Generating {replay_depth.LABELS[method]} depth for every saved frame",
              ctx.survey.build_frame_depth, ctx.run, index, [method])


def run_depth(ctx):
    return depth_frames(ctx, ctx.params["model"])


def run_gaussian_depth(ctx):
    splats = ctx.ref("splats")
    if splats and variant(splats.get("params") or {}, GAUSSIAN_PARAMS):
        raise ValueError("Rendered depth uses the Gaussian map built with default settings.")
    return depth_frames(ctx, "gaussian")


def depth_method(ref):
    if not ref:
        raise ValueError("Connect depth frames from a depth operator.")
    if ref["op"] == "gaussian-depth":
        return "gaussian"
    if ref["op"] == "depth":
        return validate(OPERATORS_BY_ID["depth"]["params"], ref.get("params"))["model"]
    raise ValueError("Backprojection reads saved depth frames: connect a depth operator directly.")


def run_backproject(ctx):
    import depth_cloud
    if ctx.body.get("peek"):
        raise Pending("Not built yet")
    method = depth_method(ctx.ref("depth"))
    index = ctx.map()["feature_index"]
    trained, path = ctx.params["trained"], ctx.params["regenerate_path"]
    if trained and method != "anything":
        raise ValueError("Depth correction training is available for Depth Anything V2.")
    cloud = depth_cloud.reconstruct(ctx.run, index, method, saved_only=not path, trained=trained)
    if path and not cloud.get("path"):
        cloud = depth_cloud.reconstruct(ctx.run, index, method, regenerate=True, trained=trained)
    outputs = dict(points=dict(data=cloud["points"], summary={k: v for k, v in cloud.items()
                                                              if k not in ("points", "path")}))
    if cloud.get("path"):
        outputs["path"] = dict(data=cloud["path"]["poses"])
    return outputs


def surface_mesh_output(model, method, distance, extra=None):
    mesh = model.get("surfaces") or {}
    built = mesh.get("growth_distance_m", mesh.get("max_edge_m"))
    if mesh.get("method") == method and built is not None and abs(built - distance) < 1e-9:
        # Meshes from before settings were recorded used the defaults.
        settings = {**SURFACE_DEFAULTS.get(method, {}), **(mesh.get("settings") or {})}
        if all(settings.get(k) == v for k, v in (extra or {}).items()):
            return dict(mesh=dict(data=mesh["patches"], summary={k: v for k, v in mesh.items() if k != "patches"}))
    return None


def run_landmark_surfaces(ctx):
    model, method, distance = ctx.map(), ctx.params["method"], ctx.params["distance_cm"]/100
    extra = {"min_support": ctx.params["min_support"]} if method == cloud_planes.METHOD else {}
    done = surface_mesh_output(model, method, distance, extra)
    if done:
        return done
    if model["units"] != "m" or (method != cloud_planes.METHOD and not model.get("landmark_views")):
        raise ValueError("Surface growth needs metric scale, and seed methods need landmark tracks.")
    ctx.start("Rebuilding surfaces from saved landmarks", ctx.survey.rebuild_surfaces, ctx.run, model, distance, method,
              {"min_support": ctx.params["min_support"]} if method == cloud_planes.METHOD else {})


def run_ai_planes(ctx):
    model, distance = ctx.map(), ctx.params["distance_cm"]/100
    done = surface_mesh_output(model, ai_surfaces.METHOD, distance)
    if done:
        return done
    if "predictions" in ctx.body:
        ai_surfaces.save_predictions(ctx.run, model, ctx.body["predictions"])
    elif not ai_surfaces.cached_frames(ctx.run, model["feature_index"]):
        candidates = [p["frame"] for p in model["poses"]
                      if p.get("rotation") and len((model.get("landmark_views") or {}).get(p["frame"], [])) >= 15]
        if len(candidates) < 2:
            raise ValueError("This walk needs two mapped camera views with landmark tracks.")
        count = min(ctx.params["views"], len(candidates))
        frames = [candidates[round(i*(len(candidates)-1)/max(1, count-1))] for i in range(count)]
        if ctx.body.get("peek"):
            raise Pending("Not built yet")
        return {"request": dict(kind="depth-predictions", frames=frames, index=model["feature_index"])}
    ctx.start("Fitting planes to depth predictions", ctx.survey.rebuild_surfaces, ctx.run, model, distance,
              ai_surfaces.METHOD)


GAUSSIAN_PARAMS = (number("steps", "Training steps", 1500, 50, 100000, kind="int"),
                   flag("dense_seed", "Seed from dense depth points", True,
                        help="Off: seed only from the sparse camera-map landmarks."))


def splat_root(ctx, ref):
    """Folder and variant of the upstream Gaussian map."""
    suffix = variant(validate(GAUSSIAN_PARAMS, (ref or {}).get("params")), GAUSSIAN_PARAMS)
    return gaussian_map.artifact_dir(ctx.run, ctx.map()["feature_index"], suffix), suffix


def run_gaussian_map(ctx):
    index = ctx.map()["feature_index"]
    suffix = variant(ctx.params, GAUSSIAN_PARAMS)
    root = gaussian_map.artifact_dir(ctx.run, index, suffix)
    if (root/"gaussians.ply").is_file():
        return dict(splats=dict(url=artifact("/gaussian-artifact", ctx.run, index, "gaussians.ply", variant=suffix),
                                format="splat-ply", index=index, variant=suffix))
    ctx.start(f"Training Gaussian splats for {ctx.params['steps']} steps on this Mac", ctx.survey.train_gaussians,
              ctx.run, ctx.model, root, ctx.params["steps"], ctx.params["dense_seed"])


def range_scale(report, units):
    """Metres per map unit that the mesh's ToF check measured, for a map without a scale of its own, else None.

    gaussian_mesh.range_evidence casts each paired ToF beam into the mesh and compares the hit with the reading;
    a consumer (Send to World) can size the surface from it and show how well it held on held-out readings.
    """
    if units == "m":
        return None
    try:
        tof = json.loads(Path(report).read_text()).get("tof") or {}
    except (OSError, ValueError):
        return None
    k = tof.get("candidate_metres_per_unit")
    if not isinstance(k, (int, float)) or not math.isfinite(k) or k <= 0:
        return None
    errors = {key: tof[key] for key in ("median_error_m", "p80_error_m", "hits", "eligible")
              if isinstance(tof.get(key), (int, float)) and math.isfinite(tof[key])}
    return dict(metres_per_unit=float(k), source="ToF readings against this mesh", **errors)


def run_gaussian_mesh(ctx):
    root, suffix = splat_root(ctx, ctx.ref("splats"))
    if not (root/"gaussians.ply").is_file():
        raise ValueError("Run the Gaussian map first.")
    import gaussian_mesh
    index = ctx.model["feature_index"]
    name = gaussian_mesh.folder(ctx.params)
    report = root/name/"report.json"
    if report.is_file():
        mesh = dict(url=artifact("/gaussian-artifact", ctx.run, index, name + "/mesh.ply", variant=suffix),
                    report=artifact("/gaussian-artifact", ctx.run, index, name + "/report.json", variant=suffix),
                    format="mesh-ply", index=index, variant=suffix, mesh=name)
        scale = range_scale(report, ctx.model.get("units"))
        if scale:
            mesh["scale"] = scale
        return dict(mesh=mesh)
    ctx.start("Opening trained Gaussian splats", ctx.survey.build_gaussian_geometry, ctx.run, ctx.model, root, False,
              ctx.params)


def run_clearance(ctx):
    import clearance
    root, suffix = splat_root(ctx, ctx.ref("splats"))
    if not (root/"gaussians.ply").is_file():
        raise ValueError("Run the Gaussian map first.")
    folder = clearance.artifact_dir(ctx.run, root, ctx.params)
    if (folder/"report.json").is_file():
        index = ctx.model["feature_index"]
        return dict(clearance=dict(url=artifact("/gaussian-artifact", ctx.run, index, "clearance-v1/report.json",
                                                variant=suffix, clearance=folder.name),
                                   index=index, variant=suffix, clearance=folder.name))
    ctx.start("Opening trained Gaussian splats", ctx.survey.build_gaussian_geometry, ctx.run, ctx.model, root, True,
              ctx.params)


import clearance as _clearance
import gaussian_mesh as _gaussian_mesh
CLEARANCE_PARAMS = _clearance.PARAMS
import nav_map as _nav_map
NAV_MAP_PARAMS = (*_nav_map.PARAMS,
                  choice("unknown_as", "Space the walk didn't see through",
                         [["unknown", "Unknown: the drone goes in slowly, half a metre at a time"],
                          ["free", "Free: trust that the walk saw the room's surfaces"]],
                         help="The clearance grid only calls a cell free where two views saw through it; a walk with "
                              "the camera at hand height leaves most of a room unknown."))


def _clearance_folder(ctx, ref):
    """The clearance grid's folder from its node (its settings and the Gaussian map it came from)."""
    if not ref or ref.get("op") != "clearance":
        raise ValueError("Connect the Walk clearance node.")
    root, suffix = splat_root(ctx, (ref.get("inputs") or {}).get("splats"))
    folder = _clearance.artifact_dir(ctx.run, root, validate(CLEARANCE_PARAMS, ref.get("params")))
    if not (folder/"grid.npz").is_file():
        raise ValueError("Run Walk clearance first.")
    return suffix, folder


def _nav_map_file(ctx, ref):
    """The map file a Navigation map node makes (its folder, the file, its settings)."""
    if not ref or ref.get("op") != "navigation-map":
        raise ValueError("Connect a Navigation map node.")
    params = validate(NAV_MAP_PARAMS, ref.get("params"))
    _, folder = _clearance_folder(ctx, (ref.get("inputs") or {}).get("clearance"))
    suffix = variant(params, NAV_MAP_PARAMS)
    return folder, folder/("navmap" + (f"-{suffix}" if suffix else "") + ".dmp"), params


def run_navigation_map(ctx):
    import numpy as np
    splat_variant, folder = _clearance_folder(ctx, ctx.ref("clearance"))
    suffix = variant(ctx.params, NAV_MAP_PARAMS)
    file = folder/("navmap" + (f"-{suffix}" if suffix else "") + ".dmp")
    g = np.load(folder/"grid.npz")
    grid = folder/"grid.npz"
    if ctx.params["unknown_as"] == "free":                # (a copy of the grid with the unknown called free)
        import tempfile
        cells = g["cells"].copy(); cells[cells == _nav_map.UNKNOWN] = _nav_map.FREE
        tmp = Path(tempfile.mkdtemp()) / "grid.npz"
        np.savez(tmp, cells=cells, origin_m=g["origin_m"], voxel_m=g["voxel_m"])
        grid = tmp
    m = _nav_map.from_clearance(grid, ctx.params)
    file.write_bytes(m["data"])
    report = json.loads((folder/"report.json").read_text()) if (folder/"report.json").is_file() else {}
    model = ctx.model or {}
    out = dict(file=file.name, bytes=len(m["data"]), cells=m["n"], cell_m=round(m["res"], 4), origin_m=[round(v, 3) for v in m["origin"]],
               occupied=m["occupied"], free=m["free"], unknown=m["unknown"],
               levelled=bool(model.get("gravity_aligned")), units=report.get("scale_source") or model.get("units"),
               url=artifact("/gaussian-artifact", ctx.run, model.get("feature_index", ""), "clearance-v1/" + file.name,
                            variant=splat_variant, clearance=folder.name))
    if not out["levelled"]:
        out["warning"] = "This walk's map isn't levelled from the IMU: its Z may not be straight up, and the drone's floor and ceiling would tilt."
    return dict(navmap=out)


def run_send_to_drone(ctx):
    if ctx.body.get("peek"):
        raise Pending("Not sent: run this node to send the map to the drone")
    folder, file, mp = _nav_map_file(ctx, ctx.ref("navmap"))
    if not file.is_file():
        raise ValueError("Run the Navigation map first.")
    data = file.read_bytes()
    anchor, heading = None, ctx.params["heading_deg"]
    if ctx.params["anchor"] == "start":
        anchor = (0.0, 0.0, 0.0, heading)                 # (the walk's map starts at its first camera)
    elif ctx.params["anchor"] == "floor":
        res, origin, cells = _nav_map.decode(data)
        floor = _nav_map.floor_below(cells, res, origin)
        if floor is None:
            raise ValueError("No floor found under where the walk started: say where the drone is another way.")
        anchor = (0.0, 0.0, floor + 0.05, heading)
    host = str(ctx.survey.pi).split("@")[-1]
    lines = _nav_map.lines(data, ctx.params["radius_m"], anchor)
    said = _nav_map.send(host, lines)
    return dict(answers=[dict(drone=host, said=a) for a in said] or [dict(drone=host, said="no answer (is dfb_pi running, on port 14560?)")])


SPLAT_FIT_PARAMS = (   # gaussian-view.js fitGaussianAttributes
    number("extent_trim_pct", "Ignore the outermost points when sizing cells", 2.0, 0.0, 49.0, unit="%", group="Cells",
           help="On each side of each axis."),
    number("cells_per_sqrt_point", "Cells across per square root of the point count", .5, .001, 100.0, group="Cells"),
    number("min_cells", "Fewest cells across", 8, 1, 10000, kind="int", group="Cells"),
    number("cell_samples", "Points sampled from each neighbouring cell", 32, 1, 10000, kind="int", group="Neighbours"),
    number("neighbours", "Nearest points in each fit", 12, 1, 1000, kind="int", group="Neighbours"),
    number("radius_neighbour", "Radius from the distance to the Nth nearest point", 3, 1, 1000, kind="int", group="Size"),
    number("radius_scale", "Radius scale", .78, .001, 100.0, group="Size"),
    number("min_radius", "Smallest radius", .04, 0.0, 100.0, unit="× cell", group="Size"),
    number("max_radius", "Largest radius", .7, .001, 100.0, unit="× cell", group="Size"),
    number("lone_radius", "Radius of a point with no neighbours", .15, 0.0, 100.0, unit="× cell", group="Size"),
    number("min_thickness", "Thinnest splat", .12, 0.0, 1.0, unit="× radius", group="Size",
           help="Flat neighbourhoods give thin splats, down to this."),
    number("opacity", "Opacity", .82, 0.0, 1.0, group="Look"),
)

REPLAY_PATH_PARAMS = (
    flag("fill_gaps", "Fill unmapped stretches with straight lines", True,
         help="Frames between two given poses get a position on the line between them, at their time."),
    flag("extend_ends", "Extend to the walk's first and last frames", True,
         help="Frames before the first and after the last given pose move on at the speed measured at that end."),
    number("end_frames", "Given poses that measure an end's speed", 4, 2, 1000, kind="int",
           help="Consecutive given poses at each end of the walk."),
    flag("blend_directions", "Blend camera directions between given poses", True,
         help="Frames without a saved direction turn smoothly between the nearest given poses."),
)

LINGBOT_PARAMS = (number("stride", "Use every Nth rectified frame", 8, 1, 100, kind="int"),)


def run_lingbot(ctx):
    index = ctx.map()["feature_index"]
    suffix = variant(ctx.params, LINGBOT_PARAMS)
    target = ctx.run/("lingbot-v1-" + ai_surfaces.index_path(ctx.run, index).stem + (f"-{suffix}" if suffix else ""))
    if (target/"trajectory.json").is_file():
        return dict(result=dict(url=artifact("/lingbot-artifact", ctx.run, index, "trajectory.json", variant=suffix),
                                view=artifact("/lingbot-artifact", ctx.run, index, "view.html", variant=suffix),
                                cloud=artifact("/lingbot-artifact", ctx.run, index, "cloud.ply", variant=suffix),
                                index=index, variant=suffix))
    ctx.start("Running LingBot-Map on Apple Metal", ctx.survey.train_lingbot, ctx.run, ctx.model, target,
              ctx.params["stride"])


def run_depth_surfaces(ctx):
    if ctx.body.get("peek"):
        raise Pending("Not built yet")
    model = ctx.map()
    p = ctx.params
    ref, points = ctx.ref("points"), ctx.data("points")
    if ref and ref["op"] == "backproject":
        settings = validate(OPERATORS_BY_ID["backproject"]["params"], ref.get("params"))
        mesh = depth_surfaces.build(ctx.run, model["feature_index"], depth_method((ref.get("inputs") or {}).get("depth")),
                                    p["method"], p["tof"], p["detail"], p["reach"],
                                    saved_only=not settings["regenerate_path"], trained=settings["trained"])
    elif isinstance(points, list):
        if not 3 <= len(points) <= 2_000_000 or any(not isinstance(r, list) or len(r) != 6 for r in points):
            raise ValueError("Supply at least 3 colored points.")
        cloud = dict(points=points, units=model.get("units", "relative"), method="input")
        mesh = depth_surfaces.fit_cloud(ctx.run, model, cloud, p["method"], p["tof"], p["detail"], p["reach"])
    else:
        raise ValueError("Connect points from Backproject depth or a code node.")
    return dict(mesh=dict(data=mesh["patches"], summary={k: v for k, v in mesh.items() if k != "patches"}))


OPERATORS = [
    dict(id="walk", label="Recorded walk", group="Sources", inputs=[],
         outputs=[port("tables", "Recording tables"), port("photos", "Recorded photos"), port("telemetry", "Telemetry")],
         params=[], walk_params=list(SENSOR_PARAMS), run=run_walk,
         description="The open walk's photos and sensor logs. Sensor settings are saved with the walk."),
    dict(id="camera-map", label="Camera map", group="Mapping",
         inputs=[port("photos", "Recorded photos"), port("telemetry", "Telemetry", optional=True)],
         outputs=[port("map", "Camera map")], params=[*STREAM_PARAMS, *CAMERA_MAP_PARAMS],
         run=run_camera_map, description="Structure from motion with COLMAP, levelled and scaled from the sensors."),
    dict(id="depth", label="Estimate depth", group="Depth",
         inputs=[port("photos", "Recorded photos"), port("map", "Camera map")], outputs=[port("depth", "Depth frames")],
         params=[choice("model", "Depth model", DEPTH_MODELS)], run=run_depth,
         description="Monocular depth for every saved frame, rectified with the camera map's lens."),
    dict(id="gaussian-depth", label="Render Gaussian depth", group="Depth",
         inputs=[port("splats", "Trained Gaussian map"), port("map", "Camera map")],
         outputs=[port("depth", "Depth frames")], params=[], run=run_gaussian_depth),
    dict(id="backproject", label="Backproject depth", group="Depth",
         inputs=[port("depth", "Depth frames"), port("map", "Camera map")],
         outputs=[port("points", "Registered depth points"), port("path", "Camera path")],
         params=[flag("trained", "Train depth correction before fitting points",
                      help="Depth Anything V2 only: fits a correction against the map's landmarks."),
                 flag("regenerate_path", "Regenerate the camera path from depth")],
         run=run_backproject),
    dict(id="landmark-surfaces", label="Landmark surfaces", group="Surfaces", inputs=[port("map", "Camera map")],
         outputs=[port("mesh", "Polygon mesh")],
         params=[choice("method", "Method", [(m.METHOD, m.LABEL) for m in SURFACE_MODULES]),
                 number("distance_cm", "Distance", 20, .1, 1000, "cm",
                        label_by=dict(param="method", labels={m.METHOD: m.DISTANCE_LABEL for m in SURFACE_MODULES})),
                 number("min_support", "Fewest landmarks per plane", cloud_planes.MIN_SUPPORT, 3, 10000, kind="int",
                        when=dict(param="method", values=[cloud_planes.METHOD]))],
         run=run_landmark_surfaces, description="Surfaces grown from the camera map's landmarks. Needs metric scale."),
    dict(id="ai-depth-planes", label=ai_surfaces.LABEL, group="Surfaces",
         inputs=[port("map", "Camera map"), port("photos", "Recorded photos")], outputs=[port("mesh", "Polygon mesh")],
         params=[number("distance_cm", ai_surfaces.DISTANCE_LABEL, 20, .1, 1000, "cm"),
                 number("views", "Views to infer", 36, 2, 400, kind="int")],
         run=run_ai_planes, description="Depth Anything V2 runs in this browser on sampled views; the Mac fits planes."),
    dict(id="depth-surfaces", label="Depth surfaces", group="Surfaces",
         inputs=[port("points", "Registered depth points", "Colored points"), port("map", "Camera map")],
         outputs=[port("mesh", "Polygon mesh")],
         params=[choice("method", "Method", [(k, v[0]) for k, v in depth_surfaces.METHODS.items()]),
                 flag("tof", "Include ToF returns", help="Needs a metric camera map."),
                 number("detail", "Mesh detail", 8, .001, 10000, help="Centimetres on metric maps, else % of map span.",
                        when=dict(param="method", values=["balloon"])),
                 number("reach", "Inflation limit", 200, .001, 100000,
                        help="Centimetres on metric maps, else % of map span.",
                        when=dict(param="method", values=["balloon"]))],
         run=run_depth_surfaces),
    dict(id="gaussian-map", label="Gaussian map", group="Gaussians",
         inputs=[port("map", "Camera map"), port("photos", "Recorded photos")],
         outputs=[port("splats", "Trained Gaussian map")], params=list(GAUSSIAN_PARAMS), run=run_gaussian_map,
         description="Brush trains 3D Gaussian splats from the registered photos."),
    dict(id="gaussian-mesh", label="Gaussian surface mesh", group="Gaussians",
         inputs=[port("splats", "Trained Gaussian map")], outputs=[port("mesh", "Polygon mesh")], params=list(_gaussian_mesh.PARAMS),
         run=run_gaussian_mesh, description="Depth rendered from several views, fused, meshed and simplified."),
    dict(id="clearance", label="Walk clearance", group="Gaussians",
         inputs=[port("splats", "Trained Gaussian map"), port("map", "Camera map")],
         outputs=[port("clearance", "Clearance map")], params=[*CLEARANCE_PARAMS], run=run_clearance,
         description="Free and occupied space around the walked path, from the splats and ToF sightlines."),
    dict(id="navigation-map", label="Navigation map", group="Drone",
         inputs=[port("clearance", "Clearance map")], outputs=[port("navmap", "Navigation map")], params=list(NAV_MAP_PARAMS),
         run=run_navigation_map,
         description="The walk's free and occupied space as the drone's map (map.dmp): merged into the drone's cells, "
                     "occupied wherever any part was."),
    dict(id="send-to-drone", label="Send to the drone", group="Drone",
         inputs=[port("navmap", "Navigation map")], outputs=[port("answers", "JSON records")], params=list(_nav_map.SEND_PARAMS),
         run=run_send_to_drone,
         description="The map to the drone's Pi over the Wi-Fi (dfb_pi's command port), with the guard's radius and where "
                     "the drone is. Its answers come back."),
    dict(id="lingbot-map", label="LingBot-Map", group="Mapping",
         inputs=[port("photos", "Recorded photos"), port("map", "Camera map")],
         outputs=[port("result", "LingBot trajectory and points")], params=list(LINGBOT_PARAMS), run=run_lingbot,
         description="Learned reconstruction from the image sequence, on Apple Metal."),
]
# Operators the browser runs: declared here so the catalogue is complete.
OPERATORS += [
    dict(id="code", label="Code transform", group="Code", runner="code", inputs=[], outputs=[], params=[],
         default_type="JSON records",
         description="JavaScript over typed inputs. Its ports, output type and controls are part of the node."),
    dict(id="replay-path", label="Replay path", group="Camera", runner="browser", params=list(REPLAY_PATH_PARAMS),
         source=["replay-path.js"],
         inputs=[port("poses", "Camera map", "Camera path"), port("frames", "Telemetry")],
         outputs=[port("path", "Camera path")],
         description="One camera pose per recorded frame, in this browser: the given poses as they are, the frames "
                     "between and beyond them estimated as the settings say. Each pose says how it was made."),
    dict(id="splat-fit", label="Fit splats to points", group="Gaussians", runner="browser", params=list(SPLAT_FIT_PARAMS),
         source=["gaussian-view.js"],
         inputs=[port("points", "Colored points", "Registered depth points")], outputs=[port("splats", "Point-fitted splats")],
         description="Gaussian splats sized from the local point spacing, in this browser."),
    dict(id="viewer", label="Viewer", group="Display", runner="viewer", inputs=[], outputs=[], params=[],
         source=["ui.js", "graph.js"],
         description="Draws its connected outputs. Edit its code to compose plot layers."),
]
OPERATORS_BY_ID = {o["id"]: o for o in OPERATORS}


HERE = Path(__file__).resolve().parent


def sources(operator):
    """The code that does an operator's work, for the inspector to show: its run function, the operators.py helpers and
    survey server methods it calls (ctx.survey.…), and the survey modules any of them use, else the browser files it
    names (source)."""
    import inspect
    import re as _re
    import server
    if "run" not in operator:
        return [dict(file=f) for f in operator.get("source", [])]
    found, files, queue = [], [], [("operators.py", operator["run"].__name__, operator["run"])]
    seen = set()
    while queue:
        file, name, target = queue.pop(0)
        if (file, name) in seen:
            continue
        seen.add((file, name))
        found.append(dict(file=file, name=name))
        code = inspect.getsource(target)
        scope = getattr(target, "__globals__", {})
        for call in _re.findall(r"\b([a-z_][a-z0-9_]*)\(", code):
            helper = scope.get(call)
            if not inspect.isfunction(helper):
                continue
            home = Path(inspect.getsourcefile(helper)).resolve()
            if home == Path(inspect.getsourcefile(target)).resolve():
                queue.append((home.name, call, helper))   # a helper beside it (operators.py, server.py)
            elif home.parent == HERE and home.name not in files:
                files.append(home.name)                    # a survey module it imported the function from
        for method in _re.findall(r"\b(?:ctx\.survey|self\.survey|survey)\.([a-z_][a-z0-9_]*)", code) + \
                (_re.findall(r"\bself\.([a-z_][a-z0-9_]*)", code) if name.startswith("Survey.") else []):
            target = getattr(server.Survey, method, None)
            if callable(target):
                queue.append(("server.py", "Survey." + method, target))
        for method in _re.findall(r"\bctx\.([a-z_][a-z0-9_]*)\(", code):
            target = getattr(Context, method, None)
            if callable(target):
                queue.append(("operators.py", "Context." + method, target))
        names = set(_re.findall(r"\b([a-z_][a-z0-9_]*)\.", code)) | \
            set(_re.findall(r"^\s*(?:import|from)\s+([a-z_][a-z0-9_]*)", code, _re.M))
        for module in sorted(names):
            if module not in ("operators", "server") and (HERE/(module + ".py")).is_file() and module + ".py" not in files:
                files.append(module + ".py")
    return found + [dict(file=f) for f in files]


def source_text(file, name=None):
    """A survey source file's text, or one function of it (name)."""
    import inspect
    import re as _re
    if not _re.fullmatch(r"[a-z0-9_\-]+\.(py|js)", file or "") or not (HERE/file).is_file():
        raise ValueError("Unknown source file")
    text = (HERE/file).read_text()
    if name:
        if not file.endswith(".py") or not _re.fullmatch(r"[A-Za-z_]\w*(\.[A-Za-z_]\w*)?", name):
            raise ValueError("Unknown function")
        import importlib
        target = importlib.import_module(file[:-3])
        for part in name.split("."):
            target = getattr(target, part, None)
        # Only a function or class written in that file (not one it imported).
        if not (inspect.isfunction(target) or inspect.isclass(target)) or \
                Path(inspect.getsourcefile(target) or "").resolve() != (HERE/file).resolve():
            raise ValueError("Unknown function")
        text = inspect.getsource(target)
    return text


def catalog():
    public = [{**{k: v for k, v in o.items() if k != "run"}, "sources": sources(o)} for o in OPERATORS]
    depth = [dict(id=k, label=v, units=replay_depth.UNITS[k]) for k, v in replay_depth.LABELS.items()]
    return dict(operators=public, types=TYPES, compatible=COMPATIBLE, depth_methods=depth)


def execute(survey, body):
    operator = OPERATORS_BY_ID.get(body.get("op"))
    if not operator or "run" not in operator:
        raise ValueError("Unknown server operator.")
    ctx = Context(survey, operator, body)
    try:
        result = operator["run"](ctx)
    except Pending as pending:
        return dict(pending=str(pending) or "Working")
    if "request" in (result or {}):
        return result
    with survey.lock:
        survey.operator_jobs.pop(ctx.key(), None)
    declared = {p["name"]: p["types"][0] for p in operator["outputs"]}
    return dict(outputs={name: dict(value, type=declared[name]) for name, value in result.items()})
