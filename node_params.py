"""Settings that pipeline nodes own, with their ranges and the previous fixed values as defaults.

Sensor settings describe how a recording is read, so they live with the walk
(``sensors.json`` in the walk folder, falling back to ``sensors.json`` in the
survey folder for new walks). Camera map settings travel with the node and are
stored in ``survey.json`` under ``params`` so a node can tell whether the saved
map was built with its settings.
"""
import json
import math
from pathlib import Path

SENSOR_PARAMS = (
    dict(key="tof_counts_per_metre", label="ToF counts per metre", kind="number", default=104.0,
         min=1.0, max=100000.0, help="Raw TOF250 reading at one metre. Wall checks gave 52 at 0.50 m and 104 at 1.00 m."),
    dict(key="tof_min_raw", label="Lowest valid ToF reading", kind="int", default=1, min=0, max=65535,
         help="Raw readings below this are treated as no return."),
    dict(key="tof_max_raw", label="Highest valid ToF reading", kind="int", default=249, min=1, max=65535,
         help="Raw readings above this are treated as out of range."),
    dict(key="pairing_window_ms", label="Sensor pairing window", kind="number", unit="ms", default=80.0,
         min=1.0, max=1000.0, help="Largest time difference when pairing a frame with an IMU or ToF sample."),
)

CAMERA_MAP_PARAMS = (
    dict(key="frame_stride", label="Use every Nth frame", kind="int", default=2, min=1, max=100,
         help="Frames considered when rebuilding a finished walk."),
    dict(key="max_views", label="Most views in the solve", kind="int", default=400, min=8, max=5000,
         help="Spread evenly over the walk. More views cost time and memory."),
    dict(key="sift_features", label="SIFT features per view", kind="int", default=2500, min=200, max=30000),
    dict(key="neighbour_matches", label="Match with the next N views", kind="int", default=8, min=1, max=100),
    dict(key="similar_matches", label="Similar distant views to try", kind="int", default=8, min=0, max=100,
         help="Visually similar non-neighbours per view. Each candidate must pass geometric verification."),
    dict(key="loop_matches", label="Match walk start with its end", kind="bool", default=True),
    dict(key="exhaustive_below", label="Match all pairs below N views", kind="int", default=45, min=0, max=500),
    dict(key="init_min_tri_angle", label="Minimum initial triangulation angle", kind="number", unit="°",
         default=6.0, min=.5, max=45.0),
    dict(key="focal_guess_px", label="Focal length guess without calibration", kind="number", unit="px",
         default=260.0, min=20.0, max=10000.0, help="Starting value for the self-estimated fisheye lens."),
    dict(key="use_calibration", label="Use the checkerboard lens calibration", kind="bool", default=True),
    dict(key="recover_skipped", label="Recover skipped frames against landmarks", kind="bool", default=True),
    dict(key="recover_gaps", label="Re-solve unmapped stretches", kind="bool", default=True),
    dict(key="up_max_uncertainty_deg", label="Largest accepted up-axis uncertainty", kind="number", unit="°",
         default=10.0, min=.5, max=90.0, help="IMU levelling is skipped when up is less certain than this."),
)


def defaults(schema):
    return {p["key"]: p["default"] for p in schema}


def validate(schema, values):
    """Complete values with defaults, rejecting unknown keys and out-of-range numbers."""
    values = dict(values or {})
    known = {p["key"]: p for p in schema}
    unknown = sorted(set(values) - set(known))
    if unknown:
        raise ValueError("Unknown setting: " + ", ".join(unknown))
    result = defaults(schema)
    for key, value in values.items():
        spec = known[key]
        if spec["kind"] == "bool":
            if not isinstance(value, bool):
                raise ValueError(f"{spec['label']} must be on or off.")
        elif spec["kind"] == "choice":
            if value not in [option[0] for option in spec["options"]]:
                raise ValueError(f"{spec['label']} must be one of: " + ", ".join(o[1] for o in spec["options"]) + ".")
        else:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{spec['label']} must be a number.")
            if spec["kind"] == "int":
                if value != int(value):
                    raise ValueError(f"{spec['label']} must be a whole number.")
                value = int(value)
            else:
                value = float(value)
            if not spec["min"] <= value <= spec["max"]:
                raise ValueError(f"{spec['label']} must be between {spec['min']} and {spec['max']}.")
        result[key] = value
    if "tof_min_raw" in result and result["tof_min_raw"] > result["tof_max_raw"]:
        raise ValueError("The lowest valid ToF reading must not exceed the highest.")
    return result


def sensor_settings(run):
    """The walk's sensor settings, else the survey folder's defaults for new walks, else built-ins."""
    run = Path(run)
    for path in (run/"sensors.json", run.parent/"sensors.json"):
        if path.is_file():
            try:
                return validate(SENSOR_PARAMS, json.loads(path.read_text()))
            except (ValueError, json.JSONDecodeError):
                continue
    return defaults(SENSOR_PARAMS)


def default_sensor_settings(root):
    """Settings new walks start from, as saved in the survey folder."""
    return sensor_settings(Path(root)/".new-walk")


def tof_metres(raw, sensors):
    """Metres for a valid raw ToF reading, else None."""
    try:
        raw = int(raw)
    except (TypeError, ValueError):
        return None
    if not sensors["tof_min_raw"] <= raw <= sensors["tof_max_raw"]:
        return None
    return raw / sensors["tof_counts_per_metre"]


def pairing_ns(sensors):
    return int(sensors["pairing_window_ms"]*1e6)
