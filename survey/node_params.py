"""Settings that pipeline nodes own, with their ranges and the previous fixed values as defaults.

Sensor settings describe how a recording is read, so they live with the walk
(``sensors.json`` in the walk folder, falling back to ``sensors.json`` in the
survey folder for new walks). Camera map settings travel with the node and are
stored in ``survey.json`` under ``params`` so a node can tell whether the saved
map was built with its settings.
"""
import json
import math
import re
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
    dict(group="Solver", key="frame_stride", label="Use every Nth frame", kind="int", default=2, min=1, max=100,
         help="Frames considered when rebuilding a finished walk."),
    dict(group="Solver", key="max_views", label="Most views in the solve", kind="int", default=400, min=8, max=5000,
         help="Spread evenly over the walk. More views cost time and memory."),
    dict(group="Solver", key="sift_features", label="SIFT features per view", kind="int", default=2500, min=200, max=30000),
    dict(group="Solver", key="neighbour_matches", label="Match with the next N views", kind="int", default=8, min=1, max=100),
    dict(group="Solver", key="similar_matches", label="Similar distant views to try", kind="int", default=8, min=0, max=100,
         help="Visually similar non-neighbours per view. Each candidate must pass geometric verification."),
    dict(group="Solver", key="loop_matches", label="Match walk start with its end", kind="bool", default=True),
    dict(group="Solver", key="exhaustive_below", label="Match all pairs below N views", kind="int", default=45, min=0, max=500),
    dict(group="Solver", key="init_min_tri_angle", label="Minimum initial triangulation angle", kind="number", unit="°",
         default=6.0, min=.5, max=45.0),
    dict(group="Solver", key="focal_guess_px", label="Focal length guess without calibration", kind="number", unit="px",
         default=260.0, min=20.0, max=10000.0, help="Starting value for the self-estimated fisheye lens."),
    dict(group="Solver", key="use_calibration", label="Use the checkerboard lens calibration", kind="bool", default=True),
    dict(group="Frame recovery", key="recover_skipped", label="Recover skipped frames against landmarks", kind="bool", default=True),
    dict(group="Frame recovery", key="recover_gaps", label="Re-solve unmapped stretches", kind="bool", default=True),
    dict(group="Scale and level", key="up_max_uncertainty_deg", label="Largest accepted up-axis uncertainty", kind="number", unit="°",
         default=10.0, min=.5, max=90.0, help="IMU levelling is skipped when up is less certain than this."),
    dict(group="Scale and level", key="up_max_holdout_deg", label="Largest held-out up error (80th percentile)", kind="number",
         unit="°", default=6.0, min=.5, max=90.0),
    dict(group="Scale and level", key="scale_min_views", label="Fewest views for a ToF scale", kind="int", default=12, min=3,
         max=10000),
    dict(group="Scale and level", key="scale_min_spread_m", label="Smallest spread of ToF distances", kind="number", unit="m",
         default=.30, min=0.0, max=20.0, help="Scale needs readings at different distances."),
    dict(group="Scale and level", key="scale_max_error_pct", label="Largest held-out scale error (80th percentile)",
         kind="number", unit="%", default=15.0, min=1.0, max=100.0),
    dict(group="Frame recovery", key="recover_max_frames", label="Farthest frame from a mapped view", kind="int", default=4,
         min=1, max=100),
    dict(group="Frame recovery", key="recover_min_inliers", label="Fewest tracked landmarks", kind="int", default=20, min=6,
         max=10000),
    dict(group="Frame recovery", key="recover_min_inlier_ratio", label="Smallest share of geometric inliers", kind="number",
         default=.65, min=.05, max=1.0),
    dict(group="Frame recovery", key="recover_min_spread_px", label="Smallest image spread of inliers", kind="number",
         unit="px", default=65.0, min=0.0, max=2000.0),
    dict(group="Frame recovery", key="recover_max_error_px", label="Largest reprojection error", kind="number", unit="px",
         default=2.5, min=.1, max=50.0),
    dict(group="Frame recovery", key="recover_max_jump", label="Largest jump as a share of scene depth", kind="number",
         default=.20, min=.01, max=5.0),
    dict(group="Frame recovery", key="gap_min_frames", label="Shortest unmapped stretch to re-solve", kind="int", unit="frames",
         default=20, min=2, max=100000),
    dict(group="Frame recovery", key="gap_max_frames", label="Longest unmapped stretch to re-solve", kind="int",
         unit="frames", default=200, min=2, max=100000),
    dict(group="Frame recovery", key="gap_max_repairs", label="Stretches re-solved per build", kind="int", default=3, min=0,
         max=100),
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
        elif spec["kind"] == "text":
            if not isinstance(value, str) or not 0 < len(value) <= 200 or (
                    spec.get("pattern") and not re.fullmatch(spec["pattern"], value)):
                raise ValueError(f"{spec['label']} is not valid.")
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


# How this Mac reaches and drives the recording hardware. Saved as device.json in the survey folder.
DEVICE_PARAMS = (
    dict(group="Connection", key="pi_host", label="Pi SSH destination", kind="text", default="pi@192.168.2.127",
         pattern=r"[A-Za-z0-9_.@:\-\[\]]+", help="user@host for SSH. Key login must work without a password."),
    dict(group="Camera", key="camera_source", label="Camera", kind="choice", default="picamera",
         options=[["picamera", "The Pi camera through Picamera2 (the drone's flow program stopped)"],
                  ["drone", "The drone's downward camera through its flow program, while it runs"]],
         help="The drone's: dfb_flow sends every Nth frame (its --frames port) and dfb_pi the telemetry and the ToF; "
              "nothing on the drone stops. Its pictures are 320 × 240: use a lens calibration file of their own."),
    dict(group="Camera", key="frames_port", label="The flow program's picture port", kind="int", default=14682,
         min=1024, max=65535, help="dfb_flow --frames PORT, on the Pi (the drone's camera only)."),
    dict(group="Camera", key="camera_width", label="Image width", kind="int", unit="px", default=640, min=64, max=4096),
    dict(group="Camera", key="camera_height", label="Image height", kind="int", unit="px", default=480, min=48, max=4096),
    dict(group="Camera", key="camera_fps", label="Frame rate", kind="number", unit="fps", default=8.0, min=1.0, max=120.0),
    dict(group="Camera", key="jpeg_bitrate", label="JPEG encoder bitrate", kind="int", unit="bit/s", default=2800000,
         min=100000, max=50000000),
    dict(group="Camera", key="rotate_180", label="Rotate the image 180°", kind="bool", default=True,
         help="For a camera mounted upside down."),
    dict(group="Camera", key="calibration_file", label="Lens calibration file", kind="text", default="ov5647-640x480.json",
         pattern=r"[A-Za-z0-9_.\-]+\.json", help="Saved in the survey folder. Use one per camera and resolution."),
    dict(group="Sensors", key="tof_address", label="ToF I²C address", kind="choice", default=0x52,
         options=[[0x29, "0x29"], [0x52, "0x52"], [0x64, "0x64"]]),
    dict(group="Sensors", key="tof_poll_hz", label="ToF poll rate", kind="number", unit="Hz", default=60.0, min=1.0, max=500.0),
    dict(group="Sensors", key="telemetry_port", label="Flight telemetry UDP port on the Pi", kind="int", default=14681,
         min=1024, max=65535),
    dict(group="Live map", key="live_view_interval_s", label="Time between live map views", kind="number", unit="s",
         default=.5, min=.05, max=10.0),
    dict(group="Live map", key="live_max_views", label="Most views in a live map", kind="int", default=160, min=8, max=2000),
    dict(group="Live map", key="live_first_build", label="Views before the first live map", kind="int", default=12, min=4,
         max=1000),
    dict(group="Calibration", key="board_columns", label="Checkerboard inner corners across", kind="int", default=9, min=3,
         max=40),
    dict(group="Calibration", key="board_rows", label="Checkerboard inner corners down", kind="int", default=6, min=3, max=40),
)


def device_settings(root):
    path = Path(root)/"device.json"
    try:
        return validate(DEVICE_PARAMS, json.loads(path.read_text()))
    except (OSError, ValueError, json.JSONDecodeError):
        return defaults(DEVICE_PARAMS)


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
