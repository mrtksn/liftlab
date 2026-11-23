"""Read a saved recording using camera timestamps and the Pi receive clock."""
from bisect import bisect_left
import csv
import math

from node_params import pairing_ns, sensor_settings, tof_metres


def read_timeline(run):
    sensors = sensor_settings(run)
    window = pairing_ns(sensors)*1e-9
    def read(name):
        path = run / name
        if path.is_file():
            with path.open() as stream:
                return list(csv.DictReader(stream))
        return []

    def number(row, key):
        try:
            value = float(row[key])
            return value if math.isfinite(value) else None
        except (KeyError, ValueError, TypeError):
            return None

    motion = [(t, row) for row in read("motion.csv")
              if (t := number(row, "pi_uart_monotonic_s")) is not None]
    motion.sort(key=lambda sample: sample[0])
    times = [t for t, _ in motion]
    result = []
    origin = None
    for row in read("timestamps.csv"):
        stamp = number(row, "pi_camera_relative_us")
        if stamp is None or not row.get("frame"):
            continue
        if origin is None:
            origin = stamp
        frame = dict(id=row["frame"], ms=round((stamp-origin)/1000),
                     tof=number(row, "tof_raw"), tof_m=tof_metres(row.get("tof_raw"), sensors))
        clock = number(row, "pi_encoder_callback_ns")
        if clock is not None and times:
            t = clock * 1e-9
            right = bisect_left(times, t)
            index = min({max(0, right-1), min(right, len(times)-1)}, key=lambda i: abs(times[i]-t))
            if abs(times[index]-t) <= window:
                sample = motion[index][1]
                frame.update(
                    gyro=[number(sample, k+"_rad_s") for k in ("gx","gy","gz")],
                    accel=[number(sample, k+"_m_s2") for k in ("ax","ay","az")],
                    attitude=[number(sample, k) for k in ("qw","qx","qy","qz")],
                    height=number(sample,"height_m") if number(sample,"height_valid") else None,
                    lag_ms=round((times[index]-t)*1000),
                )
        result.append(frame)
    return result


def read_recording_tables(run):
    """Read independent acquisition logs without resampling them to camera rate."""
    def cell(value):
        if value is None or value == "":
            return None
        try:
            n = float(value)
            return n if math.isfinite(n) else None
        except ValueError:
            return value

    tables = {}
    for path in sorted(run.glob("*.csv")):
        if path.is_symlink() or not path.is_file():
            continue
        with path.open(newline="") as stream:
            tables[path.name] = [{k: cell(v) for k, v in row.items() if k is not None}
                                 for row in csv.DictReader(stream)]
    return tables
