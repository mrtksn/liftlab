"""Read a saved recording using camera timestamps and the Pi receive clock."""
from bisect import bisect_left
import csv
import math


def read_timeline(run):
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
                     tof=number(row, "tof_raw"))
        clock = number(row, "pi_encoder_callback_ns")
        if clock is not None and times:
            t = clock * 1e-9
            right = bisect_left(times, t)
            index = min({max(0, right-1), min(right, len(times)-1)}, key=lambda i: abs(times[i]-t))
            if abs(times[index]-t) <= .08:
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
