"""Sent over SSH to the drone's Pi, in place of pi_camera.py (Device and recording: the camera "The drone's downward
camera, while it flies"). stdout contains the same timestamped JPEG packets.

The pictures come from the drone's own programs, which keep running: dfb_flow's (runner/pi/frame_out.h, its --frames
port: every Nth frame of the downward camera, 2×2 Bayer blocks as RGB, linear), the flight telemetry dfb_pi sends
Room Survey (UDP, the same as for pi_camera.py), and each ToF reading dfb_pi takes (UDP 127.0.0.1:14683, the same
range samples pi_camera.py makes). So a walk records what the drone sees in flight, the camera as it flies it, and
nothing on the drone stops.

dfb_flow balances each picture (grey world, a gamma: frame_out.h); this only compresses it.
"""
import argparse
import os
import signal
import socket
import struct
import sys
import threading
import time
from collections import deque


def stale_senders(proc="/proc", me=None):
    """PIDs of other running copies of this sender (python3 -u - ... --frames-port=N)."""
    me = os.getpid() if me is None else me
    found = []
    for entry in os.listdir(proc):
        if not entry.isdigit() or int(entry) == me:
            continue
        try:
            with open(os.path.join(proc, entry, "cmdline"), "rb") as f:
                argv = f.read().decode(errors="replace").rstrip("\0").split("\0")
        except OSError:
            continue
        if (len(argv) > 3 and os.path.basename(argv[0]).startswith("python3") and argv[1:3] == ["-u", "-"]
                and any(a.startswith("--frames-port=") or a.startswith("--telemetry-port=") for a in argv[3:])):
            found.append(int(entry))
    return found


FRAME = struct.Struct("<4sIIQIIII")        # frame_out.h: "DFF1", width, height, time [ns], frame number, exposure [µs], gain ×100, bytes
MOTION = struct.Struct("<d17f")
RANGE = struct.Struct(">QQi")


def listen(port, size, into, lock):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", port))
        while True:
            packet = s.recv(1024)
            if len(packet) == size:
                with lock:
                    into.append(packet)


def read_exact(sock, n):
    out = bytearray()
    while len(out) < n:
        chunk = sock.recv(n - len(out))
        if not chunk:
            raise ConnectionError("dfb_flow closed the pictures")
        out += chunk
    return bytes(out)


if __name__ == "__main__":
    import numpy as np
    import simplejpeg

    parser = argparse.ArgumentParser()
    parser.add_argument("--frames-port", type=int, default=14682)
    parser.add_argument("--telemetry-port", type=int, default=14681)
    parser.add_argument("--range-port", type=int, default=14683)
    parser.add_argument("--quality", type=int, default=88)
    parser.add_argument("--rotate", type=int, default=0, help="1 rotates the picture 180 degrees")
    args = parser.parse_args()

    stale = stale_senders()
    for pid in stale:
        try:
            os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    if stale:
        time.sleep(1)
        print(f"Stopped earlier sender(s) {stale}", file=sys.stderr, flush=True)

    motion, ranges, lock = deque(maxlen=1024), deque(maxlen=1024), threading.Lock()
    threading.Thread(target=listen, args=(args.telemetry_port, MOTION.size, motion, lock), daemon=True).start()
    threading.Thread(target=listen, args=(args.range_port, RANGE.size, ranges, lock), daemon=True).start()
    stream = sys.stdout.buffer
    spent, spent_t0 = [], time.monotonic()
    while True:
        try:
            sock = socket.create_connection(("127.0.0.1", args.frames_port), timeout=5)
        except OSError:
            print("Waiting for dfb_flow's pictures (is the flow program running with --frames "
                  f"{args.frames_port}?)", file=sys.stderr, flush=True)
            time.sleep(2)
            continue
        print(f"Connected to dfb_flow on port {args.frames_port}", file=sys.stderr, flush=True)
        try:
            while True:
                magic, w, h, t_ns, seq, exp_us, gain, size = FRAME.unpack(read_exact(sock, FRAME.size))
                if magic != b"DFF1" or size != w * h * 3:
                    raise ConnectionError("not dfb_flow's pictures")
                rgb = np.frombuffer(read_exact(sock, size), np.uint8).reshape(h, w, 3)
                got = time.monotonic()
                pic = rgb
                if args.rotate:
                    pic = pic[::-1, ::-1]
                jpeg = simplejpeg.encode_jpeg(np.ascontiguousarray(pic), quality=args.quality, colorspace="RGB",
                                              colorsubsampling="420", fastdct=True)
                with lock:
                    m, r = list(motion), list(ranges)
                    motion.clear(); ranges.clear()
                # (the frame's time: the camera driver's stamp, on the clock the telemetry and ranges carry)
                stream.write(struct.pack(">4sQIQHH", b"IMU2", t_ns // 1000, len(jpeg), t_ns, len(m), len(r)))
                stream.write(b"".join(m)); stream.write(b"".join(r)); stream.write(jpeg)
                stream.flush()
                spent.append(time.monotonic() - got)
                if len(spent) == 30:
                    print(f"{30 / max(1e-6, time.monotonic() - spent_t0):.1f} pictures a second, {1e3 * sum(spent) / 30:.0f} ms each on this Pi", file=sys.stderr, flush=True)
                    spent.clear(); spent_t0 = time.monotonic()
        except BrokenPipeError:
            raise SystemExit(0)                          # (the Mac went away: stdout is closed)
        except (ConnectionError, OSError, socket.timeout) as e:   # (dfb_flow restarted, or stalled: connect again)
            print(f"Pictures stopped: {e}", file=sys.stderr, flush=True)
            sock.close()
            time.sleep(1)
