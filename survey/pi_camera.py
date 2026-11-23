"""Sent over SSH to the Pi. stdout contains timestamped JPEG packets.

The Mac passes the device settings as arguments: python3 -u - --width 640 ...
A new sender first stops earlier copies of itself: a connection that dropped
without closing can leave one running, holding the camera and telemetry port.
"""
import argparse
import fcntl
import os
import signal
import time
import socket
import struct
from collections import deque
import sys
import threading


def stale_senders(proc="/proc", me=None):
    """PIDs of other running copies of this sender (python3 -u - ... --telemetry-port=N)."""
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
                and any(a.startswith("--telemetry-port=") for a in argv[3:])):
            found.append(int(entry))
    return found


def running(pid, proc="/proc"):
    """False once the process has exited, including while it waits to be reaped."""
    try:
        with open(os.path.join(proc, str(pid), "stat")) as f:
            return f.read().rsplit(")", 1)[1].split()[0] != "Z"
    except (OSError, IndexError):
        return False


def stop_senders(pids, timeout=5.0, proc="/proc"):
    for sig in (signal.SIGTERM, signal.SIGKILL):
        for pid in pids:
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError):
                pass
        deadline = time.monotonic() + timeout
        while any(running(pid, proc) for pid in pids) and time.monotonic() < deadline:
            time.sleep(.1)
        pids = [pid for pid in pids if running(pid, proc)]
        if not pids:
            return []
    return pids


class RangeReader:
    def __init__(self):
        self.bus = None
        self.retry = 0

    def sample(self):
        try:
            if self.bus is None:
                if time.monotonic() < self.retry:
                    return -1
                self.bus = open('/dev/i2c-1','r+b',buffering=0)
                fcntl.ioctl(self.bus, 0x0703, args.tof_address)
            self.bus.write(bytes([0]))
            time.sleep(.001)
            data = self.bus.read(2)
            return int.from_bytes(data,'big') if len(data) == 2 else -1
        except OSError:
            if self.bus:
                self.bus.close()
                self.bus = None
            self.retry = time.monotonic() + 1
            return -1


def receive_motion():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as listener:
        listener.bind(("127.0.0.1", args.telemetry_port))
        while True:
            packet = listener.recv(1024)
            if len(packet) == imu_packet.size:
                with sample_lock:
                    samples.append(packet)


def poll_range():
    period = 1/args.tof_hz
    deadline = time.monotonic()
    try:
        while not done.is_set():
            start_ns = time.monotonic_ns()
            raw = ranges.sample()
            with range_lock:
                range_samples.append(range_packet.pack(start_ns, time.monotonic_ns(), raw))
            deadline += period
            now = time.monotonic()
            if deadline < now:
                deadline = now
            done.wait(deadline-now)
    finally:
        if ranges.bus:
            ranges.bus.close()


if __name__ == "__main__":
    from libcamera import Transform
    from picamera2 import Picamera2
    from picamera2.encoders import MJPEGEncoder
    from picamera2.outputs import Output

    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=8)
    parser.add_argument("--bitrate", type=int, default=2800000)
    parser.add_argument("--rotate", type=int, default=1, help="1 rotates the image 180 degrees")
    parser.add_argument("--tof-address", type=int, default=0x52)
    parser.add_argument("--tof-hz", type=float, default=60)
    parser.add_argument("--telemetry-port", type=int, default=14681)
    args = parser.parse_args()
    ranges = RangeReader()
    imu_packet = struct.Struct("<d17f")
    range_packet = struct.Struct(">QQi")
    samples = deque(maxlen=1024)
    range_samples = deque(maxlen=1024)
    range_lock = threading.Lock()
    sample_lock = threading.Lock()

    stale = stale_senders()
    if stale:
        left = stop_senders(stale)
        print(f"Stopped earlier sender(s) {stale}" + (f"; still running: {left}" if left else ""), file=sys.stderr, flush=True)

    class PacketOutput(Output):
        def outputframe(self, frame, keyframe=True, timestamp=None, packet=None, audio=False):
            if self.recording and timestamp is not None:
                try:
                    callback_ns = time.monotonic_ns()
                    with range_lock:
                        ranging = list(range_samples)
                        range_samples.clear()
                    with sample_lock:
                        motion = list(samples)
                        samples.clear()
                    stream.write(struct.pack(">4sQIQHH", b"IMU2", timestamp, len(frame),
                                             callback_ns, len(motion), len(ranging)))
                    stream.write(b"".join(motion))
                    stream.write(b"".join(ranging))
                    stream.write(frame)
                    stream.flush()
                except (BrokenPipeError, ConnectionResetError):
                    done.set()

    done = threading.Event()
    stream = sys.stdout.buffer
    threading.Thread(target=receive_motion, daemon=True).start()
    threading.Thread(target=poll_range, daemon=True).start()
    with Picamera2() as camera:
        config = camera.create_video_configuration(main={"size": (args.width, args.height), "format": "YUV420"},
                                                   transform=Transform(hflip=bool(args.rotate), vflip=bool(args.rotate)),
                                                   controls={"FrameRate": args.fps})
        camera.configure(config)
        camera.start_recording(MJPEGEncoder(bitrate=args.bitrate), PacketOutput())
        try:
            done.wait()
        finally:
            camera.stop_recording()
