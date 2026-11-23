"""Sent over SSH to the Pi. stdout contains timestamped JPEG packets.

The Mac passes the device settings as arguments: python3 -u - --width 640 ...
"""
import argparse
import fcntl
import time
import socket
import struct
from collections import deque
import sys
import threading
from libcamera import Transform
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import Output


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
