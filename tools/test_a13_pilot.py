"""a13_pilot's packets against ones captured from the stock app, and the sender's rate, heartbeat and goodbye."""
import importlib.util
from pathlib import Path
import socket
import time
import unittest

spec = importlib.util.spec_from_file_location("a13_pilot", Path(__file__).resolve().parent / "a13_pilot.py")
pilot = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pilot)

# Byte-exact packets from captures/a13_app_1.pcapng, with the stick bytes they carry.
CAPTURED = [
    ((0x80, 0x80, 0x80, 0x80, 0x00), "036614808080800002000000000000000000000299"),   # neutral
    ((0x80, 0x80, 0xf8, 0x80, 0x00), "0366148080f8800002000000000000000000007a99"),   # throttle up
    ((0x80, 0x80, 0x6a, 0x0c, 0x00), "03661480806a0c0002000000000000000000006499"),   # yaw left
    ((0x7b, 0xa8, 0x80, 0x80, 0x00), "0366147ba88080000200000000000000000000d199"),   # pitch forward
    ((0xa7, 0x81, 0x80, 0x80, 0x00), "036614a78180800002000000000000000000002499"),   # roll right
    ((0x80, 0x80, 0x80, 0x80, 0x01), "036614808080800102000000000000000000000399"),   # take off
    ((0x80, 0x80, 0x80, 0x80, 0x02), "036614808080800202000000000000000000000099"),   # land
]

def from_byte(b):
    """The stick value that stick_byte turns into b."""
    return (b - 0x80) / (0x7F if b > 0x80 else 0x80)


class PacketTests(unittest.TestCase):
    def test_matches_captured_app_packets(self):
        for (roll, pitch, throttle, yaw, flags), hexed in CAPTURED:
            pkt = pilot.control_packet(from_byte(roll), from_byte(pitch), from_byte(throttle), from_byte(yaw), flags)
            self.assertEqual(pkt.hex(), hexed)

    def test_stick_range(self):
        self.assertEqual(pilot.stick_byte(-1), 0x00)
        self.assertEqual(pilot.stick_byte(0), 0x80)
        self.assertEqual(pilot.stick_byte(1), 0xFF)
        self.assertEqual(pilot.stick_byte(7), 0xFF)

    def test_short_packets(self):
        self.assertEqual(pilot.camera_packet(2), b"\x06\x02")
        self.assertEqual(pilot.HEARTBEAT, b"\x01\x01")
        self.assertEqual(pilot.GOODBYE, b"\x08\x01")


class LinkTests(unittest.TestCase):
    def test_rate_heartbeat_flag_and_goodbye(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as drone:
            drone.bind(("127.0.0.1", 0))
            drone.settimeout(0.5)
            pilot.CONTROL_PORT = drone.getsockname()[1]
            link = pilot.Link("127.0.0.1").start()
            link.command(pilot.TAKEOFF)
            got, t0 = [], time.time()
            while time.time() - t0 < 1.5:
                data, addr = drone.recvfrom(64)
                got.append((time.time() - t0, data))
                drone.sendto(bytes.fromhex("6502314631460000000000"), addr)
            link.stop()
            try:
                while True: got.append((9, drone.recvfrom(64)[0]))
            except socket.timeout:
                pass
        controls = [(t, d) for t, d in got if len(d) == 21]
        rate = sum(1 for t, _ in controls if t < 1.5) / 1.5
        self.assertTrue(14 <= rate <= 19, rate)
        self.assertGreaterEqual(sum(1 for _, d in got if d == pilot.HEARTBEAT), 2)
        self.assertEqual(controls[0][1][7], pilot.TAKEOFF)
        self.assertEqual(controls[-1][1][7], pilot.LAND)                              # quitting lands
        self.assertTrue(all(d[3:7] == b"\x80" * 4 for _, d in controls[-3:]))         # with the sticks centred
        flagged = [t for t, d in controls if t < 1.4 and d[7] == pilot.TAKEOFF]
        self.assertTrue(0.9 <= flagged[-1] <= 1.15, flagged[-1])                       # take-off held about 1 s
        self.assertEqual(got[-1][1], pilot.GOODBYE)
        self.assertGreater(link.replies, 10)


if __name__ == "__main__":
    unittest.main()
