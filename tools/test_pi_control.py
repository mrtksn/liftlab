"""The production client's UDP request/reply, validation and no-retry timeout."""
import importlib.util
from pathlib import Path
import socket
import threading
import unittest

spec = importlib.util.spec_from_file_location("control", Path(__file__).resolve().parents[1] / "runner/pi/control.py")
control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)


class ClientTests(unittest.TestCase):
    def test_reply(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
            server.bind(("127.0.0.1", 0))
            received = []

            def answer():
                data, address = server.recvfrom(512)
                received.append(data)
                server.sendto(b"recommendation ready\n", address)

            worker = threading.Thread(target=answer)
            worker.start()
            reply = control.request("127.0.0.1", server.getsockname()[1], "autotune attitude")
            worker.join(2)
            self.assertEqual(received, [b"autotune attitude\n"])
            self.assertEqual(reply, "recommendation ready")

    def test_timeout_sends_once(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
            server.bind(("127.0.0.1", 0))
            with self.assertRaises(TimeoutError):
                control.request("127.0.0.1", server.getsockname()[1], "autotune apply", .05)
            self.assertEqual(server.recvfrom(512)[0], b"autotune apply\n")
            server.settimeout(.05)
            with self.assertRaises(TimeoutError):
                server.recvfrom(512)

    def test_single_command(self):
        for command in ("", "status\narm", "status\rarm", "status\0arm", "x" * 500):
            with self.assertRaises(ValueError):
                control.request("127.0.0.1", 14560, command)


if __name__ == "__main__":
    unittest.main()
