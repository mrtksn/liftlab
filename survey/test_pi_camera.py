"""The Pi sender replaces earlier copies of itself and nothing else."""
from pathlib import Path
import tempfile
import unittest

from pi_camera import running, stale_senders, stop_senders


def fake_process(proc, pid, argv, state="S"):
    folder = Path(proc)/str(pid)
    folder.mkdir()
    (folder/"cmdline").write_bytes("\0".join(argv).encode() + b"\0")
    (folder/"stat").write_text(f"{pid} ({Path(argv[0]).name}) {state} 1 1 1")


class SenderTests(unittest.TestCase):
    def test_finds_only_other_senders(self):
        with tempfile.TemporaryDirectory() as proc:
            sender = ["python3", "-u", "-", "--width=640", "--telemetry-port=14681"]
            fake_process(proc, 4436, sender)
            fake_process(proc, 5000, sender)                                     # this process
            fake_process(proc, 391, ["python3", "/home/pi/liftlab/liftlab", "watch"])
            fake_process(proc, 4426, ["bash", "-c", "python3 -u - --width=640 --telemetry-port=14681"])
            fake_process(proc, 777, ["/usr/bin/python3.11", "-u", "-", "--telemetry-port=14681"])
            fake_process(proc, 888, ["python3", "-u", "-", "--width=640"])         # another stdin script
            (Path(proc)/"self").mkdir()
            self.assertEqual(sorted(stale_senders(proc, me=5000)), [777, 4436])

    def test_exited_and_zombie_processes_count_as_stopped(self):
        with tempfile.TemporaryDirectory() as proc:
            fake_process(proc, 12, ["python3", "-u", "-", "--telemetry-port=1"], state="Z")
            self.assertFalse(running(12, proc))
            self.assertFalse(running(13, proc))
            self.assertEqual(stop_senders([2**22 + 12345], timeout=.1, proc=proc), [])


if __name__ == "__main__":
    unittest.main()
