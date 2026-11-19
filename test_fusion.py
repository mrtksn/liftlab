from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from fusion import gravity_alignment, load_observations


class FusionTests(unittest.TestCase):
    def test_mount_alignment_and_rejection(self):
        rng = np.random.default_rng(234)
        def rotation(v):
            return cv2.Rodrigues(np.asarray(v, dtype=float))[0]
        world = rotation([.45,-.6,.4])
        mount = rotation([.5,.4,-.3])
        visual, imu = [], []
        for axis in rng.normal(size=(60,3))*.45:
            angle = np.linalg.norm(axis)
            imu.append(np.r_[np.cos(angle/2), axis*np.sin(angle/2)/angle])
            visual.append(world.T @ rotation(axis) @ mount)
        np.testing.assert_allclose(gravity_alignment(visual,imu), world, atol=1e-7)
        self.assertIsNone(gravity_alignment(visual,imu[::-1]))
        self.assertIsNone(gravity_alignment([np.eye(3)]*60, [[1,0,0,0]]*60))

    def test_clock_and_range(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run/"timestamps.csv").write_text("frame,pi_encoder_callback_ns,tof_raw\na,500000000,52\nb,1400000000,104\n")
            (run/"motion.csv").write_text("pi_uart_monotonic_s,qw,qx,qy,qz,height_m,height_valid\n0.51,1,0,0,0,2,1\n")
            obs = load_observations(run)
            self.assertEqual(obs["a"]["height"], 2)
            self.assertEqual(obs["a"]["range"], .5)
            self.assertEqual(obs["b"], {"range":1.0})
