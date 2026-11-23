from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from fusion import body_up, gravity_alignment, levelling_rotation, load_observations, up_alignment


def rotation(v):
    return cv2.Rodrigues(np.asarray(v, dtype=float))[0]


def handheld_walk(rng, count=40, noise_deg=1.):
    """Mostly turning about the vertical, a few degrees of tilt, a pitched camera mount."""
    visual_from_inertial = rotation([.45,-.6,.4])
    # Camera (x right, y down, z forward) to body (x forward, y left, z up),
    # pitched 35 degrees down about the body's left axis.
    level = np.array([[0,0,1],[-1,0,0],[0,-1,0]], dtype=float)
    mount = rotation([0,np.radians(35),0]) @ level
    rotations, ups = [], []
    for yaw in np.linspace(-1.2, 1.4, count):
        tilt = rng.normal(size=2)*np.radians(3)
        body = rotation([0,0,yaw]) @ rotation([*tilt,0])       # world from body, Z up
        rotations.append(visual_from_inertial @ body @ mount)  # visual world from camera
        up = body.T @ [0,0,1] + rng.normal(size=3)*np.radians(noise_deg)
        ups.append(up/np.linalg.norm(up))
    return rotations, ups, visual_from_inertial @ [0,0,1]


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

    def test_body_up_matches_quaternion_rotation(self):
        q = np.array([.44,-.52,.37,.63]); q /= np.linalg.norm(q)
        w, x, y, z = q
        v, skew = q[1:], np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
        world_from_body = (w*w-v@v)*np.eye(3) + 2*np.outer(v,v) + 2*w*skew
        np.testing.assert_allclose(body_up(q), world_from_body.T @ [0,0,1], atol=1e-12)

    def test_up_from_mostly_vertical_turns(self):
        rotations, ups, truth = handheld_walk(np.random.default_rng(234))
        fit = up_alignment(rotations, ups)
        self.assertIsNotNone(fit)
        self.assertLess(np.degrees(np.arccos(fit["up"] @ truth)), 2)
        self.assertLessEqual(fit["uncertainty_deg"], 10)
        self.assertAlmostEqual(fit["camera_pitch_down_deg"], 35, delta=3)

    def test_up_rejects_unconstrained_or_inconsistent_data(self):
        rotations, ups, _ = handheld_walk(np.random.default_rng(5))
        self.assertIsNone(up_alignment(rotations, ups[::-1]))
        self.assertIsNone(up_alignment([np.eye(3)]*60, [[0,0,1]]*60))
        self.assertIsNone(up_alignment(rotations[:8], ups[:8]))

    def test_levelling_rotation_keeps_heading(self):
        for up in ([.2,-.5,.84], [0,0,1], [0,0,-1], [.6,0,-.8]):
            up = np.asarray(up)/np.linalg.norm(up)
            level = levelling_rotation(up)
            np.testing.assert_allclose(level @ up, [0,0,1], atol=1e-9)
            np.testing.assert_allclose(level @ level.T, np.eye(3), atol=1e-9)
        # A direction perpendicular to the tilt plane stays put.
        np.testing.assert_allclose(levelling_rotation([.3,0,.95]) @ [0,1,0], [0,1,0], atol=1e-9)

    def test_clock_and_range(self):
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run/"timestamps.csv").write_text("frame,pi_encoder_callback_ns,tof_raw\na,500000000,52\nb,1400000000,104\n")
            (run/"motion.csv").write_text("pi_uart_monotonic_s,qw,qx,qy,qz,height_m,height_valid\n0.51,1,0,0,0,2,1\n")
            obs = load_observations(run)
            self.assertEqual(obs["a"]["height"], 2)
            self.assertEqual(obs["a"]["range"], .5)
            self.assertEqual(obs["b"], {"range":1.0})
