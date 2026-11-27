"""Checks the drone's live fixes (drone_fixes.py): the flight core's heading from its attitude, the nose in the camera
for a turned camera, the attitude at a moment, and PSI from a located picture (a fake locator)."""
import math
import unittest

import numpy as np

import drone_fixes as df


def quat_yaw(a):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


class DroneFixesTest(unittest.TestCase):
    def test_yaw(self):
        for a in (-3.0, -1.2, 0.0, 0.7, 2.9):
            self.assertAlmostEqual(df.yaw_of(quat_yaw(a)), a, places=6)

    def test_nose(self):
        # a camera turned 90 deg on the drone (flowcal's figure for the toy drone): in the map's camera axes (x right,
        # y down in the picture) the nose is down the picture; unturned, to its right
        self.assertTrue(np.allclose(df.nose_in_camera(90), [0, 1, 0], atol=1e-9))
        self.assertTrue(np.allclose(df.nose_in_camera(0), [1, 0, 0], atol=1e-9))

    def test_attitudes(self):
        a = df.Attitudes()
        for k in range(10):
            a.add(k * 0.05, quat_yaw(k * 0.1))
        self.assertAlmostEqual(df.yaw_of(a.at(0.21)), 0.4, places=6)        # nearest sample
        self.assertIsNone(a.at(5.0))                                         # too far from any

    def test_psi(self):
        """A drone facing map +X (heading 0) whose flight core says 30 deg: PSI 30 deg; turned, PSI the same."""
        class FakeLocator:
            def __init__(self, R): self.R = R
            def locate(self, jpeg, minimum): return dict(status="Located", xyz=[1, 2, 0.3], rotation=self.R.tolist(), inliers=60)
        fixer = df.Fixer.__new__(df.Fixer)
        fixer.nose, fixer.minimum, fixer.n, fixer.located = df.nose_in_camera(90), 12, 0, 0
        att = df.Attitudes()
        for drone_map_heading, fc_heading in ((0.0, 0.5236), (1.0, 1.5236), (-2.0, -1.4764)):
            # camera looking straight down, its picture's +y (the nose) along the map heading
            h = drone_map_heading
            y_axis = np.array([math.cos(h), math.sin(h), 0])                  # camera y (down in the picture) = the nose
            z_axis = np.array([0, 0, -1.0])                                    # looking down
            x_axis = np.cross(y_axis, z_axis)
            R = np.column_stack([x_axis, y_axis, z_axis])                      # camera -> map
            fixer.locator = FakeLocator(R)
            att.add(10.0, quat_yaw(fc_heading))
            r = fixer.fix(b"", 10.0, att)
            self.assertEqual(r["status"], "Located")
            self.assertAlmostEqual(r["heading"], df.wrap(h), places=6)
            self.assertAlmostEqual(math.degrees(r["psi"]), 30.0, places=3)
            self.assertTrue(r["line"].startswith("mapfix 10.000 1.000 2.000 0.300 0.5236"))
            att = df.Attitudes()


if __name__ == "__main__":
    unittest.main()
