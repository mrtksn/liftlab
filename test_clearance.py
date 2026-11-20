import tempfile
import unittest
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from clearance import (FREE, OCCUPIED, UNKNOWN, integrate, range_samples,
                       replay_distances, sample_poses)


class ClearanceTest(unittest.TestCase):
    def test_sphere_distances_include_unknown_and_outside(self):
        grid = np.full((9, 9, 9), FREE, dtype='u1')
        grid[6, 4, 4] = OCCUPIED
        grid[4, 3, 4] = UNKNOWN
        points = np.array([[4.5, 4.5, 4.5], [2.5, 4.5, 4.5], [0.1, 4, 4], [0, 0, 0]])
        a, b, edge, missing = replay_distances(grid, np.zeros(3), 1, points, [1, 1, 1, 0])
        self.assertEqual(a, dict(obstacle_m=1.5, observed_m=.5))
        self.assertGreater(b['observed_m'], 1)
        self.assertAlmostEqual(edge['observed_m'], .1)
        self.assertIsNone(missing)

    def test_interpolation_excludes_unobserved_intervals(self):
        times = np.array([0., .25, 2.])
        positions = np.array([[0., 0, 0], [.1, 0, 0], [1., 0, 0]])
        path, _, valid, exact = sample_poses(times, positions, Rotation.identity(3),
                                              [-.1, .125, .9, 2., 2.1], 1.)
        self.assertEqual(valid.tolist(), [False, True, False, True, False])
        self.assertEqual(exact.tolist(), [False, False, False, True, False])
        self.assertAlmostEqual(path[1, 0], .05)

    def test_free_votes_require_distinct_viewpoints(self):
        occupied = np.zeros((8, 3, 3), bool)
        votes = np.zeros_like(occupied, dtype='u1')
        last = np.full(occupied.shape, -1, dtype='i4')
        for group in [0, 0, 1]:
            integrate(np.array([[.5, 1.5, 1.5]]), np.array([[6.5, 1.5, 1.5]]),
                      np.zeros(3), 1., occupied, votes, last, np.array([group]))
            self.assertEqual(votes[3, 1, 1], group + 1)
        self.assertTrue(occupied[6, 1, 1])
        self.assertFalse(votes[7, 1, 1])

    def test_independent_ranges_replace_legacy_pairs(self):
        with tempfile.TemporaryDirectory() as folder:
            run = Path(folder)
            frames = [dict(tof_raw='104', pi_tof_read_end_ns='10', pi_encoder_callback_ns='20')]
            self.assertEqual(range_samples(run, frames)[0], [(10, 1.)])
            (run/'range.csv').write_text('tof_raw,pi_read_start_ns,pi_read_end_ns\n208,40,60\n8888,60,70\n')
            self.assertEqual(range_samples(run, frames)[0], [(50, 2.)])
