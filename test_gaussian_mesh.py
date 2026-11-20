import unittest

import numpy as np
from gaussian_mesh import (CX, CY, FOCAL, HEIGHT, WIDTH, composite, confirm_depths,
                           fuse, ray_ranges)


class GaussianMeshTests(unittest.TestCase):
    def test_separated_layers_choose_one_return(self):
        uv = np.array([[50., 50.], [50., 50.]])
        covariance = np.array([[1.,0.,1.]]*2)
        depth = composite(uv, np.array([1.,3.]), covariance, np.array([.8,.95]), 10.)
        self.assertAlmostEqual(depth[50,50], 1., places=4)
        self.assertEqual(depth[0,0], 0.)

    def test_observed_wall_keeps_opening_after_simplification(self):
        y, x = np.indices((HEIGHT,WIDTH))
        offsets = [-.24, -.12, 0., .12, .24]
        eyes = np.column_stack([offsets, np.zeros((5,2))])
        rotations = np.tile(np.eye(3), (5,1,1))
        depths = []
        for eye in eyes:
            wall_x = eye[0] + (x-CX)*2/FOCAL
            wall_y = (y-CY)*2/FOCAL
            hole = (np.abs(wall_x)<.25) & (np.abs(wall_y)<.55)
            depths.append(np.where(hole, 0., 2.).astype('f'))
        accepted = confirm_depths(depths, eyes, rotations, 3.)
        vertices, faces, _, _, _ = fuse(accepted, eyes, rotations)
        self.assertLess(float(np.max(np.abs(vertices[:,2]-2))), .02)
        ranges = ray_ranges(vertices, faces, [[0,0,0], [.6,0,0]], [[0,0,1]]*2)
        self.assertTrue(np.isnan(ranges[0]))
        self.assertAlmostEqual(ranges[1], 2., places=2)

    def test_repeated_pose_cannot_confirm_depth(self):
        depth = np.ones((HEIGHT,WIDTH),'f')
        accepted = confirm_depths([depth]*3, np.zeros((3,3)), np.tile(np.eye(3),(3,1,1)), 1.)
        self.assertFalse(any(image.any() for image in accepted))
