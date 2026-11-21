import unittest

import numpy as np

from depth_cloud import align_depth


class CameraDepthAlignmentTests(unittest.TestCase):
    def test_relative_scale_is_estimated_from_landmarks(self):
        y,x = np.mgrid[0:72,0:96]
        raw = np.float32(1+x/96+y/72)
        x,y = x[4::10,4::10],y[4::10,4::10]
        z = raw[y,x]*3
        camera = np.stack(((x+.5-48)*z/64,(y+.5-36)*z/64,z),axis=-1).reshape(-1,3)
        aligned = align_depth(raw,'lingbot','relative',camera)
        np.testing.assert_allclose(aligned,raw*3,rtol=1e-5)
        # Metric Apple depth preserves measured map scale.
        self.assertIs(align_depth(raw,'apple','m',camera),raw)
        self.assertIsNone(align_depth(raw,'anything','relative',camera[:3]))
