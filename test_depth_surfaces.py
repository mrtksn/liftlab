import unittest

import numpy as np

from depth_surfaces import planes, voxel_skin


class DepthSurfaceTests(unittest.TestCase):
    def test_plane_orientation_and_gap_in_relative_space(self):
        y,z = np.mgrid[:15,:15] * .03
        patch = np.column_stack((np.ones(y.size),y.ravel(),z.ravel()))
        points = np.vstack((patch,patch+[0,1,0]))
        faces = planes(points,.04,'relative')
        self.assertGreaterEqual(len(faces),2)
        for face in faces:
            self.assertGreater(abs(face['normal'][0]),.99)
            self.assertLess(np.ptp(np.array(face['vertices'])[:,1]),.5)

    def test_voxel_envelope_keeps_distant_clusters_separate(self):
        x,y = np.mgrid[:12,:12] * .05
        patch = np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size)))
        faces = voxel_skin(np.vstack((patch,patch+[2,0,0])),.05,'m')
        self.assertTrue(faces)
        self.assertTrue(all(np.ptp(np.array(f['vertices'])[:,0])<1 for f in faces))
