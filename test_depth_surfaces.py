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

    def test_balloon_contacts_wall_and_closes_over_missing_data(self):
        from balloon import build_balloon, closed_sphere
        y,z = np.mgrid[-1.2:1.2:.04,-1.2:1.2:.04]
        wall = np.column_stack((np.ones(y.size),y.ravel(),z.ravel()))
        wall = wall[np.linalg.norm(wall[:,1:],axis=1)>.18]
        poses = [dict(xyz=p,rotation=np.eye(3)) for p in
                 ([0,-.6,0],[0,.2,0],[-.45,.2,.15])]
        patches,stats = build_balloon(wall,poses,.08,1.7)
        tri = np.array([p['vertices'] for p in patches])
        verts,ids = np.unique(tri.reshape(-1,3),axis=0,return_inverse=True)
        self.assertTrue(closed_sphere(verts,ids.reshape(-1,3)))
        front = verts[(abs(verts[:,1])<.4)&(abs(verts[:,2])<.4)&(verts[:,0]>0)]
        self.assertTrue(len(front))
        self.assertAlmostEqual(np.median(front[:,0]),1,delta=.12)
        self.assertGreater(stats['supported'],0)
        self.assertLess(stats['supported'],len(tri))
