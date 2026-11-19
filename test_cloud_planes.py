import unittest

import cv2
import numpy as np

from cloud_planes import build_planes


def fit(points, gap=.22):
    return build_planes(dict(units="m", points=points.tolist(),
                             landmark_views={"frame-a":list(range(len(points)))}), gap)


class PlaneTests(unittest.TestCase):
    def test_clouds_produce_one_polygon_each_with_true_normals(self):
        uv = np.array([(x/10,y/10) for x in range(10) for y in range(10)])
        a = np.column_stack((uv,np.zeros(len(uv))))
        b = np.column_stack((np.full(len(uv),3),uv))
        c = np.column_stack((uv[:,0]+5, uv[:,1], .55*uv[:,0]))
        points = np.vstack((a,b,c,[[40,-10,8]]))
        rng = np.random.default_rng(42)
        points += rng.normal(0,.002,points.shape)
        mesh = fit(points)
        self.assertEqual(len(mesh['patches']), 3)
        for patch, expected in zip(sorted(mesh['patches'], key=lambda p:min(p['landmarks'])),
                                   ([0,0,1],[1,0,0],[-.55,0,1])):
            normal = np.asarray(patch['normal'])
            self.assertGreater(abs(normal @ expected)/np.linalg.norm(expected), .999)
            self.assertGreaterEqual(len(patch['landmarks']), 94)
            self.assertLessEqual(len(patch['vertices']), 8)
            self.assertNotIn(300, patch['landmarks'])
            self.assertLess(np.ptp(np.asarray(patch['vertices']) @ normal), 1e-8)

    def test_hole_gap_and_timing(self):
        points = np.array([(x*.1,y*.1,0) for x in range(21) for y in range(21)
                           if not (6<=x<=14 and 6<=y<=14)])
        other = points + [4,0,0]
        model = dict(units='m', points=np.vstack((points,other)).tolist(), path_frames=['A','B','C'],
                     landmark_views={'A':list(range(len(points))), 'B':list(range(len(points),2*len(points)-1))})
        faces = sorted(build_planes(model,.24)['patches'],key=lambda p:min(p['landmarks']))
        self.assertEqual([p['first'] for p in faces],['A','C'])
        for face, center in zip(faces, [(1,1),(5,1)]):
            self.assertEqual(len(face['holes']),1)
            hole = np.asarray(face['holes'][0], dtype=np.float32)[:,:2].copy()
            self.assertGreater(cv2.pointPolygonTest(hole,center,False),0)

    def test_line_has_no_surface(self):
        self.assertFalse(fit(np.array([[t/50,0,0] for t in range(100)]))['patches'])

    def test_range_moves_only_a_supported_plate(self):
        points = [[x/10,y/10,1] for x in range(-5,6) for y in range(-5,6)]
        poses = [dict(frame=str(i),xyz=[i*.05,0,0],rotation=np.eye(3).tolist()) for i in range(4)]
        model = dict(units='m',points=points,poses=poses)
        observations = {p['frame']:dict(range=1.02) for p in poses}
        mesh = build_planes(model,.25,observations)
        self.assertEqual(mesh['indoor']['range_adjusted'],1)
        self.assertEqual(mesh['indoor']['range_matches'],4)
        face = mesh['patches'][0]
        np.testing.assert_allclose(np.array(face['vertices'])[:,2],1.01)
        for obs in observations.values():
            obs['range'] = .5
        mesh = build_planes(model,.25,observations)
        self.assertEqual(mesh['indoor']['range_matches'],0)
        np.testing.assert_allclose(np.array(mesh['patches'][0]['vertices'])[:,2],1.)
