import unittest

import numpy as np

from cloth_surfaces import build_cloth, ray_lattice


class ClothTests(unittest.TestCase):
    def test_first_supported_layer_and_replay(self):
        rays, faces = ray_lattice()
        rays = rays[faces[0]]
        # Same angular sectors contain a wall and a farther wall. The isolated
        # close return shares a sector with the near wall and must not shadow it.
        points = np.vstack((rays, 3*rays, .2*rays[0], [-1,0,0]))
        model = dict(units="m", points=points.tolist(),
                     poses=[dict(frame="0", xyz=[0,0,0]), dict(frame="1", xyz=[.011,0,0])],
                     landmark_views={"0":list(range(7)), "1":list(range(8))})
        mesh = build_cloth(model, .9)
        self.assertEqual(len(mesh["patches"]), 1)
        face = mesh["patches"][0]
        self.assertEqual(set(face["landmarks"]), {0,1,2})
        np.testing.assert_array_equal(face["vertices"], points[face["landmarks"]])
        self.assertEqual(face["first"], "1")
        self.assertEqual((mesh["isolated"], mesh["unconfirmed"]), (1,1))
        self.assertEqual(len(model["points"]), 8)
        self.assertFalse(build_cloth(model, .02)["patches"])
        model['poses'][1]['xyz'] = [0,0,0]
        self.assertFalse(build_cloth(model, .9)["patches"])

    def test_gap_and_depth_boundary(self):
        rays, faces = ray_lattice()
        near = rays[faces[0]]
        points = np.vstack((near, -near, [0,.1,.2]))
        model = dict(units="m", points=points.tolist(),
                     poses=[dict(frame="a", xyz=[0,0,0]), dict(frame="b", xyz=[.011,0,0])],
                     landmark_views={f:list(range(7)) for f in ('a','b')})
        mesh = build_cloth(model, 5)
        self.assertEqual({tuple(sorted(p["landmarks"])) for p in mesh["patches"]}, {(0,1,2),(3,4,5)})
        self.assertEqual(build_cloth(dict(model, units="relative"), 1)["status"], "needs_scale")
