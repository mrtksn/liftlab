import unittest

import numpy as np

from surfaces import build_surfaces, supported_footprint


class SurveySurfacesTest(unittest.TestCase):
    def test_coplanar_clusters_connect_without_spanning_opening(self):
        grid = np.arange(.08, 5, .18)
        x, y = np.meshgrid(grid, grid)
        points = np.column_stack([x.ravel(), y.ravel()])
        points = points[(points[:,0] < 1.9) | (points[:,0] > 3.1)]
        cells = supported_footprint(points, 1.1)
        self.assertTrue({(0,1),(1,1),(3,1),(4,1)}.issubset(cells))
        self.assertFalse(any(x==2 for x,_ in cells))

    def test_gravity_identifies_floor_beneath_path(self):
        rng = np.random.default_rng(71)
        floor = np.column_stack([rng.uniform(-2,2,(2200,2)), rng.normal(0,.006,2200)])
        ceiling = np.column_stack([rng.uniform(-2,2,(1400,2)), rng.normal(2.6,.006,1400)])
        table = np.column_stack([rng.uniform(-.48,.48,(600,2)), rng.normal(.8,.006,600)])
        path = [dict(xyz=[float(x), .5, 1.2]) for x in np.linspace(-1.5,1.5,20)]
        model = dict(units="m",points=np.vstack([ceiling,table,floor]).tolist(),
                     poses=path,gravity_aligned=True)
        mesh = build_surfaces(model)
        patches = [p for p in mesh["patches"] if p.get("kind")=="floor"]
        self.assertTrue(patches)
        self.assertTrue(all(abs(v[2])<.05 for p in patches for v in p["vertices"]))
        model["gravity_aligned"] = False
        self.assertNotIn("floor_plane", build_surfaces(model))
