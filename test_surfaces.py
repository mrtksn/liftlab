import unittest

import numpy as np

from surfaces import build_surfaces


class SurveySurfacesTest(unittest.TestCase):
    def test_coplanar_clusters_connect_without_spanning_opening(self):
        grid = np.arange(.08, 5, .18)
        x, y = np.meshgrid(grid, grid)
        points = np.column_stack([x.ravel(), y.ravel()])
        points = points[(points[:,0] < 1.9) | (points[:,0] > 3.1)]
        model = dict(units="m", poses=[], points=np.column_stack([points, np.zeros(len(points))]))
        # Use the same doorway at the density of the captured surface samples.
        model["points"] = (model["points"] * .35).tolist()
        faces = build_surfaces(model)["patches"]
        self.assertTrue(faces)
        x = np.array([p["vertices"] for p in faces])[:, :, 0]
        self.assertTrue((x < .665).any() and (x > 1.085).any())
        self.assertFalse(((x.min(axis=1) < .875) & (x.max(axis=1) > .875)).any())

    def test_unbiased_3d_planes_and_uniform_normalization(self):
        rng = np.random.default_rng(42)
        ramp_normal = np.array([.48, -.64, .60])
        normals = np.array([[0, 0, 1], [.8, .6, 0], ramp_normal])
        centres = np.array([[-3, 1, 0], [0, 0, 2], [3, 2, 3]])
        points = []
        for centre, normal in zip(centres, normals):
            _, _, axes = np.linalg.svd([normal], full_matrices=True)
            uv = rng.uniform(-.85, .85, (1800, 2))
            points.extend(centre + uv @ axes[1:] + rng.normal(0, .002, (len(uv), 1)) * normal)
        for scale, offset, gravity in [(1., 0., True), (19., 1200., False)]:
            model = dict(units="relative", poses=[], gravity_aligned=gravity,
                         points=np.asarray(points) * scale + offset)
            patches = build_surfaces(model)["patches"]
            triangles = (np.array([p["vertices"] for p in patches]) - offset) / scale
            for centre, normal in zip(centres, normals):
                local = np.linalg.norm(triangles.mean(axis=1) - centre, axis=1) < 1.3
                faces = triangles[local]
                self.assertGreater(len(faces), 10)
                self.assertLess(np.max(np.abs((faces-centre) @ normal)), .02)
                n = np.cross(faces[:, 1]-faces[:, 0], faces[:, 2]-faces[:, 0])
                self.assertTrue(np.all(abs(n @ normal) > .99 * np.linalg.norm(n, axis=1)))
            if gravity:
                self.assertEqual({p["orientation"] for p in patches}, {"level", "upright", "sloped"})
            else:
                self.assertFalse(any("orientation" in p for p in patches))

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
