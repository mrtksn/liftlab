import unittest
import numpy as np

from seed_surfaces import MAX_EDGE, build_surfaces


def survey(points, frames, units="m"):
    return dict(units=units, points=points,
                poses=[dict(frame=f, xyz=eye) for f, eye, ids in frames],
                landmark_views={f: ids for f, eye, ids in frames})


class SeedGrowthTest(unittest.TestCase):
    def test_seed_pair_then_triangle_and_distance_cutoff(self):
        points = [[0, 0, 1], [.12, 0, 1], [0, .12, 1], [2, 0, 1]]
        for n in range(1, 5):
            mesh = build_surfaces(survey(points, [('1', [0, 0, 0], list(range(n)))]))
            self.assertEqual(len(mesh["patches"]), int(n >= 3))
            self.assertEqual(mesh["seeds"], int(n in (1, 4)))
            self.assertEqual(mesh["open_pairs"], int(n == 2))
        a,b,c = np.array(mesh["patches"][0]["vertices"])
        self.assertGreater(np.cross(b-a, c-a) @ -a, 0)

    def test_collinear_or_overlong_third_edge_cannot_create_face(self):
        for third in [[.05, 0, 1], [-.14, .05, 1]]:
            m = survey([[0,0,1], [.13,0,1], third], [('1',[0,0,0],[0,1,2])])
            self.assertEqual(build_surfaces(m)["patches"], [])

    def test_custom_cutoff_smaller_and_larger_than_default(self):
        model = survey([[0,0,1], [.12,0,1], [0,.19,1]], [('1',[0,0,0],[0,1,2])])
        self.assertFalse(build_surfaces(model, .10)["patches"])
        self.assertFalse(build_surfaces(model)["patches"])
        mesh = build_surfaces(model, .25)
        self.assertEqual(mesh["max_edge_m"], .25)
        self.assertEqual(len(mesh["patches"]), 1)

    def test_front_occludes_through_back_face_then_retry_from_other_view(self):
        points = [[-.07,-.05,1], [.07,-.05,1], [0,.08,1], [0,0,1.3]]
        frames = [('0',[0,0,0],[0,1,2]), ('1',[0,0,0],[3])]
        model = survey(points,frames)
        mesh = build_surfaces(model)
        self.assertEqual(mesh["deferred_landmarks"],[3])
        self.assertEqual(model["points"],points)
        frames.append(('2',[1,0,1.3],[3]))
        later = build_surfaces(survey(points,frames))
        self.assertEqual(later["accepted"],4)
        self.assertEqual(later["deferred_landmarks"],[])

    def test_refine_and_grow_across_multiple_edges(self):
        points = [[0,0,1], [.12,0,1], [0,.12,1], [.04,.04,1], [.12,.12,1], [.22,.1,1]]
        frames = [(str(i),[0,0,0],[i]) for i in range(len(points))]
        mesh = build_surfaces(survey(points,frames))
        faces = mesh["patches"]
        self.assertEqual(len(faces),5)
        self.assertEqual([(p["first"], p["until"]) for p in mesh["retired"]], [("2","3")])
        self.assertFalse(any(set(f["landmarks"])=={0,1,2} for f in faces))
        for face in faces:
            v = np.array(face["vertices"])
            self.assertLessEqual(max(np.linalg.norm(v-np.roll(v,1,axis=0),axis=1)), MAX_EDGE)
        self.assertEqual(build_surfaces(survey(points,frames,'relative'))['status'],'needs_scale')
