import tempfile
from pathlib import Path
import unittest

import numpy as np
from gaussian_mesh import (CX, CY, FOCAL, HEIGHT, WIDTH, composite, confirm_depths,
                           fuse, ray_ranges, saved_range_evidence, write_ply)


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

    def test_saved_mesh_evidence_pairs_registered_fresh_rays(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            positions = [0, 4, 1, 0, 0]
            poses = [dict(frame=f'{i:07}.jpg', xyz=[x,0,0], rotation=np.eye(3).tolist())
                     for i,x in enumerate(positions)]
            poses[2]['method'] = 'interpolated'
            (run/'timestamps.csv').write_text('frame,tof_raw,pi_encoder_callback_ns,pi_tof_read_end_ns\n'+
                ''.join(f"{p['frame']},{8888 if i==3 else 104},100,{90000100 if i==4 else 110}\n"
                        for i,p in enumerate(poses)))
            mesh = run/'mesh.ply'
            write_ply(mesh, np.array([[-2.,-2.,2],[2.,-2.,2],[0.,2.,2]]), np.array([[0,1,2]]))
            model = dict(units='m', poses=poses[::-1])
            result = saved_range_evidence(run,model,mesh)
            first, second = result['readings']
            self.assertFalse(first['validation'])
            self.assertEqual(first['endpoint'],[0,0,1])
            self.assertEqual(first['intersection'],[0,0,2])
            self.assertEqual(first['mesh_m'],2)
            self.assertEqual(second['frame'],'0000001.jpg')
            self.assertTrue(second['validation'])
            self.assertFalse(second['hit'])
            self.assertIsNone(second['intersection'])
            self.assertEqual(result['summary']['eligible'],1)
            model['units'] = 'relative'
            relative = saved_range_evidence(run,model,mesh)
            self.assertIsNone(relative['readings'][0]['endpoint'])
            self.assertIn('reason',relative['summary'])

    def test_repeated_pose_cannot_confirm_depth(self):
        depth = np.ones((HEIGHT,WIDTH),'f')
        accepted = confirm_depths([depth]*3, np.zeros((3,3)), np.tile(np.eye(3),(3,1,1)), 1.)
        self.assertFalse(any(image.any() for image in accepted))
