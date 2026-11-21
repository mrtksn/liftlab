import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from replay_depth import gaussian_poses, inventory, output, preview


class FrameDepthTests(unittest.TestCase):
    def test_pose_gap_uses_capture_time_and_marks_untracked_endpoints(self):
        names = [f'{i:07d}.jpg' for i in range(5)]
        model = {'poses': [
            {'frame': names[i], 'rotation': np.eye(3), 'xyz': [x, 0, 0]}
            for i, x in [(1, 0), (3, 10)]
        ]}
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            (run/'timestamps.csv').write_text('frame,pi_camera_relative_us\n' +
                ''.join(f'{f},{t}\n' for f, t in zip(names,[0,100,110,200,300])))
            poses = list(gaussian_poses(run, names, model))
        self.assertEqual([p[3] for p in poses], ['Nearest saved pose', 'Saved pose',
                         'Interpolated pose', 'Saved pose', 'Nearest saved pose'])
        np.testing.assert_allclose([p[1][0] for p in poses], [0,0,1,10,10])

    def test_missing_depth_stays_black(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'depth.npy'
            np.save(path, [[0, np.nan, 1, 8]])
            png = cv2.imdecode(np.frombuffer(preview(path, 'apple'),np.uint8), cv2.IMREAD_COLOR)
        self.assertFalse(png[0,:2].any())
        self.assertTrue(png[0,2:].any())
        self.assertFalse(np.array_equal(png[0,2], png[0,3]))

    def test_batch_status_separates_missing_model_and_ready_archive(self):
        with tempfile.TemporaryDirectory() as tmp:
            run = Path(tmp)
            index = 'features-' + 'a'*32 + '.npz'
            dest = output(run, index, 'apple')
            dest.mkdir(parents=True)
            (dest/'complete.json').write_text('{}')
            results = inventory(run, index)
            self.assertTrue(results['apple']['complete'])
            self.assertFalse(results['gaussian']['available'])
            self.assertTrue(results['lingbot']['available'])
            with self.assertRaises(ValueError):
                output(run, index, '../frames')
