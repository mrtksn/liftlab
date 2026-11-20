import base64
import tempfile
import unittest
from pathlib import Path

import numpy as np

from ai_surfaces import H, W, cached_frames, metric_fit, preview_png, projection, save_predictions


class DepthGeometryTests(unittest.TestCase):
    def test_disparity_is_scaled_and_shifted(self):
        pred = np.linspace(10,240,75)
        z = 1/(.005*pred + .13)
        z[3::13] *= 2
        np.testing.assert_allclose(metric_fit(pred,z), [.005,.13], rtol=.02)
        self.assertIsNone(metric_fit(np.zeros(75),z))
        self.assertIsNone(metric_fit(pred,1/(2-.005*pred)))

    def test_projection_uses_camera_rotation(self):
        pose = dict(xyz=[1,2,3],rotation=[[0,0,1],[1,0,0],[0,1,0]])
        pixels,z,inside = projection(np.array([[3,2,3],[0,2,3]]),pose)
        self.assertEqual(pixels[0].tolist(), [W//2,H//2])
        self.assertEqual(z.tolist(),[2,-1])
        self.assertEqual(inside.tolist(),[True,False])

    def test_prediction_cache_validated_before_replacement(self):
        frames=['0000001.jpg','0000002.jpg']
        model=dict(feature_index='features-'+'a'*32+'.npz', poses=[dict(frame=f,rotation=np.eye(3)) for f in frames])
        raw=base64.b64encode(np.zeros((H,W),np.uint8)).decode()
        with tempfile.TemporaryDirectory() as directory:
            run=Path(directory)
            save_predictions(run,model,dict.fromkeys(frames,raw))
            invalid=dict.fromkeys(frames,raw)
            invalid[frames[1]]='AA=='
            with self.assertRaises(ValueError):
                save_predictions(run,model,invalid)
            self.assertEqual(cached_frames(run,model['feature_index']),frames)
            self.assertTrue(preview_png(run,model['feature_index'],frames[0]).startswith(b'\x89PNG'))
