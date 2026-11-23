import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from gaussian_map import write_brush_image


class BrushInputTests(unittest.TestCase):
    def test_short_and_long_jpegs_preserve_decoded_pixels(self):
        rng = np.random.default_rng(7)
        for image in (np.zeros((288,384,3), np.uint8), rng.integers(256, size=(288,384,3), dtype=np.uint8)):
            original = cv2.imencode('.jpg', image)[1]
            with tempfile.TemporaryDirectory() as tmp:
                path = Path(tmp)/'frame.jpg'
                write_brush_image(path, image)
                self.assertGreaterEqual(path.stat().st_size, 16387)
                np.testing.assert_array_equal(cv2.imread(str(path)), cv2.imdecode(original, cv2.IMREAD_COLOR))
