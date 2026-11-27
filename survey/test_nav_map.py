"""Checks the drone's map from a walk (nav_map.py): the file reads back cell for cell, merging keeps what's occupied,
the floor is found under the start, and the lines that send it."""
import base64
import unittest
import zlib

import numpy as np

import nav_map as nm


class NavMapTest(unittest.TestCase):
    def test_round_trip(self):
        rng = np.random.default_rng(3)
        for shape in [(23, 17, 9), (1, 1, 1), (5, 1, 7), (40, 30, 12)]:
            c = rng.integers(0, 3, shape).astype("u1")
            c[:, :, 1:] = np.where(rng.random(shape)[:, :, 1:] < 0.6, c[:, :, :1], c[:, :, 1:])   # layers repeat
            data = nm.encode(0.1, (-1, 2, 0.5), c)
            res, origin, back = nm.decode(data)
            self.assertAlmostEqual(res, 0.1, places=6)
            self.assertEqual(origin, (-1.0, 2.0, 0.5))
            self.assertTrue(np.array_equal(back, c), shape)

    def test_small_room(self):
        c = np.full((60, 40, 20), nm.FREE, "u1")
        c[40:42] = nm.OCCUPIED
        c[55:] = nm.UNKNOWN
        data = nm.encode(0.1, (0, 0, 0), c)
        self.assertLess(len(data), 120)                   # rows and layers repeat: a few bytes
        self.assertTrue(np.array_equal(nm.decode(data)[2], c))

    def test_damaged(self):
        data = bytearray(nm.encode(0.1, (0, 0, 0), np.ones((4, 4, 4), "u1")))
        data[len(data) // 2] ^= 1
        with self.assertRaises(ValueError):
            nm.decode(bytes(data))

    def test_coarsen(self):
        c = np.full((4, 4, 4), nm.FREE, "u1")
        c[0, 0, 0] = nm.OCCUPIED                          # one occupied cell: the whole block is
        c[2:, 2:, 2:] = nm.UNKNOWN                        # a block all unknown stays unknown
        c[2, 0, 0] = nm.UNKNOWN                           # a block mostly free is free
        k = nm.coarsen(c, 2, 0.5)
        self.assertEqual(k.shape, (2, 2, 2))
        self.assertEqual(k[0, 0, 0], nm.OCCUPIED)
        self.assertEqual(k[1, 1, 1], nm.UNKNOWN)
        self.assertEqual(k[1, 0, 0], nm.FREE)
        self.assertEqual(nm.coarsen(np.zeros((3, 3, 3), "u1"), 2).shape, (2, 2, 2))   # padded with unknown

    def test_floor_and_lines(self):
        c = np.full((10, 10, 10), nm.FREE, "u1")
        c[:, :, :3] = nm.OCCUPIED                         # a floor 0.3 m thick from z −1
        self.assertAlmostEqual(nm.floor_below(c, 0.1, (-0.5, -0.5, -1.0)), -0.7, places=6)
        data = nm.encode(0.1, (0, 0, 0), c)
        ls = nm.lines(data, 0.35, (0, 0, -0.65, 90))
        self.assertTrue(ls[0].startswith(f"file map.dmp {len(data)} "))
        pieces = b"".join(base64.b64decode(ln.split()[2]) for ln in ls if ln.startswith("fd "))
        self.assertEqual(pieces, data)
        self.assertTrue(all(len(ln) <= 57 for ln in ls))  # each fits one radio text command
        self.assertIn("mapguard 0.35", ls)
        self.assertIn("mapanchor 0.000 0.000 -0.650 90.0", ls)
        self.assertEqual(int(ls[0].split()[3], 16), zlib.crc32(data))


if __name__ == "__main__":
    unittest.main()
