import json
from pathlib import Path
import tempfile
import unittest

from fusion import load_observations
from node_params import (CAMERA_MAP_PARAMS, DEVICE_PARAMS, SENSOR_PARAMS, defaults, device_settings, pairing_ns,
                         sensor_settings, tof_metres, validate)


class NodeParamTests(unittest.TestCase):
    def test_defaults_match_previous_fixed_values(self):
        sensors = defaults(SENSOR_PARAMS)
        self.assertEqual(tof_metres(104, sensors), 1)
        self.assertIsNone(tof_metres(0, sensors))
        self.assertIsNone(tof_metres(250, sensors))
        self.assertEqual(tof_metres(249, sensors), 249/104)
        self.assertEqual(pairing_ns(sensors), 80_000_000)
        camera = defaults(CAMERA_MAP_PARAMS)
        self.assertEqual((camera["frame_stride"], camera["max_views"], camera["sift_features"]), (2, 400, 2500))

    def test_validate_rejects_bad_values(self):
        for bad in ({"sift_features": 1.5}, {"sift_features": 10}, {"loop_matches": 1},
                    {"unknown": 1}, {"focal_guess_px": float("nan")}, {"max_views": True}):
            with self.assertRaises(ValueError):
                validate(CAMERA_MAP_PARAMS, bad)
        with self.assertRaises(ValueError):
            validate(SENSOR_PARAMS, {"tof_min_raw": 300, "tof_max_raw": 200})
        self.assertEqual(validate(CAMERA_MAP_PARAMS, {"max_views": 120.0})["max_views"], 120)

    def test_walk_settings_override_survey_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            run = root/"walk"
            run.mkdir()
            (root/"sensors.json").write_text(json.dumps({"tof_counts_per_metre": 50}))
            self.assertEqual(sensor_settings(run)["tof_counts_per_metre"], 50)
            (run/"sensors.json").write_text(json.dumps({"tof_counts_per_metre": 200, "tof_max_raw": 1000}))
            (run/"timestamps.csv").write_text("frame,pi_encoder_callback_ns,tof_raw\na,500000000,400\n")
            self.assertEqual(load_observations(run)["a"]["range"], 2)
            (run/"sensors.json").write_text("not json")
            self.assertEqual(sensor_settings(run)["tof_counts_per_metre"], 50)


    def test_device_settings(self):
        device = defaults(DEVICE_PARAMS)
        self.assertEqual((device["camera_width"], device["camera_height"], device["tof_address"]), (640, 480, 0x52))
        self.assertEqual(validate(DEVICE_PARAMS, {"pi_host": "pi@10.0.0.2"})["pi_host"], "pi@10.0.0.2")
        for bad in ({"pi_host": "pi@host; rm -rf ~"}, {"calibration_file": "../escape.json"}, {"tof_address": 0x30},
                    {"camera_fps": 0}):
            with self.assertRaises(ValueError):
                validate(DEVICE_PARAMS, bad)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.assertEqual(device_settings(root), device)
            (root/"device.json").write_text(json.dumps({"camera_fps": 15}))
            self.assertEqual(device_settings(root)["camera_fps"], 15)
            (root/"device.json").write_text(json.dumps({"camera_fps": -1}))
            self.assertEqual(device_settings(root), device)


if __name__ == "__main__":
    unittest.main()
