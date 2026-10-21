import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
import cv2
from rgbd_geometry.configuration import DEFAULTS, load_config, validate_config, runtime_config, save_config, apply_preset
from rgbd_geometry.geometry import Primitive, fit_table, camera_rays, Intrinsics, footprint_box
from rgbd_geometry.synthetic import render
from rgbd_geometry.vision import ModelFitter, Tracker, View, segment, color_estimate
from app import save_workspace, load_workspace, calibrate_workspace


def observation(dimensions=(.1, .1, .1), flags=None, orientation="upright", quality=1., shape="cube"):
    scores = {s: .1 for s in ("cube", "cuboid", "triangular_pyramid", "triangular_prism", "cone", "cylinder")}
    scores[shape] = .95
    return dict(shape=shape, scores=scores, dimensions=np.array(dimensions), center=np.array([0, 0, .05]),
                quality=quality, flags=flags or [], color={"name": "red", "confidence": .9}, orientation=orientation,
                metrics={}, surface_error_m=.001, features={}, box=np.zeros((8, 3)), mask=np.ones((10, 10)))


class ConfigurationTests(unittest.TestCase):
    def test_invalid_values_rejected(self):
        cases = [{"max_fit_points": 0}, {"max_fit_points": True}, {"surface_noise_m": float("nan")},
                 {"weights": {"surface": 0, "depth": 0, "rgb": 0}}, {"analysis_hz": -1},
                 {"min_confidence": 1.1}, {"cube_relative_tolerance": -1}, {"refine_top_k": 9},
                 {"temporal_depth_filter": "false"}, {"roi": [0, 0, 1000, 1000]},
                 {"color": {"hue_boundaries": [9, 23, 20, 85, 100, 135, 172]}}, {"typo": 1}]
        cases += [{"color": {"min_color_fraction": "bad"}}, {"color": {"min_color_fraction": float("nan")}},
                  {"weights": {"surface": 1e308}}, {"weights": {"rgb": 1e-320}}]
        for changes in cases:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                validate_config(changes)

    def test_presets_and_atomic_roundtrip(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"config.json"
            for name in ("fast", "balanced", "precise"):
                config = apply_preset(validate_config({}), name)
                save_config(path, config)
                self.assertEqual(load_config(path), config)
                invalid = copy.deepcopy(config)
                invalid["min_confidence"] = 2
                with self.assertRaises(ValueError):
                    save_config(path, invalid)
                self.assertEqual(load_config(path), config)

    def test_runtime_snapshot_and_capture_guard(self):
        config = validate_config({})
        changed = copy.deepcopy(config)
        changed["weights"]["rgb"] = .8
        snapshot = runtime_config(config, changed)
        changed["weights"]["rgb"] = 0
        self.assertEqual(snapshot["weights"]["rgb"], .8)
        changed["width"] = config["width"] + 208
        with self.assertRaises(ValueError):
            runtime_config(config, changed)
        changed = copy.deepcopy(config)
        changed["roi"] = [0, 0, 600, 300]
        with self.assertRaises(ValueError):
            runtime_config(config, changed, (424, 240))


class RobustnessTests(unittest.TestCase):
    def setUp(self):
        self.config = validate_config({})

    def test_bad_observation_does_not_pollute_history(self):
        tracker = Tracker(self.config)
        for index in range(3):
            result = tracker.update([observation()], index*.2)[0]
        self.assertEqual(result["shape"], "cube")
        bad = observation(dimensions=(.9, .9, .9), flags=["poor_model_fit"])
        result = tracker.update([bad], .6)[0]
        self.assertEqual(result["shape"], "unknown")
        self.assertEqual(len(tracker.tracks[result["id"]]["history"]), 3)
        self.assertEqual(result["stable_observations"], 0)
        for index in range(3):
            result = tracker.update([observation()], .8+index*.2)[0]
        self.assertEqual(result["dimensions_mm"], [100., 100., 100.])

    def test_orientation_switch_and_size_jump_reacquire(self):
        tracker = Tracker(self.config)
        for index in range(3):
            tracker.update([observation(shape="cylinder", dimensions=(.12, .12, .20))], index*.1)
        result = tracker.update([observation(shape="cylinder", orientation="lying", dimensions=(.20, .12, .12))], .4)[0]
        self.assertEqual(result["shape"], "unknown")
        self.assertEqual(result["stable_observations"], 1)
        for index in range(2):
            result = tracker.update([observation(shape="cylinder", orientation="lying", dimensions=(.20, .12, .12))], .5+index*.1)[0]
        self.assertEqual(result["dimensions_mm"], [200., 120., 120.])
        result = tracker.update([observation(shape="cylinder", orientation="lying", dimensions=(.30, .18, .18))], .8)[0]
        self.assertEqual(result["shape"], "unknown")
        self.assertIn("dimensions_reacquiring", result["quality_flags"])

    def test_nan_inf_and_severe_holes(self):
        model, params = Primitive("cube"), np.array([0, 0, .2, .10])
        frame, masks, table = render([(model, params, [35, 40, 215])], holes=.4)
        frame.depth[5:10, 5:10] = np.nan
        frame.depth[15:20, 5:10] = np.inf
        detections = segment(frame.rgb, frame.depth, frame.intrinsics, table, self.config)
        self.assertEqual(len(detections), 1)
        self.assertTrue(np.isfinite(detections[0].points).all())
        detections[0].mask = masks[0]
        fit = ModelFitter(self.config).fit(detections[0], View(masks[0], frame.intrinsics, table))
        fit["color"] = color_estimate(frame.rgb, masks[0], self.config)
        tracker = Tracker(self.config)
        for index in range(4):
            result = tracker.update([fit], index*.2)[0]
        self.assertEqual(result["shape"], "unknown")
        self.assertIsNone(result["dimensions_mm"])

    def test_plane_rejects_collinear_points(self):
        points = np.c_[np.linspace(-.3, .3, 1000), np.zeros(1000), np.ones(1000)]
        with self.assertRaises(ValueError):
            fit_table(points)

    def test_uniform_brightness_does_not_expand_object(self):
        model, params = Primitive("cube"), np.array([0, 0, .3, .1])
        background, _, table = render([], holes=0)
        frame, _, _ = render([(model, params, [30, 50, 220])], holes=0)
        shifted = np.clip(frame.rgb.astype(int)+15, 0, 255).astype(np.uint8)
        first = segment(frame.rgb, frame.depth, frame.intrinsics, table, self.config, background.rgb)[0]
        second = segment(shifted, frame.depth, frame.intrinsics, table, self.config, background.rgb)[0]
        self.assertLess(abs(int(second.mask.sum())-int(first.mask.sum()))/first.mask.sum(), .04)

    def test_calibration_roundtrip_and_multiframe(self):
        frames = [render([], noise=.0018, seed=index)[0] for index in range(7)]
        table = calibrate_workspace(frames[-1], self.config, frames)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"table.data"
            save_workspace(path, frames[-1], table, frames[-1].rgb, frames[-1].rgb, "old")
            saved, bg, secondary = load_workspace(path, frames[-1].intrinsics, "new")
            self.assertTrue(np.allclose(table.origin, saved.origin))
            self.assertEqual(bg.shape, frames[-1].rgb.shape)
            self.assertIsNone(secondary)
            changed = copy.copy(frames[-1].intrinsics)
            changed.fx += 2
            with self.assertRaises(ValueError):
                load_workspace(path, changed)

    def test_periodic_full_search_and_changed_shape(self):
        config = validate_config({"full_search_interval": 2})
        fitter = ModelFitter(config)
        modes = []
        for shape, values in [("cube", [.10])]*3 + [("cylinder", [.10, .1])]:
            frame, masks, table = render([(Primitive(shape), np.array([0, 0, .2]+values), [30, 40, 220])])
            detection = segment(frame.rgb, frame.depth, frame.intrinsics, table, config)[0]
            result = fitter.fit(detection, View(masks[0], frame.intrinsics, table))
            modes.append(result["search_mode"])
            self.assertEqual(result["shape"], shape)
        self.assertEqual(modes[:3], ["full", "temporal", "full"])

    def test_rays_cached_and_readonly(self):
        intr = Intrinsics(20, 15, 20, 20, 10, 7)
        self.assertIs(camera_rays(20, 15, 20, 20, 10, 7), camera_rays(20, 15, 20, 20, 10, 7))
        self.assertFalse(camera_rays(20, 15, 20, 20, 10, 7).flags.writeable)
        self.assertEqual(intr.deproject(np.ones((15, 20))).shape, (15, 20, 3))

    def test_triangle_equal_area_box_is_stable_under_rotation(self):
        model = Primitive("triangular_pyramid")
        dimensions = []
        for angle in np.linspace(-np.pi, np.pi, 30):
            dims, corners = footprint_box(model.vertices(np.array([0, 0, angle, .1, .095, .11])))
            dimensions.append(dims)
            self.assertEqual(corners.shape, (8, 3))
        self.assertLess(np.ptp(np.array(dimensions), axis=0).max(), .0001)


if __name__ == "__main__":
    unittest.main()
