import json
import unittest
from pathlib import Path
import numpy as np
from rgbd_geometry.geometry import Primitive, SHAPES, footprint_box, fit_table, TableFrame
from rgbd_geometry.synthetic import render, scene_camera
from rgbd_geometry.vision import segment, View, ModelFitter, Tracker, color_estimate

ROOT = Path(__file__).resolve().parents[1]


class PipelineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))
        cls.cfg["max_fit_points"] = 350
        cls.fitter = ModelFitter(cls.cfg)

    def test_all_six_shapes_with_depth_noise(self):
        dimensions = {
            "cube": [.09], "cuboid": [.13, .075, .055],
            "triangular_pyramid": [.12, .10, .13],
            "triangular_prism": [.12, .10, .09], "cone": [.11, .14], "cylinder": [.11, .10]}
        for shape in SHAPES:
            with self.subTest(shape=shape):
                model = Primitive(shape)
                params = np.array([0, 0, .28] + dimensions[shape])
                frame, masks, table = render([(model, params, [35, 40, 215])], holes=.06)
                detections = segment(frame.rgb, frame.depth, frame.intrinsics, table, self.cfg)
                self.assertEqual(len(detections), 1)
                # Exact RGB segmentation tests the estimator without attributing
                # missing-depth rim erosion to model fitting.
                detections[0].mask = masks[0]
                observation = self.fitter.fit(detections[0], View(masks[0], frame.intrinsics, table))
                expected, _ = footprint_box(model.vertices(params))
                error = float(np.max(np.abs(observation["dimensions"]-expected)))
                print(shape, observation["shape"], round(observation["score"], 3),
                      round(observation["margin"], 3), "dimension_error_mm", round(error*1000, 2), flush=True)
                self.assertEqual(observation["shape"], shape)
                self.assertLess(error, .014)

    def test_support_plane(self):
        frame, _, known = render([], noise=.001)
        xyz = frame.intrinsics.deproject(frame.depth)
        plane = fit_table(xyz.reshape(-1, 3), threshold=.004)
        angle = np.arccos(np.clip(plane.basis[:, 2] @ known.basis[:, 2], -1, 1))
        self.assertLess(angle, .015)
        # Origins may differ within the same plane; compare the normal offset.
        self.assertLess(abs((plane.origin-known.origin) @ known.basis[:, 2]), .003)

    def test_color_red_hue_wrap_and_black(self):
        import cv2
        hsv = np.zeros((20, 20, 3), np.uint8)
        hsv[:, :10] = [1, 230, 220]
        hsv[:, 10:] = [179, 230, 220]
        mask = np.ones((20, 20), np.uint8)
        self.assertEqual(color_estimate(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), mask)["name"], "red")
        self.assertEqual(color_estimate(np.zeros((20, 20, 3), np.uint8), mask)["name"], "black")

    def test_tracker_unknown_and_expiry(self):
        tracker = Tracker(self.cfg)
        scores = {s: .2 for s in SHAPES}
        scores["cube"] = .9
        observation = dict(shape="cube", scores=scores, dimensions=np.array([.1]*3), center=np.array([0, 0, .05]),
                           quality=1., flags=[], color={"name": "red", "confidence": .9}, orientation="upright",
                           metrics={}, surface_error_m=.001, features={}, box=np.zeros((8, 3)), mask=np.ones((10, 10)))
        self.assertEqual(tracker.update([observation], 0)[0]["shape"], "unknown")
        tracker.update([observation], .1)
        stable = tracker.update([observation], .2)[0]
        self.assertEqual(stable["shape"], "cube")
        # Slow inference still observes the same object; its ID must stay stable.
        self.assertEqual(tracker.update([observation], 1.3)[0]["id"], stable["id"])
        tracker.update([], 2.3)
        self.assertEqual(len(tracker.tracks), 0)
        observation["flags"] = ["image_border_truncated"]
        tracker.update([observation], 3)
        tracker.update([observation], 3.1)
        self.assertEqual(tracker.update([observation], 3.2)[0]["shape"], "unknown")

    def test_rotated_and_lying_shapes(self):
        cases = [("triangular_pyramid", "upright", [0, 0, -2.3, .12, .10, .13]),
                 ("triangular_prism", "lying", [0, 0, 1.1, .11, .08, .15]),
                 ("cylinder", "lying", [0, 0, -1.0, .09, .16])]
        for shape, orientation, values in cases:
            with self.subTest(shape=shape, orientation=orientation):
                self.fitter.cache = []
                model, params = Primitive(shape, orientation), np.array(values)
                frame, masks, table = render([(model, params, [40, 210, 40])])
                detection = segment(frame.rgb, frame.depth, frame.intrinsics, table, self.cfg)[0]
                detection.mask = masks[0]
                result = self.fitter.fit(detection, View(masks[0], frame.intrinsics, table))
                print("rotated", shape, result["shape"], result["orientation"], round(result["score"], 3), flush=True)
                self.assertEqual(result["shape"], shape)
                self.assertEqual(result["orientation"], orientation)
                expected, _ = footprint_box(model.vertices(params))
                self.assertLess(np.max(np.abs(result["dimensions"]-expected)), .015)

    def test_secondary_view_and_warm_start(self):
        intr, table = scene_camera()
        eye = np.array([.40, -.52, .26])
        forward = -eye/np.linalg.norm(eye)
        right = np.cross(forward, [0, 0, 1.])
        right /= np.linalg.norm(right)
        down = np.cross(forward, right)
        camera_to_world = np.column_stack((right, down, forward))
        second_table = TableFrame(-eye @ camera_to_world, camera_to_world.T)
        rotation = second_table.basis @ table.basis.T
        translation = second_table.origin-table.origin @ rotation.T
        model, params = Primitive("cone"), np.array([0, 0, .3, .12, .15])
        frame, masks, _ = render([(model, params, [40, 40, 220])], intr, table)
        _, second_masks, _ = render([(model, params, [40, 40, 220])], intr, second_table)
        detection = segment(frame.rgb, frame.depth, intr, table, self.cfg)[0]
        primary = View(masks[0], intr, table)
        secondary = View(second_masks[0], intr, table, rotation, translation)
        self.fitter.cache = []
        result = self.fitter.fit(detection, primary, secondary)
        self.assertEqual(result["shape"], "cone")
        self.assertGreater(result["metrics"]["secondary_rgb"], .85)
        self.assertNotIn("single_view_hidden_dimensions_inferred", result["flags"])
        result2 = self.fitter.fit(detection, primary, secondary)
        self.assertEqual(result2["shape"], "cone")

    def test_multitarget_segmentation_and_unique_ids(self):
        objects = [(Primitive("cube"), np.array([x, 0, 0, .07]), [30, 40, 210]) for x in (-.15, .15)]
        frame, _, table = render(objects)
        detections = segment(frame.rgb, frame.depth, frame.intrinsics, table, self.cfg)
        self.assertEqual(len(detections), 2)

    def test_first_hit_rejects_wrong_back_surface(self):
        model, params = Primitive("cube"), np.array([0, 0, 0, .1])
        _, table = scene_camera()
        vertices = model.vertices(params)
        camera_points = table.to_camera(vertices)
        hit, valid = model.first_hit(camera_points, params, table)
        # Some back vertices lie on a model surface but are behind the first visible surface.
        self.assertTrue(np.any(valid & (hit < .95)))


if __name__ == "__main__":
    unittest.main()
