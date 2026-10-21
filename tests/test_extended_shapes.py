import copy
import unittest
import numpy as np
from rgbd_geometry.configuration import validate_config
from rgbd_geometry.geometry import EXTRA_SHAPES, SHAPES, Primitive, model_bank
from rgbd_geometry.vision import Tracker


class ExtendedShapeTests(unittest.TestCase):
    def test_optional_bank_keeps_original_mode_and_has_valid_metric_models(self):
        self.assertEqual({model.shape for model in model_bank()}, set(SHAPES))
        self.assertEqual({model.shape for model in model_bank(EXTRA_SHAPES)}, set(SHAPES+EXTRA_SHAPES))
        for shape in EXTRA_SHAPES:
            model = Primitive(shape)
            params = np.array([0., 0., .2, .04] + ([] if shape == "sphere" else [.05]))
            vertices, normals, offsets = model.prepared(params)
            self.assertEqual(model.parameter_count, len(params))
            self.assertAlmostEqual(float(vertices[:, 2].min()), 0., places=8)
            self.assertAlmostEqual(float(vertices[:, 2].max()), .04 if shape == "sphere" else .05, places=8)
            self.assertTrue(np.isfinite(offsets).all())
            np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1., atol=1e-8)
        for value in (["bogus"], ["sphere", "sphere"], "sphere", [None]):
            with self.assertRaises(ValueError):
                validate_config({"extra_shapes": value})

    @staticmethod
    def observation(margin):
        scores = {shape: .2 for shape in SHAPES+EXTRA_SHAPES}
        scores["hexagonal_pyramid"], scores["cone"] = .88, .88-margin
        return dict(shape="hexagonal_pyramid", scores=scores, dimensions=np.array([.04, .035, .04]),
                    center=np.array([0., 0., .02]), orientation="upright", quality=.99, flags=[],
                    color={"name": "blue", "confidence": .99}, box=np.zeros((8, 3)), mask=np.ones((4, 4), np.uint8),
                    metrics={}, surface_error_m=.0005, features={})

    def test_score_window_accumulates_consistent_evidence_without_lowering_margin(self):
        config = validate_config({"extra_shapes": list(EXTRA_SHAPES), "score_smoothing_window": 7})
        tracker = Tracker(config)
        for index, margin in enumerate([.08, .055, .075, .055, .075, .055, .075]):
            result = tracker.update([self.observation(margin)], index*.5)[0]
        self.assertEqual(result["shape"], "hexagonal_pyramid")
        self.assertGreaterEqual(result["score_margin"], config["min_score_margin"])
        poor = self.observation(.08)
        poor["flags"] = ["poor_model_fit"]
        self.assertEqual(tracker.update([poor], 4.)[0]["shape"], "unknown")
        ambiguous = Tracker(config)
        for index in range(15):
            result = ambiguous.update([self.observation(.01)], index*.5)[0]
            self.assertEqual(result["shape"], "unknown")


if __name__ == "__main__":
    unittest.main()
