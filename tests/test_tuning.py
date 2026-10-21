import tempfile
from pathlib import Path
import unittest
from rgbd_geometry.configuration import validate_config, save_config, load_config


class TuningPanelTests(unittest.TestCase):
    def setUp(self):
        try:
            from rgbd_geometry.tuning import TuningPanel
            self.folder = tempfile.TemporaryDirectory()
            self.path = Path(self.folder.name)/"config.json"
            self.config = validate_config({})
            save_config(self.path, self.config)
            self.panel = TuningPanel(self.config, self.path, visible=False)
        except (ImportError, RuntimeError) as error:
            self.skipTest(str(error))

    def tearDown(self):
        if hasattr(self, "panel"):
            self.panel.close()
        if hasattr(self, "folder"):
            self.folder.cleanup()

    def test_units_save_reload_and_profiles(self):
        self.panel.variables["min_object_height_m"].set(12)
        self.panel.variables["min_confidence"].set(70)
        self.panel.roi.set("[10, 20, 200, 150]")
        self.panel.apply()
        self.assertEqual(self.panel.pending["min_object_height_m"], .012)
        self.assertEqual(self.panel.pending["min_confidence"], .7)
        self.panel.save()
        self.assertEqual(load_config(self.path)["roi"], [10, 20, 200, 150])
        self.panel.variables["min_confidence"].set(50)
        self.panel.reload()
        self.assertEqual(self.panel.collect()["min_confidence"], .7)
        self.panel.preset("fast")
        self.assertEqual(self.panel.pending["max_fit_points"], 220)
        self.panel.pump()

    def test_invalid_edits_do_not_replace_valid_config(self):
        self.panel.variables["max_fit_points"].set(0)
        self.panel.save()
        self.assertIsNone(self.panel.pending)
        self.assertEqual(load_config(self.path), self.config)
        self.assertIn("未保存", self.panel.status.get())
