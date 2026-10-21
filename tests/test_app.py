import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import app
from rgbd_geometry.configuration import validate_config, save_config
from rgbd_geometry.synthetic import DemoSource


class ApplicationTests(unittest.TestCase):
    def test_hot_reload_at_frame_boundary_and_finite_json(self):
        with tempfile.TemporaryDirectory() as folder:
            config_path, output = Path(folder)/"config.json", Path(folder)/"results.jsonl"
            save_config(config_path, validate_config({}))

            class EditingSource(DemoSource):
                def read(self):
                    if self.index == 1:
                        save_config(config_path, validate_config({"min_confidence": .7}))
                    return super().read()

            argv = ["app.py", "--demo", "--headless", "--every-frame", "--frames", "4", "--watch-config",
                    "--config", str(config_path), "--output", str(output)]
            with patch("sys.argv", argv), patch("app.DemoSource", EditingSource), contextlib.redirect_stdout(io.StringIO()):
                app.main()
            records = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
            self.assertEqual(len(records), 4)
            self.assertEqual(records[0]["configuration_revision"], 0)
            self.assertTrue(any(record["configuration_revision"] == 1 for record in records))
            self.assertEqual({obj["shape"] for obj in records[-1]["objects"]},
                             {"cube", "cuboid", "triangular_pyramid", "triangular_prism", "cone", "cylinder"})
            self.assertNotIn("NaN", output.read_text(encoding="utf-8"))

    def test_gui_render_and_roi_keys_without_external_windows(self):
        with tempfile.TemporaryDirectory() as folder:
            config_path, output = Path(folder)/"config.json", Path(folder)/"results.jsonl"
            save_config(config_path, validate_config({}))
            keys = iter([ord("r"), ord("1")] + [-1]*45)
            argv = ["app.py", "--demo", "--no-tuning", "--frames", "45", "--config", str(config_path), "--output", str(output)]
            with patch("sys.argv", argv), patch("app.cv2.imshow") as display, \
                 patch("app.cv2.waitKey", side_effect=lambda delay: next(keys, -1)), \
                 patch("app.cv2.selectROI", return_value=(10, 20, 500, 300)), patch("app.cv2.destroyWindow"), \
                 contextlib.redirect_stdout(io.StringIO()):
                app.main()
            self.assertTrue(display.called)
            records = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
            self.assertTrue(records)
            self.assertTrue(any(record["configuration_revision"] > 0 for record in records))


if __name__ == "__main__":
    unittest.main()
