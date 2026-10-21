import contextlib
import copy
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import tkinter as tk
import unittest
from unittest.mock import patch

import numpy as np

import app
from rgbd_geometry.cad_simulation import (
    CadSceneSource, OBJECT_DEFAULTS, SCENE_DEFAULTS, load_scene,
    fit_scene_camera, place_mesh, rasterize, read_step, validate_scene, virtual_camera,
)
from rgbd_geometry.configuration import save_config, validate_config
from rgbd_geometry.vision import ModelFitter, Tracker


HAS_OCP = importlib.util.find_spec("OCP") is not None


@unittest.skipUnless(HAS_OCP, "optional STEP dependency not installed")
class StepSimulationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.path = self.folder/"模型.step"
        self._write_box(self.path)
        self.config = validate_config({"width": 424, "height": 320})
        self.scene = copy.deepcopy(SCENE_DEFAULTS)
        self.scene.update(noise_sigma_mm=0., hole_fraction=0., rgb_noise_sigma=0., shading=0.)
        self.scene["objects"] = [{**copy.deepcopy(OBJECT_DEFAULTS), "file": self.path.name,
                                  "name": "misleading_cone_name"}]
        self.scene_path = self.folder/"scene.json"
        self._save_scene()

    @staticmethod
    def _write_box(path, unit="MM"):
        from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.Interface import Interface_Static
        from OCP.STEPControl import STEPControl_Writer, STEPControl_AsIs
        writer = STEPControl_Writer()
        old = Interface_Static.CVal_s("write.step.unit")
        try:
            if not Interface_Static.SetCVal_s("write.step.unit", unit):
                raise RuntimeError("STEP unit configuration failed")
            assert writer.Transfer(BRepPrimAPI_MakeBox(75., 75., 75.).Shape(), STEPControl_AsIs) == IFSelect_RetDone
            assert writer.Write(str(path)) == IFSelect_RetDone
        finally:
            Interface_Static.SetCVal_s("write.step.unit", old)

    def _save_scene(self):
        self.scene_path.write_text(json.dumps(self.scene), encoding="utf-8")

    def test_step_units_mm_and_inches_are_metric_and_unicode_paths_work(self):
        inch_path = self.folder/"inch.step"
        self._write_box(inch_path, "INCH")
        for path in (self.path, inch_path):
            mesh = read_step(path)
            np.testing.assert_allclose(np.ptp(mesh.vertices_m, axis=0), [.075]*3, atol=1e-8)
            self.assertEqual(len(mesh.triangles), 12)
            self.assertFalse(mesh.vertices_m.flags.writeable)

    def test_optical_z_reconstruction_lies_on_actual_box_surfaces(self):
        source = CadSceneSource(self.scene_path, self.config)
        frame = source.read()
        points = source.table.to_world(frame.intrinsics.deproject(frame.depth)[source._buffers[2] == 0])
        self.assertGreater(len(points), 500)
        surface = np.maximum.reduce([np.abs(points[:, 0])-.0375, np.abs(points[:, 1])-.0375,
                                     -points[:, 2], points[:, 2]-.075])
        self.assertLess(float(np.max(np.abs(surface))), 1e-6)

    def test_z_buffer_occlusion_does_not_depend_on_draw_order(self):
        intr, table = virtual_camera(200, 160, self.scene["camera"])
        square = np.array([[-.10, -.1, .05], [.10, -.1, .05], [.10, .1, .05], [-.1, .1, .05]])
        tris = np.array([[0, 1, 2], [0, 2, 3]], np.int32)
        low = (square, tris, [220, 30, 30])
        high = (square + [0, 0, .05], tris, [30, 220, 30])
        _, depth1, owners1 = rasterize([low, high], intr, table, shading=0)
        _, depth2, owners2 = rasterize([high, low], intr, table, shading=0)
        np.testing.assert_allclose(depth1, depth2, atol=1e-7)
        self.assertEqual(owners1[80, 100], 1)
        self.assertEqual(owners2[80, 100], 0)

    def test_mesh_hole_shows_table_instead_of_convex_hull_fill(self):
        camera = copy.deepcopy(self.scene["camera"])
        camera["target_mm"] = [0., 0., 100.]
        intr, table = virtual_camera(200, 160, camera)
        points, triangles = [], []
        for x0, x1, y0, y1 in ((-.1, -.04, -.1, .1), (.04, .1, -.1, .1),
                                (-.04, .04, -.1, -.04), (-.04, .04, .04, .1)):
            start = len(points)
            points.extend([[x0, y0, .1], [x1, y0, .1], [x1, y1, .1], [x0, y1, .1]])
            triangles.extend([[start, start+1, start+2], [start, start+2, start+3]])
        _, depth, owners = rasterize([(np.array(points), np.array(triangles), [220, 30, 30])], intr, table)
        self.assertGreater(int((owners == 0).sum()), 100)
        self.assertEqual(owners[80, 100], -1)
        self.assertGreater(float(depth[80, 100]), .85)

    def test_noise_is_reproducible_and_invalid_scene_is_rejected(self):
        self.scene.update(noise_sigma_mm=1., hole_fraction=.2)
        self._save_scene()
        first, second = CadSceneSource(self.scene_path, self.config), CadSceneSource(self.scene_path, self.config)
        np.testing.assert_array_equal(first.read().depth, second.read().depth)
        self.assertGreater(float((first.read().depth == 0).mean()), .15)
        for key, value in (("noise_sigma_mm", float("nan")), ("hole_fraction", 2), ("seed", True)):
            invalid = copy.deepcopy(self.scene)
            invalid[key] = value
            with self.assertRaises(ValueError):
                validate_scene(invalid, self.folder)
        invalid = copy.deepcopy(self.scene)
        invalid["objects"][0]["file"] = "missing.step"
        with self.assertRaises(ValueError):
            validate_scene(invalid, self.folder)

    def test_rotation_grounding_and_mesh_scale(self):
        mesh = read_step(self.path)
        obj = load_scene(self.scene_path)["objects"][0]
        obj.update(rotation_deg=[90., 0., 30.], scale=2., position_mm=[100., -50.])
        vertices = place_mesh(mesh, obj)
        self.assertAlmostEqual(float(vertices[:, 2].min()), 0.)
        self.assertAlmostEqual(float(np.ptp(vertices[:, 2])), .15)
        np.testing.assert_allclose((vertices[:, :2].min(axis=0)+vertices[:, :2].max(axis=0))/2, [.1, -.05])

    def test_recognizer_uses_pixels_not_step_filename_or_name(self):
        source = CadSceneSource(self.scene_path, self.config)
        fitter, tracker = ModelFitter(self.config), Tracker(self.config)
        for _ in range(3):
            frame = source.read()
            observations, _, _, _ = app.analyze(frame, source.table, source.background, self.config, fitter)
            result = tracker.update(observations, frame.timestamp)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]["shape"], "cube")
        np.testing.assert_allclose(result[0]["dimensions_mm"], [75.]*3, atol=2.)

    def test_virtual_calibration_hotkey_never_overwrites_real_workspace(self):
        config_path = self.folder/"config.json"
        save_config(config_path, self.config)
        workspace = self.folder/"workspace.npz"
        workspace.write_bytes(b"real calibration sentinel")
        output = self.folder/"results.jsonl"
        keys = iter([ord("b"), -1, -1])
        argv = ["app.py", "--scene", str(self.scene_path), "--config", str(config_path),
                "--workspace", str(workspace), "--no-tuning", "--every-frame", "--quiet",
                "--frames", "3", "--output", str(output)]
        with patch("sys.argv", argv), patch("app.cv2.imshow"), patch("app.cv2.waitKey", side_effect=lambda _: next(keys, -1)), contextlib.redirect_stdout(io.StringIO()):
            app.main()
        self.assertEqual(workspace.read_bytes(), b"real calibration sentinel")
        records = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
        self.assertTrue(all(record["input_source"] == "step_mesh_simulation" for record in records))
        self.assertTrue(all(obj["color"]["source"] == "simulated_rgb" for record in records for obj in record["objects"]))
        self.assertEqual(records[-1]["objects"][0]["shape"], "cube")

    def test_gui_import_properties_preview_and_launch_arguments(self):
        from step_simulator import SceneEditor
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        save_config(self.folder/"config.json", self.config)
        with patch("step_simulator.ROOT", self.folder):
            editor = SceneEditor(root, self.scene_path)
            editor.object_vars["x"].set("25")
            editor.color.set("#ff0000")
            scene = editor.collect_scene()
            self.assertEqual(scene["objects"][0]["position_mm"][0], 25)
            self.assertEqual(scene["objects"][0]["color_rgb"], [255, 0, 0])
            with patch("step_simulator.messagebox.showerror") as error:
                editor.preview()
                error.assert_not_called()
            self.assertIsNotNone(editor.photo)
            self.assertIn("75.0", editor.status.get())
            with patch("step_simulator.subprocess.Popen") as launch:
                editor.launch()
                args = launch.call_args.args[0]
                self.assertIn("--scene", args)
                self.assertTrue(Path(args[args.index("--scene")+1]).is_file())
                self.assertIn("--output", args)
                launched_config = Path(args[args.index("--config")+1])
                self.assertTrue(launched_config.is_file())
                self.assertEqual(launched_config.parent, self.folder/"simulations")
                self.assertEqual(json.loads(launched_config.read_text(encoding="utf-8"))["extra_shapes"], [])
            editor.selected = None

    def test_prepare_external_library_and_load_from_gui_without_modifying_step(self):
        from prepare_step_library import prepare
        from step_simulator import SceneEditor
        from tkinter import ttk
        original = self.path.read_bytes()
        output = self.folder/"models"/"user_blocks"
        with contextlib.redirect_stdout(io.StringIO()):
            manifest = prepare(self.folder, output)
        self.assertEqual(manifest["imported"], 1)
        self.assertEqual(manifest["failed"], 0)
        self.assertEqual(self.path.read_bytes(), original)
        self.assertTrue(Path(manifest["default_scene"]).is_file())
        self.assertTrue(Path(manifest["algorithm_config"]).is_file())
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        save_config(self.folder/"config.json", self.config)
        with patch("step_simulator.ROOT", self.folder):
            editor = SceneEditor(root)
            editor.open_library()
            dialog = next(child for child in root.winfo_children() if isinstance(child, tk.Toplevel))
            tree = next(child for child in dialog.winfo_children() if isinstance(child, ttk.Treeview))
            tree.selection_set("0")
            button = next(child for child in dialog.winfo_children() if isinstance(child, ttk.Button))
            with patch("step_simulator.messagebox.showerror") as error:
                button.invoke()
                error.assert_not_called()
            self.assertEqual(len(editor.scene["objects"]), 1)
            self.assertEqual(Path(editor.scene["objects"][0]["file"]), self.path)
            self.assertEqual(editor.config_path, Path(manifest["algorithm_config"]))

    def test_auto_fit_prevents_near_depth_clipping_and_updates_gui(self):
        self.scene["camera"]["distance_mm"] = 220.
        self._save_scene()
        before = CadSceneSource(self.scene_path, self.config)
        self.assertGreater(before.camera_health()[0]["outside_depth_fraction"], .01)
        fitted = fit_scene_camera(load_scene(self.scene_path), self.config)
        self.assertGreater(fitted["camera"]["distance_mm"], 220.)
        self.scene_path.write_text(json.dumps(fitted), encoding="utf-8")
        after = CadSceneSource(self.scene_path, self.config)
        self.assertEqual(after.camera_health()[0]["outside_depth_fraction"], 0.)
        self.assertFalse(after.camera_health()[0]["border_truncated"])
        from step_simulator import SceneEditor
        root = tk.Tk()
        root.withdraw()
        self.addCleanup(root.destroy)
        config_path = self.folder/"config.json"
        save_config(config_path, self.config)
        editor = SceneEditor(root, self.scene_path, config_path)
        editor.camera_vars["distance_mm"].set("220")
        with patch("step_simulator.messagebox.showerror") as error:
            editor.auto_fit()
            error.assert_not_called()
        self.assertGreater(float(editor.camera_vars["distance_mm"].get()), 220.)

    def test_virtual_secondary_rgb_extrinsics_and_fusion_are_consistent(self):
        self.scene["secondary_camera"] = {**self.scene["camera"], "azimuth_deg": -20., "elevation_deg": 75.}
        self._save_scene()
        source = CadSceneSource(self.scene_path, self.config)
        world = np.array([[0., 0., .03], [.02, -.01, .05]])
        fusion = source.secondary_fusion
        projected = source.table.to_camera(world) @ fusion.rotation.T + fusion.translation
        np.testing.assert_allclose(projected, source.secondary_table.to_camera(world), atol=1e-10)
        frame = source.read()
        secondary = source.secondary_frame(frame)
        self.assertEqual(secondary[1], frame.timestamp)
        observations, _, _, _ = app.analyze(frame, source.table, source.background, self.config, ModelFitter(self.config), fusion, secondary)
        self.assertEqual(len(observations), 1)
        self.assertIn("secondary_rgb", observations[0]["metrics"])
        self.assertEqual(observations[0]["shape"], "cube")


if __name__ == "__main__":
    unittest.main()
