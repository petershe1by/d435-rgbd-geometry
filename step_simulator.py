"""Desktop STEP scene editor and launcher for the existing RGB-D recognizer."""
from __future__ import annotations

import argparse
import base64
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import tkinter as tk
from tkinter import colorchooser, filedialog, messagebox, ttk
import uuid

import cv2
import numpy as np

from rgbd_geometry.cad_simulation import CadSceneSource, OBJECT_DEFAULTS, SCENE_DEFAULTS, fit_scene_camera, load_scene, validate_scene
from rgbd_geometry.configuration import load_config
from rgbd_geometry.geometry import EXTRA_SHAPES

ROOT = Path(__file__).resolve().parent


class SceneEditor:
    def __init__(self, root, scene_path=None, config_path=None):
        self.root = root
        self.scene = copy.deepcopy(SCENE_DEFAULTS)
        self.config_path = Path(config_path or ROOT/"config.json").resolve()
        self.extended_models = tk.BooleanVar(value=bool(load_config(self.config_path).get("extra_shapes")))
        self.dual_rgb = tk.BooleanVar(value=False)
        self.selected = None
        self.child = None
        self.photo = None
        self.object_vars = {}
        self.camera_vars = {}
        self.status = tk.StringVar(value="导入 STEP，设置位置/角度/颜色，先预览，再启动识别。")
        root.title("STEP 几何体虚拟工位 · RGB-D 仿真")
        root.geometry(f"{min(1120, root.winfo_screenwidth()-80)}x{min(790, root.winfo_screenheight()-90)}")
        root.minsize(900, 650)
        root.columnconfigure(0, weight=1)
        root.rowconfigure(1, weight=1)
        toolbar = ttk.Frame(root, padding=8)
        toolbar.grid(row=0, column=0, sticky="ew")
        for text, command in (("新建场景", self.new_scene), ("导入 STEP", self.add_files), ("物块库", self.open_library), ("打开场景", self.open_scene),
                               ("保存场景", self.save_scene), ("自动取景", self.auto_fit), ("更新预览", self.preview),
                               ("启动仿真识别", self.launch)):
            ttk.Button(toolbar, text=text, command=command).pack(side="left", padx=3)
        body = ttk.Frame(root, padding=(8, 0))
        body.grid(row=1, column=0, sticky="nsew")
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)
        left = ttk.Frame(body)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 10))
        ttk.Label(left, text="物体列表（一个 STEP 作为一个刚性目标）").pack(anchor="w")
        self.listbox = tk.Listbox(left, width=35, height=7, exportselection=False)
        self.listbox.pack(fill="x", pady=4)
        self.listbox.bind("<<ListboxSelect>>", self.select_object)
        ttk.Button(left, text="移除所选物体", command=self.remove_object).pack(anchor="w", pady=(0, 4))
        properties = ttk.LabelFrame(left, text="所选物体（长度 mm，角度 °）", padding=8)
        properties.pack(fill="x")
        fields = (("x", "桌面 X"), ("y", "桌面 Y"), ("rx", "绕 X / 翻滚"),
                  ("ry", "绕 Y / 俯仰"), ("rz", "绕 Z / 朝向"),
                  ("scale", "模型缩放倍率"), ("speed", "旋转速度 °/仿真秒"))
        for row, (key, label) in enumerate(fields):
            variable = tk.StringVar(value="1" if key == "scale" else "0")
            self.object_vars[key] = variable
            ttk.Label(properties, text=label).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Entry(properties, textvariable=variable, width=12).grid(row=row, column=1, padx=(10, 0))
        self.color = tk.StringVar(value="#dc2d23")
        ttk.Label(properties, text="表面颜色").grid(row=7, column=0, sticky="w", pady=3)
        ttk.Entry(properties, textvariable=self.color, width=12).grid(row=7, column=1)
        ttk.Button(properties, text="选择颜色", command=self.choose_color).grid(row=8, column=1, pady=4)
        ttk.Button(properties, text="应用所选物体", command=self.apply_object).grid(row=9, column=0, columnspan=2, pady=4)
        right = ttk.Frame(body)
        right.grid(row=0, column=1, sticky="nsew")
        right.columnconfigure(0, weight=1)
        ttk.Label(right, text="左：虚拟 RGB   右：虚拟深度（预览不等于识别结果）").grid(row=0, column=0, sticky="w")
        self.preview_label = ttk.Label(right, text="尚未生成预览", anchor="center")
        self.preview_label.grid(row=1, column=0, sticky="nsew", pady=6)
        right.rowconfigure(1, weight=1)
        settings = ttk.LabelFrame(right, text="虚拟相机与噪声", padding=8)
        settings.grid(row=2, column=0, sticky="ew")
        camera_fields = (("azimuth_deg", "方位角 °", -90), ("elevation_deg", "俯视角 °", 52),
                         ("distance_mm", "相机距离 mm", 850), ("horizontal_fov_deg", "水平视场 °", 58),
                         ("noise_sigma_mm", "深度噪声 σ mm", .8), ("holes_percent", "深度孔洞 %", 2))
        for index, (key, label, default) in enumerate(camera_fields):
            row, pair = divmod(index, 2)
            variable = tk.StringVar(value=str(default))
            self.camera_vars[key] = variable
            ttk.Label(settings, text=label).grid(row=row, column=pair*2, sticky="w", pady=5, padx=3)
            ttk.Entry(settings, textvariable=variable, width=9).grid(row=row, column=pair*2+1, padx=(2, 12))
        ttk.Checkbutton(settings, text="第二路 RGB（俯视轮廓补充）", variable=self.dual_rgb).grid(row=3, column=0, columnspan=4, sticky="w", pady=4)
        ttk.Checkbutton(settings, text="扩展 12 类仿真预设（球体、四/五/六棱形）", variable=self.extended_models).grid(row=4, column=0, columnspan=4, sticky="w", pady=4)
        ttk.Label(right, text="STEP 声明的单位自动换算；颜色在此指定，不读取 STEP 材质。\n"
                  "物体自动落在桌面。支持网格遮挡，不含重力/碰撞、反光/透明及真实 D435 噪声模型。\n"
                  "启动时自动检查测距/视野；修改场景后需重新启动识别。\n"
                  "关闭此编辑器会保留已启动的识别窗口；请在识别窗口按 q 结束。",
                  wraplength=650).grid(row=3, column=0, sticky="w", pady=10)
        ttk.Label(root, textvariable=self.status, wraplength=1000, padding=8).grid(row=2, column=0, sticky="ew")
        if scene_path:
            self._load(scene_path)

    def open_library(self):
        path = ROOT/"models"/"user_blocks"/"library.json"
        if not path.is_file():
            self.status.set("尚未准备用户物块库；请先用 prepare_step_library.py 导入 STEP 目录。")
            return
        try:
            manifest = json.loads(path.read_text(encoding="utf-8"))
            items = [item for item in manifest["items"] if "error" not in item]
        except (ValueError, OSError, KeyError) as exc:
            messagebox.showerror("物块库无效", str(exc), parent=self.root)
            return
        dialog = tk.Toplevel(self.root)
        dialog.title("用户物块库 · 选择一个或多个 STEP")
        dialog.geometry("830x600")
        dialog.columnconfigure(0, weight=1)
        dialog.rowconfigure(1, weight=1)
        ttk.Label(dialog, text=f"已导入 {len(items)} 个模型；Ctrl / Shift 多选，每次最多 6 个。\n"
                  "可选基础 6 类或扩展 12 类；镂空体、组合体等仍未完整建模。",
                  padding=8).grid(row=0, column=0, columnspan=2, sticky="w")
        tree = ttk.Treeview(dialog, columns=("name", "size", "support"), show="headings", selectmode="extended")
        for key, title, width in (("name", "模型", 280), ("size", "网格外接 L/W/H（mm）", 220), ("support", "当前类别支持", 220)):
            tree.heading(key, text=title)
            tree.column(key, width=width)
        tree.grid(row=1, column=0, sticky="nsew", padx=(8, 0))
        scrollbar = ttk.Scrollbar(dialog, orient="vertical", command=tree.yview)
        scrollbar.grid(row=1, column=1, sticky="ns", padx=(0, 8))
        tree.configure(yscrollcommand=scrollbar.set)
        for index, item in enumerate(items):
            tree.insert("", "end", iid=str(index), values=(item["name"], " × ".join(f"{value:.1f}" for value in item["table_bbox_mm"]),
                                                           item.get("reference_shape") or "扩展模型（未完整建模）"))

        def load_selected():
            try:
                selected = tree.selection()
                if not 1 <= len(selected) <= 6:
                    raise ValueError("请选择 1–6 个物块")
                from prepare_step_library import scene_for
                scene = validate_scene(scene_for([items[int(index)] for index in selected]), ROOT)
                folder = ROOT/"simulations"
                folder.mkdir(exist_ok=True)
                scene_path = folder/f"library_selection_{uuid.uuid4().hex[:8]}.json"
                scene_path.write_text(json.dumps(scene, ensure_ascii=False, indent=2), encoding="utf-8")
                config_path = Path(manifest.get("algorithm_config", ROOT/"config.json"))
                load_config(config_path)
                self.config_path = config_path.resolve()
                self.extended_models.set(bool(load_config(self.config_path).get("extra_shapes")))
                self._load(scene_path)
                self.status.set(f"已加载 {len(selected)} 个用户物块；算法配置 {self.config_path.name}。")
                dialog.destroy()
            except (ValueError, OSError) as exc:
                messagebox.showerror("加载失败", str(exc), parent=dialog)

        ttk.Button(dialog, text="载入所选物块到场景", command=load_selected).grid(row=2, column=0, columnspan=2, pady=10)
        tree.bind("<Double-1>", lambda event: load_selected())

    def _refresh_list(self, select=0):
        self.listbox.delete(0, "end")
        self.selected = None
        for index, obj in enumerate(self.scene["objects"]):
            self.listbox.insert("end", f"{index+1}. {obj['name']}")
        if self.scene["objects"]:
            self.listbox.selection_set(min(select, len(self.scene["objects"])-1))
            self.select_object()

    def new_scene(self):
        self.scene = copy.deepcopy(SCENE_DEFAULTS)
        self.dual_rgb.set(False)
        self._refresh_list()
        for key in ("azimuth_deg", "elevation_deg", "distance_mm", "horizontal_fov_deg"):
            self.camera_vars[key].set(str(self.scene["camera"][key]))
        self.camera_vars["noise_sigma_mm"].set(str(self.scene["noise_sigma_mm"]))
        self.camera_vars["holes_percent"].set(str(self.scene["hole_fraction"]*100))
        self.preview_label.configure(image="", text="新场景：请导入 STEP")
        self.photo = None
        self.status.set("已新建空场景。导入自己的 STEP 文件后预览。正在运行的识别不受影响。")

    def _commit_object(self):
        if self.selected is None:
            return
        values = {key: float(var.get()) for key, var in self.object_vars.items()}
        try:
            rgb = [value//257 for value in self.root.winfo_rgb(self.color.get())]
        except tk.TclError as exc:
            raise ValueError("颜色请使用 #RRGGBB 格式或点击选择颜色") from exc
        obj = copy.deepcopy(self.scene["objects"][self.selected])
        obj.update(position_mm=[values["x"], values["y"]], rotation_deg=[values["rx"], values["ry"], values["rz"]],
                   scale=values["scale"], yaw_speed_deg_s=values["speed"], color_rgb=rgb)
        # Validate edits before replacing the previous valid object.
        validate_scene({**self.scene, "objects": [obj]}, ROOT)
        self.scene["objects"][self.selected] = obj

    def select_object(self, event=None):
        selected = self.listbox.curselection()
        if not selected:
            return
        index = selected[0]
        if index == self.selected:
            return
        try:
            if self.selected != index:
                self._commit_object()
        except (ValueError, OSError) as exc:
            self.listbox.selection_clear(0, "end")
            if self.selected is not None:
                self.listbox.selection_set(self.selected)
            messagebox.showerror("参数无效", str(exc), parent=self.root)
            return
        self.selected = index
        obj = self.scene["objects"][index]
        values = dict(zip(("x", "y", "rx", "ry", "rz", "scale", "speed"),
                          [*obj["position_mm"], *obj["rotation_deg"], obj["scale"], obj["yaw_speed_deg_s"]]))
        for key, value in values.items():
            self.object_vars[key].set(f"{value:g}")
        self.color.set("#"+"".join(f"{int(channel):02x}" for channel in obj["color_rgb"]))

    def add_files(self):
        paths = filedialog.askopenfilenames(parent=self.root, title="导入 STEP / STP",
                                            filetypes=[("STEP", "*.step *.stp *.STEP *.STP"), ("所有文件", "*.*")])
        if not paths:
            return
        try:
            self._commit_object()
            objects = copy.deepcopy(self.scene["objects"])
            for path in paths:
                obj = copy.deepcopy(OBJECT_DEFAULTS)
                obj.update(file=str(Path(path).resolve()), name=Path(path).stem,
                           position_mm=[(len(objects)%3-1)*180, (len(objects)//3)*180-90])
                objects.append(obj)
            self.scene = validate_scene({**self.scene, "objects": objects}, ROOT)
            self._refresh_list(len(objects)-1)
            self.status.set(f"已导入 {len(paths)} 个文件；点击更新预览检查尺寸和可见范围。")
        except (ValueError, OSError) as exc:
            messagebox.showerror("导入失败", str(exc), parent=self.root)

    def remove_object(self):
        if self.selected is not None:
            self.scene["objects"].pop(self.selected)
            self._refresh_list()

    def choose_color(self):
        result = colorchooser.askcolor(parent=self.root, initialcolor=self.color.get())
        if result[1]:
            self.color.set(result[1])

    def apply_object(self):
        try:
            self._commit_object()
            self.status.set("物体参数已应用。更新预览或重新启动识别查看效果。")
        except (ValueError, OSError) as exc:
            messagebox.showerror("参数无效", str(exc), parent=self.root)

    def collect_scene(self):
        self._commit_object()
        scene = copy.deepcopy(self.scene)
        for key in ("azimuth_deg", "elevation_deg", "distance_mm", "horizontal_fov_deg"):
            scene["camera"][key] = float(self.camera_vars[key].get())
        scene["noise_sigma_mm"] = float(self.camera_vars["noise_sigma_mm"].get())
        scene["hole_fraction"] = float(self.camera_vars["holes_percent"].get())/100
        if self.dual_rgb.get():
            scene["secondary_camera"] = scene.get("secondary_camera") or {
                **copy.deepcopy(scene["camera"]), "azimuth_deg": -20., "elevation_deg": 75.,
                "distance_mm": max(450., scene["camera"]["distance_mm"])}
        else:
            scene["secondary_camera"] = None
        return validate_scene(scene, ROOT)

    def runtime_config(self):
        config = load_config(self.config_path)
        if self.extended_models.get():
            if not config.get("extra_shapes"):
                config.update(surface_score_sigma=.6, depth_score_sigma=.8, score_smoothing_window=7,
                              weights={"surface": .30, "depth": .25, "rgb": .10, "secondary_rgb": .35})
            config["extra_shapes"] = list(EXTRA_SHAPES)
        else:
            config["extra_shapes"] = []
        return config

    def auto_fit(self):
        try:
            scene = fit_scene_camera(self.collect_scene(), self.runtime_config())
            self.scene = scene
            self.camera_vars["distance_mm"].set(f"{scene['camera']['distance_mm']:.1f}")
            self.status.set(f"自动取景完成：相机距离 {scene['camera']['distance_mm']:.1f} mm，已检查有效测距和画面边缘。")
        except (ValueError, RuntimeError, OSError) as exc:
            messagebox.showerror("自动取景失败", str(exc), parent=self.root)

    def _load(self, path):
        scene = load_scene(path)
        self.scene = scene
        self.dual_rgb.set(scene.get("secondary_camera") is not None)
        for key in ("azimuth_deg", "elevation_deg", "distance_mm", "horizontal_fov_deg"):
            self.camera_vars[key].set(f"{scene['camera'][key]:g}")
        self.camera_vars["noise_sigma_mm"].set(f"{scene['noise_sigma_mm']:g}")
        self.camera_vars["holes_percent"].set(f"{scene['hole_fraction']*100:g}")
        self._refresh_list()
        self.status.set(f"已打开场景: {path}；算法配置: {self.config_path.name}")

    def open_scene(self):
        path = filedialog.askopenfilename(parent=self.root, filetypes=[("场景 JSON", "*.json")])
        if path:
            try:
                self._load(path)
            except (ValueError, OSError) as exc:
                messagebox.showerror("场景无效", str(exc), parent=self.root)

    def _snapshot(self, auto_fit=False):
        scene = self.collect_scene()
        if auto_fit:
            scene = fit_scene_camera(scene, self.runtime_config())
            self.scene = scene
            self.camera_vars["distance_mm"].set(f"{scene['camera']['distance_mm']:.1f}")
        folder = ROOT/"simulations"
        folder.mkdir(exist_ok=True)
        prefix = f"scene_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"
        path = folder/f"{prefix}.json"
        path.write_text(json.dumps(scene, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        return path

    def save_scene(self):
        try:
            scene = self.collect_scene()
            path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".json", filetypes=[("场景 JSON", "*.json")])
            if path:
                Path(path).write_text(json.dumps(scene, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
                self.status.set(f"场景已保存: {path}")
        except (ValueError, OSError) as exc:
            messagebox.showerror("保存失败", str(exc), parent=self.root)

    def preview(self):
        source = None
        try:
            self.status.set("正在读取 STEP 并生成预览……")
            self.root.update_idletasks()
            path = self._snapshot()
            source = CadSceneSource(path, self.runtime_config())
            frame = source.read()
            depth = cv2.applyColorMap(cv2.convertScaleAbs(frame.depth, alpha=150), cv2.COLORMAP_TURBO)
            depth[frame.depth <= 0] = 0
            preview = np.hstack((frame.rgb, depth))
            preview_width = max(440, min(680, self.root.winfo_width()-380))
            preview = cv2.resize(preview, (preview_width, int(preview.shape[0]*preview_width/preview.shape[1])), interpolation=cv2.INTER_AREA)
            png = cv2.imencode(".png", preview)[1].tobytes()
            self.photo = tk.PhotoImage(data=base64.b64encode(png).decode("ascii"))
            self.preview_label.configure(image=self.photo, text="")
            dimensions = "; ".join(f"{obj['name']}: {'×'.join(f'{v:.1f}' for v in obj['dimensions_mm'])} mm" for obj in source.reference_objects)
            invalid = [item for item in source.camera_health() if item["outside_depth_fraction"] > .01 or item["border_truncated"] or item["visible_pixels"] == 0]
            warning = f"注意：{len(invalid)} 个物块超出有效深度/画面，请点自动取景。 " if invalid else ""
            self.status.set(warning+"网格外接尺寸（非识别结果）: "+dimensions)
        except Exception as exc:
            self.status.set(f"预览失败: {exc}")
            messagebox.showerror("预览失败", str(exc), parent=self.root)
        finally:
            if source:
                source.close()

    def launch(self):
        try:
            if self.child is not None and self.child.poll() is None:
                self.status.set("仿真仍在运行；请先在识别窗口按 q 结束，再启动新场景。")
                return
            path = self._snapshot(auto_fit=True)
            output, log = path.with_suffix(".jsonl"), path.with_suffix(".log")
            config_path = path.with_suffix(".config.json")
            config_path.write_text(json.dumps(self.runtime_config(), ensure_ascii=False, indent=2), encoding="utf-8")
            with log.open("w", encoding="utf-8") as handle:
                self.child = subprocess.Popen([sys.executable, "-u", str(ROOT/"app.py"), "--scene", str(path), "--config", str(config_path),
                                               "--output", str(output)], cwd=ROOT, stdout=handle,
                                              stderr=subprocess.STDOUT, env={**os.environ, "PYTHONIOENCODING": "utf-8"})
            self.status.set(f"仿真已启动，相机 {self.camera_vars['distance_mm'].get()} mm，结果: {output.name}；按 q 退出。")
            self.root.after(1000, lambda: self._poll(log))
        except (ValueError, OSError) as exc:
            messagebox.showerror("启动失败", str(exc), parent=self.root)

    def _poll(self, log):
        if self.child is not None and self.child.poll() is None:
            self.root.after(1000, lambda: self._poll(log))
        elif self.child is not None and self.child.returncode:
            self.status.set(f"识别退出码 {self.child.returncode}，请查看 {log}")
        else:
            self.status.set(f"本次识别已结束，日志: {log}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    user_scene = ROOT/"models"/"user_blocks"/"basic_six.json"
    initial = args.scene or (user_scene if user_scene.is_file() else ROOT/"models"/"examples"/"scene.json")
    config_path = args.config
    if config_path is None and initial.resolve() == user_scene.resolve():
        candidate = user_scene.parent/"config_extended.json"
        if not candidate.is_file():
            candidate = user_scene.parent/"config_40mm.json"
        if candidate.is_file():
            config_path = candidate
    root = tk.Tk()
    SceneEditor(root, initial if initial.is_file() else None, config_path)
    root.mainloop()
