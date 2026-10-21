"""Chinese Tk tuning panel. No worker thread accesses widgets or live config."""
from __future__ import annotations
import copy
import json
from .configuration import validate_config, load_config, save_config, apply_preset

# key, Chinese label, display multiplier, minimum, maximum, integer
FIELDS = {
    "分割与深度": [
        ("min_object_height_m", "最小离桌高度 / mm", 1000, 0, 40, False),
        ("max_object_height_m", "最大离桌高度 / mm", 1000, 20, 800, False),
        ("min_depth_m", "最近深度 / m", 1, .1, 1, False),
        ("max_depth_m", "最远深度 / m", 1, .2, 4, False),
        ("min_component_pixels", "最小目标面积 / px", 1, 20, 2000, True),
        ("segmentation_close_pixels", "轮廓闭运算核 / px", 1, 1, 15, True),
        ("rgb_expand_pixels", "RGB 补边半径 / px", 1, 0, 20, True),
        ("background_threshold", "背景色差阈值", 1, 5, 80, False),
        ("plane_threshold_m", "桌面标定容差 / mm", 1000, .5, 12, False)],
    "速度与模型": [
        ("surface_noise_m", "表面噪声容差 / mm", 1000, .5, 15, False),
        ("max_fit_points", "评分点数上限", 1, 90, 1000, True),
        ("fit_sample_points", "优化采样点数", 1, 60, 600, True),
        ("coarse_fit_nfev", "初筛迭代上限", 1, 3, 40, True),
        ("fit_max_nfev", "精拟合迭代上限", 1, 8, 90, True),
        ("warm_fit_nfev", "连续帧迭代上限", 1, 3, 50, True),
        ("refine_top_k", "精拟合候选数", 1, 1, 8, True),
        ("full_search_interval", "每几次观测全面搜索", 1, 1, 30, True),
        ("analysis_hz", "分析提交频率上限 / Hz", 1, .5, 15, False),
        ("cube_relative_tolerance", "正方体相对等边容差 / %", 100, 0, 30, False),
        ("cube_absolute_tolerance_m", "正方体绝对等边容差 / mm", 1000, 0, 15, False)],
    "评分与稳定": [
        ("surface_score_sigma", "表面评分噪声倍率", 1, .25, 5, False),
        ("depth_score_sigma", "深度评分噪声倍率", 1, .25, 5, False),
        ("score_smoothing_window", "类别评分平均窗口", 1, 1, 21, True),
        ("weights.surface", "3D 表面权重", 100, 0, 100, False),
        ("weights.depth", "可见深度权重", 100, 0, 100, False),
        ("weights.rgb", "主 RGB 轮廓权重", 100, 0, 100, False),
        ("weights.secondary_rgb", "侧视 RGB 轮廓权重", 100, 0, 100, False),
        ("min_confidence", "最低确认分数 / %", 100, 0, 100, False),
        ("min_score_margin", "最小类别分差 / %", 100, 0, 40, False),
        ("min_depth_quality", "最低深度质量 / %", 100, 0, 100, False),
        ("min_stable_frames", "连续确认观测数", 1, 1, 15, True),
        ("smoothing_window", "尺寸平滑窗口", 1, 1, 21, True),
        ("track_distance_m", "跟踪关联距离 / mm", 1000, 5, 250, False),
        ("min_solidity", "最低轮廓凸实度 / %", 100, 50, 100, False)],
    "颜色与区域": [
        ("color.black_value", "黑色明度上限", 1, 0, 120, True),
        ("color.white_value", "白色明度下限", 1, 100, 255, True),
        ("color.neutral_saturation", "彩色饱和度下限", 1, 0, 160, True),
        ("color.min_color_fraction", "主颜色最小比例 / %", 100, 0, 100, False),
        ("max_secondary_skew_s", "两路接收时差门限 / ms", 1000, 5, 300, False),
        ("dimension_jump_relative", "尺寸跳变重新确认 / %", 100, 0, 100, False)]}


def _get(config, key):
    value = config
    for part in key.split("."):
        value = value[part]
    return value


def _set(config, key, value):
    parts = key.split(".")
    target = config
    for part in parts[:-1]:
        target = target[part]
    target[parts[-1]] = value


class TuningPanel:
    def __init__(self, config, path, visible=True):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.config, self.path = tk, copy.deepcopy(config), path
        self.pending, self.closed = None, False
        self.root = tk.Tk()
        self.root.title("D435 几何体识别 · 参数调节")
        self.root.option_add("*Font", ("Microsoft YaHei UI", 9))
        self.root.geometry(f"660x{min(730, max(520, self.root.winfo_screenheight()-90))}")
        self.root.minsize(520, 490)
        if not visible:
            self.root.withdraw()
        self.root.protocol("WM_DELETE_WINDOW", self.hide)
        header = ttk.Frame(self.root, padding=8)
        header.pack(fill="x")
        ttk.Label(header, text="修改后点击应用；当前分析结束后生效。尺寸参数显示为 mm。", wraplength=630).pack(anchor="w")
        presets = ttk.Frame(header)
        presets.pack(fill="x", pady=6)
        for name, label in (("fast", "快速"), ("balanced", "均衡"), ("precise", "精细")):
            ttk.Button(presets, text=label, command=lambda n=name: self.preset(n)).pack(side="left", padx=3)
        self.stats = tk.StringVar(value="等待首个分析结果")
        ttk.Label(header, textvariable=self.stats).pack(anchor="w")
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill="both", expand=True, padx=8)
        self.variables = {}
        for page_name, fields in FIELDS.items():
            shell = ttk.Frame(notebook)
            notebook.add(shell, text=page_name)
            canvas = tk.Canvas(shell, highlightthickness=0)
            scroll = ttk.Scrollbar(shell, orient="vertical", command=canvas.yview)
            canvas.configure(yscrollcommand=scroll.set)
            scroll.pack(side="right", fill="y")
            canvas.pack(side="left", fill="both", expand=True)
            page = ttk.Frame(canvas, padding=8)
            item = canvas.create_window((0, 0), window=page, anchor="nw")
            page.bind("<Configure>", lambda event, c=canvas: c.configure(scrollregion=c.bbox("all")))
            canvas.bind("<Configure>", lambda event, c=canvas, i=item: c.itemconfigure(i, width=event.width))
            canvas.bind("<MouseWheel>", lambda event, c=canvas: c.yview_scroll(int(-event.delta/120), "units"))
            page.columnconfigure(1, weight=1)
            for row, (key, label, factor, low, high, integer) in enumerate(fields):
                variable = tk.DoubleVar(value=_get(config, key)*factor)
                self.variables[key] = variable
                ttk.Label(page, text=label).grid(row=row, column=0, sticky="w", pady=7)
                ttk.Scale(page, from_=low, to=high, variable=variable,
                          command=lambda value, v=variable, whole=integer: v.set(int(round(float(value))) if whole else round(float(value), 2))).grid(row=row, column=1, sticky="ew", padx=5)
                ttk.Entry(page, textvariable=variable, width=9).grid(row=row, column=2)
            if page_name == "颜色与区域":
                row = len(fields)
                ttk.Label(page, text="ROI [x,y,w,h]；空白表示全图").grid(row=row, column=0, columnspan=3, sticky="w", pady=(14, 3))
                self.roi = tk.StringVar(value=json.dumps(config["roi"]) if config["roi"] else "")
                ttk.Entry(page, textvariable=self.roi).grid(row=row+1, column=0, columnspan=3, sticky="ew")
                ttk.Label(page, text="色相边界（0–180）：橙、黄、绿、青、蓝、紫、红").grid(row=row+2, column=0, columnspan=3, sticky="w", pady=(14, 3))
                self.hues = tk.StringVar(value=json.dumps(config["color"]["hue_boundaries"]))
                ttk.Entry(page, textvariable=self.hues).grid(row=row+3, column=0, columnspan=3, sticky="ew")
                self.temporal = tk.BooleanVar(value=config["temporal_depth_filter"])
                ttk.Checkbutton(page, text="启用深度时域滤波（移动目标建议关闭）", variable=self.temporal).grid(row=row+4, column=0, columnspan=3, sticky="w", pady=12)
                ttk.Label(page, text="RGB 窗口快捷键：r 框选区域；s 保存；l 重载；1/2/3 选预设；t 显示此面板。",
                          wraplength=590).grid(row=row+5, column=0, columnspan=3, sticky="w")
        footer = ttk.Frame(self.root, padding=8)
        footer.pack(fill="x")
        for label, action in (("应用参数", self.apply), ("保存到配置", self.save), ("重载配置", self.reload)):
            ttk.Button(footer, text=label, command=action).pack(side="left", padx=3)
        self.status = tk.StringVar(value=f"配置：{path}")
        ttk.Label(self.root, textvariable=self.status, wraplength=630, padding=8).pack(fill="x")

    def collect(self):
        config = copy.deepcopy(self.config)
        for fields in FIELDS.values():
            for key, _, factor, _, _, integer in fields:
                value = self.variables[key].get()/factor
                _set(config, key, int(round(value)) if integer else round(value, 6))
        config["roi"] = json.loads(self.roi.get()) if self.roi.get().strip() else None
        config["color"]["hue_boundaries"] = json.loads(self.hues.get())
        config["temporal_depth_filter"] = self.temporal.get()
        return validate_config(config)

    def set_config(self, config):
        self.config = copy.deepcopy(config)
        for fields in FIELDS.values():
            for key, _, factor, *_ in fields:
                self.variables[key].set(_get(config, key)*factor)
        self.roi.set(json.dumps(config["roi"]) if config["roi"] else "")
        self.hues.set(json.dumps(config["color"]["hue_boundaries"]))
        self.temporal.set(config["temporal_depth_filter"])

    def queue(self, config):
        self.pending = validate_config(config)
        self.status.set("参数有效，等待当前帧分析结束后应用")

    def apply(self):
        try:
            self.queue(self.collect())
        except (ValueError, TypeError, self.tk.TclError) as error:
            self.status.set(f"未应用：{error}")

    def save(self):
        try:
            config = self.collect()
            save_config(self.path, config)
            self.queue(config)
            self.status.set("参数已保存；等待应用。桌面标定阈值改变后请按 b 重新标定。")
        except (ValueError, TypeError, OSError, self.tk.TclError) as error:
            self.status.set(f"未保存：{error}")

    def reload(self):
        try:
            config = load_config(self.path)
            self.set_config(config)
            self.queue(config)
        except (ValueError, TypeError, OSError) as error:
            self.status.set(f"未重载：{error}")

    def preset(self, name):
        try:
            config = apply_preset(self.collect(), name)
            self.set_config(config)
            self.queue(config)
        except (ValueError, TypeError, self.tk.TclError) as error:
            self.status.set(f"未选择预设：{error}")

    def pump(self):
        if not self.closed:
            self.root.update_idletasks()
            self.root.update()

    def hide(self):
        self.root.withdraw()

    def show(self):
        self.root.deiconify()
        self.root.lift()

    def close(self):
        if not self.closed:
            self.closed = True
            self.root.destroy()
