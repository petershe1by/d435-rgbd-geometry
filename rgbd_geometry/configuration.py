"""Validated configuration snapshots, atomic persistence and runtime presets."""
import copy
import json
import math
import os
from pathlib import Path
from .geometry import EXTRA_SHAPES

DEFAULTS = {
    "width": 640, "height": 480, "fps": 15, "min_depth_m": .20, "max_depth_m": 2.,
    "min_object_height_m": .008, "max_object_height_m": .40, "min_component_pixels": 180,
    "max_fit_points": 350, "surface_noise_m": .003, "plane_threshold_m": .004,
    "analysis_hz": 5, "min_confidence": .58, "min_score_margin": .06,
    "track_distance_m": .08, "track_timeout_s": .8, "max_secondary_skew_s": .10,
    "weights": {"surface": .30, "depth": .25, "rgb": .25, "secondary_rgb": .20},
    "fit_max_nfev": 30, "coarse_fit_nfev": 10, "warm_fit_nfev": 15,
    "refine_top_k": 3, "full_search_interval": 8, "fit_sample_points": 160,
    "cube_relative_tolerance": .08, "cube_absolute_tolerance_m": .004,
    "roi": None, "temporal_depth_filter": False, "background_threshold": 25.,
    "rgb_expand_pixels": 8, "segmentation_close_pixels": 3, "min_depth_quality": .65,
    "min_stable_frames": 3, "smoothing_window": 7, "dimension_jump_relative": .25,
    "min_solidity": .90, "max_surface_error_factor": 2., "max_objects": 16,
    "extra_shapes": [],
    "surface_score_sigma": 1.8, "depth_score_sigma": 2.5,
    "score_smoothing_window": 1,
    "color": {"black_value": 48, "white_value": 185, "neutral_saturation": 50,
              "min_color_fraction": .45, "hue_boundaries": [9, 23, 36, 85, 100, 135, 172]},
}

PRESETS = {
    "fast": {"max_fit_points": 220, "fit_sample_points": 100, "coarse_fit_nfev": 8,
             "fit_max_nfev": 20, "warm_fit_nfev": 10, "refine_top_k": 2, "full_search_interval": 10},
    "balanced": {key: DEFAULTS[key] for key in ("max_fit_points", "fit_sample_points", "coarse_fit_nfev",
                 "fit_max_nfev", "warm_fit_nfev", "refine_top_k", "full_search_interval")},
    "precise": {"max_fit_points": 600, "fit_sample_points": 260, "coarse_fit_nfev": 18,
                "fit_max_nfev": 55, "warm_fit_nfev": 25, "refine_top_k": 4, "full_search_interval": 5},
}
CAPTURE_KEYS = ("width", "height", "fps")


def _merge(base, updates):
    if not isinstance(updates, dict):
        raise ValueError("配置必须是 JSON 对象")
    result = copy.deepcopy(base)
    for key, value in updates.items():
        if key not in base:
            raise ValueError(f"未知配置项 {key}，请检查拼写")
        result[key] = _merge(base[key], value) if isinstance(base[key], dict) else copy.deepcopy(value)
    return result


def validate_config(config):
    cfg = _merge(DEFAULTS, config)
    extras = cfg["extra_shapes"]
    if not isinstance(extras, list) or any(not isinstance(s, str) or s not in EXTRA_SHAPES for s in extras) or len(set(extras)) != len(extras):
        raise ValueError(f"extra_shapes 必须为不重复的已支持扩展类别列表: {EXTRA_SHAPES}")
    for name, default in DEFAULTS.items():
        if isinstance(default, bool):
            if not isinstance(cfg[name], bool):
                raise ValueError(f"{name} 必须为 true 或 false")
        elif isinstance(default, (int, float)):
            v = cfg[name]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v < 0:
                raise ValueError(f"{name} 必须为有限非负数")
            if isinstance(default, int) and int(v) != v:
                raise ValueError(f"{name} 必须为整数")
            if isinstance(default, int):
                cfg[name] = int(v)
    positive = ("width", "height", "fps", "min_depth_m", "surface_noise_m", "plane_threshold_m",
                "analysis_hz", "track_distance_m", "track_timeout_s", "max_secondary_skew_s",
                "fit_max_nfev", "coarse_fit_nfev", "warm_fit_nfev", "refine_top_k", "full_search_interval",
                "min_stable_frames", "smoothing_window", "max_surface_error_factor", "max_objects")
    for name in positive:
        if cfg[name] <= 0:
            raise ValueError(f"{name} 必须大于零")
    for low, high in (("min_depth_m", "max_depth_m"), ("min_object_height_m", "max_object_height_m")):
        if cfg[low] >= cfg[high]:
            raise ValueError(f"{low} 必须小于 {high}")
    if cfg["max_fit_points"] < 90 or cfg["fit_sample_points"] < 60:
        raise ValueError("max_fit_points 至少 90，fit_sample_points 至少 60")
    if not .25 <= cfg["surface_score_sigma"] <= 5 or not .25 <= cfg["depth_score_sigma"] <= 5:
        raise ValueError("表面/深度评分噪声倍率必须为 0.25–5")
    if not 1 <= cfg["score_smoothing_window"] <= 21:
        raise ValueError("类别评分平均窗口必须为 1–21")
    if not 1 <= cfg["refine_top_k"] <= 8 or not 1 <= cfg["segmentation_close_pixels"] <= 15:
        raise ValueError("refine_top_k 范围 1–8，segmentation_close_pixels 范围 1–15")
    for name in ("min_confidence", "min_score_margin", "min_depth_quality", "min_solidity",
                 "cube_relative_tolerance", "dimension_jump_relative"):
        if not 0 <= cfg[name] <= 1:
            raise ValueError(f"{name} 必须在 0–1 之间")
    weights = cfg["weights"]
    if any(isinstance(w, bool) or not isinstance(w, (int, float)) or not math.isfinite(w) or w < 0 or w > 100 or (0 < w < 1e-8)
           for w in weights.values()) or sum(weights[k] for k in ("surface", "depth", "rgb")) <= 0:
        raise ValueError("权重必须为 0 或 1e-8–100 内的有限数，D435 三项不能全部为零")
    color = cfg["color"]
    for key in ("black_value", "white_value", "neutral_saturation"):
        value = color[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 <= value <= 255:
            raise ValueError(f"color.{key} 必须在 0–255 之间")
    fraction = color["min_color_fraction"]
    if isinstance(fraction, bool) or not isinstance(fraction, (int, float)) or not math.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError("color.min_color_fraction 必须是 0–1 内的有限数")
    if color["black_value"] >= color["white_value"]:
        raise ValueError("颜色明度阈值顺序或比例不正确")
    bounds = color["hue_boundaries"]
    if not isinstance(bounds, list) or len(bounds) != 7 or any(isinstance(v, bool) or not isinstance(v, (int, float))
            or not math.isfinite(v) for v in bounds) or not 0 < bounds[0] < bounds[-1] < 180 or any(a >= b for a, b in zip(bounds, bounds[1:])):
        raise ValueError("hue_boundaries 必须为 0–180 内严格递增的七个数")
    if cfg["roi"] is not None:
        roi = cfg["roi"]
        if not isinstance(roi, list) or len(roi) != 4 or any(isinstance(v, bool) or not isinstance(v, int) for v in roi):
            raise ValueError("roi 必须是四个整数 [x,y,width,height]")
        x, y, w, h = roi
        if x < 0 or y < 0 or w <= 0 or h <= 0 or x+w > cfg["width"] or y+h > cfg["height"]:
            raise ValueError("roi 必须位于 RGB 图像范围内")
    return cfg


def load_config(path):
    return validate_config(json.loads(Path(path).read_text(encoding="utf-8-sig")))


def save_config(path, config):
    cfg = validate_config(config)
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    try:
        with temporary.open("w", encoding="utf-8") as handle:
            json.dump(cfg, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def apply_preset(config, name):
    if name not in PRESETS:
        raise ValueError(f"未知预设 {name}")
    return validate_config(_merge(config, PRESETS[name]))


def runtime_config(current, candidate, image_size=None):
    config = validate_config(candidate)
    changed = [key for key in CAPTURE_KEYS if config[key] != current[key]]
    if changed:
        raise ValueError("采集参数 " + ", ".join(changed) + " 需重启程序才能改变")
    if image_size and config["roi"]:
        x, y, w, h = config["roi"]
        if x+w > image_size[0] or y+h > image_size[1]:
            raise ValueError("ROI 超出当前视频帧；请重新框选")
    return config
