"""Index an external STEP directory and prepare project-local simulation scenes.

Reference labels below are used only in validation reports, never in recognition.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from rgbd_geometry.cad_simulation import OBJECT_DEFAULTS, SCENE_DEFAULTS, read_step
from rgbd_geometry.geometry import footprint_box

ROOT = Path(__file__).resolve().parent
BASELINE_LABELS = {"A01": "cube", "A02": "cuboid", "A03": "cylinder",
                   "A05": "cone", "A06": "triangular_pyramid", "A10": "triangular_prism"}
REFERENCE_LABELS = {**BASELINE_LABELS, "A04": "sphere", "A07": "quadrangular_pyramid",
                    "A08": "pentagonal_pyramid", "A09": "hexagonal_pyramid",
                    "A11": "pentagonal_prism", "A12": "hexagonal_prism"}
COLORS = [[220, 40, 35], [40, 180, 45], [230, 120, 30],
          [175, 40, 180], [30, 65, 210], [210, 210, 30]]


def scene_for(items):
    scene = copy.deepcopy(SCENE_DEFAULTS)
    count = len(items)
    columns = min(3, count)
    rows = (count+columns-1)//columns
    largest = max(max(item["axis_span_mm"][:2]) for item in items)
    spacing = max(80., largest*1.7)
    extent = max(columns*spacing, rows*spacing)
    scene["camera"].update(distance_mm=max(320., extent*1.8), target_mm=[0., 0., 20.])
    scene["secondary_camera"] = {**copy.deepcopy(scene["camera"]), "azimuth_deg": -20., "elevation_deg": 75.}
    scene["objects"] = []
    for index, item in enumerate(items):
        row, column = divmod(index, columns)
        obj = copy.deepcopy(OBJECT_DEFAULTS)
        obj.update(file=item["file"], name=item["name"], color_rgb=COLORS[index%len(COLORS)],
                   position_mm=[(column-(columns-1)/2)*spacing, (row-(rows-1)/2)*spacing])
        scene["objects"].append(obj)
    return scene


def prepare(directory, output):
    directory, output = Path(directory).resolve(), Path(output).resolve()
    if not directory.is_dir():
        raise ValueError(f"模型目录不存在: {directory}")
    paths = sorted(path for path in directory.rglob("*") if path.is_file() and path.suffix.lower() in (".step", ".stp"))
    if not paths:
        raise ValueError("目录中没有 STEP/STP 文件")
    output.mkdir(parents=True, exist_ok=True)
    singles = output/"single"
    singles.mkdir(exist_ok=True)
    inventory = []
    for index, path in enumerate(paths):
        item = {"file": str(path), "name": path.stem, "code": path.stem.split("_")[0]}
        try:
            mesh = read_step(path)
            item.update(axis_span_mm=(np.ptp(mesh.vertices_m, axis=0)*1000).round(3).tolist(),
                        table_bbox_mm=(footprint_box(mesh.vertices_m)[0]*1000).round(3).tolist(),
                        triangles=len(mesh.triangles))
            item["reference_shape"] = REFERENCE_LABELS.get(item["code"])
            # Numeric names cannot collide if CAD stems repeat in subfolders.
            scene_path = singles/f"{index+1:02d}_{item['code']}.json"
            scene_path.write_text(json.dumps(scene_for([item]), ensure_ascii=False, indent=2), encoding="utf-8")
            item["scene"] = str(scene_path)
        except (ValueError, RuntimeError, OSError) as exc:
            item["error"] = str(exc)
        inventory.append(item)
        print(f"[{index+1}/{len(paths)}] {item['name']}: {item.get('table_bbox_mm', item.get('error'))}", flush=True)
    valid = [item for item in inventory if "error" not in item]
    if not valid:
        raise ValueError("没有成功导入的 STEP 文件")
    selected = [item for code in BASELINE_LABELS for item in valid if item["code"] == code]
    if not selected:
        selected = valid[:6]
    overview = output/"basic_six.json"
    overview.write_text(json.dumps(scene_for(selected), ensure_ascii=False, indent=2), encoding="utf-8")
    from rgbd_geometry.configuration import load_config, save_config
    small_config = output/"config_40mm.json"
    if not small_config.exists():
        settings = load_config(ROOT/"config.json")
        settings.update(min_object_height_m=.003, surface_noise_m=.0015)
        save_config(small_config, settings)
    extended_config = output/"config_extended.json"
    if not extended_config.exists():
        from rgbd_geometry.geometry import EXTRA_SHAPES
        settings = load_config(small_config)
        settings.update(extra_shapes=list(EXTRA_SHAPES), surface_score_sigma=.6, depth_score_sigma=.8,
                        score_smoothing_window=7,
                        weights={"surface": .30, "depth": .25, "rgb": .10, "secondary_rgb": .35})
        save_config(extended_config, settings)
    groups = []
    for category in sorted({item["code"][:1] for item in valid}):
        members = [item for item in valid if item["code"].startswith(category)]
        for start in range(0, len(members), 6):
            path = output/f"group_{category}_{start//6+1:02d}.json"
            path.write_text(json.dumps(scene_for(members[start:start+6]), ensure_ascii=False, indent=2), encoding="utf-8")
            groups.append(str(path))
    manifest = {"source_directory": str(directory), "imported": len(valid),
                "failed": len(inventory)-len(valid), "default_scene": str(overview),
                "group_scenes": groups, "items": inventory, "algorithm_config": str(extended_config),
                "color_note": "场景颜色为指定的仿真颜色，不是从 STEP 材质读取。",
                "recognition_note": "导入/渲染支持不等于类别支持；reference_shape 仅作评估标签。"}
    (output/"library.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    return manifest


def validate_library(manifest, output, config_path):
    from app import analyze
    from rgbd_geometry.cad_simulation import CadSceneSource
    from rgbd_geometry.configuration import load_config
    from rgbd_geometry.vision import ModelFitter, Tracker

    config = load_config(config_path)
    reports = []
    for index, item in enumerate(manifest["items"]):
        if "error" in item:
            continue
        source = None
        record = {"name": item["name"], "code": item["code"], "reference_shape": item["reference_shape"]}
        try:
            source = CadSceneSource(item["scene"], config)
            fitter, tracker = ModelFitter(config), Tracker(config)
            for _ in range(max(8, config["min_stable_frames"], config.get("score_smoothing_window", 1))):
                frame = source.read()
                observations, elapsed, _, _ = analyze(frame, source.table, source.background, config, fitter,
                                                      source.secondary_fusion, source.secondary_frame(frame))
                results = tracker.update(observations, frame.timestamp)
            for result in results:
                result["color"]["source"] = "simulated_rgb"
            record.update(rendered=True, detected_count=len(results), reference_bbox_mm=source.reference_objects[0]["dimensions_mm"],
                          results=[{k: v for k, v in result.items() if not k.startswith("_")} for result in results])
            if item["reference_shape"]:
                record["reference_shape_correct"] = len(results) == 1 and results[0]["shape"] == item["reference_shape"]
        except (ValueError, RuntimeError, OSError) as exc:
            record["error"] = str(exc)
        finally:
            if source:
                source.close()
        reports.append(record)
        print(f"[{index+1}/{len(manifest['items'])}] {item['code']}: {[r['shape'] for r in record.get('results', [])]} {record.get('error', '')}", flush=True)
    summary = {"input_source": "step_mesh_simulation", "config_file": str(Path(config_path).resolve()),
               "observations_per_model": max(8, config["min_stable_frames"], config.get("score_smoothing_window", 1)),
               "models": len(reports), "rendered": sum(report.get("rendered", False) for report in reports),
               "basic_shape_correct": sum(report.get("reference_shape_correct", False) for report in reports),
               "basic_shape_evaluated": sum("reference_shape_correct" in report for report in reports),
               "limitation": "Fixed synthetic cameras and noise; no real-camera accuracy claim. Unsupported classes may be rejected or incorrectly mapped to one of the enabled basic classes.",
               "reports": reports}
    path = Path(output)/"recognition_validation.json"
    path.write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT/"models"/"user_blocks")
    parser.add_argument("--config", type=Path, default=ROOT/"config.json")
    parser.add_argument("--validate", action="store_true")
    args = parser.parse_args()
    manifest = prepare(args.directory, args.output)
    print("DEFAULT SCENE:", manifest["default_scene"], flush=True)
    if args.validate:
        print("VALIDATION:", validate_library(manifest, args.output, args.config), flush=True)
