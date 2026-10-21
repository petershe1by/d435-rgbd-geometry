"""Reproducible synthetic streaming benchmark; this is NOT a hardware accuracy test."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from scipy.optimize import linear_sum_assignment
from rgbd_geometry.synthetic import DemoSource
from rgbd_geometry.geometry import footprint_box
from rgbd_geometry.configuration import load_config

ROOT = Path(__file__).resolve().parent


def summarize(path, config):
    records = [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines()]
    if not records:
        raise ValueError("没有已分析帧")
    warm = [record["processing_s"] for record in records
            if record["objects"] and all(obj["search_mode"] == "temporal" for obj in record["objects"])]
    truth = DemoSource(config).objects
    centers = np.array([model.vertices(params).mean(axis=0) for model, params, _ in truth])
    sizes = [footprint_box(model.vertices(params))[0]*1000 for model, params, _ in truth]
    expected = {model.shape for model, _, _ in truth}
    complete, correct, evaluated, errors = 0, 0, 0, []
    for record in records[config["min_stable_frames"]-1:]:
        objects = record["objects"]
        if {obj["shape"] for obj in objects} == expected:
            complete += 1
        if not objects:
            continue
        measured_centers = np.array([obj["center_table_m"] for obj in objects])
        rows, cols = linear_sum_assignment(np.linalg.norm(measured_centers[:, None]-centers[None], axis=2))
        for row, col in zip(rows, cols):
            evaluated += 1
            correct += objects[row]["shape"] == truth[col][0].shape
            if objects[row]["dimensions_mm"] is not None:
                errors.extend(np.abs(np.array(objects[row]["dimensions_mm"])-sizes[col]).tolist())
    times = [r["processing_s"] for r in records]
    return {"frames": len(records), "image_size": [config["width"], config["height"]], "objects": len(truth),
            "first_frame_s": times[0], "warm_median_s": float(np.median(warm)) if warm else None,
            "warm_p95_s": float(np.percentile(warm, 95)) if warm else None,
            "overall_mean_s": float(np.mean(times)), "slowest_frame_s": max(times),
            "full_search_frames": len(records)-len(warm), "all_six_confirmed_frames_after_warmup": complete,
            "evaluated_objects_after_warmup": evaluated, "correct_objects_after_warmup": int(correct),
            "max_model_dimension_error_mm": float(max(errors)) if errors else None,
            "mean_model_dimension_error_mm": float(np.mean(errors)) if errors else None,
            "limitation": "Synthetic renderer shares model family with fitter; not real-camera generalization or accuracy evidence."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT/"config.json")
    parser.add_argument("--preset", choices=("fast", "balanced", "precise"), default="balanced")
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--output-dir", type=Path, default=ROOT/"benchmarks")
    args = parser.parse_args()
    if args.frames < 3:
        raise ValueError("至少测试 3 帧")
    config = load_config(args.config)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir/f"{args.preset}.jsonl"
    subprocess.run([sys.executable, str(ROOT/"app.py"), "--demo", "--headless", "--every-frame", "--quiet",
                    "--config", str(args.config.resolve()), "--preset", args.preset, "--frames", str(args.frames),
                    "--output", str(output.resolve())], check=True, cwd=ROOT)
    summary = summarize(output, config)
    summary["preset"] = args.preset
    path = args.output_dir/f"{args.preset}_summary.json"
    path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
