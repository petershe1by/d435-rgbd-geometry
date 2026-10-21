"""Summarize app.py JSONL logs; throughput is not the sensor's hardware FPS."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np


def summarize(path: Path, skip: int = 3) -> dict:
    if skip < 0:
        raise ValueError("skip 不能为负数")
    records = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line)
                for key in ("processing_s", "result_age_s", "output_unix_s", "capture_monotonic_s"):
                    value = record[key]
                    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
                        raise ValueError(f"无效字段 {key}")
                if record["processing_s"] < 0 or record["result_age_s"] < 0:
                    raise ValueError("耗时不能为负数")
                if not isinstance(record["objects"], list) or not all(
                    isinstance(obj, dict) and isinstance(obj.get("shape"), str)
                    for obj in record["objects"]
                ):
                    raise ValueError("无效 objects")
                records.append(record)
            except (ValueError, KeyError, TypeError) as exc:
                raise ValueError(f"{path} 第 {line_number} 行不是完整有效的结果: {exc}") from exc
    selected = records[skip:]
    if not selected:
        raise ValueError(f"共有 {len(records)} 条结果，跳过 {skip} 条后没有可统计记录")
    processing = np.array([record["processing_s"] for record in selected])
    ages = np.array([record["result_age_s"] for record in selected])
    shapes = Counter(obj["shape"] for record in selected for obj in record["objects"])
    total_objects = sum(shapes.values())
    output_span = selected[-1]["output_unix_s"] - selected[0]["output_unix_s"]
    observed_span = selected[-1]["capture_monotonic_s"] - selected[0]["capture_monotonic_s"]
    configuration_revisions = sorted({str(record.get("configuration_revision", "missing")) for record in selected})
    return {
        "file": str(path.resolve()),
        "total_records": len(records),
        "skipped_records": skip,
        "evaluated_records": len(selected),
        "processing_ms": {"mean": float(processing.mean()*1000),
                          "median": float(np.median(processing)*1000),
                          "p95": float(np.percentile(processing, 95)*1000),
                          "max": float(processing.max()*1000)},
        "host_receipt_to_output_ms": {"median": float(np.median(ages)*1000),
                                      "p95": float(np.percentile(ages, 95)*1000)},
        "result_output_hz": (len(selected)-1)/output_span if len(selected) > 1 and output_span > 0 else None,
        "analyzed_observation_hz": (len(selected)-1)/observed_span if len(selected) > 1 and observed_span > 0 else None,
        "frames_without_detections": sum(not record["objects"] for record in selected),
        "detected_object_observations": total_objects,
        "unknown_object_fraction": shapes.get("unknown", 0)/total_objects if total_objects else None,
        "shape_observations": dict(sorted(shapes.items())),
        "configuration_revisions": configuration_revisions,
        "limitations": [
            "无人工真值，不能计算识别准确率、漏检率或尺寸误差。unknown 比例仅针对已检出目标。",
            "输出频率和被分析帧频率不是相机采集 FPS；离线回放频率也不是实时吞吐率。",
            "帧龄从主机收到帧起计算，不包含此前的曝光、USB 传输或显示器延迟。",
            "多个参数版本或不同目标数量混在同一日志时，应分段比较。",
        ],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="统计实机或回放 JSONL；不把输出频率当相机 FPS")
    parser.add_argument("log", type=Path)
    parser.add_argument("--skip", type=int, default=3, help="跳过开头确认阶段的结果条数，默认 3")
    args = parser.parse_args()
    try:
        summary = summarize(args.log, args.skip)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
