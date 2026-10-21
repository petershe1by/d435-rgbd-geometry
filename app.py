"""RealSense D435 tabletop geometry recognizer. See README.md for calibration."""
from __future__ import annotations
import argparse
import copy
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
from pathlib import Path
import time
import sys
import cv2
import numpy as np
from rgbd_geometry.geometry import Intrinsics, TableFrame, fit_table
from rgbd_geometry.cameras import RealSenseSource, USBSource, SecondaryFusion
from rgbd_geometry.synthetic import DemoSource
from rgbd_geometry.vision import ModelFitter, Tracker, View, segment, color_estimate
from rgbd_geometry.configuration import load_config, save_config, apply_preset, runtime_config

ROOT = Path(__file__).resolve().parent


def save_workspace(path, frame, table, background, secondary=None, stereo_signature=None):
    data = {"origin": table.origin, "basis": table.basis,
            "plane_residual_m": np.array(table.residual_m), "background": background,
            "intrinsics_json": np.array(json.dumps(asdict(frame.intrinsics)))}
    if secondary is not None:
        data["secondary_background"] = secondary
        if stereo_signature:
            data["stereo_signature"] = np.array(stereo_signature)
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    try:
        with temporary.open("wb") as handle:
            np.savez_compressed(handle, **data)
        temporary.replace(path)
    finally:
        if temporary.exists():
            temporary.unlink()


def load_workspace(path, intrinsics, stereo_signature=None):
    with np.load(path, allow_pickle=False) as data:
        saved = json.loads(str(data["intrinsics_json"]))
        current = asdict(intrinsics)
        if any(abs(saved[k] - current[k]) > .01 for k in current):
            raise ValueError("桌面标定与当前 RGB 内参/分辨率不匹配；请重新按 b 标定")
        table = TableFrame(data["origin"].copy(), data["basis"].copy(), float(data["plane_residual_m"]))
        bg = data["background"].copy()
        secondary = data["secondary_background"].copy() if "secondary_background" in data else None
        if secondary is not None and stereo_signature and ("stereo_signature" not in data or str(data["stereo_signature"]) != stereo_signature):
            secondary = None
            print("Stereo calibration changed or background signature missing: press b on EMPTY table to recapture side background.", flush=True)
        if bg.shape != (intrinsics.height, intrinsics.width, 3) or bg.dtype != np.uint8:
            raise ValueError("桌面 RGB 背景文件格式不正确，请重新标定")
    return table, bg, secondary


def calibrate_workspace(frame, config, samples=None):
    depth = frame.depth.copy()
    if samples:
        stack = np.stack([sample.depth for sample in samples])
        valid_samples = np.isfinite(stack) & (stack > config["min_depth_m"]) & (stack < config["max_depth_m"])
        depth = np.ma.median(np.ma.array(stack, mask=~valid_samples), axis=0).filled(0).astype(np.float32)
    valid = (depth > config["min_depth_m"]) & (depth < config["max_depth_m"])
    roi = config.get("roi")
    if roi:
        x, y, w, h = map(int, roi)
        allowed = np.zeros(depth.shape, bool)
        allowed[max(0, y):y+h, max(0, x):x+w] = True
        valid &= allowed
    points = frame.intrinsics.deproject(depth)[valid]
    return fit_table(points, config["plane_threshold_m"])


def analyze(frame, table, background, config, fitter, fusion=None, secondary_frame=None):
    start = time.monotonic()
    detections = segment(frame.rgb, frame.depth, frame.intrinsics, table, config, background)
    secondary_image, associations, masks = None, {}, []
    if fusion is not None and secondary_frame is not None:
        image, stamp = secondary_frame
        if abs(frame.timestamp-stamp) <= config["max_secondary_skew_s"]:
            secondary_image = fusion.rectify(image)
            masks = fusion.masks(secondary_image, config)
            associations = fusion.associate(detections, masks, table)
    observations = []
    for i, detection in enumerate(detections):
        primary = View(detection.mask, frame.intrinsics, table)
        secondary = None
        if i in associations:
            quality = float(np.clip(1/(1+fusion.rms/3), .2, 1))
            secondary = View(masks[associations[i]], fusion.intrinsics, table,
                             fusion.rotation, fusion.translation, quality)
        observation = fitter.fit(detection, primary, secondary)
        observation["color"] = color_estimate(frame.rgb, detection.mask, config)
        observation["color"]["source"] = "d435_rgb"
        if secondary is not None:
            second_color = color_estimate(secondary_image, secondary.mask, config)
            primary_color = observation["color"].copy()
            observation["color"]["secondary"] = second_color
            if second_color["name"] == primary_color["name"]:
                observation["color"]["confidence"] = round((primary_color["confidence"] + second_color["confidence"])/2, 3)
                observation["color"]["source"] = "both_rgb_views"
            elif primary_color["confidence"] > .65 and second_color["confidence"] > .65:
                observation["color"]["name"] = "mixed"
                observation["color"]["name_zh"] = "混合色/视角不一致"
                observation["color"]["primary"] = primary_color
                observation["flags"].append("view_color_disagreement")
        observations.append(observation)
    return observations, time.monotonic()-start, frame, table


def overlay(image, results, table, intrinsics, compact=False):
    image = image.copy()
    for result in results:
        mask = result["_mask"] > 0
        color = (40, 190, 40) if result["shape"] != "unknown" else (0, 180, 255)
        image[mask] = (image[mask] * .75 + np.array(color) * .25).astype(np.uint8)
        camera_box = table.to_camera(result["_box"])
        if not np.isfinite(camera_box).all() or np.any(camera_box[:, 2] <= .02) or not mask.any():
            continue
        uv = np.clip(intrinsics.project(camera_box), -10000, 10000).astype(int)
        for a, b in [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                     (0, 4), (1, 5), (2, 6), (3, 7)]:
            cv2.line(image, tuple(uv[a]), tuple(uv[b]), color, 1, cv2.LINE_AA)
        y, x = np.where(mask)
        dims = result["dimensions_mm"]
        text = (f"#{result['id']}" if compact else
                f"#{result['id']} {result['shape']} {result['confidence']:.2f} {result['color']['name']}")
        if dims and not compact:
            text += " " + "x".join(f"{d:.0f}" for d in dims) + "mm"
        cv2.putText(image, text, (int(x.min()), max(35, int(y.min())-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1, cv2.LINE_AA)
    return image


def simulation_overlay(image, results, table, intrinsics):
    """Keep long class names and dimensions out of a crowded virtual image."""
    drawing = overlay(image, results, table, intrinsics, compact=True)
    height, width = drawing.shape[:2]
    rows_per_column = max(1, (height-65)//58)
    columns = max(1, (len(results)+rows_per_column-1)//rows_per_column)
    canvas = np.full((max(height, 125), width+columns*320, 3), 242, np.uint8)
    canvas[:height, :width] = drawing
    cv2.putText(canvas, "STEP SIMULATION", (width+12, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (50, 50, 50), 1, cv2.LINE_AA)
    for index, result in enumerate(results):
        column, row = divmod(index, rows_per_column)
        x, y = width+column*320+12, 66+row*58
        color = (40, 130, 40) if result["shape"] != "unknown" else (0, 120, 190)
        dims = result["dimensions_mm"]
        label = result['shape'] if result['shape'] != 'unknown' else f"? {result['candidate_shape']}"
        lines = [f"#{result['id']}  {label}",
                 " x ".join(f"{d:.1f}" for d in dims)+" mm" if dims else result.get("decision_reason", "pending / unknown"),
                 f"{result['color']['name']} | score {result['confidence']:.2f} gap {result['score_margin']:.2f}"]
        for offset, text in enumerate(lines):
            cv2.putText(canvas, text, (x, y+offset*15), cv2.FONT_HERSHEY_SIMPLEX, .42, color, 1, cv2.LINE_AA)
    return canvas


def public_record(results, frame, elapsed):
    return {"frame_number": int(frame.frame_number), "capture_monotonic_s": frame.timestamp,
            "output_unix_s": time.time(), "processing_s": round(elapsed, 4),
            "result_age_s": round(time.monotonic()-frame.timestamp, 4),
            "size_definition": "table_aligned_oriented_bounding_cuboid_L_W_H_mm",
            "objects": [{k: v for k, v in r.items() if not k.startswith("_")} for r in results]}


def parse_args():
    p = argparse.ArgumentParser(description="D435 RGB-D simple geometry recognition")
    p.add_argument("--config", type=Path, default=ROOT/"config.json")
    p.add_argument("--workspace", type=Path, default=ROOT/"workspace_calibration.npz")
    p.add_argument("--usb", type=int, help="USB camera index; requires --stereo for fusion")
    p.add_argument("--stereo", type=Path, help="stereo_calibration.json")
    p.add_argument("--serial", help="RealSense device serial")
    p.add_argument("--bag", type=Path, help="Replay a RealSense .bag recording")
    p.add_argument("--record", type=Path, help="Record live raw RGB-D to .bag")
    p.add_argument("--output", type=Path, default=ROOT/"results.jsonl")
    p.add_argument("--demo", action="store_true", help="No hardware required")
    p.add_argument("--scene", type=Path, help="STEP scene JSON; virtual RGB-D input without a D435")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--frames", type=int, default=0, help="Capture frame limit; 0=unlimited")
    p.add_argument("--every-frame", action="store_true", help="Analyze each frame synchronously; useful for offline validation")
    p.add_argument("--preset", choices=("fast", "balanced", "precise"), help="Runtime preset; does not overwrite the file")
    p.add_argument("--no-tuning", action="store_true", help="Disable the Tk parameter panel")
    p.add_argument("--watch-config", action="store_true", help="Reload edited config safely after the current analysis")
    p.add_argument("--quiet", action="store_true", help="Suppress per-frame console summaries")
    p.add_argument("--calibrate-plane", action="store_true", help="EMPTY table only; save and exit")
    return p.parse_args()


def main():
    args = parse_args()
    config = load_config(args.config)
    if args.preset:
        config = apply_preset(config, args.preset)
    if args.frames < 0:
        raise ValueError("--frames 必须是非负整数")
    if args.calibrate_plane and 0 < args.frames < 45:
        raise ValueError("空桌面标定需要至少 45 帧曝光稳定时间；请增大 --frames 或保留默认无限制")
    virtual_input = args.demo or args.scene is not None
    if args.demo and args.scene is not None:
        raise ValueError("--demo 与 --scene 只能选择一种")
    if virtual_input and (args.usb is not None or args.bag or args.record or args.calibrate_plane or args.serial):
        raise ValueError("虚拟输入不与实物相机/录制/标定选项组合")
    if args.bag and (args.usb is not None or args.record):
        raise ValueError("回放不能与实时 USB 或录制组合；回放双相机需要同步录制的第二视频")
    if (args.usb is None) != (args.stereo is None):
        raise ValueError("双视角融合请同时指定 --usb 和 --stereo")
    source = usb = handle = tuner = None
    executor = ThreadPoolExecutor(max_workers=1)
    future = None
    try:
        if args.scene is not None:
            from rgbd_geometry.cad_simulation import CadSceneSource
            source = CadSceneSource(args.scene, config)
        else:
            source = DemoSource(config) if args.demo else RealSenseSource(config, args.bag, args.record, args.serial)
        fusion = source.secondary_fusion if args.scene is not None else (SecondaryFusion(args.stereo) if args.stereo else None)
        usb = USBSource(args.usb, fusion.size) if args.usb is not None else None
        fitter, tracker = ModelFitter(config), Tracker(config)
        if not args.headless and not args.no_tuning:
            try:
                from rgbd_geometry.tuning import TuningPanel
                tuner = TuningPanel(config, args.config)
            except ImportError:
                print("Tkinter unavailable: use config.json and the l reload hotkey.", flush=True)
        pending_config, config_revision = None, 0
        config_stamp = args.config.stat().st_mtime_ns
        last_watch = 0.
        table, background, loaded = None, None, False
        last_submit, processed, captured = -1e9, 0, 0
        last_results, last_frame, last_table, last_elapsed = [], None, None, 0.
        calibration_samples = deque(maxlen=7)
        capture_ticks = deque(maxlen=30)
        if not args.calibrate_plane:
            handle = args.output.open("w", encoding="utf-8")

        def finish_job():
            nonlocal last_results, last_frame, last_table, last_elapsed, processed
            observations, elapsed, analyzed_frame, analyzed_table = future.result()
            results = tracker.update(observations, analyzed_frame.timestamp)
            if args.scene is not None:
                for result in results:
                    result["color"]["source"] = ("simulated_rgb_views" if result["color"].get("source") == "both_rgb_views" else "simulated_rgb")
            record = public_record(results, analyzed_frame, elapsed)
            record["configuration_revision"] = config_revision
            if args.scene is not None:
                record["input_source"] = "step_mesh_simulation"
                record["scene_file"] = str(args.scene.resolve())
            handle.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
            handle.flush()
            summary = [(r["id"], r["shape"], r["candidate_shape"], r["candidate_dimensions_mm"], r["color"]["name"])
                       for r in results]
            if not args.quiet:
                print(f"frame={analyzed_frame.frame_number} processing={elapsed:.2f}s objects={summary}", flush=True)
            last_results, last_frame, last_table, last_elapsed = results, analyzed_frame, analyzed_table, elapsed
            processed += 1
            if tuner:
                tuner.stats.set(f"分析 {elapsed:.3f}s / 帧 | {len(results)} 个目标 | 确认 {sum(r['shape'] != 'unknown' for r in results)} | 参数版本 {config_revision}")

        while not args.frames or captured < args.frames:
            if tuner:
                tuner.pump()
                if tuner.pending is not None:
                    pending_config, tuner.pending = tuner.pending, None
            frame = source.read()
            if frame is None:
                break
            captured += 1
            capture_ticks.append(time.monotonic())
            calibration_samples.append(frame)
            if not loaded:
                loaded = True
                if virtual_input:
                    table = source.table
                    background = getattr(source, "background", None)
                elif args.workspace.exists() and not args.calibrate_plane:
                    try:
                        table, background, secondary_bg = load_workspace(args.workspace, frame.intrinsics, fusion.signature if fusion else None)
                        if fusion:
                            fusion.background = secondary_bg
                    except (ValueError, KeyError, OSError) as error:
                        if args.headless:
                            raise
                        print(f"Calibration unavailable: {error}; clear table and press b.", flush=True)
                if fusion and (frame.intrinsics.width, frame.intrinsics.height) != fusion.primary_size:
                    raise ValueError("D435 分辨率与双相机标定不一致")
                if fusion and fusion.primary_matrix is not None and not np.allclose(fusion.primary_matrix, frame.intrinsics.matrix, atol=.05, rtol=0):
                    raise ValueError("D435 RGB 内参与双相机标定不一致，请重新标定")
                runtime_config(config, config, (frame.intrinsics.width, frame.intrinsics.height))
            secondary_frame = source.secondary_frame(frame) if args.scene is not None else (usb.read() if usb else None)
            if args.calibrate_plane:
                # Let auto-exposure settle, then median the last seven depth frames.
                if captured < 45:
                    continue
                table = calibrate_workspace(frame, config, list(calibration_samples))
                secondary_bg = fusion.rectify(secondary_frame[0]) if secondary_frame else None
                if fusion and secondary_bg is None:
                    raise RuntimeError("USB 没有视频帧，不能保存完整桌面标定")
                save_workspace(args.workspace, frame, table, frame.rgb, secondary_bg, fusion.signature if fusion else None)
                print(f"Saved {args.workspace}; plane RMS {table.residual_m*1000:.2f} mm")
                return
            if table is None and args.headless:
                raise RuntimeError("无桌面标定。先清空桌面运行 --calibrate-plane，或用界面按 b 标定")
            if future is not None and future.done():
                finish_job()
                future = None
            now = time.monotonic()
            if args.watch_config and now-last_watch > .5:
                last_watch = now
                try:
                    stamp = args.config.stat().st_mtime_ns
                    if stamp != config_stamp:
                        config_stamp = stamp
                        pending_config = load_config(args.config)
                except (ValueError, TypeError, OSError) as error:
                    print(f"Config reload rejected; previous parameters retained: {error}", flush=True)
            if pending_config is not None and future is None:
                try:
                    candidate = runtime_config(config, pending_config, (frame.intrinsics.width, frame.intrinsics.height))
                    if candidate != config:
                        if hasattr(source, "configure"):
                            source.configure(candidate)
                        config = candidate
                        fitter, tracker = ModelFitter(config), Tracker(config)
                        config_revision += 1
                        last_results, last_frame = [], None
                        print(f"Applied configuration revision {config_revision}", flush=True)
                    if tuner:
                        tuner.set_config(config)
                        tuner.status.set("已应用；结果重新确认中。改变桌面标定阈值后请按 b 重新标定。")
                except (ValueError, TypeError, OSError) as error:
                    print(f"Config change rejected: {error}", flush=True)
                    if tuner:
                        tuner.set_config(config)
                        tuner.status.set(f"未应用：{error}")
                pending_config = None
            if table is not None and future is None and (args.every_frame or args.bag or now-last_submit >= 1/config["analysis_hz"]):
                future = executor.submit(analyze, frame, table, background, copy.deepcopy(config), fitter, fusion, secondary_frame)
                last_submit = now
                if args.every_frame or args.bag:
                    finish_job()
                    future = None
            if not args.headless:
                live = frame.rgb.copy()
                if config["roi"]:
                    x, y, w, h = config["roi"]
                    cv2.rectangle(live, (x, y), (x+w, y+h), (0, 255, 0), 2)
                state = ("VIRTUAL STEP RGB-D | r: ROI | 1/2/3: preset | t: tuning | q: quit" if args.scene is not None else
                         "b: EMPTY calibration | r: ROI | s: save | l: reload | 1/2/3: preset | t: tuning | q: quit")
                cv2.putText(live, state, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
                if len(capture_ticks) > 1:
                    capture_fps = (len(capture_ticks)-1)/max(capture_ticks[-1]-capture_ticks[0], .001)
                    cv2.putText(live, f"capture {capture_fps:.1f}fps | config rev {config_revision}", (8, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1)
                cv2.imshow("STEP virtual RGB" if args.scene is not None else "D435 live RGB", live)
                depth_display = cv2.applyColorMap(cv2.convertScaleAbs(frame.depth, alpha=150), cv2.COLORMAP_TURBO)
                depth_display[frame.depth <= 0] = 0
                cv2.imshow("STEP virtual depth" if args.scene is not None else "D435 depth", depth_display)
                # Show detections on their own captured frame, so moving objects never
                # receive boxes copied from an earlier frame in the live window.
                if last_frame is not None:
                    renderer = simulation_overlay if args.scene is not None else overlay
                    result_image = renderer(last_frame.rgb, last_results, last_table, last_frame.intrinsics)
                    age = time.monotonic()-last_frame.timestamp
                    cv2.putText(result_image, f"ANALYZED frame {last_frame.frame_number} | age {age:.2f}s | fit {last_elapsed:.2f}s",
                                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 255, 255), 1)
                    cv2.imshow("Recognition - captured frame", result_image)
                if secondary_frame:
                    cv2.imshow("Virtual secondary RGB" if args.scene is not None else "USB RGB", secondary_frame[0])
                key = cv2.waitKey(1) & 0xff
                if key in (ord("q"), 27):
                    break
                if key == ord("t") and tuner:
                    tuner.show()
                if key == ord("s"):
                    if tuner:
                        tuner.save()
                    else:
                        save_config(args.config, config)
                if key == ord("l"):
                    try:
                        pending_config = load_config(args.config)
                    except (ValueError, TypeError, OSError) as error:
                        print(f"Config reload rejected: {error}", flush=True)
                if key in (ord("1"), ord("2"), ord("3")):
                    pending_config = apply_preset(pending_config or config, ("fast", "balanced", "precise")[key-ord("1")])
                    if tuner:
                        tuner.set_config(pending_config)
                if key == ord("r"):
                    roi = cv2.selectROI("Select workspace ROI", frame.rgb, showCrosshair=True, fromCenter=False)
                    cv2.destroyWindow("Select workspace ROI")
                    if roi[2] > 0 and roi[3] > 0:
                        pending_config = copy.deepcopy(pending_config or config)
                        pending_config["roi"] = list(map(int, roi))
                        if tuner:
                            tuner.set_config(pending_config)
                if key == ord("b"):
                    if virtual_input:
                        print("虚拟输入使用已知桌面，不写入真实相机的标定文件。", flush=True)
                        continue
                    if future is not None:
                        future.result()
                        future = None
                    try:
                        candidate_table = calibrate_workspace(frame, config, list(calibration_samples))
                    except ValueError as error:
                        print(f"Calibration rejected; previous plane retained: {error}", flush=True)
                        continue
                    candidate_background = np.median(np.stack([sample.rgb for sample in calibration_samples]), axis=0).astype(np.uint8)
                    secondary_bg = fusion.rectify(secondary_frame[0]) if secondary_frame else None
                    if fusion:
                        if secondary_bg is None:
                            raise RuntimeError("USB 没有视频帧，无法标定空桌面背景")
                    save_workspace(args.workspace, frame, candidate_table, candidate_background, secondary_bg, fusion.signature if fusion else None)
                    table, background = candidate_table, candidate_background
                    if fusion:
                        fusion.background = secondary_bg
                    tracker = Tracker(config)
                    fitter.cache = []
                    last_results, last_frame = [], None
                    print(f"Calibration saved; plane RMS={table.residual_m*1000:.2f}mm", flush=True)
            if virtual_input:
                time.sleep(1/config["fps"])
        if future is not None:
            finish_job()
            future = None
        print(f"Captured {captured} frames; analyzed {processed}; results: {args.output}")
    finally:
        executor.shutdown(wait=True, cancel_futures=True)
        if tuner:
            tuner.close()
        if handle:
            handle.close()
        if usb:
            usb.close()
        if source:
            source.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.")
    except (ValueError, RuntimeError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)
