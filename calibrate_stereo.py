"""Calibrate D435 rectified RGB -> USB RGB using a metric chessboard."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import time
import cv2
import numpy as np
from rgbd_geometry.cameras import RealSenseSource, USBSource
from rgbd_geometry.configuration import load_config

ROOT = Path(__file__).resolve().parent


def calibrate(object_points, first_points, second_points, first_matrix, first_size, second_size):
    if len(object_points) < 15:
        raise ValueError("至少需要 15 组不同位置、不同倾角的棋盘格；推荐 25 组")
    if not (len(object_points) == len(first_points) == len(second_points)):
        raise ValueError("棋盘格点和两路图像点组数不一致")
    usb_rms, usb_matrix, usb_dist, _, _ = cv2.calibrateCamera(
        object_points, second_points, second_size, None, None)
    # D435 color from RealSenseSource has already been rectified with known intrinsics.
    stereo = cv2.stereoCalibrate(object_points, first_points, second_points,
                                first_matrix, np.zeros(5), usb_matrix, usb_dist, first_size,
                                flags=cv2.CALIB_FIX_INTRINSIC,
                                criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 150, 1e-7))
    rms, _, _, _, _, rotation, translation, _, _ = stereo
    if not np.isfinite(rms) or rms > 2.5 or usb_rms > 2.5:
        raise ValueError(f"标定误差过大: stereo={rms:.2f}px USB={usb_rms:.2f}px；检查棋盘尺寸、清晰度、对应角点及相机是否移动")
    return {"d435_image_size": list(first_size), "usb_image_size": list(second_size),
            "d435_matrix_rectified": first_matrix.tolist(), "usb_matrix": usb_matrix.tolist(),
            "usb_distortion": usb_dist.tolist(), "rotation_d435_to_usb": rotation.tolist(),
            "translation_d435_to_usb_m": translation.reshape(3).tolist(),
            "stereo_rms_px": float(rms), "usb_rms_px": float(usb_rms),
            "pair_count": len(object_points), "translation_units": "metres"}


def main():
    p = argparse.ArgumentParser(description="D435 + USB chessboard stereo calibration")
    p.add_argument("--usb", type=int, required=True)
    p.add_argument("--config", type=Path, default=ROOT/"config.json")
    p.add_argument("--cols", type=int, default=9, help="INNER corners, not squares")
    p.add_argument("--rows", type=int, default=6)
    p.add_argument("--square-mm", type=float, default=25)
    p.add_argument("--usb-width", type=int, default=1280)
    p.add_argument("--usb-height", type=int, default=720)
    p.add_argument("--pairs", type=int, default=25)
    p.add_argument("--output", type=Path, default=ROOT/"stereo_calibration.json")
    args = p.parse_args()
    if args.square_mm <= 0 or min(args.cols, args.rows) < 3 or args.pairs < 15:
        raise ValueError("棋盘尺寸必须为正，内角点至少 3x3，至少 15 组")
    config = load_config(args.config)
    source = usb = None
    pattern = (args.cols, args.rows)
    points = np.zeros((args.cols*args.rows, 3), np.float32)
    points[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_mm/1000
    first_points, second_points, object_points = [], [], []
    try:
        source = RealSenseSource(config)
        usb = USBSource(args.usb, (args.usb_width, args.usb_height))
        print("固定两台相机。缓慢变换棋盘位置/倾角，静止后按空格采集，q 退出。", flush=True)
        last_pair = None
        while len(object_points) < args.pairs:
            frame, other = source.read(), usb.read()
            if frame is None:
                raise RuntimeError("D435 RGB-D 流已结束")
            if other is None:
                cv2.waitKey(1)
                continue
            image, stamp = other
            ok1, c1 = cv2.findChessboardCornersSB(cv2.cvtColor(frame.rgb, cv2.COLOR_BGR2GRAY), pattern)
            ok2, c2 = cv2.findChessboardCornersSB(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), pattern)
            display1, display2 = frame.rgb.copy(), image.copy()
            if ok1:
                cv2.drawChessboardCorners(display1, pattern, c1, True)
            if ok2:
                cv2.drawChessboardCorners(display2, pattern, c2, True)
            cv2.putText(display1, f"pairs {len(object_points)}/{args.pairs} | SPACE: capture | q: quit",
                        (8, 25), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 1)
            cv2.imshow("D435 calibration", display1)
            cv2.imshow("USB calibration", display2)
            key = cv2.waitKey(1) & 0xff
            if key in (ord("q"), 27):
                return
            if key != ord(" "):
                continue
            if not ok1 or not ok2:
                print("两路画面必须同时检测到完整棋盘格", flush=True)
                continue
            if abs(frame.timestamp-stamp) > config["max_secondary_skew_s"]:
                print("两路采集时差过大，请保持棋盘静止后重试", flush=True)
                continue
            if last_pair is not None and np.mean(np.linalg.norm(c1-last_pair, axis=2)) < 12:
                print("棋盘姿态变化不足；请更换位置或倾角", flush=True)
                continue
            first_points.append(c1.astype(np.float32))
            second_points.append(c2.astype(np.float32))
            object_points.append(points.copy())
            last_pair = c1.copy()
            print(f"Captured {len(object_points)} pairs", flush=True)
        data = calibrate(object_points, first_points, second_points, frame.intrinsics.matrix,
                         (frame.intrinsics.width, frame.intrinsics.height), (image.shape[1], image.shape[0]))
        data.update({"inner_corners": list(pattern), "square_mm": args.square_mm, "created_unix_s": time.time()})
        args.output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"Saved {args.output}; stereo RMS={data['stereo_rms_px']:.3f}px")
    finally:
        if usb:
            usb.close()
        if source:
            source.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
