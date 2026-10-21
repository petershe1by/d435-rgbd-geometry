from __future__ import annotations

from dataclasses import dataclass
import threading
import time
import cv2
import numpy as np
from .geometry import Intrinsics


@dataclass
class Frame:
    rgb: np.ndarray
    depth: np.ndarray
    timestamp: float  # host monotonic receipt time; not a hardware sync guarantee
    intrinsics: Intrinsics
    frame_number: int


class RealSenseSource:
    def __init__(self, config, bag=None, record=None, serial=None):
        try:
            import pyrealsense2 as rs
        except ImportError as exc:
            raise RuntimeError("请先安装 pyrealsense2，或使用 --demo 无相机运行") from exc
        self.rs = rs
        self.pipeline, self.align = rs.pipeline(), rs.align(rs.stream.color)
        settings = rs.config()
        if bag:
            settings.enable_device_from_file(str(bag), repeat_playback=False)
        else:
            if serial:
                settings.enable_device(serial)
            settings.enable_stream(rs.stream.depth, config["width"], config["height"], rs.format.z16, config["fps"])
            settings.enable_stream(rs.stream.color, config["width"], config["height"], rs.format.bgr8, config["fps"])
        if record:
            settings.enable_record_to_file(str(record))
        self.closed = False
        self.spatial = rs.spatial_filter()
        self.spatial.set_option(rs.option.holes_fill, 0)
        self.to_disparity, self.to_depth = rs.disparity_transform(True), rs.disparity_transform(False)
        self.temporal = rs.temporal_filter(.6, 20, 0) if config.get("temporal_depth_filter") else None
        self.stream_description = f"{config['width']}x{config['height']} @ {config['fps']} fps"
        self.usb_type = "unknown"
        try:
            self.profile = self.pipeline.start(settings)
        except RuntimeError as exc:
            if not bag and "resolve requests" in str(exc).lower():
                raise RuntimeError(
                    f"无法解析 RGB/深度流配置：{self.stream_description}。"
                    "请在 config.json 使用两路共同支持的配置，例如 640x480 @ 15 fps；"
                    "关闭 RealSense Viewer 等占用相机的程序，并检查 USB 数据线和端口。"
                    f" SDK 原因：{exc}"
                ) from exc
            raise
        self.playback = None
        try:
            if bag:
                self.playback = self.profile.get_device().as_playback()
                self.playback.set_real_time(False)
                self.playback.pause()
            device = self.profile.get_device()
            self.scale = device.first_depth_sensor().get_depth_scale()
            if not bag and device.supports(rs.camera_info.usb_type_descriptor):
                usb_type = device.get_info(rs.camera_info.usb_type_descriptor)
                if isinstance(usb_type, str):
                    self.usb_type = usb_type
                    print(f"RealSense streams: {self.stream_description}; USB {self.usb_type}", flush=True)
                    if self.usb_type.startswith("2"):
                        print("相机当前使用 USB 2.x 连接；建议换用 USB 3 数据线及端口后重新检查。", flush=True)
        except Exception:
            self.pipeline.stop()
            raise
        self.map_x = self.map_y = None
        self.config = config

    def read(self):
        try:
            if self.playback:
                if self.playback.current_status() == self.rs.playback_status.stopped:
                    return None
                self.playback.resume()
            frames = self.pipeline.wait_for_frames(5000)
            timestamp = time.monotonic()
            if self.playback and self.playback.current_status() != self.rs.playback_status.stopped:
                self.playback.pause()
        except RuntimeError as exc:
            if self.playback and self.playback.current_status() == self.rs.playback_status.stopped:
                return None
            if not self.playback and "frame didn't arrive" in str(exc).lower():
                raise RuntimeError(
                    f"相机已启动但未收到 RGB-D 帧：{self.stream_description}，USB {self.usb_type}。"
                    "请关闭其他相机程序，检查 Windows 相机访问权限、USB 数据线/端口，"
                    "并使用 RealSense Viewer 检查两路是否实际出图。"
                    f" SDK 原因：{exc}"
                ) from exc
            raise
        # Alignment guarantees matching RGB/depth pixels. Avoid fabricated hole fills.
        aligned = self.align.process(frames)
        raw, color = aligned.get_depth_frame(), aligned.get_color_frame()
        if not raw or not color:
            raise RuntimeError("RGB-D 帧缺少颜色或深度，检查流配置和设备连接")
        raw_valid = np.asanyarray(raw.get_data()) > 0
        filtered = self.to_disparity.process(raw)
        filtered = self.spatial.process(filtered)
        if self.temporal:
            filtered = self.temporal.process(filtered)
        filtered = self.to_depth.process(filtered)
        depth = np.asanyarray(filtered.get_data()).astype(np.float32) * self.scale
        depth[~raw_valid] = 0
        rgb = np.asanyarray(color.get_data()).copy()
        intr = color.profile.as_video_stream_profile().get_intrinsics()
        pinhole = Intrinsics(intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
        if np.any(np.abs(intr.coeffs) > 1e-9):
            if intr.model != self.rs.distortion.brown_conrady:
                raise RuntimeError(f"不支持的非零 RGB 畸变模型 {intr.model}；请使用 SDK 校正后的流")
            if self.map_x is None:
                self.map_x, self.map_y = cv2.initUndistortRectifyMap(
                    pinhole.matrix, np.array(intr.coeffs), None, pinhole.matrix,
                    (intr.width, intr.height), cv2.CV_32FC1)
            rgb = cv2.remap(rgb, self.map_x, self.map_y, cv2.INTER_LINEAR)
            depth = cv2.remap(depth, self.map_x, self.map_y, cv2.INTER_NEAREST)
        return Frame(rgb, depth, timestamp, pinhole, raw.get_frame_number())

    def configure(self, config):
        enabled = bool(config["temporal_depth_filter"])
        if enabled != (self.temporal is not None):
            self.temporal = self.rs.temporal_filter(.6, 20, 0) if enabled else None
        self.config = config.copy()

    def close(self):
        if not self.closed:
            self.closed = True
            self.pipeline.stop()


class USBSource:
    """Capture on a separate thread so a slow USB camera does not block depth."""
    def __init__(self, index, size=None):
        self.capture = cv2.VideoCapture(index)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(f"无法打开 USB 相机 {index}")
        if size:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self.lock, self.latest, self.stopped = threading.Lock(), None, False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stopped:
                ok, image = self.capture.read()
                stamp = time.monotonic()
                if ok:
                    with self.lock:
                        self.latest = (image, stamp)
                else:
                    time.sleep(.02)
        finally:
            self.capture.release()

    def read(self):
        with self.lock:
            if self.latest is None:
                return None
            return self.latest[0].copy(), self.latest[1]

    def close(self):
        self.stopped = True
        self.thread.join(timeout=2)
        # Do not release a handle on a different thread while a driver is in read().


class SecondaryFusion:
    def __init__(self, path):
        import json
        import hashlib
        if isinstance(path, dict):
            data = path
        else:
            with open(path, encoding="utf-8") as handle:
                data = json.load(handle)
        self.signature = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()[:24]
        self.K = np.array(data["usb_matrix"], float)
        self.distortion = np.array(data["usb_distortion"], float)
        self.rotation = np.array(data["rotation_d435_to_usb"], float)
        self.translation = np.array(data["translation_d435_to_usb_m"], float).reshape(3)
        self.size = tuple(data["usb_image_size"])
        self.primary_size = tuple(data["d435_image_size"])
        self.rms = float(data["stereo_rms_px"])
        self.primary_matrix = np.array(data["d435_matrix_rectified"], float) if "d435_matrix_rectified" in data else None
        if self.K.shape != (3, 3) or self.rotation.shape != (3, 3) or not np.isfinite(self.K).all() or not np.isfinite(self.translation).all():
            raise ValueError("双相机标定矩阵格式或数值不正确")
        if not np.isfinite(self.rotation).all() or not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-4) or np.linalg.det(self.rotation) < .999:
            raise ValueError("双相机标定旋转矩阵不合法")
        if not np.isfinite(self.distortion).all() or self.distortion.size not in (4, 5, 8, 12, 14):
            raise ValueError("USB 畸变参数不合法")
        if not np.isfinite(self.rms) or not 0 <= self.rms <= 2.5 or any(not isinstance(v, int) or v <= 0 for v in (*self.size, *self.primary_size)):
            raise ValueError("双相机标定误差或图像尺寸不合法")
        self.intrinsics = Intrinsics(*self.size, self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2])
        self.background = None
        self.map_x, self.map_y = cv2.initUndistortRectifyMap(self.K, self.distortion, None, self.K, self.size, cv2.CV_32FC1)

    def rectify(self, image):
        if (image.shape[1], image.shape[0]) != self.size:
            raise ValueError("USB 分辨率与标定文件不一致；请重新标定")
        return cv2.remap(image, self.map_x, self.map_y, cv2.INTER_LINEAR)

    def masks(self, image, config=None):
        if self.background is None:
            return []
        if image.shape != self.background.shape:
            raise ValueError("侧视背景分辨率不匹配，请清空桌面重新按 b 标定")
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB).astype(float)
        reference = cv2.cvtColor(self.background, cv2.COLOR_BGR2LAB).astype(float)
        difference = lab-reference
        difference[:, :, 0] -= np.median(difference[:, :, 0])
        fg = (np.linalg.norm(difference, axis=2) > (config or {}).get("background_threshold", 25)).astype(np.uint8)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        masks = []
        for contour in contours:
            if cv2.contourArea(contour) < 120:
                continue
            mask = np.zeros(fg.shape, np.uint8)
            cv2.drawContours(mask, [contour], -1, 1, -1)
            masks.append(mask)
        return masks

    def associate(self, detections, masks, table):
        from scipy.optimize import linear_sum_assignment
        if not detections or not masks:
            return {}
        centers, radii = [], []
        for mask in masks:
            y, x = np.where(mask > 0)
            centers.append([x.mean(), y.mean()])
            radii.append(max(15., float(np.hypot(np.ptp(x), np.ptp(y))) / 2))
        projected = []
        for d in detections:
            camera = table.to_camera(d.points) @ self.rotation.T + self.translation
            uv = self.intrinsics.project(camera)
            projected.append(np.mean(uv, axis=0) if np.all(camera[:, 2] > 0) else [1e6, 1e6])
        cost = np.linalg.norm(np.array(projected)[:, None] - np.array(centers)[None], axis=2)
        cost /= np.array(radii)[None]
        allowed = cost < 1.1
        for row in range(len(detections)):
            ordered = np.sort(cost[row])
            if len(ordered) > 1 and ordered[1]-ordered[0] < .20:
                allowed[row] = False
        for col in range(len(masks)):
            if np.sum(cost[:, col] < .8) > 1:
                allowed[:, col] = False
        rows, cols = linear_sum_assignment(np.where(allowed, cost, 1e6))
        output = {}
        for row, col in zip(rows, cols):
            if allowed[row, col]:
                output[int(row)] = int(col)
        return output
