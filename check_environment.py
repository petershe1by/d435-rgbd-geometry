"""Check the project-local runtime; a missing D435 is reported separately."""
from __future__ import annotations

import argparse
import importlib
from importlib.metadata import version
from pathlib import Path
import struct
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description="检查项目虚拟环境、依赖、GUI 和 RealSense 枚举")
    parser.add_argument("--gui", action="store_true", help="实际测试 Tk 与 OpenCV 窗口创建和关闭")
    parser.add_argument("--require-camera", action="store_true", help="未连接相机时返回非零退出码")
    parser.add_argument("--profiles", action="store_true", help="列出设备支持的深度/RGB 流组合")
    parser.add_argument("--capture", action="store_true", help="按配置实际采集 5 帧 RGB-D，不保存图像")
    parser.add_argument("--config", type=Path, default=Path(__file__).resolve().parent / "config.json")
    args = parser.parse_args()
    expected_prefix = Path(__file__).resolve().parent / ".venv"
    failures = []
    print(f"Python: {sys.version.split()[0]} ({struct.calcsize('P')*8} bit)")
    print(f"Interpreter: {sys.executable}")
    if Path(sys.prefix).resolve() != expected_prefix.resolve():
        failures.append("请使用本项目 .venv\\Scripts\\python.exe")
    if struct.calcsize("P") != 8:
        failures.append("需要 64 位 Python")
    config_path = expected_prefix / "pyvenv.cfg"
    if not config_path.exists() or "include-system-site-packages = false" not in config_path.read_text(encoding="utf-8").lower():
        failures.append("虚拟环境尚未与全局第三方包隔离")
    modules = {}
    for package, module_name in (("numpy", "numpy"), ("scipy", "scipy"),
                                 ("opencv-python", "cv2"), ("pyrealsense2", "pyrealsense2")):
        try:
            module = importlib.import_module(module_name)
            location = Path(module.__file__).resolve()
            print(f"{package}: {version(package)} | {location}")
            if not location.is_relative_to(expected_prefix.resolve()):
                failures.append(f"{package} 未从项目虚拟环境加载")
            modules[module_name] = module
        except (ImportError, OSError, RuntimeError) as exc:
            failures.append(f"{package}: {exc}")
    try:
        import tkinter as tk
        print(f"Tk: {tk.TkVersion}")
        if args.gui:
            root = tk.Tk()
            try:
                root.withdraw()
                root.update()
            finally:
                root.destroy()
            if "cv2" in modules:
                cv2 = modules["cv2"]
                try:
                    cv2.namedWindow("RGBD environment check", cv2.WINDOW_NORMAL)
                    cv2.waitKey(1)
                finally:
                    cv2.destroyAllWindows()
            print("GUI check passed")
    except Exception as exc:
        failures.append(f"GUI: {exc}")
    if "pyrealsense2" in modules:
        try:
            rs = modules["pyrealsense2"]
            devices = rs.context().query_devices()
            print(f"RealSense devices: {len(devices)}")
            for device in devices:
                print(f"  {device.get_info(rs.camera_info.name)} | serial={device.get_info(rs.camera_info.serial_number)}")
                if device.supports(rs.camera_info.usb_type_descriptor):
                    print(f"  USB connection: {device.get_info(rs.camera_info.usb_type_descriptor)}")
                if args.profiles:
                    profiles = set()
                    for sensor in device.query_sensors():
                        for profile in sensor.get_stream_profiles():
                            if ((profile.stream_type() == rs.stream.depth and profile.format() == rs.format.z16)
                                or (profile.stream_type() == rs.stream.color and profile.format() == rs.format.bgr8)):
                                video = profile.as_video_stream_profile()
                                profiles.add((str(profile.stream_type()), video.width(), video.height(), profile.fps()))
                    for stream, width, height, fps in sorted(profiles):
                        print(f"  {stream}: {width}x{height} @ {fps} fps")
            if not devices:
                print("未连接 RealSense；可运行 --demo，接入 D435 后再验证采集。")
                if args.require_camera or args.capture:
                    failures.append("未找到 RealSense 相机")
        except Exception as exc:
            failures.append(f"RealSense enumeration: {exc}")
    if args.capture and not failures:
        source = None
        try:
            from rgbd_geometry.cameras import RealSenseSource
            from rgbd_geometry.configuration import load_config
            source = RealSenseSource(load_config(args.config))
            for index in range(5):
                frame = source.read()
                valid_fraction = float((frame.depth > 0).mean())
                print(f"RGB-D frame {index+1}: RGB={frame.rgb.shape}, depth={frame.depth.shape}, valid={valid_fraction:.1%}", flush=True)
            print("实际 RGB-D 采集检查通过。")
        except Exception as exc:
            failures.append(f"RGB-D capture: {exc}")
        finally:
            if source is not None:
                source.close()
    if failures:
        for failure in failures:
            print(f"FAILED: {failure}", file=sys.stderr)
        return 1
    print("项目运行环境检查通过。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
