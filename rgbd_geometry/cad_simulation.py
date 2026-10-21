"""STEP -> independent triangle z-buffer -> RGB-D, using metric optical-Z depth.

CAD filenames and reference dimensions are never supplied to the recognizer.
OCP is optional and imported only when a STEP file is opened.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass
from functools import lru_cache
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from .cameras import Frame, SecondaryFusion
from .geometry import Intrinsics, TableFrame, footprint_box

SCENE_DEFAULTS = {
    "camera": {"azimuth_deg": -90., "elevation_deg": 52., "distance_mm": 850.,
               "target_mm": [0., 0., 35.], "horizontal_fov_deg": 58.},
    "secondary_camera": None,
    "noise_sigma_mm": .8, "hole_fraction": .02, "rgb_noise_sigma": 1.,
    "shading": .12, "mesh_tolerance_mm": .15, "seed": 1,
    "table_color_rgb": [170, 170, 170], "objects": [],
}
OBJECT_DEFAULTS = {"name": "STEP object", "position_mm": [0., 0.],
                   "rotation_deg": [0., 0., 0.], "color_rgb": [220, 45, 35],
                   "scale": 1., "yaw_speed_deg_s": 0.}


def _number(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{name} 必须为 {low}–{high} 范围内的有限数")
    return float(value)


def _vector(value, name, count, low, high):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{name} 需要 {count} 个数")
    return [_number(v, name, low, high) for v in value]


def validate_scene(data, base_dir):
    if not isinstance(data, dict):
        raise ValueError("场景必须是 JSON 对象")
    unknown = set(data) - set(SCENE_DEFAULTS)
    if unknown:
        raise ValueError(f"未知场景参数: {sorted(unknown)}")
    scene = copy.deepcopy(SCENE_DEFAULTS)
    if not isinstance(data.get("camera", {}), dict):
        raise ValueError("camera 必须为对象")
    if set(data.get("camera", {})) - set(scene["camera"]):
        raise ValueError("未知 camera 参数")
    scene.update(copy.deepcopy({k: v for k, v in data.items() if k != "camera"}))
    scene["camera"].update(data.get("camera", {}))
    camera = scene["camera"]
    for key, low, high in (("azimuth_deg", -360, 360), ("elevation_deg", 15, 89),
                           ("distance_mm", 200, 3000), ("horizontal_fov_deg", 20, 100)):
        camera[key] = _number(camera[key], key, low, high)
    camera["target_mm"] = _vector(camera["target_mm"], "target_mm", 3, -3000, 3000)
    if scene["secondary_camera"] is not None:
        second = scene["secondary_camera"]
        if not isinstance(second, dict) or set(second)-set(SCENE_DEFAULTS["camera"]):
            raise ValueError("secondary_camera 参数格式不正确")
        second = {**copy.deepcopy(SCENE_DEFAULTS["camera"]), **second}
        for key, low, high in (("azimuth_deg", -360, 360), ("elevation_deg", 15, 89),
                               ("distance_mm", 200, 3000), ("horizontal_fov_deg", 20, 100)):
            second[key] = _number(second[key], key, low, high)
        second["target_mm"] = _vector(second["target_mm"], "target_mm", 3, -3000, 3000)
        scene["secondary_camera"] = second
    for key, low, high in (("noise_sigma_mm", 0, 20), ("hole_fraction", 0, .9),
                           ("rgb_noise_sigma", 0, 50), ("shading", 0, .8),
                           ("mesh_tolerance_mm", .01, 2)):
        scene[key] = _number(scene[key], key, low, high)
    scene["table_color_rgb"] = _vector(scene["table_color_rgb"], "table_color_rgb", 3, 0, 255)
    if isinstance(scene["seed"], bool) or not isinstance(scene["seed"], int) or not 0 <= scene["seed"] < 2**32:
        raise ValueError("seed 必须是 0–2^32-1 范围的整数")
    if not isinstance(scene["objects"], list) or not 1 <= len(scene["objects"]) <= 32:
        raise ValueError("场景需要 1–32 个 STEP 物体")
    for index, item in enumerate(scene["objects"]):
        if not isinstance(item, dict) or set(item) - (set(OBJECT_DEFAULTS) | {"file"}):
            raise ValueError(f"物体 {index+1} 格式或参数不正确")
        obj = {**copy.deepcopy(OBJECT_DEFAULTS), **item}
        if not isinstance(obj.get("file"), str) or not obj["file"].strip():
            raise ValueError("每个物体需要 STEP 文件路径")
        path = Path(obj["file"])
        if not path.is_absolute():
            path = Path(base_dir) / path
        path = path.resolve()
        if path.suffix.lower() not in (".step", ".stp") or not path.is_file():
            raise ValueError(f"STEP 文件不存在或扩展名不正确: {path}")
        obj["file"] = str(path)
        if not isinstance(obj["name"], str) or not obj["name"].strip():
            raise ValueError("物体 name 不能为空")
        obj["position_mm"] = _vector(obj["position_mm"], "position_mm", 2, -3000, 3000)
        obj["rotation_deg"] = _vector(obj["rotation_deg"], "rotation_deg", 3, -360, 360)
        obj["color_rgb"] = _vector(obj["color_rgb"], "color_rgb", 3, 0, 255)
        obj["scale"] = _number(obj["scale"], "scale", .0001, 1000)
        obj["yaw_speed_deg_s"] = _number(obj["yaw_speed_deg_s"], "yaw_speed_deg_s", -180, 180)
        scene["objects"][index] = obj
    return scene


def load_scene(path):
    path = Path(path).resolve()
    return validate_scene(json.loads(path.read_text(encoding="utf-8-sig")), path.parent)


@dataclass(frozen=True)
class StepMesh:
    vertices_m: np.ndarray
    triangles: np.ndarray


@lru_cache(maxsize=12)
def _cached_step(path_string, modified_ns, file_size, tolerance_mm):
    del modified_ns, file_size  # Included in the cache key to invalidate edited CAD files.
    try:
        from OCP.BRep import BRep_Tool
        from OCP.BRepMesh import BRepMesh_IncrementalMesh
        from OCP.IFSelect import IFSelect_RetDone
        from OCP.STEPControl import STEPControl_Reader
        from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
        from OCP.TopExp import TopExp_Explorer
        from OCP.TopLoc import TopLoc_Location
        from OCP.TopoDS import TopoDS
    except ImportError as exc:
        raise RuntimeError("STEP 仿真需要安装 requirements-simulation.txt，真实相机模式不需要此依赖") from exc
    reader = STEPControl_Reader()
    if reader.ReadFile(path_string) != IFSelect_RetDone:
        raise ValueError(f"无法读取 STEP 文件: {path_string}")
    # OCCT STEP translator converts declared source units into this millimetre system.
    reader.SetSystemLengthUnit(1.)
    if reader.TransferRoots() == 0:
        raise ValueError("STEP 中没有可转换的几何体")
    shape = reader.OneShape()
    mesher = BRepMesh_IncrementalMesh(shape, tolerance_mm, False, .20, False)
    if not mesher.IsDone():
        raise ValueError("STEP 三角化失败，请检查几何有效性")
    vertices, faces, offset = [], [], 0
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        face = TopoDS.Face_s(explorer.Current())
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, location)
        if mesh is not None:
            transform = location.Transformation()
            nodes = np.array([list(mesh.Node(i).Transformed(transform).Coord()) for i in range(1, mesh.NbNodes()+1)])
            indices = np.array([mesh.Triangle(i).Get() for i in range(1, mesh.NbTriangles()+1)], dtype=np.int32) - 1
            if face.Orientation() == TopAbs_REVERSED:
                indices = indices[:, [0, 2, 1]]
            vertices.append(nodes)
            faces.append(indices + offset)
            offset += len(nodes)
            if sum(len(part) for part in faces) > 100_000 or offset > 300_000:
                raise ValueError("模型网格过大（超过 10 万三角形），请简化 CAD 或增大 mesh_tolerance_mm")
        explorer.Next()
    if not vertices or not faces:
        raise ValueError("STEP 中没有可渲染的面")
    points = np.vstack(vertices).astype(np.float64) * .001
    triangles = np.vstack(faces)
    if not np.isfinite(points).all() or len(triangles) == 0:
        raise ValueError("STEP 网格为空或包含无效坐标")
    points.setflags(write=False)
    triangles.setflags(write=False)
    return StepMesh(points, triangles)


def read_step(path, tolerance_mm=.15):
    path = Path(path).resolve()
    stat = path.stat()
    return _cached_step(str(path), stat.st_mtime_ns, stat.st_size, float(tolerance_mm))


def virtual_camera(width, height, camera):
    azimuth, elevation = np.radians([camera["azimuth_deg"], camera["elevation_deg"]])
    target = np.asarray(camera["target_mm"], float) * .001
    offset = camera["distance_mm"]*.001 * np.array([
        np.cos(elevation)*np.cos(azimuth), np.cos(elevation)*np.sin(azimuth), np.sin(elevation)])
    eye = target + offset
    if eye[2] <= .02:
        raise ValueError("虚拟相机必须位于桌面上方")
    forward = -offset / np.linalg.norm(offset)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    camera_to_world = np.column_stack((right, down, forward))
    focal = width / (2*np.tan(np.radians(camera["horizontal_fov_deg"])/2))
    intrinsics = Intrinsics(width, height, focal, focal, width/2, height/2)
    return intrinsics, TableFrame(-eye @ camera_to_world, camera_to_world.T, .0006)


def place_mesh(mesh, spec, elapsed=0.):
    angles = np.asarray(spec["rotation_deg"], float).copy()
    angles[2] += spec["yaw_speed_deg_s"]*elapsed
    rotation = Rotation.from_euler("xyz", angles, degrees=True).as_matrix()
    vertices = (mesh.vertices_m*spec["scale"]) @ rotation.T
    # One STEP file is one rigid object; preserve relative geometry of its solids.
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    vertices -= [(low[0]+high[0])/2, (low[1]+high[1])/2, low[2]]
    vertices[:, :2] += np.asarray(spec["position_mm"])*.001
    if np.max(np.ptp(vertices, axis=0)) > 5 or np.max(np.ptp(vertices, axis=0)) < .001:
        raise ValueError("模型尺寸不在 1 mm–5 m 范围；请检查 STEP 单位和 scale")
    return vertices


def fit_scene_camera(scene, config):
    """Frame known CAD assets; only images/depth, not CAD labels, enter recognition."""
    scene = copy.deepcopy(scene)
    placed = [place_mesh(read_step(obj["file"], scene["mesh_tolerance_mm"]), obj) for obj in scene["objects"]]
    vertices = np.vstack(placed)
    for key in ("camera", "secondary_camera"):
        camera = scene[key]
        if camera is None:
            continue
        near = config["min_depth_m"]+.015 if key == "camera" else .04
        far = config["max_depth_m"]-.015 if key == "camera" else 10.
        for _ in range(35):
            intr, table = virtual_camera(config["width"], config["height"], camera)
            points = table.to_camera(vertices)
            uv = intr.project(points)
            depth_ok = points[:, 2].min() > near and points[:, 2].max() < far
            frame_ok = (uv[:, 0].min() > 8 and uv[:, 1].min() > 8
                        and uv[:, 0].max() < intr.width-8 and uv[:, 1].max() < intr.height-8)
            if depth_ok and frame_ok:
                break
            camera["distance_mm"] *= 1.10
            if camera["distance_mm"] > 3000 or points[:, 2].max() > far:
                raise ValueError("无法在当前测距与视场内容纳所有物块；请减少物块间距或调整相机目标点")
        else:
            raise ValueError("自动取景未收敛，请调整物块摆放范围")
    return scene


def rasterize(meshes, intrinsics, table, table_rgb=(170, 170, 170), shading=.12):
    """Perspective-correct, double-sided triangle z-buffer; no convex hull shortcut."""
    height, width = intrinsics.height, intrinsics.width
    rays = intrinsics.deproject(np.ones((height, width))) @ table.basis
    eye = table.to_world(np.zeros((1, 3)))[0]
    valid = rays[..., 2] < -1e-8
    depth = np.zeros((height, width), float)
    depth[valid] = -eye[2]/rays[..., 2][valid]
    rgb = np.empty((height, width, 3), np.uint8)
    rgb[:] = np.asarray(table_rgb, dtype=np.uint8)[::-1]
    owners = np.full((height, width), -1, np.int16)
    light = np.array([.3, -.5, 1.])
    light /= np.linalg.norm(light)
    for identity, (vertices, triangles, color_rgb) in enumerate(meshes):
        camera_vertices = table.to_camera(vertices)
        if np.any(camera_vertices[:, 2] <= .02):
            raise ValueError("模型穿过虚拟相机近裁剪面，请调整相机距离或摆放位置")
        projected = intrinsics.project(camera_vertices)
        for indices in triangles:
            uv, z = projected[indices], camera_vertices[indices, 2]
            low = np.maximum(np.floor(uv.min(axis=0)).astype(int), [0, 0])
            high = np.minimum(np.ceil(uv.max(axis=0)).astype(int)+1, [width, height])
            if np.any(high <= low):
                continue
            a, b, c = uv
            denominator = (b[1]-c[1])*(a[0]-c[0])+(c[0]-b[0])*(a[1]-c[1])
            if abs(denominator) < 1e-10:
                continue
            y, x = np.mgrid[low[1]:high[1], low[0]:high[0]]
            w0 = ((b[1]-c[1])*(x-c[0])+(c[0]-b[0])*(y-c[1])) / denominator
            w1 = ((c[1]-a[1])*(x-c[0])+(a[0]-c[0])*(y-c[1])) / denominator
            w2 = 1-w0-w1
            inside = (w0 >= -1e-8) & (w1 >= -1e-8) & (w2 >= -1e-8)
            inverse_z = w0/z[0]+w1/z[1]+w2/z[2]
            candidate = np.divide(1., inverse_z, out=np.zeros_like(inverse_z), where=inverse_z > 0)
            current = depth[low[1]:high[1], low[0]:high[0]]
            visible = inside & (candidate > 0) & ((current == 0) | (candidate < current))
            if not visible.any():
                continue
            current[visible] = candidate[visible]
            tri = vertices[indices]
            normal = np.cross(tri[1]-tri[0], tri[2]-tri[0])
            normal /= max(np.linalg.norm(normal), 1e-15)
            brightness = 1-shading+shading*max(float(normal @ light), 0.)
            rgb[low[1]:high[1], low[0]:high[0]][visible] = np.clip(np.asarray(color_rgb)[::-1]*brightness, 0, 255).astype(np.uint8)
            owners[low[1]:high[1], low[0]:high[0]][visible] = identity
    return rgb, depth.astype(np.float32), owners


class CadSceneSource:
    def __init__(self, scene_path, config):
        self.scene_path = Path(scene_path).resolve()
        self.scene = load_scene(self.scene_path)
        self.config = config.copy()
        self.intrinsics, self.table = virtual_camera(config["width"], config["height"], self.scene["camera"])
        self.meshes = [read_step(obj["file"], self.scene["mesh_tolerance_mm"]) for obj in self.scene["objects"]]
        self.background = np.full((config["height"], config["width"], 3),
                                  np.asarray(self.scene["table_color_rgb"])[::-1], dtype=np.uint8)
        self.index = 0
        self.secondary_fusion = None
        if self.scene["secondary_camera"] is not None:
            self.secondary_intrinsics, self.secondary_table = virtual_camera(config["width"], config["height"], self.scene["secondary_camera"])
            rotation = self.secondary_table.basis @ self.table.basis.T
            translation = self.secondary_table.origin - self.table.origin @ rotation.T
            self.secondary_fusion = SecondaryFusion({
                "usb_matrix": self.secondary_intrinsics.matrix.tolist(), "usb_distortion": [0.]*5,
                "rotation_d435_to_usb": rotation.tolist(), "translation_d435_to_usb_m": translation.tolist(),
                "usb_image_size": [config["width"], config["height"]],
                "d435_image_size": [config["width"], config["height"]],
                "d435_matrix_rectified": self.intrinsics.matrix.tolist(), "stereo_rms_px": 0.})
            self.secondary_fusion.background = self.background.copy()
        self.dynamic = any(obj["yaw_speed_deg_s"] for obj in self.scene["objects"])
        self._buffers = None
        self._render(0.)

    def _render(self, elapsed):
        placed = [(place_mesh(mesh, obj, elapsed), mesh.triangles, obj["color_rgb"])
                  for mesh, obj in zip(self.meshes, self.scene["objects"])]
        self._buffers = rasterize(placed, self.intrinsics, self.table,
                                  self.scene["table_color_rgb"], self.scene["shading"])
        if self.secondary_fusion is not None:
            self._secondary_rgb = rasterize(placed, self.secondary_intrinsics, self.secondary_table,
                                            self.scene["table_color_rgb"], self.scene["shading"])[0]
        self.reference_objects = [{"name": obj["name"], "file": obj["file"],
                                   "dimensions_mm": (footprint_box(vertices)[0]*1000).round(3).tolist(),
                                   "color_rgb": obj["color_rgb"], "triangles": len(triangles)}
                                  for obj, (vertices, triangles, _) in zip(self.scene["objects"], placed)]

    def read(self):
        if self.dynamic:
            self._render(self.index/self.config["fps"])
        self.index += 1
        rgb, depth, _ = self._buffers
        rng = np.random.default_rng(np.random.SeedSequence([self.scene["seed"], self.index]))
        result_depth = depth.copy()
        valid = depth > 0
        result_depth[valid] += rng.normal(0, self.scene["noise_sigma_mm"]*.001, valid.sum())
        result_depth[(rng.random(depth.shape) < self.scene["hole_fraction"]) | ~valid] = 0
        result_depth[result_depth < 0] = 0
        result_rgb = rgb.copy()
        if self.scene["rgb_noise_sigma"]:
            result_rgb = np.clip(result_rgb.astype(float)+rng.normal(0, self.scene["rgb_noise_sigma"], rgb.shape), 0, 255).astype(np.uint8)
        return Frame(result_rgb, result_depth, time.monotonic(), self.intrinsics, self.index)

    def configure(self, config):
        self.config = config.copy()

    def secondary_frame(self, frame):
        if self.secondary_fusion is None:
            return None
        image = self._secondary_rgb.copy()
        if self.scene["rgb_noise_sigma"]:
            rng = np.random.default_rng(np.random.SeedSequence([self.scene["seed"], self.index, 3]))
            image = np.clip(image.astype(float)+rng.normal(0, self.scene["rgb_noise_sigma"], image.shape), 0, 255).astype(np.uint8)
        return image, frame.timestamp

    def camera_health(self):
        _, depth, owners = self._buffers
        result = []
        for index, spec in enumerate(self.scene["objects"]):
            mask = owners == index
            count = int(mask.sum())
            outside = mask & ((depth <= self.config["min_depth_m"]) | (depth >= self.config["max_depth_m"]))
            border = bool(mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any())
            result.append({"name": spec["name"], "visible_pixels": count,
                           "outside_depth_fraction": int(outside.sum())/max(count, 1),
                           "border_truncated": border,
                           "closest_depth_mm": float(depth[mask].min()*1000) if count else None})
        return result

    def close(self):
        pass
