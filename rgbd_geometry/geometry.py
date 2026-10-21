from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import numpy as np
from scipy.spatial import ConvexHull

SHAPES = ("cube", "cuboid", "triangular_pyramid", "triangular_prism", "cone", "cylinder")
SHAPE_ZH = dict(zip(SHAPES, ("正方体", "长方体", "三棱锥", "三棱柱", "圆锥", "圆柱")))
EXTRA_SHAPES = ("sphere", "quadrangular_pyramid", "pentagonal_pyramid", "hexagonal_pyramid",
                "pentagonal_prism", "hexagonal_prism")
SHAPE_ZH.update(dict(zip(EXTRA_SHAPES, ("球体", "正四棱锥", "正五棱锥", "正六棱锥", "正五棱柱", "正六棱柱"))))
REGULAR_POLYGONS = {"quadrangular_pyramid": 4, "pentagonal_pyramid": 5, "hexagonal_pyramid": 6,
                    "pentagonal_prism": 5, "hexagonal_prism": 6}


@lru_cache(maxsize=8)
def camera_rays(width, height, fx, fy, cx, cy):
    y, x = np.indices((height, width), dtype=np.float32)
    rays = np.stack(((x-cx)/fx, (y-cy)/fy, np.ones_like(x)), axis=-1)
    rays.setflags(write=False)
    return rays


def intersect_rays(directions, eye, normals, offsets):
    a = directions @ normals.T
    b = eye @ normals.T + offsets
    with np.errstate(divide="ignore", invalid="ignore"):
        t = -b / a
    low = np.max(np.where(a < -1e-10, t, -np.inf), axis=1)
    high = np.min(np.where(a > 1e-10, t, np.inf), axis=1)
    parallel_out = np.any((np.abs(a) <= 1e-10) & (b > 0), axis=1)
    valid = (high >= np.maximum(low, 0)) & np.isfinite(low) & ~parallel_out
    return low, valid


@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        if self.width <= 0 or self.height <= 0 or self.fx <= 0 or self.fy <= 0 or not np.isfinite(
                [self.fx, self.fy, self.cx, self.cy]).all():
            raise ValueError("相机内参不合法")

    @property
    def matrix(self):
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.]])

    def deproject(self, depth):
        if depth.shape != (self.height, self.width):
            raise ValueError("深度图尺寸与相机内参不一致")
        return camera_rays(self.width, self.height, self.fx, self.fy, self.cx, self.cy) * depth[..., None]

    def project(self, points):
        z = points[:, 2]
        uv = np.column_stack((self.fx * points[:, 0] / np.maximum(z, 1e-6) + self.cx,
                              self.fy * points[:, 1] / np.maximum(z, 1e-6) + self.cy))
        return uv


@dataclass
class TableFrame:
    origin: np.ndarray
    basis: np.ndarray  # columns: table X, Y, upward Z in D435 color optical coordinates
    residual_m: float = 0.0

    def __post_init__(self):
        self.origin, self.basis = np.asarray(self.origin, float), np.asarray(self.basis, float)
        if self.origin.shape != (3,) or self.basis.shape != (3, 3) or not np.isfinite(self.origin).all() or not np.isfinite(self.basis).all():
            raise ValueError("桌面坐标矩阵不合法")
        if not np.allclose(self.basis.T @ self.basis, np.eye(3), atol=1e-5) or np.linalg.det(self.basis) < .999:
            raise ValueError("桌面坐标旋转矩阵必须正交且保持右手系")
        if not np.isfinite(self.residual_m) or self.residual_m < 0:
            raise ValueError("桌面拟合误差不合法")

    def to_world(self, camera_points):
        return (camera_points - self.origin) @ self.basis

    def to_camera(self, world_points):
        return world_points @ self.basis.T + self.origin


def fit_table(points, threshold=0.004, iterations=220, seed=7):
    """RANSAC dominant support plane; normal points toward the camera."""
    p = np.asarray(points, dtype=float)
    p = p[np.isfinite(p).all(axis=1) & (p[:, 2] > 0)]
    if len(p) < 300:
        raise ValueError("有效桌面深度点不足，请清空桌面并调整距离/ROI")
    rng = np.random.default_rng(seed)
    p = p[rng.choice(len(p), min(6000, len(p)), replace=False)]
    best = np.zeros(len(p), bool)
    for _ in range(iterations):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        inside = np.abs((p - a) @ n) < threshold
        if inside.sum() > best.sum():
            best = inside
    if best.mean() < 0.35:
        raise ValueError("没有找到可靠桌面平面；请清空桌面，使用 ROI 排除墙面")
    q = p[best]
    center = np.median(q, axis=0)
    _, singular, vh = np.linalg.svd(q - center, full_matrices=False)
    if singular[1] / np.sqrt(len(q)) < .005:
        raise ValueError("桌面样本接近一条线，无法可靠标定")
    n = vh[-1]
    if n @ center > 0:
        n = -n
    if abs(n[2]) < 0.25:
        raise ValueError("主平面近似竖直；D435 应斜俯视桌面")
    origin = n * (n @ center)
    x = np.array([1., 0, 0]) - n * n[0]
    x /= np.linalg.norm(x)
    basis = np.column_stack((x, np.cross(n, x), n))
    error = float(np.sqrt(np.mean(((q - origin) @ n) ** 2)))
    return TableFrame(origin, basis, error)


class Primitive:
    """Convex models with metric halfspaces. Cube has one size parameter;
    round models have diameter and axial height; others have three dimensions.
    A triangular pyramid uses a centred apex and a triangular base.
    """
    def __init__(self, shape, orientation="upright"):
        if shape not in SHAPES + EXTRA_SHAPES or orientation not in ("upright", "lying"):
            raise ValueError("未知几何体模型或放置方向")
        self.shape, self.orientation = shape, orientation
        triangle = np.array([[-.5, -.5], [.5, -.5], [0, .5]])
        if shape == "sphere":
            latitude = np.linspace(-np.pi/2, np.pi/2, 13)
            longitude = np.linspace(0, 2*np.pi, 24, endpoint=False)
            vertices = np.array([[.5*np.cos(phi)*np.cos(theta), .5*np.cos(phi)*np.sin(theta),
                                  .5+.5*np.sin(phi)] for phi in latitude for theta in longitude])
            vertices = np.unique(np.round(vertices, 12), axis=0)
        elif shape in REGULAR_POLYGONS:
            angle = np.linspace(0, 2*np.pi, REGULAR_POLYGONS[shape], endpoint=False)
            ring = np.c_[.5*np.cos(angle), .5*np.sin(angle), np.zeros(len(angle))]
            vertices = (np.vstack((ring, [0., 0., 1.])) if shape.endswith("pyramid")
                        else np.vstack((ring, ring+[0., 0., 1.])))
        elif shape in ("cube", "cuboid"):
            vertices = np.array([[x, y, z] for x in (-.5, .5)
                                 for y in (-.5, .5) for z in (0, 1.)])
        elif shape == "triangular_prism":
            vertices = np.array([[x, y, z] for x, y in triangle for z in (0, 1.)])
        elif shape == "triangular_pyramid":
            vertices = np.vstack((np.c_[triangle, np.zeros(3)], [0, -1/6, 1]))
        else:
            a = np.linspace(0, 2 * np.pi, 48, endpoint=False)
            ring = np.c_[.5 * np.cos(a), .5 * np.sin(a), np.zeros(len(a))]
            vertices = (np.vstack((ring, ring + [0, 0, 1])) if shape == "cylinder"
                        else np.vstack((ring, [0, 0, 1])))
        self.base_vertices = vertices
        equations = ConvexHull(vertices).equations
        # QHull triangulates coplanar faces. Deduplicate their halfspaces: a
        # 48-sided cylinder needs 50 planes, rather than ~188 repeated planes.
        _, unique = np.unique(np.round(equations, 10), axis=0, return_index=True)
        equations = equations[np.sort(unique)]
        self.normals = equations[:, :3]
        self.offsets = equations[:, 3]
        self.tilt = (np.array([[1., 0, 0], [0, 0, -1], [0, 1, 0]])
                     if orientation == "lying" else np.eye(3))

    @property
    def parameter_count(self):
        return 4 if self.shape in ("cube", "sphere") else (5 if self.axial else 6)

    @property
    def axial(self):
        return self.shape in ("cone", "cylinder") or self.shape in REGULAR_POLYGONS

    def scales(self, params):
        if self.shape in ("cube", "sphere"):
            return np.repeat(params[3], 3)
        if self.axial:
            return np.array([params[3], params[3], params[4]])
        return np.array(params[3:6])

    def transform(self, params):
        c, s = np.cos(params[2]), np.sin(params[2])
        yaw = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
        rotation = yaw @ self.tilt
        local = self.base_vertices * self.scales(params)
        rotated = local @ rotation.T
        footprint_center = (rotated[:, :2].min(axis=0) + rotated[:, :2].max(axis=0)) / 2
        translation = np.array([params[0]-footprint_center[0], params[1]-footprint_center[1], -rotated[:, 2].min()])
        return rotation, translation

    def vertices(self, params):
        rotation, translation = self.transform(params)
        return (self.base_vertices * self.scales(params)) @ rotation.T + translation

    def planes(self, params):
        rotation, translation = self.transform(params)
        n = (self.normals / self.scales(params)) @ rotation.T
        d = self.offsets - n @ translation
        length = np.linalg.norm(n, axis=1)
        return n / length[:, None], d / length

    def prepared(self, params):
        rotation, translation = self.transform(params)
        scale = self.scales(params)
        vertices = (self.base_vertices * scale) @ rotation.T + translation
        normals = (self.normals / scale) @ rotation.T
        offsets = self.offsets - normals @ translation
        length = np.linalg.norm(normals, axis=1)
        return vertices, normals / length[:, None], offsets / length

    def surface_residual(self, points, params):
        n, d = self.planes(params)
        return np.max(points @ n.T + d, axis=1)

    def first_hit(self, camera_points, params, table):
        """Ray/convex solid intersection; parameter t=1 is the measured point."""
        n, d = self.planes(params)
        eye = table.to_world(np.zeros((1, 3)))[0]
        directions = camera_points @ table.basis
        return intersect_rays(directions, eye, n, d)


def model_bank(extra_shapes=()):
    models = [Primitive(s) for s in (*SHAPES, *extra_shapes)]
    models += [Primitive(s, "lying") for s in ("triangular_prism", "cylinder")]
    return models


def footprint_box(vertices):
    import cv2
    # A triangle can have several equal-area enclosing rectangles with different
    # L/W. Resolve ties by the shortest long side, avoiding frame-to-frame jumps.
    hull = cv2.convexHull(vertices[:, :2].astype(np.float32)).reshape(-1, 2).astype(float)
    edges = np.roll(hull, -1, axis=0)-hull
    angles = np.arctan2(edges[:, 1], edges[:, 0])
    axes_x = np.c_[np.cos(angles), np.sin(angles)]
    axes_y = np.c_[-np.sin(angles), np.cos(angles)]
    px, py = hull @ axes_x.T, hull @ axes_y.T
    low_x, high_x, low_y, high_y = px.min(axis=0), px.max(axis=0), py.min(axis=0), py.max(axis=0)
    sizes = np.c_[high_x-low_x, high_y-low_y]
    areas = sizes.prod(axis=1)
    tied = np.where(areas <= areas.min()+max(1e-10, areas.min()*1e-5))[0]
    selected = min(tied, key=lambda i: (round(float(sizes[i].max()), 7),
                                        round(float(sizes[i].min()), 7), float(angles[i] % (np.pi/2))))
    local = np.array([[low_x[selected], low_y[selected]], [high_x[selected], low_y[selected]],
                      [high_x[selected], high_y[selected]], [low_x[selected], high_y[selected]]])
    corners = local @ np.vstack((axes_x[selected], axes_y[selected]))
    low, high = vertices[:, 2].min(), vertices[:, 2].max()
    box = np.vstack((np.c_[corners, np.repeat(low, 4)], np.c_[corners, np.repeat(high, 4)]))
    length, width = sorted(sizes[selected], reverse=True)
    return np.array([length, width, high - low]), box
