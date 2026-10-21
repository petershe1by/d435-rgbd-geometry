"""Deterministic convex-solid renderer for offline regression and demo."""
from __future__ import annotations
import time
import numpy as np
from .geometry import Intrinsics, Primitive, TableFrame
from .cameras import Frame


def scene_camera(width=424, height=240):
    # Camera above and in front of the workspace, optical Z looks at the table.
    eye = np.array([0., -.50, .65])
    forward = -eye / np.linalg.norm(eye)
    right = np.array([1., 0, 0])
    down = np.cross(forward, right)
    camera_to_world = np.column_stack((right, down, forward))
    table = TableFrame(-eye @ camera_to_world, camera_to_world.T, .0006)
    intr = Intrinsics(width, height, width*.95, width*.95, width/2, height/2)
    return intr, table


def render(objects, intr=None, table=None, noise=.0008, holes=.02, seed=1):
    if intr is None:
        intr, table = scene_camera()
    rays = intr.deproject(np.ones((intr.height, intr.width))).reshape(-1, 3)
    eye = table.to_world(np.zeros((1, 3)))[0]
    directions = rays @ table.basis
    depth = -eye[2] / directions[:, 2]
    depth[depth <= 0] = 0
    rgb = np.repeat(np.array([[170, 170, 170]], np.uint8), len(rays), axis=0)
    owners = np.full(len(rays), -1, dtype=np.int16)
    for identity, (model, params, bgr) in enumerate(objects):
        camera_vertices = table.to_camera(model.vertices(params))
        if np.any(camera_vertices[:, 2] <= 0):
            continue
        uv = intr.project(camera_vertices)
        low = np.maximum(np.floor(uv.min(axis=0)).astype(int)-1, [0, 0])
        high = np.minimum(np.ceil(uv.max(axis=0)).astype(int)+2, [intr.width, intr.height])
        if np.any(high <= low):
            continue
        y, x = np.mgrid[low[1]:high[1], low[0]:high[0]]
        selected = (y*intr.width+x).ravel()
        hit, valid = model.first_hit(rays[selected], params, table)
        visible = valid & (hit > 0) & ((depth[selected] == 0) | (hit < depth[selected]))
        indices = selected[visible]
        depth[indices] = hit[visible]
        rgb[indices] = bgr
        owners[indices] = identity
    masks = [(owners == identity).reshape(intr.height, intr.width).astype(np.uint8) for identity in range(len(objects))]
    rng = np.random.default_rng(seed)
    depth += rng.normal(0, noise, len(depth))
    depth[rng.random(len(depth)) < holes] = 0
    return Frame(rgb.reshape(intr.height, intr.width, 3), depth.reshape(intr.height, intr.width).astype(np.float32),
                 time.monotonic(), intr, seed), masks, table


class DemoSource:
    def __init__(self, config=None):
        self.index = 0
        self.intrinsics, self.table = scene_camera(config["width"], config["height"]) if config else scene_camera()
        definitions = [
            ("cube", "upright", [-.20, -.10, .25, .075], [35, 40, 210]),
            ("cuboid", "upright", [0, -.10, -.2, .115, .065, .06], [40, 180, 45]),
            ("triangular_pyramid", "upright", [.20, -.10, .3, .10, .095, .11], [200, 65, 30]),
            ("triangular_prism", "lying", [-.20, .14, .2, .095, .075, .12], [30, 210, 210]),
            ("cone", "upright", [0, .14, 0, .10, .13], [180, 40, 175]),
            ("cylinder", "upright", [.20, .14, 0, .085, .10], [30, 120, 230])]
        self.objects = [(Primitive(shape, orientation), np.array(params, float), color)
                   for shape, orientation, params, color in definitions]

    def read(self):
        self.index += 1
        frame, _, _ = render(self.objects, self.intrinsics, self.table, seed=self.index)
        return frame

    def close(self):
        pass
