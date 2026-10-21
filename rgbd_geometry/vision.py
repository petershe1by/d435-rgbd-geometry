from __future__ import annotations

from dataclasses import dataclass, field
import cv2
import numpy as np
from scipy.optimize import least_squares, linear_sum_assignment
from scipy.spatial import cKDTree
from .geometry import Intrinsics, TableFrame, model_bank, footprint_box, SHAPES, SHAPE_ZH, intersect_rays


@dataclass
class View:
    mask: np.ndarray
    intrinsics: Intrinsics
    table: TableFrame
    rotation: np.ndarray = field(default_factory=lambda: np.eye(3))
    translation: np.ndarray = field(default_factory=lambda: np.zeros(3))
    quality: float = 1.0

    def project(self, vertices):
        p = self.table.to_camera(vertices) @ self.rotation.T + self.translation
        return self.intrinsics.project(p), bool(np.all(p[:, 2] > .02))


@dataclass
class Detection:
    mask: np.ndarray
    points: np.ndarray
    camera_points: np.ndarray
    center: np.ndarray
    quality: float
    flags: list


def segment(rgb, depth, intrinsics, table, config, background=None):
    if rgb.shape != (*depth.shape, 3) or (background is not None and background.shape != rgb.shape):
        raise ValueError("RGB、深度或背景图像尺寸不一致")
    depth = np.where(np.isfinite(depth), depth, 0).astype(np.float32)
    xyz = intrinsics.deproject(depth)
    world = table.to_world(xyz.reshape(-1, 3)).reshape(xyz.shape)
    valid = ((depth > config["min_depth_m"]) & (depth < config["max_depth_m"]))
    fg = valid & (world[:, :, 2] > config["min_object_height_m"]) & (
        world[:, :, 2] < config["max_object_height_m"])
    roi = config.get("roi")
    if roi:
        x, y, w, h = map(int, roi)
        allowed = np.zeros(fg.shape, np.uint8)
        allowed[max(0, y):y+h, max(0, x):x+w] = 1
        fg &= allowed.astype(bool)
    size = config.get("segmentation_close_pixels", 3) | 1
    binary = cv2.morphologyEx(fg.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((size, size), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
    lab_difference = None
    if background is not None:
        delta = cv2.cvtColor(rgb, cv2.COLOR_BGR2LAB).astype(np.float32) - cv2.cvtColor(background, cv2.COLOR_BGR2LAB)
        stable = valid & (np.abs(world[:, :, 2]) < config["plane_threshold_m"]*2)
        if stable.sum() > 300:
            delta[:, :, 0] -= np.median(delta[:, :, 0][stable])
        lab_difference = np.linalg.norm(delta, axis=2) > config.get("background_threshold", 25)
    detections = []
    rng = np.random.default_rng(17)
    components = sorted(range(1, count), key=lambda i: stats[i, cv2.CC_STAT_AREA], reverse=True)
    for i in components[:config.get("max_objects", 16)]:
        if stats[i, cv2.CC_STAT_AREA] < config["min_component_pixels"]:
            continue
        mask = (labels == i).astype(np.uint8)
        # RGB can restore the thin base rim excluded by the height threshold.
        # Restrict expansion to a local band and never consume another component.
        if lab_difference is not None:
            size = config.get("rgb_expand_pixels", 8)*2+1
            band = cv2.dilate(mask, np.ones((size, size), np.uint8)).astype(bool)
            restore = band & lab_difference & ((labels == 0) | (labels == i))
            restore &= (~valid | ((world[:, :, 2] > config["min_object_height_m"]*.25) &
                                  (world[:, :, 2] < config["max_object_height_m"])))
            if roi:
                restore &= allowed.astype(bool)
            mask[restore] = 1
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mask[:] = 0
        cv2.drawContours(mask, [max(contours, key=cv2.contourArea)], -1, 1, -1)
        usable = mask.astype(bool) & valid & (world[:, :, 2] > config["min_object_height_m"] * .7)
        points, camera = world[usable], xyz[usable]
        if len(points) < 90:
            continue
        if len(points) > config["max_fit_points"]:
            selected = rng.choice(len(points), config["max_fit_points"], replace=False)
            points, camera = points[selected], camera[selected]
        distances, _ = cKDTree(points).query(points, k=5)
        near = distances[:, -1]
        keep = near < max(.008, float(np.median(near) * 3))
        points, camera = points[keep], camera[keep]
        if len(points) < 70:
            continue
        fill = float(np.count_nonzero(valid & mask.astype(bool)) / max(mask.sum(), 1))
        flags = []
        if fill < .75:
            flags.append("depth_holes")
        if mask[0].any() or mask[-1].any() or mask[:, 0].any() or mask[:, -1].any():
            flags.append("image_border_truncated")
        if roi:
            if np.any(mask & (cv2.erode(allowed, np.ones((3, 3), np.uint8)) == 0)):
                flags.append("roi_border_truncated")
        center = np.median(points, axis=0)
        detections.append(Detection(mask, points, camera, center, min(1., fill), flags))
    return detections


def color_estimate(rgb, mask, config=None):
    from .configuration import DEFAULTS
    settings = (config or DEFAULTS)["color"] if config is None or "color" in config else DEFAULTS["color"]
    inner = cv2.erode(mask, np.ones((5, 5), np.uint8)) > 0
    if inner.sum() < 30:
        inner = mask > 0
    pixels = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)[inner]
    if len(pixels) == 0:
        return {"name": "unknown", "name_zh": "未知", "confidence": 0.0}
    h, s, v = pixels.T.astype(float)
    black, white, neutral = settings["black_value"], settings["white_value"], settings["neutral_saturation"]
    edges = settings["hue_boundaries"]
    chromatic = (s >= neutral) & (v >= black)
    votes = {
        "black": v < black,
        "white": (s < neutral) & (v > white),
        "gray": (s < neutral) & (v >= black) & (v <= white),
        "red": ((h < edges[0]) | (h >= edges[-1])) & chromatic,
        "orange": (h >= edges[0]) & (h < edges[1]) & chromatic,
        "yellow": (h >= edges[1]) & (h < edges[2]) & chromatic,
        "green": (h >= edges[2]) & (h < edges[3]) & chromatic,
        "cyan": (h >= edges[3]) & (h < edges[4]) & chromatic,
        "blue": (h >= edges[4]) & (h < edges[5]) & chromatic,
        "purple": (h >= edges[5]) & (h < edges[6]) & chromatic
    }
    counts = {k: float(m.mean()) for k, m in votes.items()}
    name = max(counts, key=counts.get)
    names = dict(zip(votes, ("黑色", "白色", "灰色", "红色", "橙色", "黄色", "绿色", "青色", "蓝色", "紫色")))
    confidence = counts[name]
    bgr = np.median(rgb[inner], axis=0).astype(int)
    return {"name": name if confidence >= settings["min_color_fraction"] else "mixed", "name_zh": names[name] if confidence >= settings["min_color_fraction"] else "混合色",
            "confidence": round(confidence, 3), "rgb": bgr[::-1].tolist(), "votes": counts}


def contour_features(mask):
    contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    polygon = cv2.approxPolyDP(contour, .025 * perimeter, True)
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    return {"vertices_2d": int(len(polygon)), "circularity": float(4*np.pi*area/max(perimeter**2, 1)),
            "solidity": float(area/max(hull_area, 1))}


def mask_hull(mask):
    y, x = np.where(mask > 0)
    return cv2.convexHull(np.c_[x, y].astype(np.float32)).reshape(-1, 2)


def silhouette_iou(view, vertices):
    uv, valid = view.project(vertices)
    if not valid or not np.isfinite(uv).all():
        return 0.
    prediction = np.zeros(view.mask.shape, np.uint8)
    hull = cv2.convexHull(np.clip(uv, -10000, 10000).astype(np.float32)).astype(np.int32)
    cv2.fillConvexPoly(prediction, hull, 1)
    observed = view.mask > 0
    return float(np.count_nonzero((prediction > 0) & observed) /
                 max(np.count_nonzero((prediction > 0) | observed), 1))


class ModelFitter:
    def __init__(self, config):
        self.config = config
        self.models = model_bank(config.get("extra_shapes", ()))
        self.cache = []
        angle = np.linspace(0, 2*np.pi, 16, endpoint=False)
        self.directions = np.c_[np.cos(angle), np.sin(angle)]

    def fit(self, detection, primary, secondary=None, allow_cache=True):
        cfg = self.config
        points = detection.points
        lo, hi = np.percentile(points, [1, 99], axis=0)
        span = np.maximum(hi - lo, .015)
        height = float(np.percentile(points[:, 2], 99.5))
        center = (lo + hi) / 2
        views = [primary] + ([secondary] if secondary is not None else [])
        targets = []
        for view in views:
            hull = mask_hull(view.mask)
            support = np.max(hull @ self.directions.T, axis=0)
            diagonal = max(float(np.linalg.norm(np.ptp(hull, axis=0))), 15)
            targets.append((view, support, diagonal))
        noise = max(cfg["surface_noise_m"], primary.table.residual_m * 2)
        sample_count = min(len(points), cfg.get("fit_sample_points", 160))
        indices = np.linspace(0, len(points)-1, sample_count, dtype=int)
        fit_points = points[indices]
        ray_indices = np.linspace(0, len(points)-1, min(sample_count, 90), dtype=int)
        ray_sample = detection.camera_points[ray_indices]
        ray_directions = ray_sample @ primary.table.basis
        full_directions = detection.camera_points @ primary.table.basis
        eye = primary.table.to_world(np.zeros((1, 3)))[0]
        available = ("surface", "depth", "rgb", "secondary_rgb") if secondary is not None else ("surface", "depth", "rgb")
        weight_scale = (1. if secondary is not None else .8)/sum(cfg["weights"][k] for k in available)
        surface_weight = np.sqrt(cfg["weights"]["surface"]*weight_scale/.30)
        depth_weight = np.sqrt(cfg["weights"]["depth"]*weight_scale/.25)
        results = []
        cached = None
        if self.cache and allow_cache:
            distances = [np.linalg.norm(detection.center[:2]-c["observed_center"][:2]) for c in self.cache]
            nearest = int(np.argmin(distances))
            if distances[nearest] < cfg["track_distance_m"] * .6:
                candidate = self.cache[nearest]
                if np.linalg.norm(span-candidate["span"]) / max(np.linalg.norm(span), .01) < .25:
                    cached = candidate
        warm_prior = cached
        if cached is not None and cached.get("generation", 1) >= cfg.get("full_search_interval", 8):
            cached = None
        selected = set()
        if cached is not None:
            ordered = sorted(cached.get("model_scores", {}), key=cached.get("model_scores", {}).get, reverse=True)
            selected = set(ordered[:cfg.get("refine_top_k", 3)])

        def evaluate(model, params):
            vertices, normals, offsets = model.prepared(params)
            distance = np.abs(np.max(points @ normals.T + offsets, axis=1))
            surface_score = float(np.mean(np.exp(-.5*(distance/(noise*cfg.get("surface_score_sigma", 1.8)))**2)))
            hit, valid = intersect_rays(full_directions, eye, normals, offsets)
            depth_error = np.where(valid, np.abs((hit-1)*detection.camera_points[:, 2]), np.inf)
            depth_score = float(np.mean(np.exp(-.5*(depth_error/(noise*cfg.get("depth_score_sigma", 2.5)))**2)))
            metrics = {"surface": surface_score, "depth": depth_score, "rgb": silhouette_iou(primary, vertices)}
            if secondary is not None:
                metrics["secondary_rgb"] = silhouette_iou(secondary, vertices)
            weights = {k: cfg["weights"][k]*(secondary.quality if k == "secondary_rgb" else 1) for k in metrics}
            score = sum(metrics[k]*weights[k] for k in metrics)/sum(weights.values())
            if model.shape == "cuboid":
                score -= .008
            dimensions, corners = footprint_box(vertices)
            return {"shape": model.shape, "orientation": model.orientation,
                    "score": float(np.clip(score, 0, 1)), "metrics": metrics,
                    "surface_error_m": float(np.median(distance)), "parameters": params,
                    "vertices": vertices, "box": corners, "dimensions": dimensions,
                    "center": np.mean(vertices, axis=0), "_model": model}
        for model in self.models:
            attempts = []
            key = (model.shape, model.orientation)

            def make_initial(yaw):
                cosine, sine = np.cos(yaw), np.sin(yaw)
                axes = np.array([[cosine, -sine], [sine, cosine]])
                local = points[:, :2] @ axes
                xy = np.maximum(np.ptp(np.percentile(local, [1, 99], axis=0), axis=0), .012)
                initial = [center[0], center[1], yaw]
                if model.shape in ("cube", "sphere"):
                    initial += [float(np.median([xy[0], xy[1], height]))]
                elif model.axial:
                    initial += ([max(xy), height] if model.orientation == "upright"
                                else [height, xy[1]])
                else:
                    initial += ([xy[0], xy[1], height] if model.orientation == "upright"
                                else [xy[0], height, xy[1]])
                return np.array(initial, float)

            def residual(params, model=model):
                vertices, normals, offsets = model.prepared(params)
                # Equal modality scale regardless of how many depth samples exist.
                surface = np.max(fit_points @ normals.T + offsets, axis=1) / noise / np.sqrt(len(fit_points))
                out = [surface*surface_weight]
                hit, hit_valid = intersect_rays(ray_directions, eye, normals, offsets)
                ray_error = np.where(hit_valid, (hit-1)*ray_sample[:, 2], .025)
                out.append(depth_weight*np.clip(ray_error, -.05, .05) / (noise*2) / np.sqrt(len(ray_sample)))
                for view, support, diagonal in targets:
                    uv, visible = view.project(vertices)
                    pred = np.max(uv @ self.directions.T, axis=0) if visible else support + diagonal
                    weight = (np.sqrt(cfg["weights"]["rgb"]*weight_scale/.25) if view is primary else
                              .75*np.sqrt(cfg["weights"]["secondary_rgb"]*weight_scale/.20)*view.quality)
                    out.append(weight * (pred - support) / (.03 * diagonal) / 4)
                height_error = vertices[:, 2].max()-height
                if model.shape == "cone" or model.shape.endswith("_pyramid"):
                    # A sampled point-cloud percentile is a lower bound for a
                    # sparsely observed apex, not an unbiased apex measurement.
                    height_error = min(height_error, 0.)
                out.append(np.array([height_error / (noise * 3)]))
                return np.concatenate(out)

            if cached is not None and key in cached["parameters"]:
                warm = cached["parameters"][key].copy()
                warm[:2] += detection.center[:2] - cached["observed_center"][:2]
                initials = [warm]
            elif model.shape in ("cone", "cylinder", "sphere") and model.orientation == "upright":
                initials = [make_initial(0)]
            else:
                # Test orientations cheaply before nonlinear refinement; triangular
                # footprints need all directions, including the opposite apex.
                coarse = [make_initial(a) for a in np.linspace(-np.pi, np.pi, 12, endpoint=False)]
                coarse.sort(key=lambda p: float(np.mean(residual(p)**2)))
                initials = [coarse[0]]
                for candidate in coarse[1:]:
                    if abs(np.arctan2(np.sin(candidate[2]-coarse[0][2]), np.cos(candidate[2]-coarse[0][2]))) > .6:
                        initials.append(candidate)
                        break
            if cached is None and warm_prior is not None and key in warm_prior["parameters"]:
                # Periodic global search adds fresh hypotheses but must not throw
                # away a good fit for a still-observed object before comparing it.
                warm = warm_prior["parameters"][key].copy()
                warm[:2] += detection.center[:2]-warm_prior["observed_center"][:2]
                initials.append(warm)
            for initial in initials:
                lower = np.r_[center[:2] - np.maximum(span[:2], .05), -2*np.pi,
                              np.repeat(.008, len(initial)-3)]
                upper = np.r_[center[:2] + np.maximum(span[:2], .05), 2*np.pi,
                              np.repeat(cfg["max_object_height_m"] * 2, len(initial)-3)]
                initial = np.clip(initial, lower + 1e-5, upper - 1e-5)
                if cached is not None and key not in selected:
                    params = initial
                else:
                    budget = cfg.get("warm_fit_nfev", 15) if cached is not None else cfg.get("coarse_fit_nfev", 10)
                    fit = least_squares(residual, initial, bounds=(lower, upper), loss="soft_l1",
                                        f_scale=.5, max_nfev=min(budget, cfg["fit_max_nfev"]), diff_step=1e-4,
                                        x_scale="jac", ftol=.002, xtol=.002)
                    params = fit.x
                attempts.append((float(np.mean(residual(params)**2)), params))
            _, params = min(attempts, key=lambda item: item[0])
            result = evaluate(model, params)
            result["_residual"], result["_bounds"] = residual, (lower, upper)
            results.append(result)
        if cached is None:
            shortlisted = sorted(results, key=lambda r: r["score"], reverse=True)[:cfg.get("refine_top_k", 3)]
            # Both hypotheses are needed for metric cube/cuboid semantics.
            if any(r["shape"] in ("cube", "cuboid") for r in shortlisted):
                shortlisted.extend(r for r in results if r["shape"] in ("cube", "cuboid") and
                                   not any(r is item for item in shortlisted))
            for result in shortlisted:
                refined = least_squares(result["_residual"], result["parameters"], bounds=result["_bounds"],
                                        loss="soft_l1", f_scale=.5, max_nfev=cfg["fit_max_nfev"],
                                        diff_step=1e-4, x_scale="jac", ftol=.0005, xtol=.0005)
                updated = evaluate(result["_model"], refined.x)
                if updated["score"] >= result["score"]:
                    result.update(updated)
        cache_item = {"observed_center": detection.center.copy(), "span": span.copy(),
                      "parameters": {(r["shape"], r["orientation"]): r["parameters"].copy() for r in results},
                      "model_scores": {(r["shape"], r["orientation"]): r["score"] for r in results},
                      "generation": cached.get("generation", 1)+1 if cached is not None else 1}
        self.cache = [c for c in self.cache
                      if np.linalg.norm(c["observed_center"][:2]-detection.center[:2]) >= cfg["track_distance_m"]*.6]
        self.cache.append(cache_item)
        self.cache = self.cache[-30:]
        best_by_shape = {}
        for r in results:
            if r["shape"] not in best_by_shape or r["score"] > best_by_shape[r["shape"]]["score"]:
                best_by_shape[r["shape"]] = r
        ranked = sorted(best_by_shape.values(), key=lambda r: r["score"], reverse=True)
        # Cube is a constrained cuboid: those two hypotheses have identical evidence
        # for equal edges. Resolve that semantic ambiguity using fitted metric sizes.
        cuboid = best_by_shape["cuboid"]
        cube = best_by_shape["cube"]
        edges = cuboid["parameters"][3:6]
        near_equal = np.ptp(edges) <= max(cfg.get("cube_absolute_tolerance_m", .004),
                                        cfg.get("cube_relative_tolerance", .08) * np.mean(edges))
        if near_equal and cube["score"] >= cuboid["score"] - .03:
            cuboid["score"] = min(cuboid["score"], cube["score"] - .10)
        elif cuboid["score"] >= .65:
            cube["score"] = min(cube["score"], cuboid["score"] - .10)
        ranked = sorted(best_by_shape.values(), key=lambda r: r["score"], reverse=True)
        best = ranked[0]
        if cached is not None and best["score"] < .75:
            return self.fit(detection, primary, secondary, allow_cache=False)
        best["margin"] = best["score"] - ranked[1]["score"]
        best["scores"] = {r["shape"]: r["score"] for r in ranked}
        best["quality"] = detection.quality
        best["flags"] = list(detection.flags)
        best["features"] = contour_features(detection.mask)
        if best["features"]["solidity"] < cfg.get("min_solidity", .90):
            best["flags"].append("occluded_or_touching_or_nonconvex")
        if best["surface_error_m"] > noise * cfg.get("max_surface_error_factor", 2):
            best["flags"].append("poor_model_fit")
        if secondary is None:
            best["flags"].append("single_view_hidden_dimensions_inferred")
        best["mask"] = detection.mask
        best["search_mode"] = "temporal" if cached is not None else "full"
        for key in ("_model", "_residual", "_bounds"):
            best.pop(key, None)
        return best


class Tracker:
    """Unique assignment, score EMA, dimensions median, and explicit stale expiry."""
    def __init__(self, config):
        self.config, self.tracks, self.next_id = config, {}, 1
        self.last_update = None

    def update(self, observations, now):
        interval = 0 if self.last_update is None else max(0, now-self.last_update)
        self.last_update = now
        # Inference can be slower than capture. A continuously observed object
        # must not expire solely because a model fit took more than 0.8 seconds.
        timeout = max(self.config["track_timeout_s"], interval*1.5) if observations else self.config["track_timeout_s"]
        self.tracks = {k: t for k, t in self.tracks.items()
                       if now - t["last_seen"] <= timeout}
        ids = list(self.tracks)
        matches = {}
        if ids and observations:
            cost = np.array([[np.linalg.norm(self.tracks[k]["center"][:2] - o["center"][:2])
                              for o in observations] for k in ids])
            gated = np.where(cost < self.config["track_distance_m"], cost, 1e6)
            rows, cols = linear_sum_assignment(gated)
            for row, col in zip(rows, cols):
                if cost[row, col] < self.config["track_distance_m"]:
                    matches[col] = ids[row]
        output = []
        for j, observation in enumerate(observations):
            identity = matches.get(j)
            if identity is None:
                identity, self.next_id = self.next_id, self.next_id + 1
                self.tracks[identity] = {"scores": observation["scores"].copy(), "history": [],
                                        "shape": observation["shape"], "hits": 0, "colors": [],
                                        "orientation": observation["orientation"]}
            track = self.tracks[identity]
            # Fast motion or a long gap reduces temporal lag.
            alpha = .55 if now - track.get("last_seen", now) < .4 else .85
            score_window = self.config.get("score_smoothing_window", 1)
            if score_window > 1:
                history = track.setdefault("score_history", [])
                raw_winner = max(observation["scores"], key=observation["scores"].get)
                moved = "center" in track and np.linalg.norm(track["center"][:2]-observation["center"][:2]) > self.config["track_distance_m"]*.3
                poor = observation["quality"] < self.config.get("min_depth_quality", .65) or any(
                    flag in observation["flags"] for flag in ("poor_model_fit", "occluded_or_touching_or_nonconvex", "image_border_truncated", "roi_border_truncated"))
                if raw_winner != track["shape"] or moved or poor or observation["orientation"] != track["orientation"]:
                    history.clear()
                history.append(observation["scores"].copy())
                del history[:-score_window]
                track["scores"] = {s: float(np.mean([item[s] for item in history])) for s in observation["scores"]}
            else:
                for s in track["scores"]:
                    track["scores"][s] = (1-alpha)*track["scores"][s] + alpha*observation["scores"][s]
            ranked = sorted(track["scores"], key=track["scores"].get, reverse=True)
            winner = ranked[0]
            if winner != track["shape"] or observation["orientation"] != track["orientation"]:
                track["history"] = []
                track["hits"] = 0
                track["shape"] = winner
                track["orientation"] = observation["orientation"]
            score = track["scores"][winner]
            margin = score - track["scores"][ranked[1]]
            severe = any(flag in observation["flags"] for flag in
                         ("image_border_truncated", "roi_border_truncated", "poor_model_fit",
                          "occluded_or_touching_or_nonconvex"))
            trustworthy = (not severe and observation["quality"] >= self.config.get("min_depth_quality", .65)
                           and observation["shape"] == winner
                           and observation["scores"][winner]*observation["quality"] >= self.config["min_confidence"])
            if trustworthy:
                if track["history"]:
                    previous = np.median(track["history"], axis=0)
                    jump = np.max(np.abs(observation["dimensions"]-previous)/np.maximum(previous, .01))
                    if jump > self.config.get("dimension_jump_relative", .25):
                        track["history"], track["hits"] = [], 0
                        observation["flags"] = observation["flags"] + ["dimensions_reacquiring"]
                track["history"].append(observation["dimensions"].copy())
                track["history"] = track["history"][-self.config.get("smoothing_window", 7):]
                track["hits"] += 1
            else:
                track["hits"] = 0
            confident = (score * observation["quality"] >= self.config["min_confidence"] and margin >= self.config["min_score_margin"]
                         and track["hits"] >= self.config.get("min_stable_frames", 3)
                         and trustworthy)
            reason = "confirmed"
            if not confident:
                if severe:
                    reason = "segmentation_or_model_fit"
                elif observation["quality"] < self.config.get("min_depth_quality", .65):
                    reason = "insufficient_depth"
                elif score*observation["quality"] < self.config["min_confidence"]:
                    reason = "low_score"
                elif not trustworthy:
                    reason = "inconsistent_observation"
                elif track["hits"] < self.config.get("min_stable_frames", 3):
                    reason = "confirming_frames"
                else:
                    reason = "ambiguous_classes"
            track["center"], track["last_seen"] = observation["center"], now
            candidate_dims = np.median(track["history"], axis=0) if track["history"] and trustworthy else observation["dimensions"]
            track["colors"].append(observation["color"])
            track["colors"] = track["colors"][-self.config.get("smoothing_window", 7):]
            color_names = [c["name"] for c in track["colors"]]
            color_name = max(dict.fromkeys(reversed(color_names)), key=color_names.count)
            color = next(c for c in reversed(track["colors"]) if c["name"] == color_name).copy()
            color["temporal_agreement"] = color_names.count(color_name)/len(color_names)
            output.append({"id": identity, "shape": winner if confident else "unknown", "decision_reason": reason,
                           "shape_zh": SHAPE_ZH[winner] if confident else "不确定",
                           "candidate_shape": winner, "confidence": round(score * observation["quality"], 3),
                           "score_margin": round(margin, 3), "scores": track["scores"].copy(),
                           "dimensions_mm": (candidate_dims * 1000).round(1).tolist() if confident else None,
                           "candidate_dimensions_mm": (candidate_dims * 1000).round(1).tolist(),
                           "enclosing_cube_side_mm": round(float(np.max(candidate_dims))*1000, 1) if confident else None,
                           "bounding_corners_table_m": observation["box"].round(6).tolist() if confident else None,
                           "dimension_source": "shape_model_fitted_to_rgbd", "color": color,
                           "orientation": observation["orientation"], "center_table_m": observation["center"].tolist(),
                           "quality_flags": observation["flags"], "metrics": observation["metrics"],
                           "surface_error_mm": round(observation["surface_error_m"]*1000, 2),
                           "features": observation["features"], "_box": observation["box"], "_mask": observation["mask"]})
            output[-1]["search_mode"] = observation.get("search_mode", "full")
            output[-1]["stable_observations"] = track["hits"]
        return output
