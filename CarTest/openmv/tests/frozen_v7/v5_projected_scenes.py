"""Pinhole-projected, geometry-labeled V5 track scenes (stdlib only).

These fixtures support detector diagnostics; they do not certify a mounted
camera or a moving vehicle. Ground truth is rasterized from road geometry
before cosmetic seams and shadows are added.

Example:
    from tests.v5_projected_scenes import generate_scene
    scene = generate_scene("demo", road_type="straight", camera_down_deg=10)
    image = scene.gray_image  # width(), height(), get_pixel(x, y)
"""
from __future__ import annotations

import argparse
import json
import math
import struct
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Sequence

WIDTH, HEIGHT = 80, 60
HFOV_DEG, VFOV_DEG = 65.9, 51.8
CAMERA_HEIGHT_M = 0.22
ROAD_WIDTH_M, WHITE_WIDTH_M = 0.45, 0.40
ROAD_HALF_M, WHITE_HALF_M = ROAD_WIDTH_M / 2, WHITE_WIDTH_M / 2
BORDER_WIDTH_M = (ROAD_WIDTH_M - WHITE_WIDTH_M) / 2
SCENE_VERSION = "v5-pinhole-groundtruth-1"
POSE_SCENE_VERSION = "v5-camera-pose-course-1"
DATA_DIR = Path(__file__).resolve().parents[1] / "test-data" / "v5-projected"
POSE_DATA_DIR = DATA_DIR / "near-course"


@dataclass(frozen=True)
class _Segment:
    x0: float
    s0: float
    x1: float
    s1: float
    length: float
    ux: float
    us: float
    chainage0: float


class RasterImage:
    """Image adapter with the firmware's get_pixel((x, y)) calling convention."""

    def __init__(self, data: bytearray, width: int, height: int, channels: int):
        self._data, self._width, self._height, self._channels = data, width, height, channels

    def width(self) -> int:
        return self._width

    def height(self) -> int:
        return self._height

    def get_pixel(self, xy: tuple[int, int]) -> int | tuple[int, int, int]:
        if not isinstance(xy, tuple) or len(xy) != 2:
            raise TypeError("get_pixel expects one (x, y) tuple")
        x, y = xy
        if not (0 <= x < self._width and 0 <= y < self._height):
            raise IndexError("pixel coordinate outside raster")
        i = (y * self._width + x) * self._channels
        if self._channels == 1:
            return self._data[i]
        return self._data[i], self._data[i + 1], self._data[i + 2]


@dataclass
class SceneResult:
    scene_id: str
    rgb: bytearray
    gray: bytearray
    ground_mask: bytearray
    road_mask: bytearray
    white_mask: bytearray
    border_mask: bytearray
    floor_mask: bytearray
    boundary_truth: bytearray
    left_edge_truth: bytearray
    right_edge_truth: bytearray
    white_edge_truth: bytearray
    left_white_edge_truth: bytearray
    right_white_edge_truth: bytearray
    branch_mask: bytearray
    island_mask: bytearray
    world_distance_m: list[float]
    truth_segments: list[_Segment] = field(repr=False)
    metadata: dict[str, Any]
    width: int = WIDTH
    height: int = HEIGHT

    @property
    def rgb_image(self) -> RasterImage:
        return RasterImage(self.rgb, self.width, self.height, 3)

    @property
    def gray_image(self) -> RasterImage:
        return RasterImage(self.gray, self.width, self.height, 1)


def _focal(size: int, fov_deg: float) -> float:
    return size / (2.0 * math.tan(math.radians(fov_deg) / 2.0))


def _sample_polyline(points: Sequence[tuple[float, float]], step: float = 0.025) -> list[tuple[float, float]]:
    if len(points) < 2:
        raise ValueError("centerline needs at least two points")
    result = [points[0]]
    for a, b in zip(points, points[1:]):
        dx, ds = b[0] - a[0], b[1] - a[1]
        count = max(1, math.ceil(math.hypot(dx, ds) / step))
        result.extend((a[0] + dx * i / count, a[1] + ds * i / count) for i in range(1, count + 1))
    return result


def _integrated_path(radius: float, turn: str, s_curve: bool = False) -> list[tuple[float, float]]:
    if turn not in ("left", "right"):
        raise ValueError("turn must be left or right")
    if radius <= ROAD_HALF_M:
        raise ValueError("curve radius must exceed road half-width")
    # Keep a 1.5 m straight approach visible before the bend starts.
    step, end_station, switch, arc = 0.025, 19.0, 6.5, min(1.55, radius * 1.25)
    # With x positive to the vehicle's right, a left turn bends toward -x.
    sign, x, s, heading, station = (-1.0 if turn == "left" else 1.0), 0.0, -5.0, 0.0, 0.0
    points = [(x, s)]
    while station < end_station:
        ds = min(step, end_station - station)
        curvature = 0.0
        if switch <= station < switch + arc:
            curvature = sign / radius
        elif s_curve and switch + arc <= station < switch + 2 * arc:
            curvature = -sign / radius
        x += math.sin(heading + curvature * ds / 2) * ds
        s += math.cos(heading + curvature * ds / 2) * ds
        heading += curvature * ds
        station += ds
        points.append((x, s))
    return points


def _make_paths(
    road_type: str,
    center_offset_m: float,
    curve_radius_m: float | None,
    ring_radius_m: float,
    turn: str,
    s_first_turn: str,
) -> tuple[list[list[tuple[float, float]]], list[tuple[float, float]], tuple[float, float] | None]:
    if road_type == "straight" or road_type in ("uphill", "downhill"):
        return [[(center_offset_m, -5.0), (center_offset_m, 14.0)]], [], None
    if road_type == "curve":
        pts = _integrated_path(curve_radius_m or 0.8, turn)
        return [[(x + center_offset_m, s) for x, s in pts]], [], None
    if road_type == "s_curve":
        pts = _integrated_path(curve_radius_m or 1.4, s_first_turn, True)
        return [[(x + center_offset_m, s) for x, s in pts]], [], None
    if road_type == "ring_island":
        if turn not in ("left", "right"):
            raise ValueError("turn must be left or right")
        if ring_radius_m <= ROAD_HALF_M:
            raise ValueError("ring radius must exceed road half-width")
        entry_s, side = 0.90, (-1.0 if turn == "left" else 1.0)
        approach = [(center_offset_m, -5.0), (center_offset_m, entry_s)]
        count = max(180, math.ceil(2 * math.pi * ring_radius_m / 0.025))
        ring = [
            (center_offset_m + side * ring_radius_m * (1 - math.cos(2 * math.pi * i / count)),
             entry_s + ring_radius_m * math.sin(2 * math.pi * i / count))
            for i in range(count + 1)
        ]
        island = (center_offset_m + side * ring_radius_m, entry_s)
        return [approach, ring], [(center_offset_m, entry_s)], island
    if road_type in ("t_branch", "cross_branch"):
        junction_s, extent = 1.05, 1.65
        main = [(center_offset_m, -5.0), (center_offset_m, 9.0 if road_type == "cross_branch" else junction_s)]
        crossbar = [(-extent, junction_s), (extent, junction_s)]
        return [main, crossbar], [(center_offset_m, junction_s)], None
    raise ValueError("unsupported road_type: " + road_type)


def _build_segments(paths: Sequence[Sequence[tuple[float, float]]]) -> list[_Segment]:
    out: list[_Segment] = []
    chainage = 0.0
    for path in paths:
        sampled = _sample_polyline(path)
        local = 0.0
        for p0, p1 in zip(sampled, sampled[1:]):
            dx, ds = p1[0] - p0[0], p1[1] - p0[1]
            length = math.hypot(dx, ds)
            if length > 1e-8:
                out.append(_Segment(p0[0], p0[1], p1[0], p1[1], length, dx / length, ds / length, chainage + local))
                local += length
        chainage += local + 10.0
    return out


def _pose_at_chainage(segments: Sequence[_Segment], chainage: float) -> tuple[float, float, float, float]:
    """Interpolate (x, s, tangent_x, tangent_s) on the first continuous route."""
    if not math.isfinite(chainage) or chainage < 0:
        raise ValueError("camera route chainage must be finite and nonnegative")
    for seg in segments:
        start, end = seg.chainage0, seg.chainage0 + seg.length
        if start - 1e-10 <= chainage < end - 1e-10:
            along = max(0.0, min(seg.length, chainage - start))
            return (seg.x0 + seg.ux * along, seg.s0 + seg.us * along,
                    seg.ux, seg.us)
    # Permit the exact end of the final segment, but never cross a synthetic
    # 10 m chainage gap inserted between independent branch paths.
    for seg in reversed(segments):
        end = seg.chainage0 + seg.length
        if abs(chainage - end) <= 1e-10:
            return seg.x1, seg.s1, seg.ux, seg.us
    raise ValueError("camera route chainage is outside the first path")


def ground_point_at_pixel(scene: SceneResult, x: float, y: float) -> tuple[float, float] | None:
    """Reproject a pixel to the modeled ground plane coordinates (x, s)."""
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("x and y must be finite pixel coordinates")
    fx, fy = float(scene.metadata["fx_px"]), float(scene.metadata["fy_px"])
    nx = (x + 0.5 - scene.width / 2.0) / fx
    ny = (y + 0.5 - scene.height / 2.0) / fy
    pose = scene.metadata.get("camera_pose")
    if pose is None:
        # Preserve the original world-aligned pinhole model for all legacy
        # scenes, including its independently modeled sloped plane.
        theta = math.radians(float(scene.metadata["camera_down_deg_world_effective"]))
        sin_t, cos_t = math.sin(theta), math.cos(theta)
        dx, dy, dz = nx, cos_t - ny * sin_t, -sin_t - ny * cos_t
        norm = math.sqrt(dx * dx + dy * dy + dz * dz)
        dx, dy, dz = dx / norm, dy / norm, dz / norm
        pitch = math.radians(float(scene.metadata["plane_pitch_deg_forward_up_positive"]))
        denominator = math.tan(pitch) * dy - dz
        if denominator <= 0.0:
            return None
        distance = float(scene.metadata["camera_height_m"]) / denominator
        return distance * dx, distance * dy / math.cos(pitch)
    sin_t = float(scene.metadata["camera_plane_sin_down"])
    cos_t = float(scene.metadata["camera_plane_cos_down"])
    dx_right, dy_forward, dz_up = nx, cos_t - ny * sin_t, -sin_t - ny * cos_t
    norm = math.sqrt(dx_right * dx_right + dy_forward * dy_forward + dz_up * dz_up)
    dx_right, dy_forward, dz_up = dx_right / norm, dy_forward / norm, dz_up / norm
    denominator = -dz_up
    if denominator <= 0.0:
        return None
    distance = float(scene.metadata["camera_height_m"]) / denominator
    world_x = (pose["x_m"] + distance *
               (dx_right * pose["right_x"] + dy_forward * pose["forward_x"]))
    world_s = (pose["s_m"] + distance *
               (dx_right * pose["right_s"] + dy_forward * pose["forward_s"]))
    return world_x, world_s


def _make_bins(segments: Sequence[_Segment], cell: float = 0.12) -> dict[tuple[int, int], list[int]]:
    bins: dict[tuple[int, int], list[int]] = {}
    expand = ROAD_HALF_M + cell
    for i, seg in enumerate(segments):
        x0 = math.floor((min(seg.x0, seg.x1) - expand) / cell)
        x1 = math.floor((max(seg.x0, seg.x1) + expand) / cell)
        s0 = math.floor((min(seg.s0, seg.s1) - expand) / cell)
        s1 = math.floor((max(seg.s0, seg.s1) + expand) / cell)
        for bx in range(x0, x1 + 1):
            for bs in range(s0, s1 + 1):
                bins.setdefault((bx, bs), []).append(i)
    return bins


def _nearest(x: float, s: float, segments: Sequence[_Segment], bins: dict[tuple[int, int], list[int]], cell: float = 0.12):
    candidates = bins.get((math.floor(x / cell), math.floor(s / cell)))
    if not candidates:
        return None
    best_d2, best = math.inf, None
    for i in candidates:
        seg = segments[i]
        along = min(seg.length, max(0.0, (x - seg.x0) * seg.ux + (s - seg.s0) * seg.us))
        px, ps = seg.x0 + along * seg.ux, seg.s0 + along * seg.us
        ox, os = x - px, s - ps
        d2 = ox * ox + os * os
        if d2 < best_d2:
            signed = ox * seg.us - os * seg.ux  # positive is to the route's right
            best_d2 = d2
            best = (math.sqrt(d2), signed, seg.chainage0 + along)
    return best


def _gray(rgb: tuple[int, int, int]) -> int:
    r, g, b = rgb
    return (77 * r + 150 * g + 29 * b + 128) >> 8


def _colors(style: str):
    if style == "practice":
        return (20, 40, 150), (220, 220, 220), (20, 40, 150)
    if style == "formal":
        return (30, 30, 30), (220, 220, 220), (160, 160, 160)
    raise ValueError("track_style must be practice or formal")


def _visible_rows(road: bytearray, edge: bytearray) -> tuple[list[int], int]:
    good: list[int] = []
    for y in range(20, HEIGHT):
        xs = [x for x in range(WIDTH) if road[y * WIDTH + x]]
        if len(xs) < 3 or xs[-1] - xs[0] + 1 != len(xs) or xs[0] == 0 or xs[-1] == WIDTH - 1:
            continue
        left = any(edge[y * WIDTH + x] for x in range(max(0, xs[0] - 2), min(WIDTH, xs[0] + 3)))
        right = any(edge[y * WIDTH + x] for x in range(max(0, xs[-1] - 2), min(WIDTH, xs[-1] + 3)))
        if left and right:
            good.append(y)
    longest = current = 0
    previous = -2
    for y in good:
        current = current + 1 if y == previous + 1 else 1
        longest, previous = max(longest, current), y
    return good, longest


def generate_scene(
    scene_id: str,
    *,
    road_type: str,
    camera_down_deg: float,
    camera_height_m: float = CAMERA_HEIGHT_M,
    track_style: str = "practice",
    center_offset_m: float = 0.0,
    curve_radius_m: float | None = None,
    ring_radius_m: float = 0.78,
    turn: str = "left",
    s_first_turn: str = "left",
    plane_pitch_deg: float | None = None,
    camera_angle_reference: str = "world",
    camera_route_chainage_m: float | None = None,
    camera_lateral_offset_m: float = 0.0,
    camera_yaw_offset_deg: float = 0.0,
    camera_stage: str | None = None,
    camera_feature_distance_m: float | None = None,
    seams: bool = True,
    shadow: bool = False,
) -> SceneResult:
    """Generate an 80x60 RGB/gray image and independent geometric truth masks.

    Axes are x=lateral-right, s=forward along the local road plane, z=up.
    camera_angle_reference='world' measures camera_down_deg from the world
    horizon; 'road' measures it below the road plane. Positive plane_pitch_deg
    means the road rises forward. A ray with groundrayden <= 0 is background.
    road_type is straight, curve, s_curve, ring_island, t_branch, cross_branch,
    uphill, or downhill. expected_valid is True only when paired outer road
    edges are raster-visible for at least 8 adjacent rows; otherwise it is None
    and score_status is unscored.
    """
    if not scene_id or any(ch in scene_id for ch in '\\/:*?"<>|'):
        raise ValueError("scene_id must be a filename-safe identifier")
    if not 0 <= camera_down_deg <= 80:
        raise ValueError("camera_down_deg must be between 0 and 80")
    if camera_height_m <= 0:
        raise ValueError("camera_height_m must be positive")
    if camera_angle_reference not in ("world", "road"):
        raise ValueError("camera_angle_reference must be world or road")
    if not (math.isfinite(camera_lateral_offset_m) and math.isfinite(camera_yaw_offset_deg)):
        raise ValueError("camera offset and yaw must be finite")
    if camera_feature_distance_m is not None and not math.isfinite(camera_feature_distance_m):
        raise ValueError("camera feature distance must be finite when provided")
    if camera_route_chainage_m is None and (camera_lateral_offset_m != 0 or camera_yaw_offset_deg != 0):
        raise ValueError("camera lateral offset/yaw requires camera_route_chainage_m")
    if plane_pitch_deg is None:
        plane_pitch_deg = 20.0 if road_type == "uphill" else -20.0 if road_type == "downhill" else 0.0
    if abs(plane_pitch_deg) >= 60:
        raise ValueError("plane pitch magnitude must be below 60 degrees")

    paths, junctions, island_center = _make_paths(road_type, center_offset_m, curve_radius_m, ring_radius_m, turn, s_first_turn)
    segments, bins = _build_segments(paths), None
    bins = _make_bins(segments)
    camera_pose = None
    if camera_route_chainage_m is not None:
        route_x, route_s, tangent_x, tangent_s = _pose_at_chainage(segments, camera_route_chainage_m)
        route_right_x, route_right_s = tangent_s, -tangent_x
        camera_x = route_x + camera_lateral_offset_m * route_right_x
        camera_s = route_s + camera_lateral_offset_m * route_right_s
        yaw = math.radians(camera_yaw_offset_deg)
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        camera_forward_x = tangent_x * cos_yaw + route_right_x * sin_yaw
        camera_forward_s = tangent_s * cos_yaw + route_right_s * sin_yaw
        camera_right_x = route_right_x * cos_yaw - tangent_x * sin_yaw
        camera_right_s = route_right_s * cos_yaw - tangent_s * sin_yaw
        camera_pose = {
            "route_chainage_m": camera_route_chainage_m,
            "route_center_x_m": route_x,
            "route_center_s_m": route_s,
            "x_m": camera_x,
            "s_m": camera_s,
            "tangent_x": tangent_x,
            "tangent_s": tangent_s,
            "right_x": camera_right_x,
            "right_s": camera_right_s,
            "forward_x": camera_forward_x,
            "forward_s": camera_forward_s,
            "lateral_offset_m": camera_lateral_offset_m,
            "yaw_offset_deg": camera_yaw_offset_deg,
        }
    fx, fy = _focal(WIDTH, HFOV_DEG), _focal(HEIGHT, VFOV_DEG)
    pitch = math.radians(plane_pitch_deg)
    effective_down_deg = camera_down_deg if camera_angle_reference == "world" else camera_down_deg - plane_pitch_deg
    camera_plane_down_deg = (
        camera_down_deg + plane_pitch_deg if camera_angle_reference == "world"
        else camera_down_deg
    )
    camera_plane_down = math.radians(camera_plane_down_deg)
    sin_camera_plane_down, cos_camera_plane_down = math.sin(camera_plane_down), math.cos(camera_plane_down)
    effective_down = math.radians(effective_down_deg)
    sin_d, cos_d = math.sin(effective_down), math.cos(effective_down)
    edge_rgb, white_rgb, floor_rgb = _colors(track_style)
    sky_rgb = (16, 16, 16)
    n = WIDTH * HEIGHT
    rgb, gray = bytearray(n * 3), bytearray(n)
    ground, road, white, border, floor = (bytearray(n) for _ in range(5))
    boundary, left_edge, right_edge = (bytearray(n) for _ in range(3))
    white_edge, left_white, right_white = (bytearray(n) for _ in range(3))
    branch, island = bytearray(n), bytearray(n)
    distances = [0.0] * n
    local_x, local_s = [0.0] * n, [0.0] * n

    for y in range(HEIGHT):
        ny = (y + 0.5 - HEIGHT / 2) / fy
        for x in range(WIDTH):
            nx = (x + 0.5 - WIDTH / 2) / fx
            idx = y * WIDTH + x
            if camera_pose is None:
                dx, dy, dz = nx, cos_d - ny * sin_d, -sin_d - ny * cos_d
                inv = 1 / math.sqrt(dx * dx + dy * dy + dz * dz)
                dx, dy, dz = dx * inv, dy * inv, dz * inv
                groundrayden = math.tan(pitch) * dy - dz
                if groundrayden <= 0:
                    rgb[3 * idx:3 * idx + 3] = bytes(sky_rgb)
                    gray[idx] = _gray(sky_rgb)
                    continue
                ray_distance = camera_height_m / groundrayden
                wx, wy = ray_distance * dx, ray_distance * dy
                ss = wy / math.cos(pitch)
            else:
                dx_right = nx
                dy_forward = cos_camera_plane_down - ny * sin_camera_plane_down
                dz_up = -sin_camera_plane_down - ny * cos_camera_plane_down
                inv = 1 / math.sqrt(dx_right * dx_right + dy_forward * dy_forward + dz_up * dz_up)
                dx_right, dy_forward, dz_up = dx_right * inv, dy_forward * inv, dz_up * inv
                groundrayden = -dz_up
                if groundrayden <= 0:
                    rgb[3 * idx:3 * idx + 3] = bytes(sky_rgb)
                    gray[idx] = _gray(sky_rgb)
                    continue
                ray_distance = camera_height_m / groundrayden
                wx = camera_pose["x_m"] + ray_distance * (
                    dx_right * camera_pose["right_x"] + dy_forward * camera_pose["forward_x"])
                ss = camera_pose["s_m"] + ray_distance * (
                    dx_right * camera_pose["right_s"] + dy_forward * camera_pose["forward_s"])
            distances[idx], local_x[idx], local_s[idx] = ray_distance, wx, ss
            ground[idx] = 255
            # Approximate projected half-pixel tolerance, only for edge raster GT.
            tol = max(0.002, min(0.10, 0.60 * ray_distance / min(fx, fy)))
            if island_center is not None and math.hypot(wx - island_center[0], ss - island_center[1]) < ring_radius_m - ROAD_HALF_M:
                island[idx] = 255
            nearest = _nearest(wx, ss, segments, bins)
            if nearest is None:
                floor[idx] = 255
                rgb[3 * idx:3 * idx + 3] = bytes(floor_rgb)
                gray[idx] = _gray(floor_rgb)
                continue
            distance, signed, _chainage = nearest
            color = floor_rgb
            if distance <= ROAD_HALF_M:
                road[idx] = 255
                if distance <= WHITE_HALF_M:
                    white[idx], color = 255, white_rgb
                else:
                    border[idx], color = 255, edge_rgb
                if abs(abs(signed) - ROAD_HALF_M) <= tol:
                    boundary[idx] = 255
                    (left_edge if signed < 0 else right_edge)[idx] = 255
                if abs(abs(signed) - WHITE_HALF_M) <= tol:
                    white_edge[idx] = 255
                    (left_white if signed < 0 else right_white)[idx] = 255
                if any(math.hypot(wx - jx, ss - js) <= 0.46 for jx, js in junctions):
                    branch[idx] = 255
            else:
                floor[idx] = 255
            rgb[3 * idx:3 * idx + 3] = bytes(color)
            gray[idx] = _gray(color)

    # Appearance-only seams/shadow are drawn after every geometric label.
    if seams:
        for idx in range(n):
            if white[idx]:
                q = abs((local_s[idx] + 0.36) % 0.72 - 0.36)
                if q <= 0.006:
                    rgb[3 * idx:3 * idx + 3] = bytes((184, 184, 184))
                    gray[idx] = 184
    if shadow:
        sx, ss0 = center_offset_m + 0.015, 1.65
        for idx in range(n):
            if white[idx] and ((local_x[idx] - sx) / 0.105) ** 2 + ((local_s[idx] - ss0) / 0.34) ** 2 <= 1:
                rgb[3 * idx:3 * idx + 3] = bytes((154, 154, 154))
                gray[idx] = 154

    visible, longest = _visible_rows(road, boundary)
    topology_complex = road_type in ("ring_island", "t_branch", "cross_branch")
    expected_valid = True if longest >= 8 and not topology_complex else None
    reason = None if expected_valid else (
        "branch_or_loop_topology_requires_path-choice-scoring" if topology_complex
        else "paired_outer_edges_not_visible_for_8_adjacent_rows"
    )
    metadata = {
        "scene_version": POSE_SCENE_VERSION if camera_pose is not None else SCENE_VERSION,
        "scene_id": scene_id,
        "road_type": road_type,
        "track_style": track_style,
        "expected_valid": expected_valid,
        "score_status": "scored" if expected_valid else "unscored",
        "unscored_reason": reason,
        "assertion_scope": "synthetic_geometry_only_not_vehicle_acceptance",
        "image_width_px": WIDTH, "image_height_px": HEIGHT,
        "camera_height_m": camera_height_m,
        "camera_down_deg_input": camera_down_deg,
        "camera_angle_reference": camera_angle_reference,
        "camera_down_deg_world_effective": effective_down_deg,
        "camera_down_deg_plane_effective": camera_plane_down_deg,
        "camera_plane_sin_down": sin_camera_plane_down,
        "camera_plane_cos_down": cos_camera_plane_down,
        "camera_pose": camera_pose,
        "camera_route_chainage_m": camera_route_chainage_m,
        "camera_stage": camera_stage,
        "camera_feature_distance_m": camera_feature_distance_m,
        "camera_lateral_offset_m": camera_lateral_offset_m,
        "camera_yaw_offset_deg": camera_yaw_offset_deg,
        "plane_pitch_deg_forward_up_positive": plane_pitch_deg,
        "hfov_deg": HFOV_DEG, "vfov_deg": VFOV_DEG, "fx_px": fx, "fy_px": fy,
        "road_width_m": ROAD_WIDTH_M, "white_width_m": WHITE_WIDTH_M,
        "edge_strip_width_each_m": BORDER_WIDTH_M,
        "center_offset_m": center_offset_m,
        "curve_radius_m": curve_radius_m,
        "ring_radius_m": ring_radius_m if road_type == "ring_island" else None,
        "turn": turn if road_type in ("curve", "ring_island") else None,
        "s_first_turn": s_first_turn if road_type == "s_curve" else None,
        "seams_added_after_groundtruth": seams,
        "shadow_added_after_groundtruth": shadow,
        "ground_intersection_rule": "tan(plane_pitch)*ray_dy-ray_dz <= 0 means background, never road",
        "world_distance_map": "camera-to-plane ray distance in meters; zero means no ground intersection",
        "slope_limit": "single infinite plane; does not model crest/grade-entry transition or vehicle pitch dynamics",
        "visible_outer_edge_rows": visible,
        "longest_contiguous_paired_edge_rows": longest,
        "path_coordinates": "x lateral right; s forward along sloped road plane",
        "mask_values": "row-major bytes: 0 absent, 255 present",
    }
    return SceneResult(
        scene_id, rgb, gray, ground, road, white, border, floor, boundary,
        left_edge, right_edge, white_edge, left_white, right_white, branch,
        island, distances, segments, metadata,
    )


def centerline_distance_at_pixel(scene: SceneResult, x: float, y: float) -> tuple[float, float] | None:
    """Return (nearest centerline distance, signed lateral offset), in meters.

    x/y are image-grid coordinates (integer coordinates name pixel centers;
    fractional coordinates are accepted). The calculation reprojects the ray
    using scene geometry, then measures it against the original world route.
    It does not consult detector output or appearance masks. None means the ray
    does not intersect the modeled ground plane.
    """
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("x and y must be finite pixel coordinates")
    ground = ground_point_at_pixel(scene, x, y)
    if ground is None:
        return None
    nearest = _nearest_full(ground[0], ground[1], scene.truth_segments)
    return nearest


def centerline_truth_at_pixel(scene: SceneResult, x: float, y: float) -> tuple[float, float, float] | None:
    """Return (distance, signed offset, route chainage) for a projected pixel."""
    ground = ground_point_at_pixel(scene, x, y)
    if ground is None:
        return None
    return _nearest_full_with_chainage(ground[0], ground[1], scene.truth_segments)


def _nearest_full(x: float, s: float, segments: Sequence[_Segment]) -> tuple[float, float] | None:
    """Unaccelerated exact-route query used by the public scoring utility."""
    best_d2, best_signed = math.inf, 0.0
    for seg in segments:
        along = min(seg.length, max(0.0, (x - seg.x0) * seg.ux + (s - seg.s0) * seg.us))
        px, ps = seg.x0 + along * seg.ux, seg.s0 + along * seg.us
        ox, os = x - px, s - ps
        d2 = ox * ox + os * os
        if d2 < best_d2:
            best_d2 = d2
            best_signed = ox * seg.us - os * seg.ux
    return (math.sqrt(best_d2), best_signed) if math.isfinite(best_d2) else None


def _nearest_full_with_chainage(
    x: float, s: float, segments: Sequence[_Segment]
) -> tuple[float, float, float] | None:
    best_d2, best_signed, best_chainage = math.inf, 0.0, 0.0
    for seg in segments:
        along = min(seg.length, max(0.0, (x - seg.x0) * seg.ux + (s - seg.s0) * seg.us))
        px, ps = seg.x0 + along * seg.ux, seg.s0 + along * seg.us
        ox, os = x - px, s - ps
        d2 = ox * ox + os * os
        if d2 < best_d2:
            best_d2 = d2
            best_signed = ox * seg.us - os * seg.ux
            best_chainage = seg.chainage0 + along
    return (math.sqrt(best_d2), best_signed, best_chainage) if math.isfinite(best_d2) else None


def _write_ppm(path: Path, pixels: bytearray) -> None:
    path.write_bytes(f"P6\n{WIDTH} {HEIGHT}\n255\n".encode("ascii") + pixels)


def _write_pgm(path: Path, pixels: bytearray) -> None:
    path.write_bytes(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode("ascii") + pixels)


def _write_distance_pgm(path: Path, distances: Sequence[float]) -> None:
    body = bytearray()
    for distance in distances:
        value = 0 if distance <= 0 else min(65535, round(distance * 1000) + 1)
        body.extend(struct.pack(">H", value))
    path.write_bytes(f"P5\n{WIDTH} {HEIGHT}\n65535\n".encode("ascii") + body)


def _specs() -> list[dict[str, Any]]:
    out = []
    for offset, name in ((0, "center"), (0.08, "offset_p08"), (-0.08, "offset_m08")):
        out.append({"road_type": "straight", "center_offset_m": offset, "tag": "straight_" + name})
    for radius in (0.5, 0.8):
        for turn in ("left", "right"):
            out.append({"road_type": "curve", "curve_radius_m": radius, "turn": turn, "tag": f"curve_{turn}_r{int(radius*100)}"})
    for turn in ("left", "right"):
        out.append({"road_type": "s_curve", "curve_radius_m": 1.4, "s_first_turn": turn, "tag": f"s_curve_{turn}_first"})
    for turn in ("left", "right"):
        out.append({"road_type": "ring_island", "ring_radius_m": 0.78, "turn": turn, "tag": f"ring_{turn}"})
    out.extend((
        {"road_type": "t_branch", "tag": "t_branch"},
        {"road_type": "cross_branch", "tag": "cross_branch"},
        {"road_type": "uphill", "plane_pitch_deg": 20.0, "tag": "uphill_20deg"},
        {"road_type": "downhill", "plane_pitch_deg": -20.0, "tag": "downhill_m20deg"},
    ))
    return out


def generate_manifest(output_dir: str | Path | None = None) -> dict[str, Any]:
    """Write the complete deterministic image/mask dataset and JSON manifest."""
    root = Path(output_dir) if output_dir is not None else DATA_DIR
    image_dir, gray_dir, mask_dir = root / "images", root / "gray", root / "masks"
    for directory in (image_dir, gray_dir, mask_dir):
        directory.mkdir(parents=True, exist_ok=True)
    entries = []
    for spec in _specs():
        for down in (5.0, 10.0, 20.0, 30.0):
            for style in ("practice", "formal"):
                base = f"{spec['tag']}_d{int(down):02d}_{style}"
                scene = generate_scene(
                    base, road_type=spec["road_type"], camera_down_deg=down,
                    track_style=style, center_offset_m=spec.get("center_offset_m", 0.0),
                    curve_radius_m=spec.get("curve_radius_m"), ring_radius_m=spec.get("ring_radius_m", 0.78),
                    turn=spec.get("turn", "left"), s_first_turn=spec.get("s_first_turn", "left"),
                    plane_pitch_deg=spec.get("plane_pitch_deg"),
                    camera_angle_reference=spec.get("camera_angle_reference", "world"), seams=True,
                    shadow=spec["road_type"] in ("straight", "curve") and style == "practice",
                )
                _write_ppm(image_dir / f"{base}.ppm", scene.rgb)
                _write_pgm(gray_dir / f"{base}.pgm", scene.gray)
                data = {
                    "ground": scene.ground_mask, "road": scene.road_mask,
                    "white": scene.white_mask, "border": scene.border_mask,
                    "boundary": scene.boundary_truth, "left_edge": scene.left_edge_truth,
                    "right_edge": scene.right_edge_truth, "white_edge": scene.white_edge_truth,
                    "branch": scene.branch_mask, "island": scene.island_mask,
                }
                paths = {}
                for name, mask in data.items():
                    rel = f"masks/{base}_{name}.pgm"
                    _write_pgm(root / rel, mask)
                    paths[name] = rel
                distance_rel = f"masks/{base}_distance_mm_plus1.pgm"
                _write_distance_pgm(root / distance_rel, scene.world_distance_m)
                entry = dict(scene.metadata)
                entry.update({
                    "rgb_ppm": f"images/{base}.ppm",
                    "gray_pgm": f"gray/{base}.pgm",
                    "masks": paths,
                    "world_distance_pgm16": distance_rel,
                    "world_distance_encoding": "big-endian uint16 PGM; 0=no hit, else round(distance_m*1000)+1",
                })
                entries.append(entry)
    manifest = {
        "dataset_version": SCENE_VERSION,
        "purpose": "synthetic detector diagnostics only; not mounted-camera or vehicle acceptance",
        "slope_model_limit": "Each grade is one infinite plane; no crest, grade-entry transition, or vehicle pitch dynamics are modeled.",
        "image_size_px": [WIDTH, HEIGHT],
        "camera": {
            "model": "ideal pinhole, pixel-center rays, no lens distortion",
            "height_m": CAMERA_HEIGHT_M, "hfov_deg": HFOV_DEG, "vfov_deg": VFOV_DEG,
            "camera_down_angles_deg": [5, 10, 20, 30],
            "camera_angle_reference_for_slope_scenes": "world",
        },
        "road_geometry": {
            "outer_width_m": ROAD_WIDTH_M, "white_width_m": WHITE_WIDTH_M,
            "border_strip_width_each_m": BORDER_WIDTH_M,
            "practice_edge_and_floor_rgb": [20, 40, 150], "white_rgb": [220, 220, 220],
            "formal_edge_rgb": [30, 30, 30], "formal_floor_rgb": [160, 160, 160],
            "gray_conversion": "(77*R+150*G+29*B+128)>>8",
        },
        "truth_definitions": {
            "road_mask": "nearest centerline distance <= 0.225m after a ray-plane hit",
            "white_mask": "nearest centerline distance <= 0.200m; avoids white end-caps beyond the inner road width",
            "border_mask": "lateral magnitude 0.200m..0.225m",
            "boundary_truth": "outer +/-0.225m road edge within projected half-pixel tolerance",
            "white_edge_truth": "white/border interface at +/-0.200m within tolerance",
            "ground_mask": "ray intersects plane; rays with groundrayden <= 0 are background",
            "expected_valid": "true only for >=8 adjacent y=20..59 rows with one >=3px road run and both outer edges inside raster; otherwise null/unscored",
            "branch_ring": "unscored for single-path validity; topology labels remain available",
            "appearance_effects": "seams and shadows are added only after truth masks",
        },
        "scene_count": len(entries),
        "scenes": entries,
    }
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return manifest


def _near_course_specs() -> list[dict[str, Any]]:
    """Return camera poses on/near bends and topology entries, not remote straights."""
    specs: list[dict[str, Any]] = []
    offsets = ((0.0, "off0"), (0.08, "offp08"), (-0.08, "offm08"))
    bend_start = 6.5

    for radius in (0.5, 0.8):
        arc = min(1.55, radius * 1.25)
        for turn in ("left", "right"):
            stages = (
                ("entry_020", bend_start - 0.20, 0.20),
                ("arc_mid", bend_start + arc / 2, -arc / 2),
            )
            for stage, chainage, feature_distance in stages:
                for offset, offset_tag in offsets:
                    for down in (20.0, 30.0):
                        for style in ("practice", "formal"):
                            specs.append({
                                "road_type": "curve", "curve_radius_m": radius,
                                "turn": turn, "camera_stage": stage,
                                "camera_route_chainage_m": chainage,
                                "camera_feature_distance_m": feature_distance,
                                "camera_lateral_offset_m": offset,
                                "camera_offset_tag": offset_tag,
                                "probe_group": "base_h022",
                                "camera_down_deg": down, "track_style": style,
                                "tag": f"curve_{turn}_r{int(radius * 100)}_{stage}_d{int(down):02d}_{offset_tag}_{style}",
                            })

    s_start, s_arc = 6.5, 1.55
    for first_turn in ("left", "right"):
        stages = (
            ("entry_020", s_start - 0.20, 0.20),
            ("arc1_mid", s_start + s_arc / 2, -s_arc / 2),
            ("arc2_mid", s_start + 1.5 * s_arc, -1.5 * s_arc),
        )
        for stage, chainage, feature_distance in stages:
            for offset, offset_tag in offsets:
                for down in (20.0, 30.0):
                    for style in ("practice", "formal"):
                        specs.append({
                            "road_type": "s_curve", "curve_radius_m": 1.4,
                            "s_first_turn": first_turn, "camera_stage": stage,
                            "camera_route_chainage_m": chainage,
                            "camera_feature_distance_m": feature_distance,
                            "camera_lateral_offset_m": offset,
                            "camera_offset_tag": offset_tag,
                            "probe_group": "base_h022",
                            "camera_down_deg": down, "track_style": style,
                            "tag": f"s_{first_turn}_first_{stage}_d{int(down):02d}_{offset_tag}_{style}",
                        })

    # The approach endpoint is at s=0.9 for ring entry and s=1.05 for T/cross.
    # Camera stations are 0.25, 0.50, or 0.90 m before those visible features.
    for turn in ("left", "right"):
        entry_chainage = 0.90 - (-5.0)
        for distance in (0.25, 0.50, 0.90):
            for offset, offset_tag in offsets:
                for down in (20.0, 30.0):
                    for style in ("practice", "formal"):
                        specs.append({
                            "road_type": "ring_island", "ring_radius_m": 0.78,
                            "turn": turn, "camera_stage": f"entry_{int(distance*100):03d}",
                            "camera_route_chainage_m": entry_chainage - distance,
                            "camera_feature_distance_m": distance,
                            "camera_lateral_offset_m": offset,
                            "camera_offset_tag": offset_tag,
                            "probe_group": "base_h022",
                            "camera_down_deg": down, "track_style": style,
                            "tag": f"ring_{turn}_entry_{int(distance*100):03d}_d{int(down):02d}_{offset_tag}_{style}",
                        })

    branch_chainage = 1.05 - (-5.0)
    for road_type in ("t_branch", "cross_branch"):
        for distance in (0.25, 0.50, 0.90):
            for offset, offset_tag in offsets:
                for down in (20.0, 30.0):
                    for style in ("practice", "formal"):
                        specs.append({
                            "road_type": road_type,
                            "camera_stage": f"entry_{int(distance*100):03d}",
                            "camera_route_chainage_m": branch_chainage - distance,
                            "camera_feature_distance_m": distance,
                            "camera_lateral_offset_m": offset,
                            "camera_offset_tag": offset_tag,
                            "probe_group": "base_h022",
                            "camera_down_deg": down, "track_style": style,
                            "tag": f"{road_type}_entry_{int(distance*100):03d}_d{int(down):02d}_{offset_tag}_{style}",
                        })
    # Small stand-height experiment on the same true curve entry/interior
    # poses. Keep lateral offset centered so height is the changed mount
    # variable; the main near-course cases remain at the requested 22 cm.
    for height in (0.35, 0.40):
        for radius in (0.5, 0.8):
            arc = min(1.55, radius * 1.25)
            for turn in ("left", "right"):
                for stage, chainage, feature_distance in (
                    ("entry_020", bend_start - 0.20, 0.20),
                    ("arc_mid", bend_start + arc / 2, -arc / 2),
                ):
                    for down in (20.0, 30.0):
                        for style in ("practice", "formal"):
                            specs.append({
                                "road_type": "curve", "curve_radius_m": radius,
                                "turn": turn, "camera_stage": stage,
                                "camera_route_chainage_m": chainage,
                                "camera_feature_distance_m": feature_distance,
                                "camera_lateral_offset_m": 0.0,
                                "camera_offset_tag": "off0",
                                "probe_group": f"height_h{int(height * 100):03d}",
                                "camera_height_m": height,
                                "camera_down_deg": down, "track_style": style,
                                "tag": f"curve_{turn}_r{int(radius * 100)}_{stage}_h{int(height * 100):03d}_d{int(down):02d}_off0_{style}",
                            })
    return specs


def generate_near_course_manifest(output_dir: str | Path | None = None) -> dict[str, Any]:
    """Write independent near-bend camera-pose cases and geometric labels."""
    root = Path(output_dir) if output_dir is not None else POSE_DATA_DIR
    image_dir, gray_dir, mask_dir = root / "images", root / "gray", root / "masks"
    for directory in (image_dir, gray_dir, mask_dir):
        directory.mkdir(parents=True, exist_ok=True)
    entries = []
    for spec in _near_course_specs():
        scene_id = "near_" + spec["tag"]
        scene = generate_scene(
            scene_id,
            road_type=spec["road_type"],
            camera_down_deg=spec["camera_down_deg"],
            camera_height_m=spec.get("camera_height_m", 0.22),
            track_style=spec["track_style"],
            curve_radius_m=spec.get("curve_radius_m"),
            ring_radius_m=spec.get("ring_radius_m", 0.78),
            turn=spec.get("turn", "left"),
            s_first_turn=spec.get("s_first_turn", "left"),
            plane_pitch_deg=0.0,
            camera_angle_reference="world",
            camera_route_chainage_m=spec["camera_route_chainage_m"],
            camera_lateral_offset_m=spec["camera_lateral_offset_m"],
            camera_stage=spec["camera_stage"],
            camera_feature_distance_m=spec["camera_feature_distance_m"],
            seams=True,
            shadow=False,
        )
        _write_ppm(image_dir / f"{scene_id}.ppm", scene.rgb)
        _write_pgm(gray_dir / f"{scene_id}.pgm", scene.gray)
        mask_paths = {}
        for name, mask in (
            ("ground", scene.ground_mask), ("road", scene.road_mask),
            ("white", scene.white_mask), ("border", scene.border_mask),
            ("boundary", scene.boundary_truth), ("branch", scene.branch_mask),
            ("island", scene.island_mask),
        ):
            rel = f"masks/{scene_id}_{name}.pgm"
            _write_pgm(root / rel, mask)
            mask_paths[name] = rel
        entry = dict(scene.metadata)
        entry.update({
            "rgb_ppm": f"images/{scene_id}.ppm",
            "gray_pgm": f"gray/{scene_id}.pgm",
            "masks": mask_paths,
            "camera_feature_distance_definition": (
                "positive means before the first bend/entry; negative means this route-chainage lies after bend onset"
            ),
            "probe_group": spec.get("probe_group", "base_h022"),
        })
        entries.append(entry)

    scored = sum(entry["expected_valid"] is True for entry in entries)
    unscored = len(entries) - scored
    manifest = {
        "dataset_version": POSE_SCENE_VERSION,
        "purpose": "independent camera-pose projected samples for near bends and topology entries; synthetic diagnostics only",
        "legacy_dataset_preserved_at": "../manifest.json",
        "image_size_px": [WIDTH, HEIGHT],
        "camera": {
            "model": "ideal pinhole; pixel-center rays transformed from the camera tangent/right/up basis into the road frame",
            "height_m": CAMERA_HEIGHT_M,
            "tested_height_m": sorted({e["camera_height_m"] for e in entries}),
            "hfov_deg": HFOV_DEG,
            "vfov_deg": VFOV_DEG,
            "down_angles_deg": [20, 30],
            "pose": "camera position interpolated on original world route chainage, tangent aligned to that route, then displaced by lateral offset",
            "lateral_offsets_m_right_positive": [-0.08, 0.0, 0.08],
        },
        "road_geometry": {
            "outer_width_m": ROAD_WIDTH_M,
            "white_width_m": WHITE_WIDTH_M,
            "practice_edge_rgb": [20, 40, 150],
            "formal_edge_rgb": [30, 30, 30],
            "white_rgb": [220, 220, 220],
        },
        "truth": {
            "road_white_membership": "computed from ray-plane ground position and distance to original centerline segments before seams",
            "centerline_error": "Euclidean ground-plane distance to the nearest original route segment, independently reprojected from each detector grid point",
            "border_or_background": "not white truth; detector center trace/target may not enter blue/black border, floor, or background",
            "ring_island": "island mask is the interior of the ring and always non-road",
            "scoring": "only expected_valid=true scenes may be pass/fail scored; visibility-limited/topology cases remain explicitly unscored",
        },
        "coverage": {
            "scene_count": len(entries),
            "scored_expected_valid_true": scored,
            "unscored_expected_valid_null": unscored,
            "by_road_type": {
                road_type: {
                    "scene_count": sum(e["road_type"] == road_type for e in entries),
                    "scored": sum(e["road_type"] == road_type and e["expected_valid"] is True for e in entries),
                    "unscored": sum(e["road_type"] == road_type and e["expected_valid"] is None for e in entries),
                    "camera_stages": sorted({e["camera_stage"] for e in entries if e["road_type"] == road_type}),
                    "visible_paired_edge_row_counts": sorted({e["longest_contiguous_paired_edge_rows"] for e in entries if e["road_type"] == road_type}),
                }
                for road_type in sorted({e["road_type"] for e in entries})
            },
            "by_camera_height_m": {
                f"{height:.2f}": {
                    "scene_count": sum(e["camera_height_m"] == height for e in entries),
                    "scored": sum(e["camera_height_m"] == height and e["expected_valid"] is True for e in entries),
                    "unscored": sum(e["camera_height_m"] == height and e["expected_valid"] is None for e in entries),
                    "max_paired_edge_rows": max((e["longest_contiguous_paired_edge_rows"] for e in entries if e["camera_height_m"] == height), default=0),
                }
                for height in sorted({e["camera_height_m"] for e in entries})
            },
            "by_probe_group": {
                group: {
                    "scene_count": sum(e["probe_group"] == group for e in entries),
                    "scored": sum(e["probe_group"] == group and e["expected_valid"] is True for e in entries),
                    "unscored": sum(e["probe_group"] == group and e["expected_valid"] is None for e in entries),
                    "max_paired_edge_rows": max((e["longest_contiguous_paired_edge_rows"] for e in entries if e["probe_group"] == group), default=0),
                }
                for group in sorted({e["probe_group"] for e in entries})
            },
        },
        "limitation": "No real-camera, rolling shutter, lens distortion, vehicle dynamics, crest transition, or lighting-domain validation.",
        "scenes": entries,
    }
    (root / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return manifest


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DATA_DIR)
    args = parser.parse_args(argv)
    adapter = RasterImage(bytearray((42,)), 1, 1, 1)
    assert adapter.get_pixel((0, 0)) == 42
    try:
        adapter.get_pixel(0)  # type: ignore[arg-type]
    except TypeError:
        pass
    else:
        raise AssertionError("integer-coordinate get_pixel call must be rejected")
    manifest = generate_manifest(args.output_dir)
    scored = sum(scene["expected_valid"] is True for scene in manifest["scenes"])
    print(f"generated {manifest['scene_count']} scenes ({scored} scored; {manifest['scene_count'] - scored} unscored)")
    print(f"manifest: {args.output_dir / 'manifest.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
