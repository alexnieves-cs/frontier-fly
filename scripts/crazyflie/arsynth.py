"""A synthetic phone video of a flight, so ar_replay.py can be tried and tested before a real one.

It renders the room of an AR setup (floor, walls, the arena's boxes, the printed markers) from a
tripod camera, and flies a small Crazyflie through it along the flight log's truth: the sim's real
pose, which a real flight log does not have. The video begins `lead` seconds before the log's
first step, so the replay has to find the time offset itself, and it carries a SYNTHETIC label.

Alongside the video it writes <video>.truth.json: the camera's true intrinsics and pose and the
true lead, which the tests and the replay's self-check compare against. The replay never reads it
to measure anything.
"""
from __future__ import annotations

import json
import math
import random
import shutil
import subprocess
from pathlib import Path

import numpy as np

from arcam import Camera, Setup, aruco_dictionary, intrinsics, look_at

import cv2

WALL_H = 2.4
FLOOR_PX_PER_M = 360
WALL_PX_PER_M = 160
LIGHT = np.array([-0.2, 0.6, 0.75]) / np.linalg.norm([-0.2, 0.6, 0.75])
REST_Z = 0.015          # the body's center when the drone stands on the floor
LABEL = 'SYNTHETIC phone video: rendered from the sim log\'s true pose, not filmed'


# ---------------------------------------------------------------------------------------------
# Textures


def _floor_texture(setup: Setup, seed: int) -> np.ndarray:
    """Oak planks along x, with the markers printed on white paper on top, in BGR float 0..1."""
    a = setup.arena
    W, H = int(a['w'] * FLOOR_PX_PER_M), int(a['d'] * FLOOR_PX_PER_M)
    rng = np.random.default_rng(seed)
    tex = np.zeros((H, W, 3), np.float32)
    plank = int(0.14 * FLOOR_PX_PER_M)
    base = np.array([0.36, 0.52, 0.68], np.float32)  # BGR oak
    for r0 in range(0, H, plank):
        x0 = -int(rng.uniform(0, 1.6) * FLOOR_PX_PER_M)
        while x0 < W:
            ln = int(rng.uniform(0.9, 1.8) * FLOOR_PX_PER_M)
            tint = base * rng.uniform(0.86, 1.1) + rng.normal(0, 0.015, 3).astype(np.float32)
            xa, xb = max(0, x0), min(W, x0 + ln)
            if xb > xa:
                rows = slice(r0, min(H, r0 + plank))
                n = rows.stop - rows.start
                grain = (np.sin(np.linspace(0, rng.uniform(8, 20), xb - xa)[None, :] +
                                np.linspace(0, rng.uniform(1, 4), n)[:, None]) * 0.035).astype(np.float32)
                tex[rows, xa:xb] = tint * (1 + grain[..., None])
                tex[rows, xa:xa + 2] *= 0.7  # plank end joint
            x0 += ln
        tex[r0:r0 + 2] *= 0.72  # the seam between rows
    tex *= (1 + rng.normal(0, 0.025, (H, W, 1))).astype(np.float32)
    tex = cv2.GaussianBlur(tex, (3, 3), 0)
    # markers on paper with a white quiet zone
    d = aruco_dictionary(setup.dictionary)
    side = int(round(setup.marker_size * FLOOR_PX_PER_M))
    quiet = int(round(0.03 * FLOOR_PX_PER_M))
    for m in setup.markers:
        img = cv2.aruco.generateImageMarker(d, int(m['id']), side).astype(np.float32) / 255
        paper = np.full((side + 2 * quiet, side + 2 * quiet), 0.97, np.float32)
        paper[quiet:quiet + side, quiet:quiet + side] = img * 0.93 + 0.04
        # Texture rows run along +y, but the marker's top edge faces +y: flip it vertically.
        paper = paper[::-1]
        ang = m.get('yaw_deg', 0.0)
        if ang:
            Mr = cv2.getRotationMatrix2D(((paper.shape[1] - 1) / 2, (paper.shape[0] - 1) / 2), -ang, 1)
            paper = cv2.warpAffine(paper, Mr, paper.shape[::-1], borderValue=-1)
        cx, cy = m['x'] * FLOOR_PX_PER_M, m['y'] * FLOOR_PX_PER_M
        x0, y0 = int(round(cx - paper.shape[1] / 2)), int(round(cy - paper.shape[0] / 2))
        sub = tex[y0:y0 + paper.shape[0], x0:x0 + paper.shape[1]]
        p = paper[:sub.shape[0], :sub.shape[1]]
        keep = p >= 0
        sub[keep] = np.repeat(p[keep][:, None], 3, axis=1)
    return tex


def _wall_texture(length: float, rng, features: list[tuple]) -> np.ndarray:
    W, H = int(length * WALL_PX_PER_M), int(WALL_H * WALL_PX_PER_M)
    tex = np.ones((H, W, 3), np.float32) * np.array([0.80, 0.84, 0.86], np.float32)
    tex *= (1 + rng.normal(0, 0.012, (H, W, 1))).astype(np.float32)
    bb = int(0.09 * WALL_PX_PER_M)
    tex[H - bb:] = 0.93
    tex[H - bb:H - bb + 2] = 0.6
    for kind, u0, u1, z0, z1, color in features:
        a, b = int(u0 * WALL_PX_PER_M), int(u1 * WALL_PX_PER_M)
        r0, r1 = H - int(z1 * WALL_PX_PER_M), H - int(z0 * WALL_PX_PER_M)
        tex[r0:r1, a:b] = color
        if kind == 'window':
            tex[r0:r1, a:b] *= np.linspace(1.0, 0.92, r1 - r0)[:, None, None]
            tex[(r0 + r1) // 2 - 2:(r0 + r1) // 2 + 2, a:b] = 0.9
            tex[r0:r1, (a + b) // 2 - 2:(a + b) // 2 + 2] = 0.9
        frame = 5 if kind == 'window' else 4
        tex[r0:r0 + frame, a:b] = tex[r1 - frame:r1, a:b] = 0.25 if kind == 'picture' else 0.95
        tex[r0:r1, a:a + frame] = tex[r0:r1, b - frame:b] = 0.25 if kind == 'picture' else 0.95
    return cv2.GaussianBlur(tex, (3, 3), 0)


# ---------------------------------------------------------------------------------------------
# The static room, ray cast once (the camera does not move)


def _sample(tex: np.ndarray, x: np.ndarray, y: np.ndarray, border: int) -> np.ndarray:
    """tex at texel coordinates (x, y), bilinear: cv2.remap in rows of 4096 (it caps map sizes)."""
    n = len(x)
    cols = 4096
    pad = (-n) % cols
    mx = np.concatenate([x.astype(np.float32), np.zeros(pad, np.float32)]).reshape(-1, cols)
    my = np.concatenate([y.astype(np.float32), np.zeros(pad, np.float32)]).reshape(-1, cols)
    return cv2.remap(tex, mx, my, cv2.INTER_LINEAR, borderMode=border).reshape(-1, 3)[:n]


def _boxes(setup: Setup) -> list[tuple[float, float, float, float, float]]:
    out = []
    for i, b in enumerate(setup.arena.get('boxes', [])):
        h = setup.box_height(i) or 0.5
        out.append((b['x'], b['y'], b['x'] + b['w'], b['y'] + b['d'], h))
    return out


def render_room(cam: Camera, setup: Setup, seed: int = 1, ss: int = 2):
    """The room as the camera sees it: (BGR float image, ray distance per pixel, floor x and y per
    pixel with NaN where the pixel is not floor). Supersampled ss x ss against aliasing."""
    a = setup.arena
    Aw, Ad = a['w'], a['d']
    rng = np.random.default_rng(seed + 101)
    floor = _floor_texture(setup, seed)
    walls = {
        'x0': _wall_texture(Ad, rng, [('picture', 1.3, 1.9, 1.15, 1.6, (0.55, 0.45, 0.30))]),
        'xW': _wall_texture(Ad, rng, [('picture', 1.0, 1.7, 1.2, 1.7, (0.35, 0.55, 0.72))]),
        'y0': _wall_texture(Aw, rng, []),
        'yD': _wall_texture(Aw, rng, [('window', 1.3, 2.6, 0.85, 2.05, (1.0, 0.98, 0.95)),
                                      ('picture', 3.1, 3.6, 1.2, 1.55, (0.30, 0.38, 0.62))]),
    }
    boxes = _boxes(setup)
    K2 = cam.K.copy()
    K2[0, 0] *= ss
    K2[1, 1] *= ss
    K2[0, 2] = (cam.K[0, 2] + 0.5) * ss - 0.5
    K2[1, 2] = (cam.K[1, 2] + 0.5) * ss - 0.5
    big = Camera(K2, cam.R, cam.t, cam.w * ss, cam.h * ss)
    c = cam.center
    H2, W2 = big.h, big.w
    img = np.zeros((H2, W2, 3), np.float32)
    dist = np.zeros((H2, W2), np.float32)
    fx = np.full((H2, W2), np.nan, np.float32)
    fy = np.full((H2, W2), np.nan, np.float32)
    step = max(1, 400_000 // W2)
    us = np.arange(W2, dtype=np.float64)
    for r0 in range(0, H2, step):
        rows = np.arange(r0, min(H2, r0 + step), dtype=np.float64)
        uu, vv = np.meshgrid(us, rows)
        d = big.ray(np.column_stack([uu.ravel(), vv.ravel()]))
        n = len(d)
        best = np.full(n, np.inf)
        kind = np.full(n, -1, np.int16)
        normal = np.zeros((n, 3))
        with np.errstate(divide='ignore', invalid='ignore'):
            # floor
            s = -c[2] / d[:, 2]
            p = c + s[:, None] * d
            ok = (s > 0) & (p[:, 0] >= 0) & (p[:, 0] <= Aw) & (p[:, 1] >= 0) & (p[:, 1] <= Ad)
            upd = ok & (s < best)
            best[upd], kind[upd] = s[upd], 0
            normal[upd] = (0, 0, 1)
            # walls
            for k, (axis, val, nrm) in enumerate(((0, 0.0, (1, 0, 0)), (0, Aw, (-1, 0, 0)),
                                                  (1, 0.0, (0, 1, 0)), (1, Ad, (0, -1, 0)))):
                s = (val - c[axis]) / d[:, axis]
                p = c + s[:, None] * d
                o = 1 - axis
                lim = Ad if axis == 0 else Aw
                ok = (s > 0) & (p[:, o] >= 0) & (p[:, o] <= lim) & (p[:, 2] >= 0) & (p[:, 2] <= WALL_H)
                upd = ok & (s < best)
                best[upd], kind[upd] = s[upd], 1 + k
                normal[upd] = nrm
            # boxes: slab test
            for bi, (x0, y0, x1, y1, h) in enumerate(boxes):
                lo = np.array([x0, y0, 0.0])
                hi = np.array([x1, y1, h])
                t1 = (lo - c) / d
                t2 = (hi - c) / d
                tmin = np.nanmax(np.minimum(t1, t2), axis=1)
                tmax = np.nanmin(np.maximum(t1, t2), axis=1)
                hit = (tmax >= tmin) & (tmin > 0)
                upd = hit & (tmin < best)
                if upd.any():
                    best[upd], kind[upd] = tmin[upd], 10 + bi
                    ax = np.argmax(np.minimum(t1, t2)[upd], axis=1)
                    sgn = np.where(d[upd, ax] > 0, -1.0, 1.0)
                    nn = np.zeros((upd.sum(), 3))
                    nn[np.arange(len(ax)), ax] = sgn
                    normal[upd] = nn
        p = c + np.where(np.isfinite(best), best, 0)[:, None] * d
        col = np.zeros((n, 3), np.float32)
        col[:] = (0.75, 0.78, 0.8)  # anything unhit: ceiling
        lam = (0.55 + 0.45 * np.clip(normal @ LIGHT, 0, 1)).astype(np.float32)
        lam[kind < 0] = 0.9
        # floor
        m = kind == 0
        if m.any():
            col[m] = _sample(floor, p[m, 0] * FLOOR_PX_PER_M, p[m, 1] * FLOOR_PX_PER_M, cv2.BORDER_REFLECT)
            # contact shadows near walls and boxes, and the window's pool of light
            dw = np.minimum.reduce([p[m, 0], Aw - p[m, 0], p[m, 1], Ad - p[m, 1]])
            ao = 1 - 0.2 * np.exp(-dw / 0.07)
            for x0, y0, x1, y1, h in boxes:
                dx = np.maximum.reduce([x0 - p[m, 0], np.zeros(m.sum()), p[m, 0] - x1])
                dy = np.maximum.reduce([y0 - p[m, 1], np.zeros(m.sum()), p[m, 1] - y1])
                ao *= 1 - 0.35 * np.exp(-np.hypot(dx, dy) / 0.06)
            pool = 1 + 0.22 * np.exp(-((p[m, 0] - 1.95) ** 2 / 0.9 + (p[m, 1] - Ad) ** 2 / 1.4))
            lam[m] *= (ao * pool).astype(np.float32)
            rr = np.repeat(np.arange(r0, r0 + len(rows)), W2)[m]
            cc = np.tile(np.arange(W2), len(rows))[m]
            fx[rr, cc] = p[m, 0]
            fy[rr, cc] = p[m, 1]
        # walls
        for k, (key, axis) in enumerate((('x0', 1), ('xW', 1), ('y0', 0), ('yD', 0))):
            m = kind == 1 + k
            if not m.any():
                continue
            tex = walls[key]
            col[m] = _sample(tex, p[m, axis] * WALL_PX_PER_M, (WALL_H - p[m, 2]) * WALL_PX_PER_M, cv2.BORDER_REPLICATE)
            lam[m] *= (1 - 0.18 * np.exp(-p[m, 2] / 0.12)).astype(np.float32)
        # boxes: a walnut bookshelf with shelf lines, a pale side table
        for bi, (x0, y0, x1, y1, h) in enumerate(boxes):
            m = kind == 10 + bi
            if not m.any():
                continue
            if bi == 0:
                base = np.array([0.16, 0.25, 0.36], np.float32)
                z = p[m, 2]
                shelf = (np.abs(((z / h) * 3) % 1) < 0.05) & (normal[m, 2] == 0)
                cm = np.repeat(base[None], m.sum(), axis=0)
                cm[shelf] *= 0.55
                col[m] = cm
            else:
                col[m] = (0.78, 0.8, 0.82)
        img[r0:r0 + len(rows)] = (col * lam[:, None]).reshape(len(rows), W2, 3)
        dist[r0:r0 + len(rows)] = np.where(np.isfinite(best), best, 99).reshape(len(rows), W2)
    small = cv2.resize(img, (cam.w, cam.h), interpolation=cv2.INTER_AREA)
    dmin = dist.reshape(cam.h, ss, cam.w, ss).min(axis=(1, 3))
    o = ss // 2
    return small, dmin, fx[o::ss, o::ss].copy(), fy[o::ss, o::ss].copy()


# ---------------------------------------------------------------------------------------------
# The drone


class Pose:
    __slots__ = ('x', 'y', 'z', 'yaw', 'spin')

    def __init__(self, x, y, z, yaw, spin):
        self.x, self.y, self.z, self.yaw, self.spin = x, y, z, yaw, spin


def truth_track(steps: list[dict], start: dict):
    """(times, poses) from the log's truth, yaw unwrapped, in radians."""
    ts, rows = [], []
    for s in steps:
        tr = s.get('truth')
        if not tr:
            continue
        spin = s.get('sent') is not None and s.get('phase') in ('takeoff', 'fly', 'land')
        ts.append(s['t'])
        rows.append((tr['x'], tr['y'], max(REST_Z, tr['z']), math.radians(tr.get('yaw_deg', 0.0)), spin))
    if not ts:
        raise SystemExit('this log has no truth: only a sim log can drive the synthetic video')
    return np.array(ts), rows


def pose_at(t: float, ts: np.ndarray, rows: list) -> Pose:
    if t <= ts[0]:
        x, y, z, yaw, _ = rows[0]
        return Pose(x, y, REST_Z, yaw, False)
    if t >= ts[-1]:
        x, y, z, yaw, _ = rows[-1]
        return Pose(x, y, max(REST_Z, z - 0.6 * (t - ts[-1])), yaw, False)
    k = int(np.searchsorted(ts, t)) - 1
    f = (t - ts[k]) / (ts[k + 1] - ts[k])
    a, b = rows[k], rows[k + 1]
    return Pose(*(a[i] + f * (b[i] - a[i]) for i in range(4)), a[4] or b[4])


def _circle(cx, cy, cz, r, n=18):
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a), np.full(n, cz)])


def draw_drone(frame: np.ndarray, dist: np.ndarray, cam: Camera, p: Pose, rng: random.Random, ss: int = 3) -> None:
    """A Crazyflie 2.1 (92 mm motor to motor, 45 mm props), composited into frame (float BGR) where
    the room is not in front of it."""
    c, s = math.cos(p.yaw), math.sin(p.yaw)

    def W(bx, by, bz=0.0):
        return (p.x + c * bx - s * by, p.y + s * bx + c * by, p.z + bz)

    motors = [W(0.0325 * sx, 0.0325 * sy, 0.004) for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]
    center = np.array([p.x, p.y, p.z])
    uv0, z0 = cam.project(center)
    if z0[0] <= 0.05:
        return
    r_px = 0.09 * cam.focal / z0[0]
    u, v = uv0[0]
    pad = int(r_px + 6)
    x0, y0 = int(u) - pad, int(v) - pad
    x1, y1 = int(u) + pad + 1, int(v) + pad + 1
    X0, Y0, X1, Y1 = max(0, x0), max(0, y0), min(cam.w, x1), min(cam.h, y1)
    if X1 <= X0 or Y1 <= Y0:
        return
    hw, hh = (x1 - x0) * ss, (y1 - y0) * ss
    col = np.zeros((hh, hw, 3), np.float32)
    alpha = np.zeros((hh, hw), np.float32)

    def to_px(pts):
        q, _ = cam.project(np.asarray(pts))
        return np.round(((q - [x0, y0]) * ss + (ss - 1) / 2) * 8).astype(np.int32)

    def put(mask, color, a):
        nonlocal col, alpha
        m = mask.astype(np.float32) / 255 * a
        col = col * (1 - m[..., None]) + np.array(color, np.float32) * m[..., None]
        alpha = alpha + m * (1 - alpha)

    def poly(pts, color, a=1.0):
        mk = np.zeros((hh, hw), np.uint8)
        cv2.fillPoly(mk, [to_px(pts)], 255, cv2.LINE_AA, shift=3)
        put(mk, color, a)

    def line(pa, pb, color, width_m, a=1.0):
        mk = np.zeros((hh, hw), np.uint8)
        q = to_px([pa, pb])
        th = max(1, int(round(width_m * cam.focal / z0[0] * ss)))
        cv2.line(mk, tuple(q[0]), tuple(q[1]), 255, th, cv2.LINE_AA, shift=3)
        put(mk, color, a)

    def rect(bx, by, bz, hx, hy, color):
        poly([W(bx - hx, by - hy, bz), W(bx + hx, by - hy, bz), W(bx + hx, by + hy, bz), W(bx - hx, by + hy, bz)], color)

    rect(0, 0, -0.012, 0.016, 0.011, (0.55, 0.55, 0.5))     # flow deck and battery underneath
    for m in motors:
        line(W(0, 0), m, (0.82, 0.82, 0.8), 0.006)           # motor mounts
        poly(_circle(m[0], m[1], m[2], 0.0045, 10), (0.12, 0.12, 0.12))
    rect(0, 0, 0.0, 0.017, 0.017, (0.13, 0.13, 0.14))        # main board
    rect(0, 0, 0.011, 0.015, 0.015, (0.09, 0.09, 0.1))        # Multi-ranger deck
    for bx, by in ((0.013, 0), (-0.013, 0), (0, 0.013), (0, -0.013), (0, 0)):
        poly(_circle(*W(bx, by, 0.013), 0.0025, 8), (0.25, 0.25, 0.25))
    for m in motors:
        if p.spin:
            poly(_circle(m[0], m[1], m[2] + 0.006, 0.0225, 24), (0.35, 0.35, 0.36), 0.42)
        else:
            a = rng.uniform(0, math.pi)
            q = (0.0225 * math.cos(a), 0.0225 * math.sin(a))
            line((m[0] - q[0], m[1] - q[1], m[2] + 0.006), (m[0] + q[0], m[1] + q[1], m[2] + 0.006), (0.2, 0.2, 0.2), 0.005)
    if p.spin:
        for (bx, by), colr in (((0.03, 0.012), (0.2, 0.25, 1.0)), ((0.03, -0.012), (0.3, 1.0, 0.35)),
                               ((-0.03, 0.012), (1.0, 0.5, 0.25)), ((-0.03, -0.012), (1.0, 0.5, 0.25))):
            poly(_circle(*W(bx, by, 0.004), 0.003, 8), colr)

    col = cv2.resize(col, (x1 - x0, y1 - y0), interpolation=cv2.INTER_AREA)
    alpha = cv2.resize(alpha, (x1 - x0, y1 - y0), interpolation=cv2.INTER_AREA)
    sl = (slice(Y0 - y0, Y1 - y0), slice(X0 - x0, X1 - x0))
    col, alpha = col[sl], alpha[sl]
    dd = float(np.linalg.norm(center - cam.center))
    alpha = alpha * (dist[Y0:Y1, X0:X1] > dd - 0.04)
    roi = frame[Y0:Y1, X0:X1]
    roi[:] = roi * (1 - alpha[..., None]) + col * alpha[..., None]


def draw_shadow(frame: np.ndarray, fx: np.ndarray, fy: np.ndarray, cam: Camera, p: Pose) -> None:
    """A soft shadow on the floor: the window light comes from +y, high up."""
    if p.z <= REST_Z + 1e-3 and not p.spin:
        sx, sy, sig, dark = p.x, p.y, 0.035, 0.45
    else:
        sx, sy = p.x + 0.12 * p.z, p.y - 0.28 * p.z
        sig, dark = 0.035 + 0.06 * p.z, 0.42 * max(0.25, 1 - p.z / 1.2)
    uv, z = cam.project([(sx, sy, 0.0)])
    if z[0] <= 0:
        return
    r = int(4 * sig * cam.focal / z[0]) + 4
    u, v = int(uv[0, 0]), int(uv[0, 1])
    X0, Y0, X1, Y1 = max(0, u - r), max(0, v - r), min(cam.w, u + r), min(cam.h, v + r)
    if X1 <= X0 or Y1 <= Y0:
        return
    gx, gy = fx[Y0:Y1, X0:X1], fy[Y0:Y1, X0:X1]
    sh = dark * np.exp(-((gx - sx) ** 2 + (gy - sy) ** 2) / (2 * sig ** 2))
    sh = np.nan_to_num(sh, nan=0.0).astype(np.float32)
    frame[Y0:Y1, X0:X1] *= (1 - sh)[..., None]


# ---------------------------------------------------------------------------------------------
# Video out


class VideoOut:
    """Frames (BGR uint8) to an H.264 MP4 through ffmpeg when it is installed (plays in browsers),
    else through OpenCV's own mp4v writer."""

    def __init__(self, path: str | Path, w: int, h: int, fps: float, crf: int = 20):
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.proc = self.writer = None
        if shutil.which('ffmpeg'):
            self.proc = subprocess.Popen(
                ['ffmpeg', '-v', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', f'{w}x{h}', '-r', str(fps),
                 '-i', '-', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', str(crf), '-pix_fmt', 'yuv420p',
                 '-movflags', '+faststart', str(self.path)], stdin=subprocess.PIPE)
        else:
            self.writer = cv2.VideoWriter(str(self.path), cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))

    def write(self, frame: np.ndarray) -> None:
        if self.proc:
            self.proc.stdin.write(np.ascontiguousarray(frame).tobytes())
        else:
            self.writer.write(frame)

    def close(self) -> None:
        if self.proc:
            self.proc.stdin.close()
            if self.proc.wait() != 0:
                raise RuntimeError('ffmpeg failed writing ' + str(self.path))
        else:
            self.writer.release()


def default_camera(setup: Setup, w: int, h: int, hfov: float = 84.0) -> Camera:
    """A phone on a tall tripod in the arena's (0, 0) corner, looking across the room."""
    a = setup.arena
    R, t = look_at((0.15, 0.15, 2.1), (a['w'] * 0.56, a['d'] * 0.56, 0.0))
    return Camera(intrinsics(w, h, hfov), R, t, w, h)


def synthesize(log, setup: Setup, out: str | Path, *, w: int = 1920, h: int = 1080, fps: float = 30.0,
               hfov: float = 84.0, lead: float = 2.3, tail: float = 3.0, seed: int = 1, max_s: float | None = None,
               label: bool = True, progress=None) -> dict:
    """Render the synthetic video of `log` (a flightlog.Log) and write <out>.truth.json."""
    cam = default_camera(setup, w, h, hfov)
    room, dist, fx, fy = render_room(cam, setup, seed)
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    vign = 1 - 0.18 * (((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2) / 2
    room *= vign[..., None]
    if label:
        sc = h / 1080
        th_px = max(1, int(2 * sc))
        (tw, th), _ = cv2.getTextSize(LABEL, cv2.FONT_HERSHEY_SIMPLEX, 0.7 * sc, th_px)
        x0, y0 = int(18 * sc), h - int(54 * sc)
        box = np.zeros((h, w), np.float32)
        box[y0:y0 + th + int(22 * sc), x0:x0 + tw + int(12 * sc)] = 0.65
        room *= (1 - box)[..., None]
        room += (box * 0.08)[..., None]
        ink = np.zeros((h, w), np.uint8)
        cv2.putText(ink, LABEL, (int(24 * sc), y0 + th + int(10 * sc)), cv2.FONT_HERSHEY_SIMPLEX, 0.7 * sc, 255, th_px, cv2.LINE_AA)
        a = (ink.astype(np.float32) / 255)[..., None]
        room[:] = room * (1 - a) + 0.95 * a
    ts, rows = truth_track(log.steps, setup.arena['start'])
    dur = lead + float(ts[-1]) + tail
    if max_s:
        dur = min(dur, max_s)
    n = int(dur * fps)
    rng = random.Random(seed)
    nrng = np.random.default_rng(seed + 7)
    bank = [nrng.normal(0, 2.2, (h, w, 3)).astype(np.float32) for _ in range(5)]
    vid = VideoOut(out, w, h, fps, crf=18)
    try:
        for k in range(n):
            t = k / fps - lead
            p = pose_at(t, ts, rows)
            f = room.copy()
            draw_shadow(f, fx, fy, cam, p)
            draw_drone(f, dist, cam, p, rng)
            f *= 1 + 0.006 * math.sin(k / fps * 2 * math.pi * 0.7)
            f *= 255
            f += np.roll(bank[k % len(bank)], (rng.randrange(h), rng.randrange(w)), axis=(0, 1))
            f8 = np.clip(f, 0, 255).astype(np.uint8)
            vid.write(f8)
            if progress and k % 30 == 0:
                progress(k, n)
    finally:
        vid.close()
    side = {'note': 'Ground truth for a SYNTHETIC video made by ar_replay.py synth. The replay measures the camera '
                    'and the time offset itself; this file is only for checking it.',
            'video': Path(out).name, 'log': str(log.path.name), 'fps': fps, 'lead_s': lead, 'frames': n,
            'camera': cam.to_json(), 'seed': seed}
    Path(str(out) + '.truth.json').write_text(json.dumps(side, indent=1))
    return side
