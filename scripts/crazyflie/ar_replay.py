#!/usr/bin/env python3
"""AR flight replay: a phone video of a Crazyflie flight, with the flight log drawn into the room.

    python3 scripts/crazyflie/ar_replay.py run VIDEO LOG --setup arenas/example-room.ar.json [--out DIR]
    python3 scripts/crazyflie/ar_replay.py sheet arenas/example-room.ar.json [--out markers.png]
    python3 scripts/crazyflie/ar_replay.py synth LOG --setup arenas/example-room.ar.json --out video.mp4

The drone's flight log says where the drone thought it was: its flow deck's estimate, which drifts
and can jump. A phone on a tripod sees where it really was. `run` turns the second into numbers:

  1. finds the printed ArUco markers on the floor (laid out in the setup file) and fits the phone's
     focal length and pose from them, so pixels map to the arena frame;
  2. finds the drone in every frame (what moves against the room's median image), and puts it back
     into the room at the height the drone's downward range sensor logged;
  3. finds when the log starts in the video, by sliding the logged track along the camera's track;
  4. writes ar.mp4: the video with the arena, the fence, the boxes, the drone's own estimate, the
     rangers' beams, the guard's verdicts and the drift between estimate and camera drawn into it,
     beside a top-down map and a drift timeline; plus ar.json and track.csv with every number.

It also measures where the drone came to rest, the one true point compare.py --landed asks for,
and prints that command. `synth` renders a synthetic video of a sim log (see arsynth.py), labelled
SYNTHETIC, to try all of this before a real flight; `sheet` prints the markers at their true size.

Assumptions: the camera stands still (a tripod), the video is undistorted (phones do this), and at
least two markers stay in view. ar.json says how well each held.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from arcam import Camera, detect, detector, fit_focal, intrinsics, load_setup, marker_sheet, solve_pose  # noqa: E402
from flightlog import read_log  # noqa: E402

import cv2  # noqa: E402

FLOOR_REST = 0.015       # the body's center height when the drone stands on its feet
MAX_CANDIDATES = 8
JUMP_SPEED = 2.5         # m/s: a logged step faster than this is the estimate jumping, not the drone

# Colors, BGR
BG = (22, 17, 14)
PANEL = (33, 27, 23)
EDGE = (64, 56, 50)
TEXT = (236, 233, 229)
MUTED = (165, 155, 146)
SKY = (240, 190, 60)        # the drone's own estimate
CORAL = (102, 115, 249)     # what the camera sees
AMBER = (66, 185, 245)
GREEN = (110, 200, 80)
RED = (70, 70, 232)
TEAL = (170, 212, 45)
VERDICT_RGB = {'fly': GREEN, 'hover': AMBER, 'land': CORAL, 'kill': RED, 'landing': AMBER, 'idle': (120, 120, 120)}


# ---------------------------------------------------------------------------------------------
# The log as arrays


class Track:
    """The log's control steps as arrays on log time: the estimate, height, yaw, rangers, commands,
    verdicts, and the sim's truth when there is one."""

    def __init__(self, log):
        steps = [s for s in log.steps if s.get('tel')]
        if len(steps) < 10:
            raise SystemExit(f'{log.path}: fewer than ten steps with telemetry; nothing to replay')
        self.t = np.array([s['t'] for s in steps])
        tel = [s['tel'] for s in steps]
        self.x = np.array([v['x'] for v in tel])
        self.y = np.array([v['y'] for v in tel])
        self.z = np.array([v['z'] for v in tel])
        self.yaw = np.array([v.get('yaw_deg', 0.0) for v in tel])
        self.vbat = np.array([v.get('vbat') or np.nan for v in tel])
        self.age = np.array([s.get('age') or 0.0 for s in steps])

        def rng(v, k):
            r = v.get(k)
            return np.nan if r is None or r >= 3.9 else r

        self.ranges = {k: np.array([rng(v, k) for v in tel]) for k in ('front', 'back', 'left', 'right')}
        self.svx = np.array([(s.get('sent') or {}).get('vx', 0.0) for s in steps])
        self.svy = np.array([(s.get('sent') or {}).get('vy', 0.0) for s in steps])
        self.phase = [s.get('phase', '') for s in steps]
        self.verdict = [s.get('verdict', '') for s in steps]
        self.reason = [s.get('reason', '') for s in steps]
        tr = [s.get('truth') for s in steps]
        self.has_truth = all(tr)
        if self.has_truth:
            self.tx = np.array([v['x'] for v in tr])
            self.ty = np.array([v['y'] for v in tr])
        self.t0, self.t1 = float(self.t[0]), float(self.t[-1])
        self.flying = np.array([p in ('takeoff', 'fly', 'land') for p in self.phase])
        # The estimate with its jumps taken out: each step's move, except steps faster than the drone
        # can fly, summed up. Its moves are the flow deck's own motion, which stay good when its
        # position does not, so the tracker uses them to predict where the drone goes next.
        step = np.hypot(np.diff(self.x), np.diff(self.y))
        dt = np.maximum(np.diff(self.t), 1e-3)
        self.jump_steps = np.flatnonzero(step / dt > JUMP_SPEED) + 1
        dx, dy = np.diff(self.x), np.diff(self.y)
        dx[self.jump_steps - 1] = 0.0
        dy[self.jump_steps - 1] = 0.0
        self.xs = self.x[0] + np.concatenate([[0.0], np.cumsum(dx)])
        self.ys = self.y[0] + np.concatenate([[0.0], np.cumsum(dy)])

    def interp(self, arr, t):
        return np.interp(t, self.t, arr)

    def index(self, t: float) -> int:
        return int(np.clip(np.searchsorted(self.t, t, side='right') - 1, 0, len(self.t) - 1))

    def height(self, t):
        """The drone's height at log time t: logged during the flight, resting on the floor before it,
        and dropping to the floor after the last step (the motors stop)."""
        t = np.asarray(t, dtype=np.float64)
        z = np.interp(t, self.t, self.z)
        z = np.where(t < self.t0, max(FLOOR_REST, float(self.z[0])), z)
        after = np.maximum(FLOOR_REST, float(self.z[-1]) - 0.6 * (t - self.t1))
        return np.where(t > self.t1, after, np.maximum(z, FLOOR_REST))


# ---------------------------------------------------------------------------------------------
# Pass 1: the camera


def read_frames(path: Path, max_s: float | None = None):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f'cannot open the video {path}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    k = 0
    try:
        while True:
            ok, f = cap.read()
            if not ok or (max_s and k / fps >= max_s):
                break
            yield k, fps, f
            k += 1
    finally:
        cap.release()


def video_info(path: Path) -> tuple[float, int, int, int]:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f'cannot open the video {path}')
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w, h = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    return fps, n, w, h


def calibrate(video: Path, setup, *, hfov=None, focal=None, max_s=None, samples=48):
    """Sample frames: their median is the empty room, their markers give the camera."""
    fps, n, w, h = video_info(video)
    if max_s:
        n = min(n, int(max_s * fps))
    stride = max(1, n // samples)
    det = detector(setup)
    world = setup.corners()
    known = set(world)
    grays, dets = [], []
    for k, _, f in read_frames(video, max_s):
        if k % stride:
            continue
        g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
        grays.append(g)
        if len(dets) < 24:
            dets.append(detect(det, g, known))
    bg = np.median(np.stack(grays), axis=0).astype(np.float32)
    seen = sorted({i for d in dets for i in d})
    fitted = focal is None and hfov is None
    if fitted:
        focal, fit_rms = fit_focal(dets[:12], world, w, h)
    K = intrinsics(w, h, hfov_deg=hfov, focal_px=focal)
    poses = [p for p in (solve_pose(d, world, K) for d in dets) if p is not None]
    if not poses:
        raise SystemExit('no frame shows two of the setup\'s markers: check the dictionary, the ids and the lighting')
    # The camera stands still: solve once on every sampled detection together.
    obj = np.concatenate([world[i] for d in dets if len(d) >= 2 for i in sorted(d)])
    img = np.concatenate([d[i] for d in dets if len(d) >= 2 for i in sorted(d)])
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_SQPNP)
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, None, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    cam = Camera(K, R, tvec.ravel(), w, h)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    centers = np.array([-p[0].T @ p[1] for p in poses])
    moved = float(np.max(np.linalg.norm(centers - cam.center, axis=1)))
    noise = float(1.4826 * np.median(np.abs(grays[len(grays) // 2].astype(np.float32) - bg)))
    info = {'markers_seen': seen, 'markers_in_setup': sorted(known), 'focal_px': round(cam.focal, 2),
            'focal_fitted': fitted, 'hfov_deg': round(cam.hfov_deg, 2), 'pose_rms_px': round(rms, 3),
            'camera_xyz_m': [round(v, 3) for v in cam.center], 'camera_moved_m': round(moved, 3),
            'frames_sampled': len(grays), 'pixel_noise': round(noise, 2)}
    return cam, bg, fps, n, info


# ---------------------------------------------------------------------------------------------
# Pass 2: what moves


def candidates(gray: np.ndarray, bg: np.ndarray, thresh: float, min_area: int, max_area: int) -> np.ndarray:
    """Blobs that differ from the empty room: (k, 4) rows of weighted centroid u, v, area, contrast."""
    d = cv2.GaussianBlur(np.abs(gray.astype(np.float32) - bg), (5, 5), 0)
    mask = (d > thresh).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.dilate(mask, np.ones((5, 5), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    out = []
    for i in range(1, n):
        x, y, w, h, area = stats[i]
        if area < min_area or area > max_area:
            continue
        sub = d[y:y + h, x:x + w] * (lab[y:y + h, x:x + w] == i)
        m = float(sub.sum())
        if m <= 0:
            continue
        yy, xx = np.mgrid[y:y + h, x:x + w]
        out.append((float((xx * sub).sum() / m), float((yy * sub).sum() / m), float(area), m / area))
    out.sort(key=lambda r: -r[2] * r[3])
    return np.array(out[:MAX_CANDIDATES], dtype=np.float64).reshape(-1, 4)


def find_candidates(video: Path, bg: np.ndarray, noise: float, max_s=None, progress=None):
    h, w = bg.shape
    thresh = max(14.0, 6.0 * noise)
    scale = w / 1920
    min_area, max_area = max(6, int(40 * scale * scale)), int(25000 * scale * scale)
    out = []
    for k, _, f in read_frames(video, max_s):
        out.append(candidates(cv2.cvtColor(f, cv2.COLOR_BGR2GRAY), bg, thresh, min_area, max_area))
        if progress and k % 60 == 0:
            progress('track', k)
    return out


# ---------------------------------------------------------------------------------------------
# Pass 3: the time offset and the track


def find_offset(cands: list[np.ndarray], fps: float, cam: Camera, tr: Track) -> tuple[float, float]:
    """The offset o with video time = log time + o: where the logged estimate, projected into the
    video, best lands on something that moves. The cost is the mean pixel distance to the nearest
    candidate, capped so frames where the drone is hidden or the estimate is off count only a little."""
    F = len(cands)
    tv = np.arange(F) / fps
    P = np.full((F, MAX_CANDIDATES, 2), np.nan)
    for k, c in enumerate(cands):
        if len(c):
            P[k, :len(c)] = c[:, :2]
    cap = 40.0 * cam.w / 1920
    log_dur = tr.t1 - tr.t0
    need = max(10, int(0.3 * min(log_dur, F / fps) * fps))
    est3 = np.column_stack([tr.x, tr.y, tr.z])

    def cost(o: float, every: int = 2) -> float:
        ks = np.arange(0, F, every)
        lt = tv[ks] - o
        ok = (lt >= tr.t0) & (lt <= tr.t1)
        if ok.sum() * every < need:
            return math.inf
        ks, lt = ks[ok], lt[ok]
        pts = np.column_stack([np.interp(lt, tr.t, est3[:, i]) for i in range(3)])
        uv, z = cam.project(pts)
        dd = np.linalg.norm(P[ks] - uv[:, None, :], axis=2)
        dmin = np.nanmin(np.where(np.isnan(dd), np.inf, dd), axis=1)
        dmin = np.where(z > 0, np.minimum(dmin, cap), cap)
        return float(dmin.mean())

    lo, hi = -log_dur + 1.0, F / fps - 1.0
    coarse = np.arange(lo, hi, 0.1) + tr.t0 * 0  # video time of log time 0
    costs = [cost(o, 3) for o in coarse]
    o = float(coarse[int(np.argmin(costs))])
    fine = np.arange(o - 0.2, o + 0.2, 1 / (4 * fps))
    fc = [cost(v, 1) for v in fine]
    k = int(np.argmin(fc))
    return float(fine[k]), float(fc[k])


def track_drone(cands: list[np.ndarray], fps: float, cam: Camera, tr: Track, offset: float):
    """The drone in each frame, in the arena frame: a tracker on the plane at the logged height.

    It predicts each frame from its own last position moved by the estimate's own motion (with the
    estimate's jumps taken out), so a jumping estimate never drags it along. Among the blobs near
    that prediction it prefers the one that also sits where the estimate plus its slowly learned
    offset says: at takeoff the drone and its shadow split from one blob, and the shadow slides
    away on the plane while the drone rises straight up, as the estimate says it does."""
    dt = 1 / fps
    fixes = np.full((len(cands), 2), np.nan)
    p = None            # the tracker's own position, smoothed
    last_k = None
    last_lt = None
    bias = None         # camera minus jump-free estimate, learned slowly
    for k, c in enumerate(cands):
        lt = k * dt - offset
        z = float(tr.height(lt))
        lt_c = min(max(lt, tr.t0), tr.t1)
        est = np.array([tr.interp(tr.xs, lt_c), tr.interp(tr.ys, lt_c)])
        if not len(c):
            continue
        hyp = cam.to_plane(c[:, :2], z)
        good = ~np.isnan(hyp[:, 0])
        if not good.any():
            continue
        since = (k - last_k) * dt if last_k is not None else math.inf
        anchor = est + (bias if bias is not None else 0.0)
        if p is None or since > 1.0:
            pred, gate = anchor, 0.5
        else:
            lp = min(max(last_lt, tr.t0), tr.t1)
            moved = est - np.array([tr.interp(tr.xs, lp), tr.interp(tr.ys, lp)])
            pred, gate = p + moved, min(0.6, 0.08 + 0.5 * since)
        dist = np.linalg.norm(hyp - pred, axis=1)
        dist[~good] = np.inf
        contrast = c[:, 3] / max(1e-6, float(c[good, 3].max()))
        score = dist + 0.5 * np.linalg.norm(hyp - anchor, axis=1) + 0.03 * (1 - contrast)
        score[dist > gate] = np.inf
        j = int(np.argmin(score))
        if not np.isfinite(score[j]):
            continue
        h = hyp[j]
        p = h if p is None or since > 1.0 else pred + 0.6 * (h - pred)
        last_k, last_lt = k, lt
        a = min(1.0, dt / 2.0)
        bias = h - est if bias is None else bias + a * ((h - est) - bias)
        fixes[k] = h
    return fixes


# ---------------------------------------------------------------------------------------------
# Numbers


def measure(fixes, fps, tr: Track, offset, truth_side=None, cam=None):
    F = len(fixes)
    tv = np.arange(F) / fps
    lt = tv - offset
    inlog = (lt >= tr.t0) & (lt <= tr.t1)
    fly_at = np.zeros(F, bool)
    if inlog.any():
        idx = np.clip(np.searchsorted(tr.t, lt[inlog], side='right') - 1, 0, len(tr.t) - 1)
        fly_at[inlog] = tr.flying[idx]
    est = np.column_stack([np.interp(lt, tr.t, tr.x), np.interp(lt, tr.t, tr.y)])
    have = ~np.isnan(fixes[:, 0])
    drift = np.linalg.norm(est - fixes, axis=1)
    m = have & fly_at
    out = {'frames': F, 'frames_in_flight': int(fly_at.sum()),
           'tracked_share_in_flight': round(float(m.sum() / max(1, fly_at.sum())), 4)}
    if m.any():
        d = drift[m]
        kmax = int(np.flatnonzero(m)[np.argmax(d)])
        out['drift'] = {'median_m': round(float(np.median(d)), 4), 'p95_m': round(float(np.percentile(d, 95)), 4),
                        'max_m': round(float(d.max()), 4), 'max_at_video_s': round(kmax / fps, 3)}
        # A jump: a logged step faster than the drone can fly. How far the estimate moved, against
        # how far the camera saw the drone move in the same few tenths of a second.
        jumps = []
        for i in tr.jump_steps:
            tj = float(tr.t[i])
            if jumps and tj - jumps[-1]['log_s'] < 0.5:
                continue
            j_end = i
            while j_end + 1 in set(tr.jump_steps) and tr.t[j_end + 1] - tj < 0.5:
                j_end += 1
            ej = float(math.hypot(tr.x[j_end] - tr.x[i - 1], tr.y[j_end] - tr.y[i - 1]))
            vj = tj + offset
            before = have & (tv >= vj - 0.35) & (tv < vj - 0.05)
            after = have & (tv > vj + 0.05) & (tv <= vj + 0.35)
            cj = None
            if before.any() and after.any():
                # The camera's move, less the move the estimate's own motion explains (the drone was
                # flying): what is left is any jump the camera saw.
                lb, la = np.median(lt[before]), np.median(lt[after])
                flown = np.array([np.interp(la, tr.t, tr.xs) - np.interp(lb, tr.t, tr.xs),
                                  np.interp(la, tr.t, tr.ys) - np.interp(lb, tr.t, tr.ys)])
                cj = float(np.linalg.norm(np.median(fixes[after], axis=0) - np.median(fixes[before], axis=0) - flown))
            jumps.append({'video_s': round(vj, 3), 'log_s': round(tj, 3), 'estimate_moved_m': round(ej, 3),
                          'camera_moved_m': None if cj is None else round(cj, 3)})
        out['estimate_jumps'] = jumps
    # Where it came to rest: fixes after the log's last step (motors off), else the last half second.
    rest = have & (lt > tr.t1 + 0.4)
    if rest.sum() < 3:
        ks = np.flatnonzero(have & inlog)
        rest = np.zeros(F, bool)
        rest[ks[ks >= ks.max() - int(0.5 * fps)]] = True if len(ks) else False
    if rest.any():
        cx, cy = np.median(fixes[rest], axis=0)
        ex, ey = float(tr.x[-1]), float(tr.y[-1])
        out['landed'] = {'camera_xy': [round(float(cx), 3), round(float(cy), 3)], 'estimate_xy': [round(ex, 3), round(ey, 3)],
                         'offset_m': round(math.hypot(ex - cx, ey - cy), 4), 'frames': int(rest.sum())}
    if tr.has_truth and m.any():
        tx, ty = np.interp(lt, tr.t, tr.tx), np.interp(lt, tr.t, tr.ty)
        ce = np.hypot(fixes[:, 0] - tx, fixes[:, 1] - ty)[m]
        ee = np.hypot(est[:, 0] - tx, est[:, 1] - ty)[m]
        chk = {'camera_vs_truth_median_m': round(float(np.median(ce)), 4), 'camera_vs_truth_p95_m': round(float(np.percentile(ce, 95)), 4),
               'camera_vs_truth_max_m': round(float(ce.max()), 4), 'estimate_vs_truth_median_m': round(float(np.median(ee)), 4),
               'estimate_vs_truth_max_m': round(float(ee.max()), 4)}
        if truth_side is not None and cam is not None:
            tc = Camera.from_json(truth_side['camera'])
            chk['offset_error_s'] = round(offset - float(truth_side['lead_s']), 4)
            chk['focal_error_rel'] = round(cam.focal / tc.focal - 1, 5)
            chk['camera_position_error_m'] = round(float(np.linalg.norm(cam.center - tc.center)), 4)
        out['self_check'] = chk
    return out, est, drift, have


# ---------------------------------------------------------------------------------------------
# Drawing


def _font(size: int, weight: str = 'Regular'):
    try:
        from PIL import ImageFont
    except ImportError:
        return None
    for p in (f'/usr/share/fonts/opentype/inter/Inter-{weight}.otf', '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
              '/Library/Fonts/Arial.ttf', 'C:/Windows/Fonts/arial.ttf'):
        if Path(p).exists():
            return ImageFont.truetype(p, size)
    return None


class Text:
    """Text drawn in one go per frame: with Pillow and a real font when there is one, else OpenCV's."""

    def __init__(self):
        self.items = []
        self.cache = {}

    def add(self, xy, s, size=15, color=TEXT, weight='Regular', anchor='la'):
        self.items.append((xy, s, size, color, weight, anchor))

    def font(self, size, weight):
        k = (size, weight)
        if k not in self.cache:
            self.cache[k] = _font(size, weight)
        return self.cache[k]

    def width(self, s, size=15, weight='Regular') -> int:
        f = self.font(size, weight)
        if f is None:
            return cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, size / 30, 1)[0][0]
        return int(f.getlength(s))

    def render(self, img: np.ndarray) -> np.ndarray:
        if not self.items:
            return img
        if self.font(15, 'Regular') is None:
            for (x, y), s, size, color, _, anchor in self.items:
                w = self.width(s, size)
                x = x - w if anchor[0] == 'r' else x - w // 2 if anchor[0] == 'm' else x
                cv2.putText(img, s, (int(x), int(y + size * 0.8)), cv2.FONT_HERSHEY_SIMPLEX, size / 30, color, 1, cv2.LINE_AA)
            self.items = []
            return img
        from PIL import Image, ImageDraw
        im = Image.fromarray(img[..., ::-1])
        dr = ImageDraw.Draw(im)
        for xy, s, size, color, weight, anchor in self.items:
            dr.text(xy, s, font=self.font(size, weight), fill=tuple(int(c) for c in color[::-1]), anchor=anchor)
        self.items = []
        return np.ascontiguousarray(np.asarray(im)[..., ::-1])


def _seg_clip(cam: Camera, a, b):
    """A 3D segment's pixels, clipped to in front of the camera; None if wholly behind."""
    pts = np.array([a, b], dtype=np.float64)
    _, z = cam.project(pts)
    near = 0.05
    if z[0] < near and z[1] < near:
        return None
    if z[0] < near or z[1] < near:
        f = (near - z[0]) / (z[1] - z[0])
        q = pts[0] + f * (pts[1] - pts[0])
        pts[0 if z[0] < near else 1] = q
    uv, _ = cam.project(pts)
    if not np.all(np.isfinite(uv)) or np.abs(uv).max() > 1e5:
        return None
    return uv


def line3(img, cam, a, b, color, th=1, dash=0):
    uv = _seg_clip(cam, a, b)
    if uv is None:
        return
    p, q = uv
    if not dash:
        cv2.line(img, tuple(np.round(p * 16).astype(int)), tuple(np.round(q * 16).astype(int)), color, th, cv2.LINE_AA, shift=4)
        return
    L = float(np.linalg.norm(q - p))
    n = max(1, int(L / dash))
    for i in range(0, n, 2):
        a2, b2 = p + (q - p) * i / n, p + (q - p) * min(n, i + 1) / n
        cv2.line(img, tuple(np.round(a2 * 16).astype(int)), tuple(np.round(b2 * 16).astype(int)), color, th, cv2.LINE_AA, shift=4)


def poly3(img, cam, pts, color, th=1, closed=False, dash=0):
    pts = list(pts)
    for i in range(len(pts) - (0 if closed else 1)):
        line3(img, cam, pts[i], pts[(i + 1) % len(pts)], color, th, dash)


def ring3(img, cam, x, y, z, r, color, th=2, n=40):
    a = np.linspace(0, 2 * np.pi, n + 1)
    pts = np.column_stack([x + r * np.cos(a), y + r * np.sin(a), np.full(n + 1, z)])
    uv, zz = cam.project(pts)
    if (zz > 0.05).all():
        cv2.polylines(img, [np.round(uv * 16).astype(np.int32)], False, color, th, cv2.LINE_AA, shift=4)


def blend(img, over, a):
    cv2.addWeighted(over, a, img, 1 - a, 0, dst=img)


def chip(img, text: Text, x, y, s, color, size=14, fill=None, weight='SemiBold', anchor_right=False):
    w = text.width(s, size, weight) + 18
    h = size + 12
    if anchor_right:
        x -= w
    over = img.copy()
    cv2.rectangle(over, (x, y), (x + w, y + h), fill or (16, 13, 11), -1)
    blend(img, over, 0.78)
    cv2.rectangle(img, (x, y), (x + w, y + h), color, 1, cv2.LINE_AA)
    text.add((x + 9, y + h // 2), s, size, color, weight, 'lm')
    return x if anchor_right else x + w


def jump_text(j) -> str:
    seen = 'the camera lost it' if j.get('camera_moved_m') is None else f"the camera saw {j['camera_moved_m'] * 100:.0f} cm"
    return f"the estimate jumped {j['estimate_moved_m'] * 100:.0f} cm, {seen}"


def nice_ticks(lo, hi, n=4):
    span = max(hi - lo, 1e-9)
    raw = span / n
    mag = 10 ** math.floor(math.log10(raw))
    step = min((s * mag for s in (1, 2, 2.5, 5, 10)), key=lambda s: abs(s - raw))
    start = math.ceil(lo / step) * step
    return [round(start + i * step, 6) for i in range(int((hi - start) / step) + 1)]


class Dashboard:
    W, H = 1600, 900
    VID = (24, 64, 1120, 630)
    MAP = (1168, 64, 408, 360)
    STAT = (1168, 436, 408, 258)
    TL = (24, 708, 1552, 160)

    def __init__(self, cam: Camera, setup, tr: Track, fixes, est, drift, have, fps, offset, info, title, synthetic):
        self.cam, self.setup, self.tr = cam, setup, tr
        self.fixes, self.est, self.drift, self.have = fixes, est, drift, have
        self.fps, self.offset, self.info, self.title, self.synthetic = fps, offset, info, title, synthetic
        x, y, w, h = self.VID
        s = w / cam.w
        hh = int(round(cam.h * s))
        if hh > h:
            s = h / cam.h
        self.vs = s
        self.vw, self.vh = int(round(cam.w * s)), int(round(cam.h * s))
        K = cam.K.copy()
        K[0, 0] *= s
        K[1, 1] *= s
        K[0, 2] = (cam.K[0, 2] + 0.5) * s - 0.5
        K[1, 2] = (cam.K[1, 2] + 0.5) * s - 0.5
        self.vcam = Camera(K, cam.R, cam.t, self.vw, self.vh)
        a = setup.arena
        mx, my, mw, mh = self.MAP
        self.msc = min((mw - 40) / a['w'], (mh - 74) / a['d'])
        self.mo = (mx + (mw - a['w'] * self.msc) / 2, my + 40 + a['d'] * self.msc)
        self.text = Text()
        self.F = len(fixes)
        self.dur = self.F / fps
        self.static = self._static()
        self.fence = info.get('fence_margin', 0.3)

    # map helpers
    def m(self, x, y):
        return (int(round(self.mo[0] + x * self.msc)), int(round(self.mo[1] - y * self.msc)))

    def _static(self) -> np.ndarray:
        """Everything that does not change: panels, the map's room, the timeline's axes and curves."""
        img = np.full((self.H, self.W, 3), BG, np.uint8)
        for x, y, w, h in (self.MAP, self.STAT, self.TL):
            cv2.rectangle(img, (x, y), (x + w, y + h), PANEL, -1)
            cv2.rectangle(img, (x, y), (x + w, y + h), EDGE, 1)
        a = self.setup.arena
        t = self.text
        mx, my, mw, mh = self.MAP
        t.add((mx + 14, my + 12), 'Top down', 15, TEXT, 'SemiBold')
        t.add((mx + mw - 14, my + 14), f'{a["w"]:.1f} x {a["d"]:.1f} m', 13, MUTED, 'Regular', 'ra')
        # room, fence, grid
        cv2.rectangle(img, self.m(0, a['d']), self.m(a['w'], 0), (40, 34, 30), -1)
        for gx in np.arange(0.5, a['w'], 0.5):
            cv2.line(img, self.m(gx, 0), self.m(gx, a['d']), (48, 42, 37), 1)
        for gy in np.arange(0.5, a['d'], 0.5):
            cv2.line(img, self.m(0, gy), self.m(a['w'], gy), (48, 42, 37), 1)
        cv2.rectangle(img, self.m(0, a['d']), self.m(a['w'], 0), (200, 196, 190), 2, cv2.LINE_AA)
        f = 0.3
        self._dash2(img, [self.m(f, f), self.m(a['w'] - f, f), self.m(a['w'] - f, a['d'] - f), self.m(f, a['d'] - f)], AMBER, closed=True)
        for b in a.get('boxes', []):
            cv2.rectangle(img, self.m(b['x'], b['y'] + b['d']), self.m(b['x'] + b['w'], b['y']), (70, 78, 120), -1)
            cv2.rectangle(img, self.m(b['x'], b['y'] + b['d']), self.m(b['x'] + b['w'], b['y']), CORAL, 1, cv2.LINE_AA)
        for i, c in self.setup.corners().items():
            pts = np.array([self.m(*p[:2]) for p in c], np.int32)
            col = GREEN if i in self.info['markers_seen'] else MUTED
            cv2.polylines(img, [pts], True, col, 1, cv2.LINE_AA)
        # what the camera can see of the floor
        cw, ch = self.cam.w, self.cam.h
        edge = [(u, 0) for u in np.linspace(0, cw - 1, 9)] + [(cw - 1, v) for v in np.linspace(0, ch - 1, 6)] + \
               [(u, ch - 1) for u in np.linspace(cw - 1, 0, 9)] + [(0, v) for v in np.linspace(ch - 1, 0, 6)]
        fl = self.cam.to_plane(np.array(edge), 0.0)
        c0 = self.cam.center
        dirs = self.cam.ray(np.array(edge))
        far = c0[:2] + dirs[:, :2] / np.maximum(1e-6, np.linalg.norm(dirs[:, :2], axis=1, keepdims=True)) * 7
        far_ok = np.where(np.isnan(fl), far, fl)
        d = np.linalg.norm(far_ok - c0[:2], axis=1, keepdims=True)
        far_ok = c0[:2] + (far_ok - c0[:2]) * np.minimum(1, 7 / np.maximum(d, 1e-6))
        over = img.copy()
        poly = np.array([self.m(*p) for p in far_ok], np.int32)
        cv2.fillPoly(over, [poly], (90, 120, 60))
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.rectangle(mask, self.m(0, a['d']), self.m(a['w'], 0), 255, -1)
        blend_mask = (mask > 0)[..., None]
        img[:] = np.where(blend_mask, (0.82 * img + 0.18 * over).astype(np.uint8), img)
        cx, cy = self.m(*c0[:2])
        cv2.circle(img, (cx, cy), 5, TEXT, -1, cv2.LINE_AA)
        t.add((cx + 8, cy - 4), 'phone', 12, MUTED, 'Regular', 'ls')
        # legend
        ly = my + mh - 22
        cv2.line(img, (mx + 16, ly), (mx + 38, ly), SKY, 2, cv2.LINE_AA)
        t.add((mx + 44, ly), "drone's estimate", 13, TEXT, 'Regular', 'lm')
        cv2.line(img, (mx + 170, ly), (mx + 192, ly), CORAL, 2, cv2.LINE_AA)
        t.add((mx + 198, ly), 'camera', 13, TEXT, 'Regular', 'lm')
        cv2.rectangle(img, (mx + 268, ly - 6), (mx + 280, ly + 6), GREEN, 1)
        t.add((mx + 286, ly), 'markers', 13, TEXT, 'Regular', 'lm')
        # timeline
        x, y, w, h = self.TL
        t.add((x + 14, y + 12), "Drift: the drone's estimate minus what the camera sees", 15, TEXT, 'SemiBold')
        self.px0, self.px1 = x + 58, x + w - 18
        self.py0, self.py1 = y + 40, y + h - 40
        ys = self.drift[self.have & ~np.isnan(self.drift)]
        top = max(0.2, float(np.nanmax(ys)) * 1.15 if len(ys) else 0.2)
        self.ytop = top
        for v in nice_ticks(0, top * 100, 4):
            yy = self.ty(v / 100)
            if yy < self.py0 - 1:
                continue
            cv2.line(img, (self.px0, yy), (self.px1, yy), (48, 42, 37), 1)
            t.add((self.px0 - 8, yy), f'{v:g} cm', 12, MUTED, 'Regular', 'rm')
        for v in nice_ticks(0, self.dur, 8):
            xx = self.tx(v)
            cv2.line(img, (xx, self.py1), (xx, self.py1 + 4), MUTED, 1)
            t.add((xx, self.py1 + 20), f'{v:g} s', 12, MUTED, 'Regular', 'mm')
        # verdict strip: the guard's verdict at each step
        sy = self.py1 + 6
        for i in range(len(self.tr.t) - 1):
            a0, a1 = self.tx(self.tr.t[i] + self.offset), self.tx(self.tr.t[i + 1] + self.offset)
            cv2.rectangle(img, (a0, sy), (max(a0 + 1, a1), sy + 5), VERDICT_RGB.get(self.tr.verdict[i], MUTED), -1)
        # curves
        if self.tr.has_truth:
            lt = np.arange(self.F) / self.fps - self.offset
            ok = (lt >= self.tr.t0) & (lt <= self.tr.t1) & self.have
            tx, tyy = np.interp(lt, self.tr.t, self.tr.tx), np.interp(lt, self.tr.t, self.tr.ty)
            ce = np.hypot(self.fixes[:, 0] - tx, self.fixes[:, 1] - tyy)
            self._curve(img, np.where(ok, ce, np.nan), TEAL, 1)
            t.add((self.px1, y + 14), 'camera vs sim truth, the check', 12, TEAL, 'Regular', 'ra')
            t.add((self.px1 - 230, y + 14), 'estimate vs camera', 12, CORAL, 'SemiBold', 'ra')
        else:
            t.add((self.px1, y + 14), 'estimate vs camera', 12, CORAL, 'SemiBold', 'ra')
        self._curve(img, np.where(self.have, self.drift, np.nan), CORAL, 2)
        for j in self.info.get('estimate_jumps', []):
            xx = self.tx(j['video_s'])
            cv2.line(img, (xx, self.py0), (xx, self.py1), RED, 1, cv2.LINE_AA)
            s = jump_text(j)
            right = xx + 8 + t.width(s, 12, 'Medium') > self.px1
            t.add((xx - 6 if right else xx + 6, self.py0 + 4), s, 12, (150, 150, 255), 'Medium', 'ra' if right else 'la')
        # header and footer
        t.add((24, 22), 'AR flight replay', 22, TEXT, 'Bold', 'lm')
        t.add((24 + t.width('AR flight replay', 22, 'Bold') + 14, 24), self.title, 15, MUTED, 'Regular', 'lm')
        note = ('SYNTHETIC video, rendered from the sim log\'s truth. The camera track is measured from the pixels; '
                'the teal line checks it against that truth.') if self.synthetic else \
               ('Camera track measured from the pixels. Assumes a still camera and the logged height.')
        t.add((24, self.H - 14), note, 12, MUTED, 'Regular', 'lm')
        return t.render(img)

    def _dash2(self, img, pts, color, closed=False, dash=7):
        for i in range(len(pts) - (0 if closed else 1)):
            p, q = np.array(pts[i], float), np.array(pts[(i + 1) % len(pts)], float)
            L = np.linalg.norm(q - p)
            n = max(1, int(L / dash))
            for k in range(0, n, 2):
                a, b = p + (q - p) * k / n, p + (q - p) * min(n, k + 1) / n
                cv2.line(img, tuple(a.astype(int)), tuple(b.astype(int)), color, 1, cv2.LINE_AA)

    def tx(self, ts):
        return int(round(self.px0 + (self.px1 - self.px0) * ts / max(self.dur, 1e-6)))

    def ty(self, v):
        return int(round(self.py1 - (self.py1 - self.py0) * min(v, self.ytop) / self.ytop))

    def _curve(self, img, vals, color, th):
        pts = []
        for k in range(0, self.F, max(1, self.F // 1500)):
            if np.isnan(vals[k]):
                if len(pts) > 1:
                    cv2.polylines(img, [np.array(pts, np.int32)], False, color, th, cv2.LINE_AA)
                pts = []
                continue
            pts.append((self.tx(k / self.fps), self.ty(vals[k])))
        if len(pts) > 1:
            cv2.polylines(img, [np.array(pts, np.int32)], False, color, th, cv2.LINE_AA)

    # ---------------------------------------------------------------------------------------
    LOUPE, ZOOM = 220, 3.0

    def _loupe(self, img, video, fix, est, z, x0, y0):
        """A 3x look at the drone, from the full-size video: at this distance it is a few pixels
        wide, and the loupe shows that the camera follows the drone and not its shadow."""
        cam = self.vcam
        at = fix if fix is not None else est
        uv, dz = cam.project([(at[0], at[1], z)])
        if dz[0] <= 0.05:
            return
        c = uv[0]
        if not (0 <= c[0] < self.vw and 0 <= c[1] < self.vh):
            return
        L, Z = self.LOUPE, self.ZOOM
        sc = video.shape[1] / self.vw
        half = L / Z / 2 * sc
        cu, cv_ = c * sc
        x_a, y_a = int(round(cu - half)), int(round(cv_ - half))
        n = int(round(2 * half))
        pad = n
        src = cv2.copyMakeBorder(video, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=(20, 16, 13))
        crop = src[y_a + pad:y_a + pad + n, x_a + pad:x_a + pad + n]
        if crop.shape[0] != n or crop.shape[1] != n:
            return
        lo = cv2.resize(crop, (L, L), interpolation=cv2.INTER_CUBIC)

        def to_l(pts):
            q, d = cam.project(pts)
            return (q - c) * Z + L / 2, d

        ang = np.linspace(0, 2 * np.pi, 48)
        ring = [(est[0] + 0.09 * math.cos(a_), est[1] + 0.09 * math.sin(a_), z) for a_ in ang]
        q, d = to_l(ring)
        if (d > 0.05).all():
            cv2.polylines(lo, [np.round(q * 16).astype(np.int32)], True, SKY, 2, cv2.LINE_AA, shift=4)
        if fix is not None:
            r = int(max(9, 0.08 * cam.focal / dz[0]) * Z)
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                cx, cy = L // 2 + sx * r, L // 2 + sy * r
                cv2.line(lo, (cx, cy), (cx - sx * r // 2, cy), CORAL, 2, cv2.LINE_AA)
                cv2.line(lo, (cx, cy), (cx, cy - sy * r // 2), CORAL, 2, cv2.LINE_AA)
        lx, ly = x0 + self.vw - L - 12, y0 + 50
        img[ly:ly + L, lx:lx + L] = lo
        cv2.rectangle(img, (lx - 1, ly - 1), (lx + L, ly + L), (230, 230, 230), 1, cv2.LINE_AA)
        chip(img, self.text, lx + 6, ly + 6, f'{Z:.0f}x', TEXT, 12)

    def frame(self, k: int, video: np.ndarray) -> np.ndarray:
        img = self.static.copy()
        t = self.text
        tr, cam = self.tr, self.vcam
        tv = k / self.fps
        lt = tv - self.offset
        inlog = tr.t0 <= lt <= tr.t1
        i = tr.index(lt)
        z = float(tr.height(lt))
        a = self.setup.arena
        # the video, with the room drawn in
        v = cv2.resize(video, (self.vw, self.vh), interpolation=cv2.INTER_AREA)
        over = v.copy()
        for gx in np.arange(0.5, a['w'], 0.5):
            line3(over, cam, (gx, 0, 0), (gx, a['d'], 0), (235, 235, 235), 1)
        for gy in np.arange(0.5, a['d'], 0.5):
            line3(over, cam, (0, gy, 0), (a['w'], gy, 0), (235, 235, 235), 1)
        blend(v, over, 0.22)
        poly3(v, cam, [(0, 0, 0), (a['w'], 0, 0), (a['w'], a['d'], 0), (0, a['d'], 0)], (240, 240, 240), 2, closed=True)
        f = self.fence
        poly3(v, cam, [(f, f, 0), (a['w'] - f, f, 0), (a['w'] - f, a['d'] - f, 0), (f, a['d'] - f, 0)], AMBER, 1, closed=True, dash=9)
        for bi, b in enumerate(a.get('boxes', [])):
            hgt = self.setup.box_height(bi) or 0.5
            x0, y0, x1, y1 = b['x'], b['y'], b['x'] + b['w'], b['y'] + b['d']
            base = [(x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0)]
            topp = [(p[0], p[1], hgt) for p in base]
            uv, zz = cam.project(topp)
            if (zz > 0.05).all():
                o2 = v.copy()
                cv2.fillPoly(o2, [np.round(uv).astype(np.int32)], CORAL, cv2.LINE_AA)
                blend(v, o2, 0.18)
            poly3(v, cam, topp, CORAL, 1, closed=True)
            poly3(v, cam, base, CORAL, 1, closed=True)
            for p, q in zip(base, topp):
                line3(v, cam, p, q, CORAL, 1)
        for mid, c in self.setup.corners().items():
            if mid in self.info['markers_seen']:
                poly3(v, cam, [tuple(p) for p in c], GREEN, 1, closed=True)
        # trails, the last 6 s
        k0 = max(0, k - int(6 * self.fps))
        ks = np.arange(k0, k + 1)
        lts = ks / self.fps - self.offset
        okl = (lts >= tr.t0) & (lts <= tr.t1)
        if okl.sum() > 1:
            zz = tr.height(lts[okl])
            est3 = np.column_stack([self.est[ks[okl], 0], self.est[ks[okl], 1], zz])
            uv, dz = cam.project(est3)
            if (dz > 0.05).all():
                cv2.polylines(v, [np.round(uv * 16).astype(np.int32)], False, SKY, 2, cv2.LINE_AA, shift=4)
        hv = self.have[ks]
        if hv.sum() > 1:
            zz = tr.height(lts)
            cam3 = np.column_stack([self.fixes[ks, 0], self.fixes[ks, 1], zz])
            seg = []
            for j in range(len(ks)):
                if hv[j]:
                    seg.append(cam3[j])
                elif len(seg) > 1:
                    uv, _ = cam.project(np.array(seg))
                    cv2.polylines(v, [np.round(uv * 16).astype(np.int32)], False, CORAL, 2, cv2.LINE_AA, shift=4)
                    seg = []
                else:
                    seg = []
            if len(seg) > 1:
                uv, _ = cam.project(np.array(seg))
                cv2.polylines(v, [np.round(uv * 16).astype(np.int32)], False, CORAL, 2, cv2.LINE_AA, shift=4)
        ex, ey = self.est[k]
        fix = self.fixes[k] if self.have[k] else None
        lte = min(max(lt, tr.t0), tr.t1)
        # rangers: from where the camera sees the drone, along its logged heading
        origin = fix if fix is not None else (ex, ey)
        if inlog and z > 0.1:
            yaw = math.radians(tr.yaw[i])
            for key, off in (('front', 0), ('left', 90), ('back', 180), ('right', -90)):
                r = tr.ranges[key][i]
                ang = yaw + math.radians(off)
                d = (math.cos(ang), math.sin(ang))
                p0 = (origin[0], origin[1], z)
                if np.isnan(r):
                    line3(v, cam, p0, (origin[0] + 0.5 * d[0], origin[1] + 0.5 * d[1], z), (150, 150, 150), 1, dash=6)
                    continue
                col = RED if r < 0.3 else AMBER
                p1 = (origin[0] + r * d[0], origin[1] + r * d[1], z)
                line3(v, cam, p0, p1, col, 1)
                uv, dz = cam.project([p1])
                if dz[0] > 0.05:
                    cv2.circle(v, tuple(np.round(uv[0]).astype(int)), 3, col, -1, cv2.LINE_AA)
        # the drone's own estimate: a ring at its height with a drop line to the floor
        ring3(v, cam, ex, ey, z, 0.09, SKY, 2)
        ring3(v, cam, ex, ey, 0.0, 0.05, SKY, 1)
        line3(v, cam, (ex, ey, 0), (ex, ey, z), SKY, 1, dash=5)
        if inlog:
            sv = (tr.svx[i], tr.svy[i])
            if math.hypot(*sv) > 0.02:
                uv = _seg_clip(cam, (ex, ey, z), (ex + sv[0], ey + sv[1], z))
                if uv is not None:
                    cv2.arrowedLine(v, tuple(np.round(uv[0]).astype(int)), tuple(np.round(uv[1]).astype(int)), (245, 245, 245), 2, cv2.LINE_AA, tipLength=0.25)
        # what the camera sees: corner brackets around the drone, and the drift between the two
        if fix is not None:
            uv, dz = cam.project([(fix[0], fix[1], z)])
            if dz[0] > 0.05:
                u, w0 = np.round(uv[0]).astype(int)
                r = max(9, int(0.08 * cam.focal / dz[0]))
                for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    cx, cy = u + sx * r, w0 + sy * r
                    cv2.line(v, (cx, cy), (cx - sx * r // 2, cy), CORAL, 2, cv2.LINE_AA)
                    cv2.line(v, (cx, cy), (cx, cy - sy * r // 2), CORAL, 2, cv2.LINE_AA)
                dft = self.drift[k]
                if dft > 0.03:
                    line3(v, cam, (fix[0], fix[1], z), (ex, ey, z), (255, 255, 255), 1, dash=5)
                    mid, mdz = cam.project([((fix[0] + ex) / 2, (fix[1] + ey) / 2, z)])
                    if mdz[0] > 0.05:
                        lab = f'{dft * 100:.0f} cm'
                        mx, my = int(mid[0, 0]) + self.VID[0], int(mid[0, 1]) + self.VID[1] - 14
                        self._pending_label = (mx, my, lab)
        # paste and HUD
        x0, y0 = self.VID[0], self.VID[1] + (self.VID[3] - self.vh) // 2
        img[y0:y0 + self.vh, x0:x0 + self.vw] = v
        cv2.rectangle(img, (x0 - 1, y0 - 1), (x0 + self.vw, y0 + self.vh), EDGE, 1)
        if getattr(self, '_pending_label', None):
            mx, my, lab = self._pending_label
            chip(img, t, mx - 24, my - 12, lab, TEXT, 13, weight='SemiBold')
            self._pending_label = None
        self._loupe(img, video, fix, (ex, ey), z, x0, y0)
        nx = chip(img, t, x0 + 12, y0 + 12, f'video {tv:5.2f} s', TEXT, 13)
        chip(img, t, nx + 8, y0 + 12, f'log {lt:6.2f} s' if inlog else ('before the log' if lt < tr.t0 else 'after the log'), MUTED, 13)
        rx = x0 + self.vw - 12
        rx = chip(img, t, rx, y0 + 12, 'camera sees', CORAL, 13, anchor_right=True)
        chip(img, t, rx - 8, y0 + 12, "drone's estimate", SKY, 13, anchor_right=True)
        verdict, reason = (tr.verdict[i], tr.reason[i]) if inlog else ('', '')
        if verdict in ('hover', 'land', 'kill') or (verdict == 'landing' and reason and not reason.startswith('clamp')):
            col = VERDICT_RGB.get(verdict, AMBER)
            s = f'GUARD: {verdict.upper()}   {reason}'
            w = t.width(s, 17, 'SemiBold') + 28
            bx = x0 + (self.vw - w) // 2
            by = y0 + self.vh - 92
            o2 = img.copy()
            cv2.rectangle(o2, (bx, by), (bx + w, by + 36), (20, 16, 40), -1)
            blend(img, o2, 0.85)
            cv2.rectangle(img, (bx, by), (bx + w, by + 36), col, 2, cv2.LINE_AA)
            t.add((bx + 14, by + 18), s, 17, col, 'SemiBold', 'lm')
        # map: tracks so far
        if k > 1:
            okl = self.have[:k + 1]
            pts = [self.m(*self.est[j]) for j in range(0, k + 1, 2)
                   if tr.t0 <= j / self.fps - self.offset <= tr.t1]
            if len(pts) > 1:
                cv2.polylines(img, [np.array(pts, np.int32)], False, SKY, 1, cv2.LINE_AA)
            seg = []
            for j in range(0, k + 1, 2):
                if okl[j]:
                    seg.append(self.m(*self.fixes[j]))
                else:
                    if len(seg) > 1:
                        cv2.polylines(img, [np.array(seg, np.int32)], False, CORAL, 2, cv2.LINE_AA)
                    seg = []
            if len(seg) > 1:
                cv2.polylines(img, [np.array(seg, np.int32)], False, CORAL, 2, cv2.LINE_AA)
        cv2.circle(img, self.m(ex, ey), 7, SKY, 2, cv2.LINE_AA)
        if fix is not None:
            p = self.m(*fix)
            cv2.circle(img, p, 5, CORAL, -1, cv2.LINE_AA)
            if inlog:
                yaw = math.radians(tr.yaw[i])
                q = (int(p[0] + 14 * math.cos(yaw)), int(p[1] - 14 * math.sin(yaw)))
                cv2.line(img, p, q, CORAL, 2, cv2.LINE_AA)
            if self.drift[k] > 0.03:
                cv2.line(img, p, self.m(ex, ey), TEXT, 1, cv2.LINE_AA)
        # status
        sx, sy, sw, sh = self.STAT
        t.add((sx + 14, sy + 12), 'Flight log', 15, TEXT, 'SemiBold')
        phase = tr.phase[i] if inlog else ('ground' if lt < tr.t0 else 'landed')
        vcol = VERDICT_RGB.get(verdict, MUTED) if inlog else MUTED
        chip(img, t, sx + sw - 14, sy + 10, f'guard: {verdict or "-"}', vcol, 13, anchor_right=True)
        rows = [('phase', phase),
                ('height', f'{z:.2f} m'),
                ('estimate', f'{ex:.2f}, {ey:.2f} m'),
                ('camera', f'{fix[0]:.2f}, {fix[1]:.2f} m' if fix is not None else 'not in view'),
                ('drift', f'{self.drift[k] * 100:.1f} cm' if fix is not None else '-'),
                ('battery', f'{tr.vbat[i]:.2f} V' if inlog and not np.isnan(tr.vbat[i]) else '-'),
                ('rangers', '  '.join(f'{key[0].upper()} {tr.ranges[key][i]:.2f}' if not np.isnan(tr.ranges[key][i]) else f'{key[0].upper()} -'
                                      for key in ('front', 'left', 'back', 'right')) if inlog else '-')]
        for j, (kk, vv) in enumerate(rows):
            yy = sy + 48 + j * 25
            t.add((sx + 14, yy), kk, 13, MUTED, 'Regular', 'lm')
            col = CORAL if kk in ('camera', 'drift') else SKY if kk == 'estimate' else TEXT
            t.add((sx + 104, yy), vv, 14, col, 'Medium', 'lm')
        if inlog and reason:
            t.add((sx + 14, sy + sh - 18), ('reason: ' + reason)[:58], 13, vcol, 'Regular', 'lm')
        # timeline cursor
        xx = self.tx(tv)
        cv2.line(img, (xx, self.py0 - 4), (xx, self.py1 + 11), TEXT, 1, cv2.LINE_AA)
        if self.have[k]:
            cv2.circle(img, (xx, self.ty(self.drift[k])), 4, CORAL, -1, cv2.LINE_AA)
        return t.render(img)


# ---------------------------------------------------------------------------------------------
# The commands


def run(a) -> int:
    from arsynth import VideoOut

    video, logp = Path(a.video), Path(a.log)
    log = read_log(logp)
    tr = Track(log)
    setup = load_setup(a.setup, arena=log.arena)
    side_p = Path(str(video) + '.truth.json')
    side = json.loads(side_p.read_text()) if side_p.exists() else None
    out = Path(a.out) if a.out else logp.with_name(logp.stem + '-ar')
    out.mkdir(parents=True, exist_ok=True)
    t_start = time.time()

    def say(*m):
        if not a.quiet:
            print(*m, flush=True)

    say(f'AR replay: {logp.name} on {video.name}' + (' (SYNTHETIC video)' if side else ''))
    cam, bg, fps, n, cal = calibrate(video, setup, hfov=a.hfov, focal=a.focal, max_s=a.max_s)
    say(f'  camera: {len(cal["markers_seen"])} of {len(cal["markers_in_setup"])} markers, focal {cal["focal_px"]:.0f} px '
        f'({cal["hfov_deg"]:.1f} deg wide{", fitted" if cal["focal_fitted"] else ""}), pose error {cal["pose_rms_px"]:.2f} px, '
        f'{cal["camera_xyz_m"][2]:.2f} m up')
    if cal['camera_moved_m'] > 0.05:
        say(f'  warning: the camera seems to move by up to {cal["camera_moved_m"] * 100:.0f} cm; this replay assumes a tripod')
    cands = find_candidates(video, bg, cal['pixel_noise'], a.max_s)
    if a.offset is not None:
        offset, cost = a.offset, None
    else:
        offset, cost = find_offset(cands, fps, cam, tr)
    say(f'  sync:   the log starts {offset:.3f} s into the video' + (f' (fit {cost:.1f} px)' if cost is not None else ' (given)'))
    fixes = track_drone(cands, fps, cam, tr, offset)
    meas, est, drift, have = measure(fixes, fps, tr, offset, side, cam)
    say(f'  track:  the drone is found in {meas["tracked_share_in_flight"] * 100:.0f}% of the frames in flight')
    if 'drift' in meas:
        dd = meas['drift']
        line = f'  drift:  median {dd["median_m"] * 100:.1f} cm, max {dd["max_m"] * 100:.1f} cm'
        for j in meas.get('estimate_jumps', []):
            line += f'; at {j["log_s"]:.1f} s ' + jump_text(j)
        say(line)
    if 'landed' in meas:
        L = meas['landed']
        say(f'  landed: the camera puts it at {L["camera_xy"][0]:.2f}, {L["camera_xy"][1]:.2f}; its estimate said '
            f'{L["estimate_xy"][0]:.2f}, {L["estimate_xy"][1]:.2f} ({L["offset_m"] * 100:.0f} cm apart)')
    if 'self_check' in meas:
        c = meas['self_check']
        say(f'  check:  camera vs the sim\'s truth median {c["camera_vs_truth_median_m"] * 100:.1f} cm, p95 '
            f'{c["camera_vs_truth_p95_m"] * 100:.1f} cm (the estimate: median {c["estimate_vs_truth_median_m"] * 100:.1f} cm)')
    fence = (log.meta.get('limits') or {}).get('fence_margin', 0.3)
    summary = {'schema': 'frontier-fly/ar-replay', 'version': 1, 'video': video.name, 'log': logp.name,
               'log_source': log.source, 'synthetic_video': side is not None, 'setup': Path(a.setup).name,
               'camera': {**cal, **cam.to_json()}, 'sync': {'offset_s': round(offset, 4), 'fit_px': None if cost is None else round(cost, 3),
                                                            'given': a.offset is not None},
               'fps': fps, **meas, 'fence_margin': fence,
               'assumptions': ['the camera stands still', 'the video is undistorted', 'the drone is at its logged height',
                               'after the last step the drone drops to the floor']}
    if 'landed' in meas:
        L = meas['landed']['camera_xy']
        summary['compare_cmd'] = f'python3 scripts/crazyflie/compare.py {logp} --landed {L[0]:.2f},{L[1]:.2f}'
    frames = []
    for k in range(len(fixes)):
        lt = k / fps - offset
        frames.append({'video_s': round(k / fps, 4), 'log_s': round(lt, 4),
                       'camera_x': None if np.isnan(fixes[k, 0]) else round(float(fixes[k, 0]), 4),
                       'camera_y': None if np.isnan(fixes[k, 1]) else round(float(fixes[k, 1]), 4),
                       'estimate_x': round(float(est[k, 0]), 4), 'estimate_y': round(float(est[k, 1]), 4),
                       'height': round(float(tr.height(lt)), 4)})
    (out / 'ar.json').write_text(json.dumps({**summary, 'frames': frames}, indent=1))
    with open(out / 'track.csv', 'w', newline='') as fh:
        wr = csv.DictWriter(fh, fieldnames=list(frames[0]))
        wr.writeheader()
        wr.writerows(frames)
    written = [out / 'ar.json', out / 'track.csv']
    title = f'{logp.name}   {log.source} log, {log.mode or "?"}' + ('   SYNTHETIC video' if side else '')
    asked = {}
    for v_s in (a.stills or '').split(','):
        if v_s.strip():
            asked[min(len(fixes) - 1, max(0, int(round(float(v_s) * fps))))] = f'still-{float(v_s):06.2f}s.png'
    if a.no_video and asked:
        # Only the stills asked for: seek to each frame.
        dash = Dashboard(cam, setup, tr, fixes, est, drift, have, fps, offset, {**cal, **meas, 'fence_margin': fence}, title, side is not None)
        cap = cv2.VideoCapture(str(video))
        for k, name in sorted(asked.items()):
            cap.set(cv2.CAP_PROP_POS_FRAMES, k)
            ok, f = cap.read()
            if ok:
                cv2.imwrite(str(out / name), dash.frame(k, f))
                written.append(out / name)
        cap.release()
    if not a.no_video:
        dash = Dashboard(cam, setup, tr, fixes, est, drift, have, fps, offset, {**cal, **meas, 'fence_margin': fence}, title, side is not None)
        keys = dict(asked)
        if 'drift' in meas:
            keys[min(len(fixes) - 1, int((meas['drift']['max_at_video_s'] + 0.6) * fps))] = 'still-max-drift.png'
        flight = np.flatnonzero(have)
        if len(flight):
            keys[int(flight[len(flight) // 3])] = 'still-flight.png'
        if meas.get('estimate_jumps'):
            keys[min(len(fixes) - 1, int((meas['estimate_jumps'][0]['video_s'] + 0.5) * fps))] = 'still-jump.png'
        vo = VideoOut(out / 'ar.mp4', Dashboard.W, Dashboard.H, fps, crf=21)
        try:
            for k, _, f in read_frames(video, a.max_s):
                if k >= len(fixes):
                    break
                img = dash.frame(k, f)
                vo.write(img)
                if k in keys:
                    cv2.imwrite(str(out / keys[k]), img)
                    written.append(out / keys[k])
        finally:
            vo.close()
        written.insert(0, out / 'ar.mp4')
    if 'compare_cmd' in summary:
        say(f'  next:   {summary["compare_cmd"]}')
    say(f'wrote {", ".join(str(p) for p in written)} in {time.time() - t_start:.0f} s')
    return 0


def synth(a) -> int:
    from arsynth import synthesize

    log = read_log(a.log)
    setup = load_setup(a.setup, arena=log.arena)
    w, h = (int(v) for v in a.size.lower().split('x'))
    t0 = time.time()
    side = synthesize(log, setup, a.out, w=w, h=h, fps=a.fps, hfov=a.hfov, lead=a.lead, seed=a.seed, max_s=a.max_s,
                      progress=None if a.quiet else (lambda k, n: print(f'\r  frame {k}/{n}', end='', flush=True)))
    if not a.quiet:
        print(f'\rwrote {a.out} ({side["frames"]} frames, SYNTHETIC) and {a.out}.truth.json in {time.time() - t0:.0f} s')
    return 0


def sheet(a) -> int:
    setup = load_setup(a.setup)
    pages = marker_sheet(setup, a.out or Path(a.setup).with_suffix('.markers.png'), dpi=a.dpi)
    print('print at 100% scale, then check each black square measures '
          f'{setup.marker_size * 100:.1f} cm:\n  ' + '\n  '.join(str(p) for p in pages))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0], formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run', help='replay a flight log on a phone video of it')
    r.add_argument('video')
    r.add_argument('log')
    r.add_argument('--setup', required=True, help='the AR setup file: where the markers lie (arenas/*.ar.json)')
    r.add_argument('--out', help='output folder (default: next to the log, <log>-ar/)')
    g = r.add_mutually_exclusive_group()
    g.add_argument('--hfov', type=float, help="the video's horizontal field of view in degrees, if known")
    g.add_argument('--focal', type=float, help='the focal length in pixels, if known (default: fitted from the markers)')
    r.add_argument('--offset', type=float, help='seconds into the video where the log starts (default: found)')
    r.add_argument('--max-s', type=float, help='only the first seconds of the video')
    r.add_argument('--no-video', action='store_true', help='write the numbers only, no ar.mp4 (and any --stills)')
    r.add_argument('--stills', help='also write the dashboard at these video times, e.g. 12.5,40.3')
    r.add_argument('--quiet', action='store_true')
    r.set_defaults(func=run)
    s = sub.add_parser('synth', help='render a SYNTHETIC phone video of a sim log, to try the replay')
    s.add_argument('log')
    s.add_argument('--setup', required=True)
    s.add_argument('--out', required=True)
    s.add_argument('--size', default='1920x1080')
    s.add_argument('--fps', type=float, default=30.0)
    s.add_argument('--hfov', type=float, default=84.0)
    s.add_argument('--lead', type=float, default=2.3, help='seconds of video before the log starts')
    s.add_argument('--seed', type=int, default=1)
    s.add_argument('--max-s', type=float)
    s.add_argument('--quiet', action='store_true')
    s.set_defaults(func=synth)
    m = sub.add_parser('sheet', help='print the setup\'s markers at their true size')
    m.add_argument('setup')
    m.add_argument('--out')
    m.add_argument('--dpi', type=int, default=300)
    m.set_defaults(func=sheet)
    a = ap.parse_args(argv)
    return a.func(a)


if __name__ == '__main__':
    sys.exit(main())
