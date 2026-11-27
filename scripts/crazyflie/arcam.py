"""Camera geometry for the AR flight replay: a pinhole camera in the arena frame, and ArUco markers.

Arena frame: x along the arena's width, y along its depth, z up, meters, origin at a floor corner
(the frame arena JSON files use). Camera frame: OpenCV's, x right, y down, z forward. A pose is the
rotation R and translation t that take an arena point P to the camera: Pc = R @ P + t.

The AR setup file (arenas/<name>.ar.json) says where the printed markers lie on the floor:

    {"arena": "example-room.json", "dictionary": "DICT_4X4_50", "marker_size": 0.20,
     "markers": [{"id": 0, "x": 0.45, "y": 0.45, "yaw_deg": 0}, ...],
     "box_heights": [0.80, 0.55]}

x, y is the marker's center; marker_size is the black square's side, not the paper's. A marker
lies flat with its top edge (the side ArUco calls top) facing +y when yaw_deg is 0. box_heights,
one per arena box, are only for drawing: the arena file has no heights, because the drone's
rangers only see the boxes' sides.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError as e:  # pragma: no cover - the tests skip without it
    raise SystemExit('the AR replay needs OpenCV: pip install -r scripts/crazyflie/requirements-ar.txt') from e


# ---------------------------------------------------------------------------------------------
# The setup file


@dataclass
class Setup:
    path: Path | None
    arena: dict
    dictionary: str
    marker_size: float
    markers: list[dict]
    box_heights: list[float] = field(default_factory=list)

    def corners(self) -> dict[int, np.ndarray]:
        """Each marker's four corners in the arena frame (z = 0), in ArUco's order:
        top-left, top-right, bottom-right, bottom-left, as seen looking down at the printed marker."""
        h = self.marker_size / 2
        out = {}
        for m in self.markers:
            a = math.radians(m.get('yaw_deg', 0.0))
            c, s = math.cos(a), math.sin(a)
            pts = []
            for dx, dy in ((-h, h), (h, h), (h, -h), (-h, -h)):
                pts.append((m['x'] + c * dx - s * dy, m['y'] + s * dx + c * dy, 0.0))
            out[int(m['id'])] = np.array(pts, dtype=np.float64)
        return out

    def box_height(self, i: int) -> float | None:
        return self.box_heights[i] if i < len(self.box_heights) else None


def load_setup(path: str | Path, arena: dict | None = None) -> Setup:
    """Read an AR setup file. The arena comes from the file it names (next to it), or from `arena`,
    which wins: a flight log carries the arena it flew."""
    path = Path(path)
    raw = json.loads(path.read_text())
    if arena is None:
        if 'arena' not in raw:
            raise ValueError(f'{path}: no "arena" file named and no arena given')
        arena = json.loads((path.parent / raw['arena']).read_text())
    ids = [int(m['id']) for m in raw.get('markers', [])]
    if len(set(ids)) != len(ids):
        raise ValueError(f'{path}: marker ids repeat')
    if len(ids) < 2:
        raise ValueError(f'{path}: lay out at least two markers (four is better)')
    return Setup(path, arena, raw.get('dictionary', 'DICT_4X4_50'), float(raw.get('marker_size', 0.2)),
                 raw['markers'], [float(h) for h in raw.get('box_heights', [])])


def aruco_dictionary(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f'unknown ArUco dictionary {name!r}')
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


# ---------------------------------------------------------------------------------------------
# The camera


def intrinsics(w: int, h: int, hfov_deg: float | None = None, focal_px: float | None = None) -> np.ndarray:
    if focal_px is None:
        focal_px = (w / 2) / math.tan(math.radians(hfov_deg) / 2)
    return np.array([[focal_px, 0, (w - 1) / 2], [0, focal_px, (h - 1) / 2], [0, 0, 1]], dtype=np.float64)


def look_at(eye, target, up=(0.0, 0.0, 1.0)) -> tuple[np.ndarray, np.ndarray]:
    """The pose of a camera at `eye` looking at `target`, with no roll."""
    eye, target, up = (np.asarray(v, dtype=np.float64) for v in (eye, target, up))
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    R = np.stack([right, down, fwd])
    return R, -R @ eye


@dataclass
class Camera:
    K: np.ndarray
    R: np.ndarray
    t: np.ndarray
    w: int
    h: int

    @property
    def center(self) -> np.ndarray:
        """The camera's position in the arena frame."""
        return -self.R.T @ self.t

    @property
    def focal(self) -> float:
        return float(self.K[0, 0])

    @property
    def hfov_deg(self) -> float:
        return math.degrees(2 * math.atan((self.w / 2) / self.focal))

    def project(self, pts) -> tuple[np.ndarray, np.ndarray]:
        """Arena points (N, 3) to pixels (N, 2) and depth along the optical axis (N,). Points behind
        the camera come back with depth <= 0 and pixels that mean nothing."""
        P = np.atleast_2d(np.asarray(pts, dtype=np.float64))
        pc = P @ self.R.T + self.t
        z = pc[:, 2]
        zs = np.where(np.abs(z) < 1e-9, 1e-9, z)
        uv = (pc[:, :2] / zs[:, None]) * [self.K[0, 0], self.K[1, 1]] + [self.K[0, 2], self.K[1, 2]]
        return uv, z

    def ray(self, uv) -> np.ndarray:
        """Unit ray directions in the arena frame for pixels (N, 2)."""
        uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
        d = np.column_stack([(uv[:, 0] - self.K[0, 2]) / self.K[0, 0], (uv[:, 1] - self.K[1, 2]) / self.K[1, 1],
                             np.ones(len(uv))])
        d = d @ self.R  # R.T @ d for each row
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def to_plane(self, uv, z: float | np.ndarray) -> np.ndarray:
        """Pixels (N, 2) back onto the horizontal plane at height z: arena x, y (N, 2), NaN where the
        ray never reaches it in front of the camera."""
        d = self.ray(uv)
        c = self.center
        z = np.broadcast_to(np.asarray(z, dtype=np.float64), (len(d),))
        with np.errstate(divide='ignore', invalid='ignore'):
            s = (z - c[2]) / d[:, 2]
        xy = c[:2] + s[:, None] * d[:, :2]
        xy[~(s > 0)] = np.nan
        return xy

    def to_json(self) -> dict:
        return {'w': self.w, 'h': self.h, 'K': self.K.round(6).tolist(), 'R': self.R.round(9).tolist(),
                't': self.t.round(9).tolist(), 'center': self.center.round(4).tolist(),
                'hfov_deg': round(self.hfov_deg, 3)}

    @classmethod
    def from_json(cls, d: dict) -> 'Camera':
        return cls(np.array(d['K'], dtype=np.float64), np.array(d['R'], dtype=np.float64),
                   np.array(d['t'], dtype=np.float64), int(d['w']), int(d['h']))


# ---------------------------------------------------------------------------------------------
# Markers: detection and the camera's pose


def detector(setup: Setup):
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(aruco_dictionary(setup.dictionary), params)


def detect(det, gray: np.ndarray, known: set[int]) -> dict[int, np.ndarray]:
    """Marker id to its four corners in pixels, only for ids the setup lays out."""
    corners, ids, _ = det.detectMarkers(gray)
    out = {}
    if ids is None:
        return out
    for c, i in zip(corners, ids.ravel()):
        if int(i) in known:
            out[int(i)] = c.reshape(4, 2).astype(np.float64)
    return out


def _points(dets: dict[int, np.ndarray], world: dict[int, np.ndarray]):
    ids = sorted(dets)
    obj = np.concatenate([world[i] for i in ids])
    img = np.concatenate([dets[i] for i in ids])
    return obj, img


def solve_pose(dets: dict[int, np.ndarray], world: dict[int, np.ndarray], K: np.ndarray):
    """The camera pose from the markers seen in one frame: (R, t, rms_px), or None with fewer than
    two markers (one marker's pose flips too easily to trust)."""
    if len(dets) < 2:
        return None
    obj, img = _points(dets, world)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, None, flags=cv2.SOLVEPNP_SQPNP)
    if not ok:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, None, rvec, tvec)
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.ravel(), rms


def fit_focal(frames: list[dict[int, np.ndarray]], world: dict[int, np.ndarray], w: int, h: int,
              lo: float = 0.3, hi: float = 3.0) -> tuple[float, float]:
    """The focal length (pixels) that best explains the marker corners, for a camera of unknown
    zoom: a coarse scan then a golden-section search on the pose error. Square pixels, the
    principal point at the image center, no lens distortion (a phone's video is undistorted).
    Returns (focal, rms_px). Needs markers that are not all seen square on."""
    usable = [d for d in frames if len(d) >= 2]
    if not usable:
        raise ValueError('no frame shows two markers: the focal length cannot be fitted')

    def err(f: float) -> float:
        K = intrinsics(w, h, focal_px=f)
        tot, n = 0.0, 0
        for d in usable:
            p = solve_pose(d, world, K)
            if p is None:
                continue
            tot += p[2] ** 2
            n += 1
        return math.sqrt(tot / n) if n else math.inf

    grid = np.geomspace(lo * w, hi * w, 40)
    errs = [err(f) for f in grid]
    k = int(np.argmin(errs))
    a, b = grid[max(0, k - 1)], grid[min(len(grid) - 1, k + 1)]
    g = (math.sqrt(5) - 1) / 2
    c, d = b - g * (b - a), a + g * (b - a)
    fc, fd = err(c), err(d)
    for _ in range(40):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - g * (b - a)
            fc = err(c)
        else:
            a, c, fc = c, d, fd
            d = a + g * (b - a)
            fd = err(d)
    f = (a + b) / 2
    return f, err(f)


def marker_sheet(setup: Setup, out: str | Path, dpi: int = 300, page_mm=(210, 297)) -> list[Path]:
    """Printable pages (PNG at `dpi`, A4 by default) with each marker at its true size, its id
    and the side length to check with a ruler. Print at 100%, not 'fit to page'."""
    out = Path(out)
    px_per_m = dpi / 0.0254
    pw, ph = (round(v / 1000 * px_per_m) for v in page_mm)
    side = round(setup.marker_size * px_per_m)
    margin = round(0.012 * px_per_m)
    cell = side + 2 * margin
    label_h = round(0.008 * px_per_m)
    cols = max(1, (pw - margin) // cell)
    rows = max(1, (ph - margin) // (cell + label_h))
    per = cols * rows
    d = aruco_dictionary(setup.dictionary)
    pages = []
    ids = [int(m['id']) for m in setup.markers]
    for p in range(0, len(ids), per):
        page = np.full((ph, pw), 255, np.uint8)
        for k, i in enumerate(ids[p:p + per]):
            r, c = divmod(k, cols)
            x0, y0 = margin + c * cell, margin + r * (cell + label_h)
            img = cv2.aruco.generateImageMarker(d, i, side)
            page[y0 + margin:y0 + margin + side, x0 + margin:x0 + margin + side] = img
            txt = f'id {i}   {setup.dictionary}   black square {setup.marker_size * 100:.1f} cm'
            cv2.putText(page, txt, (x0 + margin, y0 + cell + label_h // 2), cv2.FONT_HERSHEY_SIMPLEX,
                        dpi / 300 * 0.9, 0, max(1, dpi // 150), cv2.LINE_AA)
        name = out if len(ids) <= per else out.with_name(f'{out.stem}-{p // per + 1}{out.suffix}')
        name.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(name), page)
        pages.append(name)
    return pages
