#!/usr/bin/env python3
"""Fly a QGroundControl .plan on a running PX4 SITL and check the flight.

    python3 scripts/px4/flight_check.py PLAN.plan [--meta PLAN.meta.json]
        [--url udpin:0.0.0.0:14540] [--speed 1] [--out report.json]

Talks to PX4 the way QGroundControl does, over MAVLink with pymavlink:

1. waits for a 3D fix, a home position and a ready-to-arm vehicle,
2. uploads the plan's geofence (mission type FENCE) and its mission items
   (mission type MISSION) with the MAVLink mission protocol,
3. switches to Mission mode, arms, and records the flight until PX4 lands
   and disarms (or a time limit runs out),
4. checks the recorded track against the plan and writes a JSON report.

The report's checks are pass/fail with the measured value next to each
limit, so a failure says by how much. Exit code 0 means every check passed.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

# MAVLink values used here (common.xml).
MAV_CMD_NAV_WAYPOINT = 16
MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
MAV_CMD_NAV_LAND = 21
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_FENCE_INCLUSION = 5001
MAV_CMD_FENCE_EXCLUSION = 5002
MAV_CMD_FENCE_CIRCLE_INCLUSION = 5003
MAV_CMD_FENCE_CIRCLE_EXCLUSION = 5004
MAV_CMD_COMPONENT_ARM_DISARM = 400
MAV_CMD_DO_SET_MODE = 176
MAV_CMD_SET_MESSAGE_INTERVAL = 511
MAV_MISSION_TYPE_MISSION = 0
MAV_MISSION_TYPE_FENCE = 1
MAV_FRAME_GLOBAL = 0
MAV_MISSION_ACCEPTED = 0
MAV_LANDED_STATE_ON_GROUND = 1
MAV_RESULT_ACCEPTED = 0
NAV_COMMANDS = {MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_LAND, MAV_CMD_NAV_TAKEOFF}

# PX4 custom modes (px4_custom_mode.h): main mode AUTO = 4, sub mode MISSION = 4.
PX4_MAIN_AUTO = 4
PX4_SUB_MISSION = 4

# Check limits. A multicopter rounds corners inside the acceptance radius and
# overshoots a little at speed, so the route tolerance is a few meters.
ROUTE_TOLERANCE_M = 6.0
FENCE_MARGIN_M = 0.0
HOME_TOLERANCE_M = 3.0
ALT_TOLERANCE_M = 2.0

EARTH_R = 6378137.0


def local_frame(lat0: float, lon0: float):
    """East-north meters around (lat0, lon0). Equirectangular; well under a centimeter of error over a few hundred meters."""
    k = math.cos(math.radians(lat0))

    def to_local(lat: float, lon: float) -> tuple[float, float]:
        return (math.radians(lon - lon0) * EARTH_R * k, math.radians(lat - lat0) * EARTH_R)

    return to_local


def seg_dist(p, a, b) -> float:
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    ll = dx * dx + dy * dy
    t = 0.0 if ll == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / ll))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def polyline_dist(p, line) -> float:
    if len(line) == 1:
        return math.hypot(p[0] - line[0][0], p[1] - line[0][1])
    return min(seg_dist(p, line[i], line[i + 1]) for i in range(len(line) - 1))


def inside(p, poly) -> bool:
    x, y = p
    hit = False
    j = len(poly) - 1
    for i in range(len(poly)):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            hit = not hit
        j = i
    return hit


def edge_dist(p, poly) -> float:
    return min(seg_dist(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))


def nan_param(v):
    return float("nan") if v is None else float(v)


def plan_mission_items(plan: dict) -> list[dict]:
    """The .plan's SimpleItems as MISSION_ITEM_INT fields, the way QGroundControl uploads them."""
    out = []
    for item in plan["mission"]["items"]:
        if item.get("type") != "SimpleItem":
            raise ValueError(f"unsupported item type {item.get('type')}")
        p = item["params"]
        out.append(
            dict(
                command=item["command"],
                frame=item["frame"],
                params=[nan_param(v) for v in p[:4]],
                x=int(round((p[4] or 0) * 1e7)),
                y=int(round((p[5] or 0) * 1e7)),
                z=float(p[6] or 0),
                autocontinue=1 if item["autoContinue"] else 0,
            )
        )
    return out


def plan_fence_items(plan: dict) -> list[dict]:
    out = []
    for poly in plan["geoFence"]["polygons"]:
        cmd = MAV_CMD_FENCE_INCLUSION if poly["inclusion"] else MAV_CMD_FENCE_EXCLUSION
        n = len(poly["polygon"])
        for lat, lon in poly["polygon"]:
            out.append(
                dict(command=cmd, frame=MAV_FRAME_GLOBAL, params=[float(n), 0.0, 0.0, 0.0],
                     x=int(round(lat * 1e7)), y=int(round(lon * 1e7)), z=0.0, autocontinue=1)
            )
    for circle in plan["geoFence"]["circles"]:
        cmd = MAV_CMD_FENCE_CIRCLE_INCLUSION if circle["inclusion"] else MAV_CMD_FENCE_CIRCLE_EXCLUSION
        lat, lon = circle["circle"]["center"]
        out.append(
            dict(command=cmd, frame=MAV_FRAME_GLOBAL, params=[float(circle["circle"]["radius"]), 0.0, 0.0, 0.0],
                 x=int(round(lat * 1e7)), y=int(round(lon * 1e7)), z=0.0, autocontinue=1)
        )
    return out


class Link:
    """A pymavlink connection with a background reader, so the flight log never drops messages while we wait."""

    def __init__(self, url: str):
        self.m = mavutil.mavlink_connection(url, source_system=255, source_component=190)
        self.lock = threading.Condition()
        self.inbox: list = []
        self.log: dict[str, list] = {"track": [], "reached": [], "statustext": [], "landed": [], "armed": [], "current": []}
        self.last: dict[str, object] = {}
        self.running = True
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.hb = threading.Thread(target=self._heartbeat, daemon=True)

    def start(self, timeout: float) -> None:
        hb = self.m.wait_heartbeat(timeout=timeout)
        if hb is None:
            raise TimeoutError("no heartbeat from PX4")
        self.reader.start()
        self.hb.start()

    def _heartbeat(self) -> None:
        while self.running:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(0.5)

    def _read(self) -> None:
        while self.running:
            msg = self.m.recv_match(blocking=True, timeout=0.2)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "BAD_DATA":
                continue
            if msg.get_srcSystem() != self.m.target_system:
                continue
            with self.lock:
                self.last[t] = msg
                if t == "GLOBAL_POSITION_INT":
                    self.log["track"].append((msg.time_boot_ms / 1000.0, msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0))
                elif t == "MISSION_ITEM_REACHED":
                    self.log["reached"].append((self.boot_s(), msg.seq))
                elif t == "MISSION_CURRENT":
                    if not self.log["current"] or self.log["current"][-1][1] != msg.seq:
                        self.log["current"].append((self.boot_s(), msg.seq))
                elif t == "STATUSTEXT":
                    self.log["statustext"].append((self.boot_s(), msg.severity, msg.text))
                elif t == "EXTENDED_SYS_STATE":
                    if not self.log["landed"] or self.log["landed"][-1][1] != msg.landed_state:
                        self.log["landed"].append((self.boot_s(), msg.landed_state))
                elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if not self.log["armed"] or self.log["armed"][-1][1] != armed:
                        self.log["armed"].append((self.boot_s(), armed))
                self.inbox.append(msg)
                if len(self.inbox) > 2000:
                    del self.inbox[:1000]
                self.lock.notify_all()

    def boot_s(self) -> float:
        g = self.last.get("GLOBAL_POSITION_INT")
        return g.time_boot_ms / 1000.0 if g is not None else 0.0

    def wait_for(self, types: set[str], pred=lambda m: True, timeout: float = 5.0):
        end = time.monotonic() + timeout
        with self.lock:
            while True:
                for i, msg in enumerate(self.inbox):
                    if msg.get_type() in types and pred(msg):
                        del self.inbox[: i + 1]
                        return msg
                self.inbox.clear()
                left = end - time.monotonic()
                if left <= 0:
                    return None
                self.lock.wait(left)

    def command(self, cmd: int, *params: float, timeout: float = 5.0) -> int | None:
        p = list(params) + [0.0] * (7 - len(params))
        for _ in range(3):
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component, cmd, 0, *p)
            ack = self.wait_for({"COMMAND_ACK"}, lambda m: m.command == cmd, timeout)
            if ack is not None:
                return ack.result
        return None

    def upload(self, items: list[dict], mission_type: int, timeout: float = 10.0) -> tuple[bool, str]:
        m = self.m
        with self.lock:
            self.inbox.clear()
        m.mav.mission_count_send(m.target_system, m.target_component, len(items), mission_type)
        sent: set[int] = set()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.wait_for({"MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"},
                                lambda x: getattr(x, "mission_type", 0) == mission_type, 1.0)
            if msg is None:
                continue
            if msg.get_type() == "MISSION_ACK":
                ok = msg.type == MAV_MISSION_ACCEPTED and len(sent) == len(items)
                return ok, f"ack type {msg.type}, {len(sent)}/{len(items)} items requested"
            seq = msg.seq
            if seq >= len(items):
                return False, f"PX4 asked for item {seq} of {len(items)}"
            it = items[seq]
            m.mav.mission_item_int_send(
                m.target_system, m.target_component, seq, it["frame"], it["command"], 0, it["autocontinue"],
                *it["params"], it["x"], it["y"], it["z"], mission_type,
            )
            sent.add(seq)
        return False, "timed out"

    def close(self) -> None:
        self.running = False
        self.reader.join(timeout=1)
        self.hb.join(timeout=1)


def wait_ready(link: Link, timeout: float) -> dict:
    """3D fix, home position, and the preflight checks passing."""
    end = time.monotonic() + timeout
    link.command(MAV_CMD_SET_MESSAGE_INTERVAL, 242, 1e6)  # HOME_POSITION at 1 Hz
    link.command(MAV_CMD_SET_MESSAGE_INTERVAL, 33, 1e5)  # GLOBAL_POSITION_INT at 10 Hz
    link.command(MAV_CMD_SET_MESSAGE_INTERVAL, 245, 2e5)  # EXTENDED_SYS_STATE at 5 Hz
    home = None
    while time.monotonic() < end:
        with link.lock:
            gps = link.last.get("GPS_RAW_INT")
            hp = link.last.get("HOME_POSITION")
            sys_status = link.last.get("SYS_STATUS")
        if hp is not None:
            home = {"lat": hp.latitude / 1e7, "lon": hp.longitude / 1e7, "altAmsl": hp.altitude / 1000.0}
        prearm_ok = sys_status is not None and bool(
            sys_status.onboard_control_sensors_health & mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        )
        if home and gps is not None and gps.fix_type >= 3 and prearm_ok:
            return home
        time.sleep(0.5)
    raise TimeoutError("PX4 never became ready to arm (fix, home position, preflight checks)")


def evaluate(plan: dict, items: list[dict], link: Link, home: dict, flight: dict) -> dict:
    # The frame is the plan's pad, where the export pinned the sim's takeoff point. PX4's own
    # home is its GNSS estimate at arming, so it can sit a meter or two away.
    pad = plan["mission"]["plannedHomePosition"]
    to_local = local_frame(pad[0], pad[1])
    home_xy = to_local(home["lat"], home["lon"])
    track = link.log["track"]
    t0 = flight["armedAt"]
    flown = [(t, *to_local(lat, lon), alt) for (t, lat, lon, alt) in track if t >= t0]

    nav_seqs = [i for i, it in enumerate(items) if it["command"] in NAV_COMMANDS]
    wp_seqs = [i for i, it in enumerate(items) if it["command"] == MAV_CMD_NAV_WAYPOINT]
    # PX4 repeats the last MISSION_ITEM_REACHED; keep the first report of each item.
    first_reached: dict[int, float] = {}
    for t, seq in link.log["reached"]:
        first_reached.setdefault(seq, t)
    reached = sorted(first_reached, key=lambda s: first_reached[s])
    in_order = reached == sorted(reached)

    # PX4 publishes MISSION_ITEM_REACHED as a single "last reached" value, so two items
    # reached close together can share one report. A waypoint PX4 did not report still
    # counts if the track passed within its acceptance radius at its height; the report
    # lists which ones were judged that way.
    def passed_over(seq: int) -> bool:
        it = items[seq]
        wx, wy = to_local(it["x"] / 1e7, it["y"] / 1e7)
        radius = it["params"][1] if math.isfinite(it["params"][1]) and it["params"][1] > 0 else 2.0
        return any(math.hypot(p[1] - wx, p[2] - wy) <= radius + 0.5 and abs(p[3] - it["z"]) <= ALT_TOLERANCE_M for p in flown)

    by_track = [s for s in wp_seqs if s not in first_reached and passed_over(s)]
    reached_wp = [s for s in wp_seqs if s in first_reached or s in by_track]

    # Planned path: pad, every navigation point, and back to the pad for a return.
    planned = [(0.0, 0.0)]
    for it in items:
        if it["command"] in NAV_COMMANDS and it["command"] != MAV_CMD_NAV_TAKEOFF:
            planned.append(to_local(it["x"] / 1e7, it["y"] / 1e7))
    rtl = bool(items) and items[-1]["command"] == MAV_CMD_NAV_RETURN_TO_LAUNCH
    if rtl:
        planned.append((0.0, 0.0))
    airborne = [p for p in flown if p[3] > 1.0]
    deviations = sorted(polyline_dist((p[1], p[2]), planned) for p in airborne) or [0.0]
    max_dev = deviations[-1]
    p95_dev = deviations[int(0.95 * (len(deviations) - 1))]

    # Height at each reached waypoint against the plan.
    alt_errors = []
    for seq, t in first_reached.items():
        it = items[seq] if seq < len(items) else None
        if not it or it["command"] != MAV_CMD_NAV_WAYPOINT:
            continue
        near = min(flown, key=lambda p: abs(p[0] - t), default=None)
        if near is not None:
            alt_errors.append(abs(near[3] - it["z"]))
    max_alt_err = max(alt_errors, default=0.0)

    # Fence: every airborne sample inside the inclusion polygon and outside every exclusion polygon.
    incl = [[to_local(a, b) for a, b in p["polygon"]] for p in plan["geoFence"]["polygons"] if p["inclusion"]]
    excl = [[to_local(a, b) for a, b in p["polygon"]] for p in plan["geoFence"]["polygons"] if not p["inclusion"]]
    worst_margin = math.inf
    breaches = 0
    for p in airborne:
        xy = (p[1], p[2])
        for poly in incl:
            m = edge_dist(xy, poly) * (1 if inside(xy, poly) else -1)
            worst_margin = min(worst_margin, m)
            breaches += m < FENCE_MARGIN_M
        for poly in excl:
            m = edge_dist(xy, poly) * (-1 if inside(xy, poly) else 1)
            worst_margin = min(worst_margin, m)
            breaches += m < FENCE_MARGIN_M

    last = flown[-1] if flown else None
    # Return to launch lands at PX4's home; a plan that lands flies to its own land point.
    land_xy = home_xy if rtl else planned[-1]
    end_dist = math.hypot(last[1] - land_xy[0], last[2] - land_xy[1]) if last else math.inf
    landed_states = [s for (_, s) in link.log["landed"]]
    landed = bool(landed_states) and landed_states[-1] == MAV_LANDED_STATE_ON_GROUND
    disarmed = bool(link.log["armed"]) and link.log["armed"][-1][1] is False
    takeoff_alt = next((it["z"] for it in items if it["command"] == MAV_CMD_NAV_TAKEOFF), 0.0)
    max_alt = max((p[3] for p in flown), default=0.0)
    failsafes = [s for (_, sev, s) in link.log["statustext"] if sev <= 4 and ("ailsafe" in s or "ence" in s)]

    checks = [
        dict(id="mission_upload", label="Mission upload accepted", ok=flight["missionUpload"][0], value=flight["missionUpload"][1]),
        dict(id="fence_upload", label="Geofence upload accepted", ok=flight["fenceUpload"][0], value=flight["fenceUpload"][1]),
        dict(id="takeoff", label="Climbed to the takeoff height", ok=max_alt >= 0.8 * takeoff_alt,
             value=round(max_alt, 2), limit=round(0.8 * takeoff_alt, 2)),
        dict(id="waypoints", label="Reached every waypoint, in order", ok=len(reached_wp) == len(wp_seqs) and in_order,
             value=f"{len(reached_wp)}/{len(wp_seqs)}" + ("" if in_order else " (out of order)"),
             reportedByPx4=reached, passedOverOnly=by_track),
        dict(id="route", label="Stayed on the planned route", ok=max_dev <= ROUTE_TOLERANCE_M,
             value=round(max_dev, 2), p95=round(p95_dev, 2), limit=ROUTE_TOLERANCE_M),
        dict(id="altitude", label="Height at each waypoint", ok=max_alt_err <= ALT_TOLERANCE_M,
             value=round(max_alt_err, 2), limit=ALT_TOLERANCE_M),
        dict(id="fence", label="Inside the geofence, outside every no-fly zone", ok=breaches == 0,
             value=round(worst_margin, 2) if math.isfinite(worst_margin) else None, limit=FENCE_MARGIN_M, breaches=breaches),
        dict(id="landed", label="Landed at the pad and disarmed" if not rtl else "Returned, landed at home and disarmed", ok=landed and disarmed and end_dist <= HOME_TOLERANCE_M,
             value=round(end_dist, 2), limit=HOME_TOLERANCE_M),
        dict(id="no_failsafe", label="No failsafe or fence warning from PX4", ok=not failsafes, value=failsafes[:5]),
    ]
    # One sample per second of sim time for plots.
    thinned, next_t = [], -1.0
    for p in flown:
        if p[0] >= next_t:
            thinned.append([round(p[0] - t0, 2), round(p[1], 2), round(p[2], 2), round(p[3], 2)])
            next_t = p[0] + 1.0
    return dict(
        ok=all(c["ok"] for c in checks),
        checks=checks,
        flightSec=round(flight["endAt"] - t0, 1),
        armedAtBootSec=round(t0, 3),
        armAttempts=flight.get("armAttempts"),
        navItems=len(nav_seqs),
        planned=[[round(x, 2), round(y, 2)] for x, y in planned],
        fence=dict(inclusion=[[[round(x, 2), round(y, 2)] for x, y in p] for p in incl],
                   exclusion=[[[round(x, 2), round(y, 2)] for x, y in p] for p in excl]),
        track=thinned,
        statustext=[[round(t - t0, 1), sev, s] for (t, sev, s) in link.log["statustext"]][-40:],
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("plan")
    ap.add_argument("--meta")
    ap.add_argument("--url", default="udpin:0.0.0.0:14540")
    ap.add_argument("--speed", type=float, default=1.0, help="PX4_SIM_SPEED_FACTOR, to scale wall-clock timeouts")
    ap.add_argument("--ready-timeout", type=float, default=120.0)
    ap.add_argument("--out")
    args = ap.parse_args()

    plan = json.loads(Path(args.plan).read_text())
    meta = json.loads(Path(args.meta).read_text()) if args.meta else {}
    items = plan_mission_items(plan)
    fence = plan_fence_items(plan)
    est = float(meta.get("summary", {}).get("estimatedSec", 300))
    flight_limit = (3 * est + 120) / max(args.speed, 0.1)

    link = Link(args.url)
    link.start(timeout=args.ready_timeout)
    print(f"PX4 heartbeat from system {link.m.target_system}", flush=True)
    home = wait_ready(link, args.ready_timeout)
    print(f"ready: home {home['lat']:.7f}, {home['lon']:.7f}", flush=True)

    pad = plan["mission"]["plannedHomePosition"]
    home_offset = math.hypot(*local_frame(pad[0], pad[1])(home["lat"], home["lon"]))

    fence_up = link.upload(fence, MAV_MISSION_TYPE_FENCE)
    mission_up = link.upload(items, MAV_MISSION_TYPE_MISSION)
    print(f"fence upload: {fence_up}; mission upload: {mission_up}", flush=True)

    flight = dict(missionUpload=mission_up, fenceUpload=fence_up, armedAt=link.boot_s(), endAt=link.boot_s())
    result = None
    if mission_up[0]:
        mode = link.command(MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_AUTO, PX4_SUB_MISSION)
        # The estimator can flag a transient preflight failure right after startup; PX4 then
        # refuses to arm with TEMPORARILY_REJECTED, so retry for a while, as a pilot would.
        arm_end = time.monotonic() + args.ready_timeout
        attempts = 0
        while True:
            attempts += 1
            arm = link.command(MAV_CMD_COMPONENT_ARM_DISARM, 1)
            if arm == MAV_RESULT_ACCEPTED or time.monotonic() > arm_end:
                break
            time.sleep(2.0)
        flight["armAttempts"] = attempts
        print(f"mission mode: {mode}; arm: {arm} after {attempts} attempt(s)", flush=True)
        flight["armedAt"] = link.boot_s()
        end = time.monotonic() + (flight_limit if arm == MAV_RESULT_ACCEPTED else 0)
        was_armed = False
        while time.monotonic() < end:
            with link.lock:
                armed = link.log["armed"][-1][1] if link.log["armed"] else False
            was_armed = was_armed or armed
            if was_armed and not armed:
                break
            time.sleep(0.2)
        time.sleep(1.0 / max(args.speed, 0.1))
        flight["endAt"] = link.boot_s()
        result = evaluate(plan, items, link, home, flight)
    link.close()

    if result is None:
        result = dict(ok=False, checks=[dict(id="mission_upload", label="Mission upload accepted", ok=False, value=mission_up[1])])
    report = dict(
        plan=Path(args.plan).name,
        id=meta.get("id", Path(args.plan).stem),
        label=meta.get("label"),
        estimatedSec=est,
        waypoints=meta.get("summary", {}).get("waypoints"),
        homeOffsetM=round(home_offset, 3),
        home=home,
        **result,
    )
    for c in report["checks"]:
        print(f"  [{'pass' if c['ok'] else 'FAIL'}] {c['label']}: {c.get('value')}" + (f" (limit {c['limit']})" if "limit" in c else ""))
    print(f"{report['id']}: {'PASS' if report['ok'] else 'FAIL'} in {report.get('flightSec', 0)} s sim (plan estimate {est:.0f} s)")
    if not report["ok"]:
        for t, sev, text in report.get("statustext", []):
            print(f"    PX4 {t:7.1f} s  {text.strip()}")
    if args.out:
        Path(args.out).write_text(json.dumps(report, indent=1) + "\n")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
