#!/usr/bin/env python3
"""Fly one physics-validation maneuver on a running PX4 SITL, in offboard mode.

    python3 scripts/physics/fly_px4.py MANEUVERS.json ID --sim sih|bridge --out log.json
        [--url udpin:0.0.0.0:14540] [--speed 1] [--marker log.json.t0]

1. Sets the parameters that put PX4 on frontier-fly's mission airframe
   (scripts/physics/maneuvers.ts: SIH's mass, inertia, thrust and drag for
   PX4's own simulator; the rotor geometry and thrust curve for the bridge).
2. Waits for a 3D fix and passing preflight checks, streams setpoints, switches
   to offboard, arms, and climbs to the maneuver's start point.
3. Waits until the hover settles, writes the start time in PX4's clock to
   --marker (the bridge starts its gusts from it), and flies the segments,
   timed by PX4's clock so the speed factor changes nothing.
4. Writes PX4's estimate (LOCAL_POSITION_NED, ATTITUDE), its ground truth
   where it has one (HIL_STATE_QUATERNION, SIH only) and when each segment
   started.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "px4"))
from flight_check import Link, MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_DO_SET_MODE, MAV_CMD_SET_MESSAGE_INTERVAL, MAV_RESULT_ACCEPTED, wait_ready  # noqa: E402
from pymavlink import mavutil  # noqa: E402

PX4_MAIN_OFFBOARD = 6
# Position and velocity used (NaN marks an unused axis), acceleration and yaw rate ignored.
TYPE_MASK = 0b0000_1001_1100_0000
NAN = float("nan")


class LogLink(Link):
    """Link that also keeps every state message for the log."""

    def __init__(self, url: str):
        super().__init__(url)
        self.state = {"local": [], "att": [], "truth": []}
        self.truth0 = None

    def _read(self) -> None:  # noqa: D401 - same loop as Link, plus the state log
        while self.running:
            msg = self.m.recv_match(blocking=True, timeout=0.2)
            if msg is None or msg.get_type() == "BAD_DATA" or msg.get_srcSystem() != self.m.target_system:
                continue
            t = msg.get_type()
            with self.lock:
                self.last[t] = msg
                if t == "LOCAL_POSITION_NED":
                    self.state["local"].append([msg.time_boot_ms, round(msg.x, 3), round(msg.y, 3), round(msg.z, 3), round(msg.vx, 3), round(msg.vy, 3), round(msg.vz, 3)])
                elif t == "ATTITUDE":
                    self.state["att"].append([msg.time_boot_ms, round(math.degrees(msg.roll), 2), round(math.degrees(msg.pitch), 2), round(math.degrees(msg.yaw), 2)])
                elif t == "HIL_STATE_QUATERNION":
                    # PX4 1.15 leaves time_usec at zero in this stream: stamp it with the latest estimate's time (one 50 Hz tick).
                    row = [self.boot_ms(), msg.lat, msg.lon, msg.alt, msg.vx, msg.vy, msg.vz, *[round(q, 5) for q in msg.attitude_quaternion]]
                    if self.truth0 is None:
                        self.truth0 = row
                    self.state["truth"].append(row)
                elif t == "STATUSTEXT":
                    self.log["statustext"].append((self.boot_ms() / 1000, msg.severity, msg.text))
                elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    if not self.log["armed"] or self.log["armed"][-1][1] != armed:
                        self.log["armed"].append((self.boot_ms() / 1000, armed))
                self.inbox.append(msg)
                if len(self.inbox) > 2000:
                    del self.inbox[:1000]
                self.lock.notify_all()

    def boot_ms(self) -> int:
        lp = self.last.get("LOCAL_POSITION_NED")
        return lp.time_boot_ms if lp is not None else 0

    def boot_s(self) -> float:
        return self.boot_ms() / 1000.0

    def set_param(self, name: str, value: float, timeout: float = 3.0) -> bool:
        is_int = isinstance(value, int) and not name.startswith(("SIH_", "CA_", "MPC_", "THR_", "COM_OF"))
        ptype = mavutil.mavlink.MAV_PARAM_TYPE_INT32 if is_int else mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        for _ in range(3):
            if is_int:
                import struct
                raw = struct.unpack("f", struct.pack("i", int(value)))[0]
            else:
                raw = float(value)
            self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), raw, ptype)
            ack = self.wait_for({"PARAM_VALUE"}, lambda m: m.param_id.rstrip("\x00") == name, timeout)
            if ack is not None:
                return True
        return False


class Setpoint:
    """The setpoint a background thread streams to PX4."""

    def __init__(self, link: LogLink, rate_hz: float):
        self.link = link
        self.period = 1.0 / rate_hz
        self.pos = [NAN, NAN, NAN]
        self.vel = [NAN, NAN, NAN]
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def set(self, pos, vel) -> None:
        self.pos = [NAN if v is None else float(v) for v in pos]
        self.vel = [NAN if v is None else float(v) for v in vel]

    def _run(self) -> None:
        m = self.link.m
        while self.running:
            m.mav.set_position_target_local_ned_send(
                0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, TYPE_MASK,
                *self.pos, *self.vel, 0, 0, 0, 0.0, 0,
            )
            time.sleep(self.period)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("maneuvers")
    ap.add_argument("id")
    ap.add_argument("--sim", choices=["sih", "bridge"], required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--url", default="udpin:0.0.0.0:14540")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--marker")
    ap.add_argument("--ready-timeout", type=float, default=180.0)
    args = ap.parse_args()

    spec = json.loads(Path(args.maneuvers).read_text())
    man = next(m for m in spec["maneuvers"] if m["id"] == args.id)
    params = spec["params"][args.sim]
    speed = max(args.speed, 0.1)

    link = LogLink(args.url)
    link.start(timeout=args.ready_timeout)
    for mid, hz in ((32, 50), (30, 50), (115, 50), (253, 0)):
        if hz:
            link.command(MAV_CMD_SET_MESSAGE_INTERVAL, mid, 1e6 / hz)
    bad = [k for k, v in params.items() if not link.set_param(k, v)]
    print(f"{args.id} on {args.sim}: set {len(params) - len(bad)} parameters" + (f", failed {bad}" if bad else ""), flush=True)
    home = wait_ready(link, args.ready_timeout)

    sp = Setpoint(link, 20 * speed)
    start = man["start"]
    sp.set(start, [None, None, None])
    time.sleep(1.0 / speed)
    mode = link.command(MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_OFFBOARD, 0)
    arm_end = time.monotonic() + args.ready_timeout / speed
    attempts = 0
    while True:
        attempts += 1
        arm = link.command(MAV_CMD_COMPONENT_ARM_DISARM, 1)
        if arm == MAV_RESULT_ACCEPTED or time.monotonic() > arm_end:
            break
        time.sleep(2.0 / speed)
        link.command(MAV_CMD_DO_SET_MODE, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, PX4_MAIN_OFFBOARD, 0)
    print(f"offboard: {mode}; arm: {arm} after {attempts} attempt(s)", flush=True)
    ok = arm == MAV_RESULT_ACCEPTED

    # Climb to the start point and wait for a settled hover (within 0.5 m and 0.3 m/s for 3 s, at most 60 s).
    settled_since = None
    t_end = link.boot_s() + 60
    hover_ok = False
    while ok and link.boot_s() < t_end:
        with link.lock:
            lp = link.last.get("LOCAL_POSITION_NED")
        if lp is not None:
            err = math.dist((lp.x, lp.y, lp.z), start)
            spd = math.hypot(lp.vx, lp.vy, lp.vz)
            if err < 0.5 and spd < 0.3:
                settled_since = settled_since if settled_since is not None else link.boot_s()
                if link.boot_s() - settled_since >= 3:
                    hover_ok = True
                    break
            else:
                settled_since = None
        time.sleep(0.01 / speed)
    # In wind the hover keeps moving; go on after the minute regardless, and say so.
    print(f"hover settled: {hover_ok}", flush=True)

    t0_ms = link.boot_ms()
    if args.marker:
        Path(args.marker).write_text(str(t0_ms * 1000))
    starts = []
    t_seg = 0.0
    for seg in man["segments"]:
        # Wait for PX4's clock to reach the segment start, then switch the setpoint.
        while ok and link.boot_ms() - t0_ms < t_seg * 1000:
            time.sleep(0.002 / speed)
        pos = seg.get("pos") or [None, None, None]
        vel = seg.get("vel") or [None, None, None]
        sp.set(pos, vel)
        starts.append(link.boot_ms())
        t_seg += seg["dur"]
    while ok and link.boot_ms() - t0_ms < t_seg * 1000:
        time.sleep(0.01 / speed)
    end_ms = link.boot_ms()
    sp.running = False
    time.sleep(0.2)
    link.close()

    with link.lock:
        state = link.state
        text = [(t, s, x.strip()) for t, s, x in link.log["statustext"]]
    failsafe = [x for _, s, x in text if s <= 4 and ("ailsafe" in x or "rejected" in x.lower())]
    out = dict(
        id=args.id,
        sim=args.sim,
        ok=ok and hover_ok or (ok and man.get("wind") is not None),
        armed=ok,
        hoverSettled=hover_ok,
        home=home,
        t0Ms=t0_ms,
        endMs=end_ms,
        segmentStartsMs=starts,
        params=params,
        failsafe=failsafe,
        statustext=text[-40:],
        local=[r for r in state["local"] if t0_ms - 5000 <= r[0] <= end_ms],
        att=[r for r in state["att"] if t0_ms - 5000 <= r[0] <= end_ms],
        truth=[r for r in state["truth"] if t0_ms - 5000 <= r[0] <= end_ms],
        truthColumns=["tMs", "lat1e7", "lon1e7", "altMm", "vnCm", "veCm", "vdCm", "qw", "qx", "qy", "qz"],
        truthOrigin=link.truth0,
    )
    Path(args.out).write_text(json.dumps(out) + "\n")
    print(f"{args.id} on {args.sim}: {'ok' if out['ok'] else 'FAILED'}, {len(out['local'])} estimate rows, {len(out['truth'])} truth rows", flush=True)
    return 0 if out["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
