import { LIDAR_MAX } from '../../shared/constants';
import type { Phase, SafetyAction, SafetyActionKind, V3 } from './contracts';
import { authorityLimits, headwindCostScale, THRUST_TO_WEIGHT } from './faults';

/**
 * Fault failsafe: a health monitor that turns symptoms into actions.
 *
 * It complements the safety monitor (safety.ts), which enforces the mission
 * envelope (geofence, altitude, battery reserve, collisions) on a healthy
 * drone. This one asks whether the drone is still healthy, from the same
 * signals a flight controller has: operator heartbeats, camera frames, lidar
 * beam validity and agreement with the drone's own map, the GNSS fix flag,
 * the indoor estimator's health, ESC telemetry, the battery gauge against
 * the energy model, and a wind estimate. It never reads the fault injector,
 * so a fault with no symptom goes unnoticed here, as it would in flight.
 *
 * Policy, one row per hazard (docs/SAFETY_CASE.md has the full table):
 *
 * | Symptom | Detect | Response |
 * |---|---|---|
 * | No operator heartbeat | `commsGraceSec` | Keep flying the mission, treat entry requests as denied. Go home after `commsRthSec` outdoors, `commsInsideSec` inside |
 * | Lidar beams flagged invalid | `lidarDetectSec` | Treat them as unknown (never open), slow down; inside, leave the building if the blind arc is at least `lidarExitDeg` |
 * | Lidar beams report open where the map has a wall | `silentScans` scans (1 s) on `silentMinBeams` adjacent beams | Same as flagged, latched |
 * | No camera frames | `cameraGraceSec` | Keep flying (seen-empty maps only grow from frames). Go home after `cameraRthSec` |
 * | GNSS fix lost, outdoors | at once | Climb clear of trees and buildings and wait `gnssHoldSec` for the fix, then go home on dead reckoning; land after `gnssLandSec` |
 * | Indoor pose estimate degraded | `estimatorDetectSec` | Leave the building slowly; land in place when lost |
 * | A motor below `motorRatioMin` of command | `motorDetectSec` | Latch, cap speeds to the remaining authority, go home; land if thrust-to-weight is under `motorLandTw` |
 * | Gauge falling faster than the energy model | `batteryStepPct` step or `batteryRateRatio` over `batteryWindowSec` | Scale every energy estimate by the measured ratio; go home when the scaled reserve check fails |
 * | Mean wind (over `windAvgSec`) above `windLimitMps` | `windPersistSec` | Go home: the planners' clearances near trees and buildings are sized for lighter air. A headwind home scales the energy estimate |
 *
 * "Go home" is exit_building when inside (the runner exits through the entry
 * opening first, as for a battery return). Land beats everything.
 */

export type HealthEventKind =
  | 'comms_loss'
  | 'sensor_fault'
  | 'motor_fault'
  | 'battery_fault'
  | 'gnss_loss'
  | 'estimator_fault'
  | 'wind';

export interface HealthEvent {
  tSec: number;
  kind: HealthEventKind;
  action: SafetyActionKind;
  message: string;
}

export type HealthFlag =
  | 'link_lost'
  | 'lidar_flagged'
  | 'lidar_silent'
  | 'camera_lost'
  | 'gnss_lost'
  | 'estimator_degraded'
  | 'estimator_lost'
  | 'motor_degraded'
  | 'battery_anomaly'
  | 'wind_limit'
  | 'wind_severe';

export interface HealthInput {
  tSec: number;
  dt: number;
  phase: Phase;
  /** Building id while inside a shell. */
  inside: string | null;
  /** An operator heartbeat arrived this tick. */
  linkUp: boolean;
  /** A detector frame arrived this tick. */
  cameraFrame: boolean;
  /**
   * Horizontal lidar ring as received, metres, beam i at yaw + i 2 pi / n.
   * NaN marks a beam the driver flagged invalid; >= LIDAR_MAX - 1e-3 is no
   * return. Null when no scan arrived this tick.
   */
  lidar?: ArrayLike<number> | null;
  /** Ranges the drone's own occupancy map predicts for the same beams; Infinity where it predicts nothing. */
  lidarExpected?: ArrayLike<number> | null;
  /** GNSS fix flag. Only read outdoors. */
  gnssFix?: boolean;
  /** Indoor estimator health (src/loc PoseHealth or a subset). Only read inside. */
  poseHealth?: { ok: boolean; sigmaM: number; state?: 'tracking' | 'degraded' | 'lost' } | null;
  /** ESC telemetry: achieved over commanded thrust per motor. Null when the airframe does not report it. */
  motorRatio?: readonly number[] | null;
  /** Battery gauge now, percent of usable energy. */
  batteryPct: number;
  /** Electrical power the healthy energy model predicts for this tick's motion, W. */
  modelPowerW: number;
  /** Usable pack energy, Wh, so model power converts to gauge percent. */
  usableWh: number;
  /** The healthy model's cost to fly home in still air, percent. */
  costHomePct: number;
  /** Mission battery budget (percent the mission may use), for the reserve. */
  budgetPct?: number;
  /** World-frame wind estimate, m/s. Null when none (indoors). */
  windEstimate?: V3 | null;
  /** Unit horizontal vector from the drone toward home. */
  toHome?: [number, number];
  /** Cruise airspeed home, m/s. */
  cruiseMps?: number;
  /** Horizontal distance to home, m. */
  homeDistM?: number;
}

export interface HealthVerdict {
  action: SafetyAction;
  /** Multiply the vehicle's speed caps by this. 1 when healthy. */
  speedScale: number;
  /** Lidar beams to treat as unknown (an obstacle at the minimum safe distance, never open). */
  unknownBeams: readonly number[];
  /** True while operator entry requests must be answered as denied. */
  denyAsks: boolean;
  /** Multiply the energy model's costs by this when planning. 1 when healthy. */
  costScale: number;
  flags: ReadonlySet<HealthFlag>;
}

export interface FailsafeConfig {
  commsGraceSec: number;
  commsRthSec: number;
  commsInsideSec: number;
  lidarDetectSec: number;
  lidarExitDeg: number;
  lidarSpeedScale: number;
  /** Expected range shorter than the measured miss by at least this much counts as a disagreement, m. */
  silentMarginM: number;
  silentScans: number;
  /**
   * Ignore map walls nearer than this. The map is 2D per floor, so a voxel
   * right beside the drone can hold a return from another height (a table
   * top seen while lower), which a level beam then passes over.
   */
  silentMinRangeM: number;
  silentMinBeams: number;
  cameraGraceSec: number;
  cameraRthSec: number;
  gnssHoldSec: number;
  gnssLandSec: number;
  estimatorDetectSec: number;
  /** Position 1-sigma above which the estimate counts as degraded, m. */
  estimatorExitSigmaM: number;
  /** Position 1-sigma above which the estimate counts as lost, m. */
  estimatorLandSigmaM: number;
  estimatorSpeedScale: number;
  motorRatioMin: number;
  motorDetectSec: number;
  /** Collective thrust-to-weight below which the drone lands at once. */
  motorLandTw: number;
  thrustToWeight: number;
  /** Gauge drop beyond the model's prediction that counts as a step fault, percent. */
  batteryStepPct: number;
  /** Observed over predicted drain that counts as a rate fault. */
  batteryRateRatio: number;
  batteryWindowSec: number;
  reservePct: number;
  /** Sustained wind above which the mission ends, m/s. */
  windLimitMps: number;
  windPersistSec: number;
  /** Averaging time of the mean wind the limit applies to, s. Gusts pass through it. */
  windAvgSec: number;
  /** Wind (on the estimator's short average) that ends the mission at once, m/s. */
  windSevereMps: number;
  windSevereSec: number;
  /** Severe wind stays latched until the short estimate is this far under `windSevereMps` for `windSevereClearSec`. */
  windSevereHysteresisMps: number;
  windSevereClearSec: number;
  /** Returning in wind and closing less than `windProgressM` on home in this long: land where it is. */
  windNoProgressSec: number;
  windProgressM: number;
}

export const DEFAULT_FAILSAFE: FailsafeConfig = {
  commsGraceSec: 2,
  commsRthSec: 20,
  commsInsideSec: 60,
  lidarDetectSec: 0.2,
  lidarExitDeg: 45,
  lidarSpeedScale: 0.6,
  silentMarginM: 1.5,
  silentScans: 20,
  silentMinRangeM: 1,
  silentMinBeams: 3,
  cameraGraceSec: 1,
  cameraRthSec: 10,
  gnssHoldSec: 5,
  gnssLandSec: 90,
  estimatorDetectSec: 1,
  estimatorExitSigmaM: 0.35,
  estimatorLandSigmaM: 1,
  estimatorSpeedScale: 0.5,
  motorRatioMin: 0.9,
  motorDetectSec: 0.3,
  motorLandTw: 1.3,
  thrustToWeight: THRUST_TO_WEIGHT,
  batteryStepPct: 3,
  batteryRateRatio: 1.15,
  batteryWindowSec: 20,
  reservePct: 20,
  windLimitMps: 3,
  windPersistSec: 5,
  windAvgSec: 20,
  windSevereMps: 6,
  windSevereSec: 2,
  windSevereHysteresisMps: 2,
  windSevereClearSec: 5,
  windNoProgressSec: 30,
  windProgressM: 3,
};

const RANK: Record<SafetyActionKind, number> = {
  none: 0,
  hold: 1,
  backtrack: 2,
  exit_building: 3,
  return_home: 4,
  land: 5,
};

/** Longest run of marked beams on a ring, wrapping around, in beams. */
export function longestArc(marked: ArrayLike<boolean>): number {
  const n = marked.length;
  let all = true;
  for (let i = 0; i < n; i++) if (!marked[i]) all = false;
  if (all) return n;
  let best = 0;
  let run = 0;
  for (let k = 0; k < 2 * n; k++) {
    if (marked[k % n]) {
      run += 1;
      best = Math.max(best, Math.min(run, n));
    } else run = 0;
  }
  return best;
}

function fmt(n: number, digits = 1): string {
  return n.toFixed(digits);
}

export interface Failsafe {
  check(input: HealthInput): HealthVerdict;
  events(): HealthEvent[];
}

export function createFailsafe(config?: Partial<FailsafeConfig>): Failsafe {
  const cfg: FailsafeConfig = { ...DEFAULT_FAILSAFE, ...config };
  const events: HealthEvent[] = [];
  const reported = new Set<string>();

  let linkLostSince: number | null = null;
  let cameraLostSince: number | null = null;
  let gnssLostSince: number | null = null;
  let estimatorBadSince: number | null = null;
  let motorLowSince: number | null = null;
  let motorLatched: number | null = null;
  // Mean of the worst ratio over the current low streak; latching the per-tick minimum would ratchet down on ESC noise.
  let motorSum = 0;
  let motorN = 0;
  let windHighSince: number | null = null;
  let windMean: [number, number] | null = null;
  let windSevereSince: number | null = null;
  // Severe wind latches until the short estimate stays under `windSevereMps - windSevereHysteresisMps` for
  // `windSevereClearSec`: gusts dip the estimate under the limit, and dropping back to the low mission path
  // for a moment in 8 m/s is how the drone gets blown into trees.
  let windSevereLatched = false;
  let windCalmSince: number | null = null;
  let windProgress: { t: number; d: number } | null = null;
  let flaggedSince: number | null = null;
  let silentCounts: Int32Array | null = null;
  let lidarKey: string | null = null;
  const silentLatched = new Set<number>();
  // Battery: model-predicted drain integrated beside the gauge.
  let predictedUsed = 0;
  let anchor: { t: number; pct: number; predicted: number } | null = null;
  const samples: { t: number; pct: number; predicted: number }[] = [];
  let batteryScale = 1;

  function emit(key: string, tSec: number, kind: HealthEventKind, action: SafetyActionKind, message: string): void {
    if (reported.has(key)) return;
    reported.add(key);
    events.push({ tSec, kind, action, message });
  }

  /** Forget an episode so a recurrence logs again. */
  function clear(...keys: string[]): void {
    for (const k of keys) reported.delete(k);
  }

  return {
    events: () => events,
    check(input: HealthInput): HealthVerdict {
      const { tSec, inside } = input;
      const flags = new Set<HealthFlag>();
      let best: SafetyAction = { kind: 'none', reason: '' };
      let speedScale = 1;
      const home: SafetyActionKind = inside ? 'exit_building' : 'return_home';
      const offer = (kind: SafetyActionKind, reason: string): void => {
        if (RANK[kind] > RANK[best.kind]) best = { kind, reason };
      };
      const done = input.phase === 'land' || input.phase === 'done';

      // ------------------------------------------------------------ comms
      if (!input.linkUp) {
        linkLostSince ??= tSec;
        const lost = tSec - linkLostSince;
        if (lost >= cfg.commsGraceSec) {
          flags.add('link_lost');
          emit('comms', linkLostSince + cfg.commsGraceSec, 'comms_loss', 'none', `Operator link lost for ${fmt(cfg.commsGraceSec, 0)} s. Continuing autonomously; entry requests will be treated as denied.`);
        }
        const limit = inside ? cfg.commsInsideSec : cfg.commsRthSec;
        if (lost >= limit) {
          emit('comms-rth', tSec, 'comms_loss', home, `Operator link lost for ${fmt(limit, 0)} s. ${inside ? 'Leaving the building, then returning home' : 'Returning home'}.`);
          offer(home, `Operator link lost ${fmt(lost, 0)} s`);
        }
      } else if (linkLostSince !== null) {
        if (reported.has('comms')) events.push({ tSec, kind: 'comms_loss', action: 'none', message: `Operator link restored after ${fmt(tSec - linkLostSince)} s.` });
        linkLostSince = null;
        clear('comms', 'comms-rth');
      }

      // ------------------------------------------------------------ lidar
      const ring = input.lidar;
      const unknown = new Set<number>(silentLatched);
      if (ring && ring.length > 0) {
        const n = ring.length;
        const flagged: boolean[] = new Array<boolean>(n);
        let anyFlagged = false;
        for (let i = 0; i < n; i++) {
          flagged[i] = Number.isNaN(ring[i]!);
          if (flagged[i]) anyFlagged = true;
        }
        if (anyFlagged) {
          flaggedSince ??= tSec;
          if (tSec - flaggedSince >= cfg.lidarDetectSec - 1e-9) {
            flags.add('lidar_flagged');
            for (let i = 0; i < n; i++) if (flagged[i]) unknown.add(i);
          }
        } else flaggedSince = null;

        // Silent: the beam reports no return where the drone's own map has a wall well inside range.
        const expected = input.lidarExpected;
        if (expected && expected.length === n) {
          if (!silentCounts || silentCounts.length !== n) silentCounts = new Int32Array(n);
          for (let i = 0; i < n; i++) {
            const r = ring[i]!;
            const e = expected[i]!;
            const miss = !Number.isNaN(r) && r >= LIDAR_MAX - 1e-3;
            const disagree = miss && Number.isFinite(e) && e >= cfg.silentMinRangeM && e < LIDAR_MAX - cfg.silentMarginM;
            silentCounts[i] = disagree ? silentCounts[i]! + 1 : 0;
          }
          const suspect: boolean[] = Array.from(silentCounts, (c) => c >= cfg.silentScans);
          if (longestArc(suspect) >= cfg.silentMinBeams) {
            const before = silentLatched.size;
            for (let i = 0; i < n; i++) if (suspect[i]) silentLatched.add(i);
            if (silentLatched.size > before) for (const i of silentLatched) unknown.add(i);
          }
        }
        if (silentLatched.size > 0) flags.add('lidar_silent');

        if (unknown.size > 0) {
          const marks = Array.from({ length: n }, (_, i) => unknown.has(i));
          const arcDeg = (longestArc(marks) * 360) / n;
          const blind = `${unknown.size} of ${n} lidar beams${flags.has('lidar_silent') ? ' (silent: reading open where the map has walls)' : ''}`;
          const exit = inside !== null && arcDeg >= cfg.lidarExitDeg;
          lidarKey = `lidar-${unknown.size}-${exit}`;
          emit(lidarKey, tSec, 'sensor_fault', exit ? 'exit_building' : 'none', `Lidar fault: ${blind}, blind arc ${fmt(arcDeg, 0)} deg. Treating them as blocked and slowing down${exit ? '; leaving the building' : ''}.`);
          speedScale = Math.min(speedScale, cfg.lidarSpeedScale * (unknown.size >= n ? 0.5 : 1));
          if (exit) offer('exit_building', `Lidar blind arc ${fmt(arcDeg, 0)} deg`);
        } else if (lidarKey) {
          clear(lidarKey);
          lidarKey = null;
        }
      }

      // ----------------------------------------------------------- camera
      if (!input.cameraFrame) {
        cameraLostSince ??= tSec;
        const lost = tSec - cameraLostSince;
        if (lost >= cfg.cameraGraceSec) {
          flags.add('camera_lost');
          emit('camera', cameraLostSince + cfg.cameraGraceSec, 'sensor_fault', 'none', 'Camera frames stopped. Nothing is marked searched until they return.');
        }
        if (lost >= cfg.cameraRthSec) {
          emit('camera-rth', tSec, 'sensor_fault', home, `No camera frames for ${fmt(cfg.cameraRthSec, 0)} s. The search cannot continue; ${inside ? 'leaving the building' : 'returning home'}.`);
          offer(home, `Camera lost ${fmt(lost, 0)} s`);
        }
      } else if (cameraLostSince !== null) {
        if (reported.has('camera')) events.push({ tSec, kind: 'sensor_fault', action: 'none', message: `Camera frames back after ${fmt(tSec - cameraLostSince)} s.` });
        cameraLostSince = null;
        clear('camera', 'camera-rth');
      }

      // ------------------------------------------------------------- GNSS
      if (!inside && input.gnssFix === false) {
        gnssLostSince ??= tSec;
        const lost = tSec - gnssLostSince;
        flags.add('gnss_lost');
        emit('gnss', tSec, 'gnss_loss', 'none', `GNSS fix lost. Climbing clear of obstacles on dead reckoning and waiting up to ${fmt(cfg.gnssHoldSec, 0)} s for the fix.`);
        if (lost >= cfg.gnssLandSec) {
          emit('gnss-land', tSec, 'gnss_loss', 'land', `GNSS fix lost for ${fmt(cfg.gnssLandSec, 0)} s; dead-reckoning error is too large to find the pad. Landing.`);
          offer('land', `GNSS lost ${fmt(lost, 0)} s`);
        } else if (lost >= cfg.gnssHoldSec) {
          emit('gnss-rth', tSec, 'gnss_loss', 'return_home', `GNSS fix still lost after ${fmt(cfg.gnssHoldSec, 0)} s. Returning home on dead reckoning.`);
          offer('return_home', `GNSS lost ${fmt(lost, 0)} s`);
          speedScale = Math.min(speedScale, 0.7);
        }
        // Before that the runner climbs clear of obstacles on dead reckoning (flag gnss_lost) and waits for the fix.
      } else if (!inside && gnssLostSince !== null) {
        events.push({ tSec, kind: 'gnss_loss', action: 'none', message: `GNSS fix back after ${fmt(tSec - gnssLostSince)} s.` });
        gnssLostSince = null;
        clear('gnss', 'gnss-rth', 'gnss-land');
      }

      // -------------------------------------------------------- estimator
      const ph = input.poseHealth;
      if (inside && ph) {
        const lost = ph.state === 'lost' || ph.sigmaM >= cfg.estimatorLandSigmaM;
        const bad = lost || !ph.ok || ph.sigmaM >= cfg.estimatorExitSigmaM;
        if (bad) {
          estimatorBadSince ??= tSec;
          if (lost) {
            flags.add('estimator_lost');
            emit('est-land', tSec, 'estimator_fault', 'land', `Indoor position estimate lost (1-sigma ${fmt(ph.sigmaM, 2)} m). Landing in place.`);
            offer('land', 'Position estimate lost');
          } else if (tSec - estimatorBadSince >= cfg.estimatorDetectSec - 1e-9) {
            flags.add('estimator_degraded');
            speedScale = Math.min(speedScale, cfg.estimatorSpeedScale);
            emit('est', tSec, 'estimator_fault', 'exit_building', `Indoor position estimate degraded (1-sigma ${fmt(ph.sigmaM, 2)} m). Leaving the building slowly.`);
            offer('exit_building', 'Position estimate degraded');
          }
        } else estimatorBadSince = null;
      }

      // ------------------------------------------------------------ motor
      const ratios = input.motorRatio;
      if (ratios && ratios.length > 0) {
        const worst = Math.min(...ratios);
        if (worst < cfg.motorRatioMin) {
          if (motorLowSince === null) {
            motorLowSince = tSec;
            motorSum = 0;
            motorN = 0;
          }
          motorSum += worst;
          motorN += 1;
          if (tSec - motorLowSince >= cfg.motorDetectSec - 1e-9) motorLatched = Math.min(motorLatched ?? 1, motorSum / motorN);
        } else motorLowSince = null;
      }
      if (motorLatched !== null) {
        flags.add('motor_degraded');
        const auth = authorityLimits({ speedH: 1, speedV: 1, accel: 1, yawRate: 1 }, [motorLatched], cfg.thrustToWeight);
        speedScale = Math.min(speedScale, Math.max(0.2, auth.limits.speedH));
        if (auth.thrustToWeight < cfg.motorLandTw) {
          emit('motor-land', tSec, 'motor_fault', 'land', `Motor at ${fmt(motorLatched * 100, 0)}% of command; thrust-to-weight ${fmt(auth.thrustToWeight, 2)} is too low to fly on. Landing.`);
          offer('land', `Motor degraded to ${fmt(motorLatched * 100, 0)}%`);
        } else {
          emit('motor', tSec, 'motor_fault', home, `Motor at ${fmt(motorLatched * 100, 0)}% of command; thrust-to-weight ${fmt(auth.thrustToWeight, 2)}. Slowing down and ${inside ? 'leaving the building' : 'returning home'}.`);
          offer(home, `Motor degraded to ${fmt(motorLatched * 100, 0)}%`);
        }
      }

      // ---------------------------------------------------------- battery
      if (input.usableWh > 0 && input.dt > 0) predictedUsed += ((input.modelPowerW * input.dt) / 3600 / input.usableWh) * 100;
      anchor ??= { t: tSec, pct: input.batteryPct, predicted: predictedUsed };
      const excess = anchor.pct - input.batteryPct - (predictedUsed - anchor.predicted);
      if (samples.length === 0 || tSec - samples[samples.length - 1]!.t >= 1) {
        samples.push({ t: tSec, pct: input.batteryPct, predicted: predictedUsed });
        while (samples.length > 1 && tSec - samples[0]!.t > cfg.batteryWindowSec) samples.shift();
      }
      const first = samples[0]!;
      const span = tSec - first.t;
      const predictedDrop = predictedUsed - first.predicted;
      const observedDrop = first.pct - input.batteryPct;
      let rateRatio = 1;
      if (span >= cfg.batteryWindowSec * 0.75 && predictedDrop > 0.2) rateRatio = observedDrop / predictedDrop;
      if (excess >= cfg.batteryStepPct) {
        batteryScale = Math.max(batteryScale, 1.25);
        emit('battery-step', tSec, 'battery_fault', 'none', `Battery gauge dropped ${fmt(excess)} points more than the energy model predicts. Planning with ${fmt(batteryScale, 2)}x energy costs until the drain rate is measured.`);
        // Measure the step once, and keep it out of the rate window.
        anchor = { t: tSec, pct: input.batteryPct, predicted: predictedUsed };
        samples.length = 0;
        samples.push({ ...anchor });
      } else if (rateRatio >= cfg.batteryRateRatio) {
        batteryScale = Math.max(batteryScale, Math.min(3, rateRatio));
        emit('battery-rate', tSec, 'battery_fault', 'none', `Battery draining ${fmt(rateRatio, 2)}x faster than the energy model predicts. Planning with ${fmt(batteryScale, 2)}x energy costs.`);
      }
      if (batteryScale > 1) flags.add('battery_anomaly');

      // ------------------------------------------------------------- wind
      let windScale = 1;
      const w = input.windEstimate;
      const cruise = input.cruiseMps ?? 8;
      if (!inside && w) {
        const k = 1 - Math.exp(-input.dt / cfg.windAvgSec);
        windMean = windMean ? [windMean[0] + (w[0] - windMean[0]) * k, windMean[1] + (w[1] - windMean[1]) * k] : [w[0] * k, w[1] * k];
        const speed = Math.hypot(windMean[0], windMean[1]);
        if (input.toHome) {
          // Headwind on the way home: the wind component opposing travel toward home.
          const head = -(windMean[0] * input.toHome[0] + windMean[1] * input.toHome[1]);
          windScale = headwindCostScale(cruise, Math.max(0, head));
        }
        if (speed >= cfg.windLimitMps) {
          windHighSince ??= tSec;
          if (tSec - windHighSince >= cfg.windPersistSec - 1e-9) {
            flags.add('wind_limit');
            emit('wind', tSec, 'wind', 'return_home', `Mean wind ${fmt(speed)} m/s is over the ${fmt(cfg.windLimitMps, 0)} m/s limit for flying close to trees and buildings. Returning home.`);
            offer('return_home', `Wind ${fmt(speed)} m/s`);
          }
        } else windHighSince = null;
        // Strong wind can't wait for the mean: well over the limit on the short average, climb clear and go home now.
        const now = Math.hypot(w[0], w[1]);
        if (windSevereLatched) {
          if (now < cfg.windSevereMps - cfg.windSevereHysteresisMps) {
            windCalmSince ??= tSec;
            if (tSec - windCalmSince >= cfg.windSevereClearSec) windSevereLatched = false;
          } else windCalmSince = null;
        }
        if (now >= cfg.windSevereMps || windSevereLatched) {
          windSevereSince ??= tSec;
          if (windSevereLatched || tSec - windSevereSince >= cfg.windSevereSec - 1e-9) {
            windSevereLatched = true;
            flags.add('wind_limit');
            flags.add('wind_severe');
            emit('wind-severe', tSec, 'wind', 'return_home', `Wind ${fmt(now)} m/s, well over the ${fmt(cfg.windLimitMps, 0)} m/s limit. Climbing clear and returning home.`);
            offer('return_home', `Wind ${fmt(now)} m/s`);
          }
        } else windSevereSince = null;
        // Going home into wind the airframe can't beat: the drone stops closing on home. Land rather than
        // hover until the pack runs out.
        if (flags.has('wind_limit') && (input.phase === 'return_home' || input.phase === 'recover') && input.homeDistM !== undefined) {
          const d = input.homeDistM;
          // Over the pad there is nothing left to close.
          if (!windProgress || d < windProgress.d - cfg.windProgressM || d < cfg.windProgressM * 2) windProgress = { t: tSec, d };
          else if (tSec - windProgress.t >= cfg.windNoProgressSec - 1e-9) {
            emit('wind-headway', tSec, 'wind', 'land', `No headway home in ${fmt(cfg.windNoProgressSec, 0)} s against ${fmt(speed)} m/s mean wind. Landing here.`);
            offer('land', 'No headway home in wind');
          }
        }
      }

      // Scaled reserve check. The safety monitor runs the healthy-model check; this one only adds the fault margin.
      const costScale = Math.min(10, batteryScale * (Number.isFinite(windScale) ? windScale : 10));
      if (costScale > 1.05) {
        const reserve = Math.max(cfg.reservePct, 100 - (input.budgetPct ?? 100));
        const need = input.costHomePct * costScale;
        if (input.batteryPct - need < reserve) {
          emit('reserve-scaled', tSec, flags.has('battery_anomaly') ? 'battery_fault' : 'wind', home, `Battery ${fmt(input.batteryPct)}%; ${fmt(need)}% needed to get home with the ${fmt(costScale, 2)}x fault margin leaves under the ${fmt(reserve, 0)}% reserve. ${inside ? 'Leaving the building, then returning home' : 'Returning home'}.`);
          offer(home, `Battery reserve with ${fmt(costScale, 2)}x fault margin`);
        }
      }

      if (done) best = { kind: 'none', reason: '' };
      return {
        action: best,
        speedScale,
        unknownBeams: [...unknown].sort((a, b) => a - b),
        denyAsks: !input.linkUp,
        costScale,
        flags,
      };
    },
  };
}

/** The stronger of two actions; ties keep the first. */
export function strongest(a: SafetyAction, b: SafetyAction): SafetyAction {
  return RANK[b.kind] > RANK[a.kind] ? b : a;
}
