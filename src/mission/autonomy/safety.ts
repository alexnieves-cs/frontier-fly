import { pointInRing } from '../polygon';
import type { Mission, Point } from '../types';
import { sampleHeight } from '../../world/outdoor/terrain';
import type {
  MissionWorld,
  Phase,
  SafetyAction,
  SafetyActionKind,
  SafetyEvent,
  SafetyInput,
  SafetyMonitor,
  V3,
} from './contracts';

/**
 * Safety monitor for autonomous missions.
 *
 * Checks run every tick and return the strongest action needed now. Each
 * distinct episode (a geofence lookahead breach, an altitude excursion, a
 * battery threshold, one collision) records one SafetyEvent, so the log does
 * not repeat every tick.
 *
 * Rules:
 * - Geofence and no-fly. A predicted breach holds; if the predicted breach
 *   persists for `breachPersistSec` the monitor sends the drone home. An
 *   actual breach sends it home at once. Mission no-fly zones block every
 *   altitude unless they come from a site-plan zone (`nf-<id>`), which uses
 *   that zone's world-z band. Site-plan zones outside the mission list are
 *   checked too.
 * - Altitude, outdoors only. AGL is pose z minus terrain height. Above
 *   `altitude.maxM` (actual or predicted) holds so the planner corrects.
 *   Below `altitude.minM` is allowed within `lowAltitudeRadiusM` of home in
 *   any phase (takeoff and landing), and within that distance of a building
 *   footprint only in close-in phases (LOW_ALTITUDE_PHASES: approach,
 *   inspect, entry, fly-through, indoor, confirm, exit, return, land,
 *   recover). Anywhere else it holds so the planner climbs.
 * - Battery. Reserve: when remainingPct - costPct(home) drops below the
 *   reserve, the drone returns home (exits the building first when inside).
 *   The reserve is the larger of `reservePct` and 100 - budgets.batteryPct, so
 *   the mission never plans to consume more than its budget. The reserve
 *   latches once tripped. Below `criticalPct` it lands where it is.
 * - Collisions escalate within `collisionWindowSec`: hold, backtrack,
 *   exit_building (or return_home outdoors), return_home, then land. The
 *   action is returned on the tick a contact starts; the runner carries it out.
 * - return_home and land phases never trigger another return_home (no
 *   re-entrant loops); land and done suppress everything.
 */

export interface SafetyConfig {
  /** Battery percent kept in hand after the trip home. */
  reservePct: number;
  /** Below this remaining percent, land immediately. */
  criticalPct: number;
  /** Predicted breach this long turns a hold into return_home, seconds. */
  breachPersistSec: number;
  /** A breach episode ends after this long without a breach, seconds. */
  clearGapSec: number;
  /** Hold length suggested for lookahead and altitude holds, seconds. */
  holdSec: number;
  /** Horizontal distance from home or a building where low flight is allowed. */
  lowAltitudeRadiusM: number;
  /** Collision escalation window, seconds. */
  collisionWindowSec: number;
  /** A contact this long after the last one is a new collision, seconds. */
  collisionGapSec: number;
  /** Action for the n-th collision in the window; the last entry repeats. */
  collisionLadder: SafetyActionKind[];
  /** Hold after a first collision, seconds. */
  collisionHoldSec: number;
  /** Backtrack distance after a second collision, meters. */
  backtrackM: number;
}

export const DEFAULT_SAFETY: SafetyConfig = {
  reservePct: 20,
  criticalPct: 8,
  breachPersistSec: 3,
  clearGapSec: 1.5,
  holdSec: 1,
  lowAltitudeRadiusM: 10,
  collisionWindowSec: 30,
  collisionGapSec: 0.5,
  collisionLadder: ['hold', 'backtrack', 'exit_building', 'return_home', 'land'],
  collisionHoldSec: 1.5,
  backtrackM: 1.5,
};

const RANK: Record<SafetyActionKind, number> = {
  none: 0,
  hold: 1,
  backtrack: 2,
  exit_building: 3,
  return_home: 4,
  land: 5,
};

/** Phases where flying below the altitude floor is allowed near home or a building. */
export const LOW_ALTITUDE_PHASES: ReadonlySet<Phase> = new Set<Phase>([
  'preflight',
  'approach',
  'inspect',
  'entry_select',
  'await_operator',
  'fly_through',
  'indoor_explore',
  'confirm',
  'exit',
  'return_home',
  'land',
  'recover',
]);

interface Zone {
  name: string;
  ring: Point[];
  minZ: number;
  maxZ: number;
}

interface Episode {
  start: number;
  last: number;
  escalated: boolean;
}

function noFlyZones(mission: Mission, world: MissionWorld): Zone[] {
  const zones: Zone[] = [];
  const used = new Set<number>();
  for (const zone of mission.noFlyZones) {
    const site = world.outdoor.noFly.find((z) => `nf-${z.id}` === zone.id);
    if (site) used.add(site.id);
    zones.push({
      name: zone.name,
      ring: zone.polygon.points,
      minZ: site ? site.minZ : -Infinity,
      maxZ: site ? site.maxZ : Infinity,
    });
  }
  for (const site of world.outdoor.noFly) {
    if (used.has(site.id)) continue;
    zones.push({ name: site.name, ring: site.polygon.map(([x, y]) => ({ x, y })), minZ: site.minZ, maxZ: site.maxZ });
  }
  return zones;
}

function distToRing(p: Point, ring: readonly Point[]): number {
  if (pointInRing(p, ring)) return 0;
  let best = Infinity;
  for (let i = 0; i < ring.length; i++) {
    const a = ring[i]!;
    const b = ring[(i + 1) % ring.length]!;
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const len2 = dx * dx + dy * dy;
    const t = len2 > 0 ? Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2)) : 0;
    best = Math.min(best, Math.hypot(p.x - a.x - t * dx, p.y - a.y - t * dy));
  }
  return best;
}

function fmt(n: number, digits = 1): string {
  return n.toFixed(digits);
}

export function createSafetyMonitor(mission: Mission, world: MissionWorld, config?: Partial<SafetyConfig>): SafetyMonitor {
  const cfg: SafetyConfig = { ...DEFAULT_SAFETY, ...config };
  const events: SafetyEvent[] = [];
  const zones = noFlyZones(mission, world);
  const fence = mission.geofence.points;
  const reservePct = Math.max(cfg.reservePct, 100 - mission.budgets.batteryPct);
  const terrain = world.outdoor.terrain;

  const episodes = new Map<string, Episode>();
  let reserveTripped = false;
  let criticalTripped = false;
  let lastContact = -Infinity;
  const collisionTimes: number[] = [];

  function record(tSec: number, kind: SafetyEvent['kind'], action: SafetyActionKind, message: string): void {
    events.push({ tSec, kind, action, message });
  }

  /** Track a condition with hysteresis. Returns the episode while it is active. */
  function episode(key: string, active: boolean, tSec: number): { ep: Episode; fresh: boolean } | null {
    const ep = episodes.get(key);
    if (!active) {
      if (ep && tSec - ep.last > cfg.clearGapSec) episodes.delete(key);
      return null;
    }
    if (ep && tSec - ep.last <= cfg.clearGapSec) {
      ep.last = tSec;
      return { ep, fresh: false };
    }
    const next = { start: tSec, last: tSec, escalated: false };
    episodes.set(key, next);
    return { ep: next, fresh: true };
  }

  function zoneAt(pos: V3): Zone | null {
    const p = { x: pos[0], y: pos[1] };
    for (const zone of zones) {
      if (pos[2] < zone.minZ || pos[2] > zone.maxZ) continue;
      if (pointInRing(p, zone.ring)) return zone;
    }
    return null;
  }

  function nearHome(pos: V3): boolean {
    return Math.hypot(pos[0] - world.home[0], pos[1] - world.home[1]) <= cfg.lowAltitudeRadiusM;
  }

  function nearBuilding(pos: V3): boolean {
    const p = { x: pos[0], y: pos[1] };
    return world.buildings.some((b) => distToRing(p, b.footprint) <= cfg.lowAltitudeRadiusM);
  }

  return {
    check(input: SafetyInput): SafetyAction {
      const { tSec, pose, predicted, phase, inside } = input;
      if (phase === 'land' || phase === 'done') return { kind: 'none', reason: '' };
      const homing = phase === 'return_home';
      let best: SafetyAction = { kind: 'none', reason: '' };
      const offer = (action: SafetyAction): void => {
        if (homing && action.kind === 'return_home') return;
        if (RANK[action.kind] > RANK[best.kind]) best = action;
      };

      // Battery.
      const remaining = input.battery.remainingPct();
      if (remaining < cfg.criticalPct) {
        if (!criticalTripped) {
          criticalTripped = true;
          record(tSec, 'battery_critical', 'land', `Battery critical at ${fmt(remaining)}%. Landing now.`);
        }
        offer({ kind: 'land', reason: `Battery critical (${fmt(remaining)}%)` });
      }
      const climb = Math.max(0, world.home[2] - pose.pos[2]);
      const costHome = input.battery.costPct(input.distanceHomeM, climb);
      const margin = remaining - costHome;
      if (!reserveTripped && margin < reservePct) {
        reserveTripped = true;
        const first = inside ? 'Exiting the building, then returning home' : 'Returning home';
        record(
          tSec,
          'battery_reserve',
          homing ? 'none' : inside ? 'exit_building' : 'return_home',
          `Battery ${fmt(remaining)}%, ${fmt(costHome)}% needed to get home ${fmt(input.distanceHomeM, 0)} m away leaves under the ${fmt(reservePct, 0)}% reserve. ${first}.`,
        );
      }
      if (reserveTripped) {
        // Inside, exit first; the exit phase is already doing that.
        const kind: SafetyActionKind = inside ? (phase === 'exit' ? 'none' : 'exit_building') : 'return_home';
        offer({ kind, reason: `Battery reserve (${fmt(remaining)}% left)` });
      }

      // Geofence and no-fly: actual breach.
      const here = { x: pose.pos[0], y: pose.pos[1] };
      const outside = !pointInRing(here, fence);
      const fenceNow = episode('fence-now', outside, tSec);
      if (fenceNow) {
        if (fenceNow.fresh) record(tSec, 'geofence', homing ? 'none' : 'return_home', 'Outside the geofence. Returning home.');
        offer({ kind: 'return_home', reason: 'Geofence breach' });
      }
      const zoneNow = zoneAt(pose.pos);
      const nfNow = episode('nofly-now', zoneNow !== null, tSec);
      if (nfNow && zoneNow) {
        if (nfNow.fresh) record(tSec, 'no_fly', homing ? 'none' : 'return_home', `Inside no-fly zone "${zoneNow.name}". Returning home.`);
        offer({ kind: 'return_home', reason: `No-fly zone ${zoneNow.name}` });
      }

      // Geofence and no-fly: lookahead. Skipped while homing so RTH is not held up.
      if (!homing) {
        const ahead = { x: predicted[0], y: predicted[1] };
        const fenceAhead = episode('fence-ahead', !outside && !pointInRing(ahead, fence), tSec);
        const zoneAhead = zoneNow ? null : zoneAt(predicted);
        const nfAhead = episode('nofly-ahead', zoneAhead !== null, tSec);
        const lookahead: [typeof fenceAhead, SafetyEvent['kind'], string][] = [
          [fenceAhead, 'geofence', 'the geofence edge'],
          [nfAhead, 'no_fly', `no-fly zone "${zoneAhead?.name ?? ''}"`],
        ];
        for (const [state, kind, what] of lookahead) {
          if (!state) continue;
          if (state.fresh) record(tSec, kind, 'hold', `Command would cross ${what}. Holding.`);
          if (tSec - state.ep.start >= cfg.breachPersistSec) {
            if (!state.ep.escalated) {
              state.ep.escalated = true;
              record(tSec, kind, 'return_home', `Still heading across ${what} after ${fmt(cfg.breachPersistSec, 0)} s. Returning home.`);
            }
            offer({ kind: 'return_home', reason: `Persistent ${kind} lookahead` });
          } else {
            offer({ kind: 'hold', reason: `Lookahead crosses ${what}`, seconds: cfg.holdSec });
          }
        }
      }

      // Altitude band, outdoors only.
      if (!inside) {
        const agl = pose.pos[2] - sampleHeight(terrain, pose.pos[0], pose.pos[1]);
        const aglAhead = predicted[2] - sampleHeight(terrain, predicted[0], predicted[1]);
        const high = Math.max(agl, homing ? agl : aglAhead) > mission.altitude.maxM;
        const lowAllowed = nearHome(pose.pos) || (LOW_ALTITUDE_PHASES.has(phase) && nearBuilding(pose.pos));
        const low = agl < mission.altitude.minM && !lowAllowed;
        const alt = episode('altitude', high || low, tSec);
        if (alt) {
          const message = high
            ? `Altitude ${fmt(Math.max(agl, aglAhead))} m AGL above the ${fmt(mission.altitude.maxM, 0)} m ceiling. Holding to descend.`
            : `Altitude ${fmt(agl)} m AGL below the ${fmt(mission.altitude.minM, 0)} m floor away from home and buildings. Holding to climb.`;
          if (alt.fresh) record(tSec, 'altitude', 'hold', message);
          offer({ kind: 'hold', reason: high ? 'Above altitude ceiling' : 'Below altitude floor', seconds: cfg.holdSec });
        }
      }

      // Collisions.
      if (input.collided) {
        const fresh = tSec - lastContact > cfg.collisionGapSec;
        lastContact = tSec;
        if (fresh) {
          while (collisionTimes.length > 0 && tSec - collisionTimes[0]! > cfg.collisionWindowSec) collisionTimes.shift();
          collisionTimes.push(tSec);
          const n = collisionTimes.length;
          const ladder = cfg.collisionLadder;
          let kind = ladder[Math.min(n, ladder.length) - 1] ?? 'land';
          if (kind === 'exit_building' && !inside) kind = 'return_home';
          if (homing && kind === 'return_home') kind = 'hold';
          const action: SafetyAction = { kind, reason: `Collision ${n} in ${fmt(cfg.collisionWindowSec, 0)} s` };
          if (kind === 'hold') action.seconds = cfg.collisionHoldSec;
          if (kind === 'backtrack') action.meters = cfg.backtrackM;
          const where = inside ? ' inside the building' : '';
          record(tSec, 'collision', kind, `Collision ${n} within ${fmt(cfg.collisionWindowSec, 0)} s${where}. Action: ${kind.replace('_', ' ')}.`);
          if (RANK[action.kind] > RANK[best.kind]) best = action;
        }
      }

      return best;
    },
    events() {
      return events;
    },
  };
}
