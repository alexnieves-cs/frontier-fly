import { separate as separateKinematic } from '../../team/run';
import { initOutdoorPhysics, OutdoorPhysics } from '../../world/outdoor/physics';
import { sampleHeight } from '../../world/outdoor/terrain';
import type { Mission } from '../types';
import type {
  ConfirmedContact,
  MissionForWorldOptions,
  MissionRunner,
  MissionWorld,
  MissionWorldOptions,
  RunRecord,
  RunnerOptions,
  TeamLink,
  V3,
} from './contracts';
import type { MeasuredOpening } from './openingSensor';
import { createPerceptionSource } from './perception';
import { requirementsMet } from './planner/confirm';
import { pointFree } from './planner/geometry';
import { createMissionRunner } from './runner';
import { buildMissionWorld, missionForWorld } from './world';

/**
 * Two ways in: two drones fly one woods-to-building mission together. Each drone is a full
 * mission runner with its own copy of the world, its own sensors and its own decisions. The board
 * below is all they share, and all of it is what a radio link would carry: where each drone is,
 * the standoff it took, the openings it measured from outside, the opening it is entering by,
 * where its camera has looked indoors, and what it confirmed.
 *
 * What that buys, per drone:
 * - the envelope inspection is split by side, so each flies about half of it;
 * - the openings measured on the far side reach the drone that did not see them;
 * - a drone never takes the opening its teammate is entering by while a free one is viable;
 * - indoors each look phase goes where neither camera has looked;
 * - the first confirmed find sends both home.
 *
 * Drones are not physics bodies in each other's worlds. As in src/team, each drone cuts the part of
 * its velocity that closes on the other (the stand-in for seeing it on lidar), and the run records
 * how close they came.
 */

export interface DuoOptions {
  /** Seconds between A's and B's takeoff. Default 4. */
  launchGapSec?: number;
  /** Distance from A's pad to B's, m. Default 3. */
  padSpacingM?: number;
  /** Share nothing: two drones that each fly the mission alone (the control arm). Default false. */
  noLink?: boolean;
}

export interface DuoDrone {
  name: string;
  record: RunRecord;
  /** Team time of takeoff, s: B's record times are shifted by this. */
  launchSec: number;
  home: V3;
}

export interface DuoResult {
  drones: DuoDrone[];
  /** Whether the team's confirmed contacts meet the mission. */
  found: boolean;
  /** Team time of the first confirmed contact that completed the mission, s; null if never. */
  tFoundSec: number | null;
  /** Team time the last drone finished, s. */
  tDoneSec: number;
  /** The opening each drone flew in by (auto or approved entries), in drone order; null when it did not enter. */
  entries: (number | null)[];
  /** Both drones entered, by different openings. */
  distinctEntries: boolean;
  /** Closest the two drone centers came while both were flying, m. */
  minSeparationM: number;
  /** Seconds the two drone centers were closer than CLOSE_M while both were flying. */
  closeSec: number;
  /** Where and when the closest approach happened, with each drone's phase. */
  closest: { tSec: number; pos: V3; phases: [string, string] } | null;
  /** Contacts with the world (not each other), summed over the drones. */
  collisions: number;
  landedHome: boolean;
}

/** Drone-to-drone distance under which the run counts time as close, m (two 0.38 m airframes, centers). */
export const CLOSE_M = 0.6;

interface Shared {
  name: string;
}

/** What the drones share. One per duo run. */
export class TeamBoard {
  readonly pos = new Map<string, V3>();
  readonly phase = new Map<string, string>();
  private readonly standoffBy = new Map<string, Map<string, V3>>();
  private readonly openingBy = new Map<string, Map<number, Shared & { opening: MeasuredOpening }>>();
  private readonly surveyed = new Map<string, Set<string>>();
  private readonly claims = new Map<string, Map<string, number>>();
  private readonly lookedBy = new Map<string, Map<string, Uint8Array>>();
  readonly confirmed: (Shared & { contact: ConfirmedContact })[] = [];
  private readonly confirmingBy = new Map<string, V3>();

  link(name: string, lead: boolean): TeamLink {
    const at = <K, V>(m: Map<string, Map<K, V>>, b: string): Map<K, V> => {
      let inner = m.get(b);
      if (!inner) m.set(b, (inner = new Map()));
      return inner;
    };
    const board = this;
    return {
      name,
      lead,
      standoffs: (b) => [...at(board.standoffBy, b)].filter(([n]) => n !== name).map(([n, pos]) => ({ name: n, pos })),
      publishStandoff: (b, pos) => void at(board.standoffBy, b).set(name, [pos[0], pos[1], pos[2]]),
      openings: (b) => [...at(board.openingBy, b).values()].filter((o) => o.name !== name),
      shareOpening: (b, opening) => {
        const known = at(board.openingBy, b);
        // First measurement of an opening stands: whoever saw it first owns the shared copy.
        if (!known.has(opening.id)) known.set(opening.id, { name, opening });
      },
      surveyDone: (b) => {
        const done = board.surveyed.get(b) ?? new Set<string>();
        return [...at(board.standoffBy, b).keys()].every((n) => n === name || done.has(n));
      },
      finishSurvey: (b) => {
        let done = board.surveyed.get(b);
        if (!done) board.surveyed.set(b, (done = new Set()));
        done.add(name);
      },
      claimedBy: (b, id) => {
        for (const [n, claimed] of at(board.claims, b)) if (n !== name && claimed === id) return n;
        return null;
      },
      claim: (b, id) => {
        if (id === null) at(board.claims, b).delete(name);
        else at(board.claims, b).set(name, id);
      },
      looked: (b) => {
        for (const [n, arr] of at(board.lookedBy, b)) if (n !== name) return arr;
        return null;
      },
      shareLooked: (b, looked) => void at(board.lookedBy, b).set(name, looked),
      teamConfirmed: () => board.confirmed.filter((c) => c.name !== name),
      publishConfirmed: (contact) => void board.confirmed.push({ name, contact }),
      nearest: (pos) => {
        let best: { name: string; pos: V3; phase: string | null } | null = null;
        let bestD = Infinity;
        for (const [n, p] of board.pos) {
          if (n === name) continue;
          const d = Math.hypot(p[0] - pos[0], p[1] - pos[1], p[2] - pos[2]);
          if (d < bestD) {
            bestD = d;
            best = { name: n, pos: p, phase: board.phase.get(n) ?? null };
          }
        }
        return best;
      },
      confirming: () => [...board.confirmingBy].filter(([n]) => n !== name).map(([n, estimate]) => ({ name: n, estimate })),
      setConfirming: (estimate) => {
        if (estimate) board.confirmingBy.set(name, [estimate[0], estimate[1], estimate[2]]);
        else board.confirmingBy.delete(name);
      },
      separate: (pos, vel) => {
        const others = [...board.pos].filter(([n]) => n !== name).map(([, p]) => p);
        if (!others.length) return vel;
        const [vx, vy] = separateKinematic(vel[0], vel[1], pos, others);
        return [vx, vy, vel[2]];
      },
    };
  }
}

/**
 * B's pad: beside A's, across the line from home to the nearest building, on open ground with a
 * clear column. Falls back to A's pad when none of the candidates is clear (B then waits out the gap).
 */
export function secondPad(world: MissionWorld, physics: OutdoorPhysics, spacing: number): V3 {
  const [hx, hy] = world.home;
  const b = [...world.buildings].sort((p, q) => dist2(p.shell.approach, world.home) - dist2(q.shell.approach, world.home))[0];
  const dx = b ? b.shell.approach[0] - hx : Math.cos(world.homeYaw);
  const dy = b ? b.shell.approach[1] - hy : Math.sin(world.homeYaw);
  const len = Math.hypot(dx, dy) || 1;
  const px = -dy / len;
  const py = dx / len;
  const off = world.home[2] - sampleHeight(world.outdoor.terrain, hx, hy);
  // Beside home first, square to the way to the nearest building, then anywhere on rings further out.
  const spots: [number, number][] = [1, -1, 1.5, -1.5, 2, -2].map((k) => [hx + px * spacing * k, hy + py * spacing * k]);
  for (let r = spacing; r <= spacing * 2; r += 0.5) {
    for (let a = 0; a < 16; a++) spots.push([hx + r * Math.cos((a * Math.PI) / 8), hy + r * Math.sin((a * Math.PI) / 8)]);
  }
  for (const [x, y] of spots) {
    const ground = sampleHeight(world.outdoor.terrain, x, y);
    const z = ground + off;
    // The whole column has to be clear: the drone lands through it, and undergrowth sits at ground level.
    const column: [number, number][] = [[ground + 0.5, 0.4], [ground + 1, 0.5], [z, 0.6], [z + 1.5, 0.6], [z + 3, 0.6]];
    if (column.every(([h, r]) => pointFree(physics, [x, y, h], r))) return [x, y, z];
  }
  return [world.home[0], world.home[1], world.home[2]];
}

function dist2(a: readonly number[], b: readonly number[]): number {
  return Math.hypot(a[0]! - b[0]!, a[1]! - b[1]!);
}

/** A duo flight that advances one control tick per step(), for pages that draw it as it flies. */
export interface DuoFlight {
  a: MissionRunner;
  b: MissionRunner;
  worldA: MissionWorld;
  worldB: MissionWorld;
  padB: V3;
  launchGapSec: number;
  dt: number;
  /** Team time, s. */
  readonly tSec: number;
  /** Closest the two drone centers have come while both were flying, m. */
  readonly minSeparationM: number;
  readonly done: boolean;
  /** One control tick for both drones (B holds on its pad for the launch gap). */
  step(): void;
  /** The scores so far; final once done. */
  result(): DuoResult;
}

/** Build both drones' worlds and runners. Same inputs give the same flight. */
export async function createDuo(
  worldOptions: MissionWorldOptions,
  missionOptions: MissionForWorldOptions,
  runnerOptions: RunnerOptions = {},
  duo: DuoOptions = {},
): Promise<DuoFlight> {
  await initOutdoorPhysics();
  const gap = duo.launchGapSec ?? 4;
  const worldA = buildMissionWorld(worldOptions);
  const worldB = buildMissionWorld(worldOptions);
  const padB = secondPad(worldA, new OutdoorPhysics(worldA.outdoor), duo.padSpacingM ?? 3);
  worldB.home = padB;
  const missionA = missionForWorld(worldA, missionOptions);
  const missionB = missionForWorld(worldB, missionOptions);
  const board = new TeamBoard();
  const make = (name: string, lead: boolean, mission: Mission, world: MissionWorld) =>
    createMissionRunner(mission, world, createPerceptionSource(world), {
      ...runnerOptions,
      ...(duo.noLink ? {} : { team: board.link(name, lead) }),
    });
  const a = await make('A', true, missionA, worldA);
  const b = await make('B', false, missionB, worldB);
  const dt = runnerOptions.dt ?? 0.05;
  const limit = Math.ceil(((runnerOptions.maxSimSec ?? missionA.budgets.timeSec) + gap + 5) / dt);
  const gapTicks = Math.round(gap / dt);
  let i = 0;
  let minSep = Infinity;
  let closeTicks = 0;
  let closest: DuoResult['closest'] = null;
  const report = (name: string, r: MissionRunner) => {
    const snap = r.snapshot();
    board.pos.set(name, snap.pose.pos);
    board.phase.set(name, snap.phase);
  };
  report('A', a);
  board.pos.set('B', padB);
  const finished = () => (a.done && b.done) || i >= limit;
  return {
    a,
    b,
    worldA,
    worldB,
    padB,
    launchGapSec: gap,
    dt,
    get tSec() {
      return i * dt;
    },
    get minSeparationM() {
      return minSep;
    },
    get done() {
      return finished();
    },
    step() {
      if (finished()) return;
      if (!a.done) {
        a.step();
        report('A', a);
      }
      if (i >= gapTicks && !b.done) {
        b.step();
        report('B', b);
      }
      if (i >= gapTicks && !a.done && !b.done) {
        const pa = board.pos.get('A')!;
        const pb = board.pos.get('B')!;
        const d = Math.hypot(pa[0] - pb[0], pa[1] - pb[1], pa[2] - pb[2]);
        if (d < minSep) {
          minSep = d;
          closest = { tSec: (i + 1) * dt, pos: [pa[0], pa[1], pa[2]], phases: [a.snapshot().phase, b.snapshot().phase] };
        }
        if (d < CLOSE_M) closeTicks += 1;
      }
      i += 1;
    },
    result() {
      const drones: DuoDrone[] = [
        { name: 'A', record: a.record(), launchSec: 0, home: worldA.home },
        { name: 'B', record: b.record(), launchSec: gap, home: padB },
      ];
      return { ...summarize(missionA, drones, minSep, closeTicks * dt), closest };
    },
  };
}

/** Fly one mission with two drones, A first and B `launchGapSec` later. Same inputs give the same result. */
export async function runDuo(
  worldOptions: MissionWorldOptions,
  missionOptions: MissionForWorldOptions,
  runnerOptions: RunnerOptions = {},
  duo: DuoOptions = {},
): Promise<DuoResult> {
  const flight = await createDuo(worldOptions, missionOptions, runnerOptions, duo);
  while (!flight.done) flight.step();
  return flight.result();
}

function summarize(mission: Mission, drones: DuoDrone[], minSep: number, closeSec: number): Omit<DuoResult, 'closest'> {
  const confirmed = drones
    .flatMap((d) => d.record.confirmed.map((c) => ({ ...c, tSec: c.tSec + d.launchSec })))
    .sort((p, q) => p.tSec - q.tSec);
  let tFound: number | null = null;
  for (let k = 1; k <= confirmed.length; k++) {
    if (requirementsMet(mission, confirmed.slice(0, k))) {
      tFound = confirmed[k - 1]!.tSec;
      break;
    }
  }
  // The opening each drone flew in by: its last go-ahead (an approval can be followed by a fallback).
  const entries = drones.map((d) => {
    const go = d.record.entries.filter((x) => x.decision === 'auto' || x.decision === 'approved');
    const inside = d.record.phases.some((p) => p.phase === 'indoor_explore');
    return inside && go.length ? go[go.length - 1]!.openingId : null;
  });
  return {
    drones,
    found: tFound !== null,
    tFoundSec: tFound,
    tDoneSec: Math.max(...drones.map((d) => d.record.tSec + d.launchSec)),
    entries,
    distinctEntries: entries.every((e) => e !== null) && new Set(entries).size === entries.length,
    minSeparationM: Number.isFinite(minSep) ? minSep : Infinity,
    closeSec,
    collisions: drones.reduce((n, d) => n + d.record.collisions.length, 0),
    landedHome: drones.every((d) => d.record.landedHome),
  };
}

/**
 * The Two ways in scenario: cabin-day's world and mission with a second way into the cabin. The
 * operator answer approves `ask` entries, as the demo's operator would after a look.
 */
export const TWO_WAYS_IN = {
  world: {
    seed: 20,
    buildings: 1,
    entrances: 2,
    timeOfDay: 13,
    flashlightOn: false,
    interiorLights: 'on',
    targets: [{ classId: 'person', state: 'lying', where: 'indoor', building: 0 }],
    decoys: 1,
  } satisfies MissionWorldOptions,
  mission: {
    missionType: 'find_person',
    name: 'Cabin search, two drones',
    targets: [{ id: 'tgt-person', kind: 'person', label: 'Injured hiker (simulated)', minCount: 1, minConfidence: null }],
    requiredConfidence: 0.8,
    entryPolicy: 'auto',
    budgets: { batteryPct: 80, timeSec: 900 },
  } satisfies MissionForWorldOptions,
  runner: { dt: 0.05, indoor: 'auto', operator: () => 'approve' } satisfies RunnerOptions,
};
