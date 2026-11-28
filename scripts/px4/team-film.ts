/**
 * Package a PX4 team film for the swarm replay page (swarm.html).
 *
 *   npx vite-node scripts/px4/team-film.ts -- --dir DIR --cond hard --seed 83101 --drones 4 [--out public/swarm]
 *
 * Reads DIR/film-<cond>-<seed>-<n>.json (scripts/px4/team.ts --film), adds the building (regenerated
 * from the seed, so the page never needs the generator), the base station position, the one-drone
 * PX4 coverage curve on the same building when DIR/film-<cond>-<seed>-1.json exists, every arm and
 * team size flown on this building, and the study's means over every building in DIR/summary.json.
 * Writes OUT/<cond>-<seed>-<n>.json.
 */
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { TEAM_CONDITIONS, teamEpisodeSetup } from '../../src/team/conditions';
import { basePosition } from '../../src/team/radio';

interface Episode {
  arm: string;
  cond: string;
  seed: number;
  drones: number;
  coverage: number;
  t50: number | null;
  t80: number | null;
  baseKnown: number;
  collisions: number;
  crashes: number;
  droneContacts: number;
  steps: number;
  failed?: string;
  px4?: { ekfErrRmsM: number; ekfErrMaxM: number; offboardDrops: number; wallStops: number }[];
}

const argv = process.argv.slice(2);
const arg = (name: string, def?: string): string => {
  const i = argv.indexOf(`--${name}`);
  if (i >= 0 && argv[i + 1]) return argv[i + 1]!;
  if (def === undefined) throw new Error(`missing --${name}`);
  return def;
};
const dir = resolve(arg('dir'));
const condId = arg('cond');
const seed = Number(arg('seed'));
const n = Number(arg('drones'));
const out = resolve(arg('out', 'public/swarm'));
const cond = TEAM_CONDITIONS.find((c) => c.id === condId);
if (!cond) throw new Error(`unknown condition ${condId}`);

const film = JSON.parse(readFileSync(join(dir, `film-${condId}-${seed}-${n}.json`), 'utf8'));
const { spec, cfg } = teamEpisodeSetup(cond, seed);
const soloPath = join(dir, `film-${condId}-${seed}-1.json`);
const solo = n > 1 && existsSync(soloPath) ? JSON.parse(readFileSync(soloPath, 'utf8')) : null;

const summary: Episode[] = existsSync(join(dir, 'summary.json')) ? JSON.parse(readFileSync(join(dir, 'summary.json'), 'utf8')).episodes : [];
const sameBuilding = summary
  .filter((e) => e.seed === seed && e.cond === condId)
  .map((e) => ({ arm: e.arm, drones: e.drones, coverage: e.coverage, t50: e.t50, t80: e.t80, collisions: e.collisions, crashes: e.crashes, contacts: e.droneContacts }))
  .sort((a, b) => a.arm.localeCompare(b.arm) || a.drones - b.drones);

// Means over every building flown so far, by arm and team size. Only buildings every arm and team
// size has finished count, so the rows compare the same buildings.
const ARMS = ['px4', 'quad', 'kinematic'];
const sizes = [1, 2, 3, 4];
const bySeed = new Map<number, Episode[]>();
for (const e of summary) if (e.cond === condId) bySeed.set(e.seed, [...(bySeed.get(e.seed) ?? []), e]);
const complete = [...bySeed.entries()].filter(([, es]) => ARMS.every((a) => sizes.every((s) => es.some((e) => e.arm === a && e.drones === s)))).map(([s]) => s);
const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : null);
const study = ARMS.flatMap((arm) =>
  sizes.map((drones) => {
    const es = summary.filter((e) => e.cond === condId && e.arm === arm && e.drones === drones && complete.includes(e.seed));
    const t80s = es.map((e) => e.t80).filter((t): t is number => t !== null);
    const ekf = es.flatMap((e) => (e.px4 ?? []).map((p) => p.ekfErrRmsM));
    return {
      arm,
      drones,
      buildings: es.length,
      coverage: mean(es.map((e) => e.coverage)),
      reached80: es.length ? t80s.length / es.length : null,
      t80Mean: mean(t80s),
      contacts: es.reduce((a, e) => a + e.droneContacts, 0),
      collisions: es.reduce((a, e) => a + e.collisions, 0),
      crashes: es.reduce((a, e) => a + e.crashes, 0),
      failed: es.filter((e) => e.failed).length,
      ekfRmsMean: mean(ekf),
    };
  }),
);

const bundle = {
  schema: 1,
  id: `${condId}-${seed}-${n}`,
  cond: condId,
  condLabel: cond.label,
  size: cond.size,
  budgetS: cfg.maxSteps * 0.05,
  seed,
  drones: n,
  dt: film.dt,
  autopilot: film.autopilot,
  columns: film.columns,
  perDrone: film.perDrone,
  frames: film.frames,
  flown: film.flown,
  result: film.result,
  spec,
  base: basePosition(spec),
  solo: solo ? { dt: solo.dt, coverage: solo.frames.map((f: number[]) => f[1]), result: solo.result } : null,
  sameBuilding,
  study: { split: 'test', buildings: complete.length, rows: study },
};
mkdirSync(out, { recursive: true });
const file = join(out, `${bundle.id}.json`);
// Typed arrays (floor cells and surfaces) go out as plain arrays; the page turns them back.
const json = JSON.stringify(bundle, (_k, v) => (ArrayBuffer.isView(v) ? Array.from(v as Uint8Array) : v));
writeFileSync(file, json);
console.log(`${file}: ${film.frames.length} frames, ${film.flown.length / 3} tiles, solo ${solo ? 'yes' : 'no'}, study over ${complete.length} buildings, ${(json.length / 1024).toFixed(0)} KB`);
