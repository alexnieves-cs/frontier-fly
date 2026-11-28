/**
 * A drone team where every drone is flown by its own PX4 SITL instance.
 *
 *   PX4_BUILD=/path/to/PX4-Autopilot/build/px4_sitl_default \
 *     npx vite-node scripts/px4/team.ts -- --dir DIR [--cond hard|large] [--split dev|test] [--count N]
 *       [--seeds 83001,83002] [--drones 1,2,3,4] [--arms px4,kinematic] [--film SEED:DRONES ...]
 *
 * The team is the drone-team track's (src/team/run.ts, src/bench/team.ts): sensor-only planners
 * that share maps over the wall-attenuated 2.4 GHz radio, split frontiers by claims and launch
 * through the entry one second apart. The kinematic arm flies them on the env's first-order
 * velocity model with the plain planner, as the bench does. The quad arm flies the env's own rigid
 * body and cascaded controller with the default quadrotor pilot (src/sim/quadPilot.ts: braking
 * filter, trajectory tracker, learned goal picker) and the braking-aware spacing rule between
 * drones (separateBraking in src/team/run.ts). The px4 arm flies that same pilot and rule, but swaps each
 * drone's airframe for PX4 v1.15.4 SITL
 * (EKF2 + multicopter position, attitude and rate control) flying frontier-fly's own rigid body in
 * lockstep (src/px4/px4Dynamics.ts): each drone boots, arms and takes off at the entry with GNSS,
 * then the fix is cut and PX4 flies the whole search on external vision (VIO-grade noise and drift
 * on the true pose). All arms fly the same buildings and seeds.
 *
 * Writes DIR/<arm>-<cond>-<seed>-<n>.json per episode and DIR/summary.json. --film SEED:DRONES
 * also writes DIR/film-<cond>-<seed>-<n>.json, every 100 ms of the px4 run, for the replay page.
 */
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { teamOptions, teamBenchSeeds, TEAM_BENCH_TASKS } from '../../src/bench/team';
import type { V3 } from '../../src/mission/autonomy/contracts';
import { PX4_REST_M, type VisionSource } from '../../src/mission/autonomy/px4Vehicle';
import { Px4Dynamics } from '../../src/px4/px4Dynamics';
import { px4Mode } from '../../src/px4/missionLink';
import { TEAM_CONDITIONS, teamEpisodeSetup } from '../../src/team/conditions';
import { quadPilotOptions, QUAD_PICKER_BRAKING } from '../../src/sim/quadPilot';
import { runTeam, TeamSim, type TeamResult } from '../../src/team/run';
import { bootPx4, type Px4Instance } from './link';

const args = process.argv.slice(2);
const opt = (name: string): string | undefined => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const all = (name: string): string[] => args.flatMap((a, i) => (a === `--${name}` && args[i + 1] ? [args[i + 1]!] : []));
const outDir = resolve(opt('dir') ?? 'test-results/px4-team');
const condId = opt('cond') ?? 'hard';
const cond = TEAM_CONDITIONS.find((c) => c.id === condId);
if (!cond) throw new Error(`unknown condition ${condId}`);
const task = TEAM_BENCH_TASKS.find((t) => t.id === condId)!;
const split = (opt('split') ?? 'dev') as 'dev' | 'test';
const seeds = opt('seeds') ? opt('seeds')!.split(',').map(Number) : teamBenchSeeds(split, task).slice(0, Number(opt('count') ?? 10));
const sizes = (opt('drones') ?? '1,2,3,4').split(',').map(Number);
const arms = (opt('arms') ?? 'px4,quad,kinematic').split(',');
const films = new Set(all('film'));
const px4Build = process.env.PX4_BUILD;
/** First PX4 instance number: two runs side by side need disjoint ranges (ports 4560+N, 14540+N, 14580+N). */
const instanceBase = Number(opt('instance-base') ?? 0);
const STEP_S = 0.05;

/** VIO-grade external vision: the true pose plus a slow random-walk drift and white noise (as scripts/px4/mission.ts). */
function visionModel(seed: number): VisionSource {
  let s = seed >>> 0 || 1;
  const uni = () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  const gauss = () => Math.sqrt(-2 * Math.log(Math.max(1e-12, uni()))) * Math.cos(2 * Math.PI * uni());
  const drift: V3 = [0, 0, 0];
  let lastT = 0;
  const DRIFT = 0.02;
  const POS_SIGMA = 0.03;
  const VEL_SIGMA = 0.05;
  return (truth) => {
    const dt = Math.max(0, truth.tSec - lastT);
    lastT = truth.tSec;
    for (let i = 0; i < 3; i++) drift[i] = drift[i]! + gauss() * DRIFT * Math.sqrt(dt);
    return {
      pos: [0, 1, 2].map((i) => truth.pos[i]! + drift[i]! + gauss() * POS_SIGMA) as V3,
      vel: [0, 1, 2].map((i) => truth.vel[i]! + gauss() * VEL_SIGMA) as V3,
      posVar: POS_SIGMA ** 2 + 0.05 ** 2,
      velVar: VEL_SIGMA ** 2,
      reset: 0,
    };
  };
}

const enu = (n: readonly number[]): V3 => [n[1]!, n[0]!, -n[2]!];
const ned = (e: readonly number[]): V3 => [e[1]!, e[0]!, -e[2]!];
const wrap = (a: number) => a - Math.round(a / (Math.PI * 2)) * Math.PI * 2;

/**
 * Climb from the floor to the spawn height and settle there, facing +x as the kinematic drone
 * starts. GNSS is on (the drone is at the entry) and vision is fed so EKF2 has both before the cut.
 */
function takeoff(p: Px4Instance, origin: V3, targetZ: number, vision: VisionSource): { sec: number } {
  let t = 0;
  let settled = 0;
  for (let i = 0; i < 600; i++) {
    const s = p.link.state();
    const pos = enu(s.posNed);
    const z = origin[2] + pos[2];
    const vz = Math.max(-0.8, Math.min(0.8, 1.5 * (targetZ - z)));
    const [w, x, y, zq] = s.q;
    const yaw = Math.PI / 2 - Math.atan2(2 * (w * zq + x * y), 1 - 2 * (y * y + zq * zq));
    const rate = Math.max(-1, Math.min(1, 1.5 * wrap(0 - yaw)));
    const world: V3 = [origin[0] + pos[0], origin[1] + pos[1], z];
    const v = vision({ pos: world, vel: enu(s.velNed), tSec: t });
    p.link.advance({
      dt: STEP_S,
      velNed: ned([0, 0, vz]),
      yawRateDown: -rate,
      windNed: [0, 0, 0],
      teleport: null,
      thrust: [1, 1, 1, 1],
      gpsFix: true,
      ev: v ? { posNed: ned([v.pos[0] - origin[0], v.pos[1] - origin[1], v.pos[2] - origin[2]]), velNed: ned(v.vel), posVar: v.posVar, velVar: v.velVar, reset: v.reset } : null,
    });
    p.drain();
    t += STEP_S;
    const after = p.link.state();
    const speed = Math.hypot(...after.velNed);
    if (Math.abs(targetZ - (origin[2] - after.posNed[2])) < 0.05 && speed < 0.1 && Math.abs(wrap(yaw)) < 0.05) settled++;
    else settled = 0;
    if (settled >= 20) return { sec: t };
  }
  throw new Error(`instance ${p.instance}: did not settle at ${targetZ} m in 30 s`);
}

interface DroneStats {
  ekfErrRmsM: number;
  ekfErrMaxM: number;
  trackErrRmsMps: number;
  wallStops: number;
  lockstepTimeouts: number;
  offboardDrops: number;
  bootWallSec: number;
  takeoffSec: number;
  finalMode: string;
}

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
  overlap: number;
  steps: number;
  wallSec: number;
  px4?: DroneStats[];
  failed?: string;
}

function episodeOf(arm: string, seed: number, n: number, r: TeamResult, wallSec: number): Episode {
  return {
    arm,
    cond: condId,
    seed,
    drones: n,
    coverage: +r.coverage.toFixed(4),
    t50: r.timeTo['50'] === null ? null : +(r.timeTo['50'] * STEP_S).toFixed(2),
    t80: r.timeTo['80'] === null ? null : +(r.timeTo['80'] * STEP_S).toFixed(2),
    baseKnown: +r.baseKnown.toFixed(4),
    collisions: r.collisions,
    crashes: r.crashes,
    droneContacts: r.droneContacts,
    overlap: +r.overlap.toFixed(3),
    steps: r.steps,
    wallSec: +wallSec.toFixed(1),
  };
}

const r3 = (x: number) => Math.round(x * 1000) / 1000;

async function flyPx4(seed: number, n: number): Promise<Episode> {
  const s = teamEpisodeSetup(cond!, seed);
  const spec = s.spec;
  const W = spec.floors[0]!.w * spec.cellSize;
  const D = spec.floors[0]!.d * spec.cellSize;
  const spawn = spec.spawn;
  const origin: V3 = [spawn[0], spawn[1], PX4_REST_M];
  const area = { x0: -20 - spawn[0], y0: -20 - spawn[1], w: W + 40, d: D + 40 };
  const wall0 = Date.now();
  const insts = await Promise.all(Array.from({ length: n }, (_, i) => bootPx4({ instance: instanceBase + i, px4Build: px4Build!, seed: seed * 10 + i, area })));
  const visions = insts.map((_, i) => visionModel((seed ^ 0x5eed) + i * 101));
  try {
    const takeoffs = insts.map((p, i) => takeoff(p, origin, spawn[2], visions[i]!));
    const dyns = insts.map((p, i) => new Px4Dynamics(p.link, p.link.state(), { origin, vision: visions[i] }));
    const film = films.has(`${seed}:${n}`);
    const sim = await TeamSim.create(spec, s.envSeed, s.cfg, { ...teamOptions({ name: `${n}`, drones: n }), makeDynamics: (i) => dyns[i]!, plannerOptions: () => quadPilotOptions(), separationBraking: QUAD_PICKER_BRAKING }, seed);
    const frames: number[][] = [];
    const flown: number[] = [];
    const prevFlown = new Uint8Array(sim.firstBy.length);
    let failed: string | undefined;
    for (let t = 0; ; t++) {
      let more = true;
      try {
        more = sim.step();
      } catch (e) {
        failed = e instanceof Error ? e.message : String(e);
        break;
      }
      for (const p of insts) p.drain();
      if (film && t % 2 === 0) {
        const v = sim.view();
        const row: number[] = [+(t * STEP_S).toFixed(2), r3(v.coverage), r3(v.baseKnown)];
        for (let i = 0; i < n; i++) {
          const d = dyns[i]!;
          const st = d.state!;
          const e = enu(d.last.ekfPosNed);
          const g = v.goal[i];
          row.push(r3(st.pos[0]), r3(st.pos[1]), r3(st.pos[2]), r3(st.yaw), r3(st.roll), r3(st.pitch), r3(origin[0] + e[0]), r3(origin[1] + e[1]), r3(origin[2] + e[2]));
          row.push(...d.last.motors.map(r3), v.launched[i] ? 1 : 0, v.done[i] ? 1 : 0, v.home[i] === 'return' ? 1 : 0, g ? r3(g[0]) : -1, g ? r3(g[1]) : -1);
        }
        frames.push(row);
        const fb = sim.firstBy;
        for (let k = 0; k < fb.length; k++) {
          if (fb[k] !== 0 && prevFlown[k] === 0) {
            prevFlown[k] = fb[k]!;
            flown.push(frames.length - 1, k, fb[k]! - 1);
          }
        }
      }
      if (!more) break;
    }
    const r = sim.result();
    sim.dispose();
    const ep = episodeOf('px4', seed, n, r, (Date.now() - wall0) / 1000);
    ep.px4 = dyns.map((d, i) => ({
      ekfErrRmsM: +d.ekfErrRms.toFixed(3),
      ekfErrMaxM: +d.ekfErrMax.toFixed(3),
      trackErrRmsMps: +d.trackErrRms.toFixed(3),
      wallStops: d.teleports,
      lockstepTimeouts: d.last.timeouts,
      offboardDrops: insts[i]!.events.filter((e) => e.text.startsWith('PX4 left offboard')).length,
      bootWallSec: +insts[i]!.bootWallSec.toFixed(1),
      takeoffSec: +takeoffs[i]!.sec.toFixed(1),
      finalMode: px4Mode(d.last.customMode),
    }));
    if (failed) ep.failed = failed;
    if (film) {
      writeFileSync(
        join(outDir, `film-${condId}-${seed}-${n}.json`),
        JSON.stringify({
          schema: 1,
          cond: condId,
          seed,
          drones: n,
          dt: STEP_S * 2,
          autopilot: 'PX4 v1.15.4 SITL, one instance per drone, external vision indoors',
          columns: ['t', 'coverage', 'baseKnown'],
          perDrone: ['x', 'y', 'z', 'yaw', 'roll', 'pitch', 'ekfX', 'ekfY', 'ekfZ', 'm0', 'm1', 'm2', 'm3', 'launched', 'done', 'returning', 'goalX', 'goalY'],
          frames,
          flown,
          result: ep,
        }),
      );
    }
    return ep;
  } finally {
    await Promise.all(insts.map((p) => p.close()));
  }
}

async function main(): Promise<void> {
  mkdirSync(outDir, { recursive: true });
  if (arms.includes('px4') && !px4Build) throw new Error('set PX4_BUILD to the px4_sitl_default build directory');
  const summaryPath = join(outDir, 'summary.json');
  const done: Episode[] = existsSync(summaryPath) ? (JSON.parse(readFileSync(summaryPath, 'utf8')) as { episodes: Episode[] }).episodes : [];
  const have = new Set(done.map((e) => `${e.arm}-${e.cond}-${e.seed}-${e.drones}`));
  for (const seed of seeds) {
    for (const n of sizes) {
      for (const arm of arms) {
        const key = `${arm}-${condId}-${seed}-${n}`;
        if (have.has(key) && !(arm === 'px4' && films.has(`${seed}:${n}`) && !existsSync(join(outDir, `film-${condId}-${seed}-${n}.json`)))) continue;
        let ep: Episode;
        if (arm === 'kinematic' || arm === 'quad') {
          const t0 = Date.now();
          const s = teamEpisodeSetup(cond!, seed);
          const base = teamOptions({ name: `${n}`, drones: n });
          if (arm === 'quad') s.cfg.dynamics = 'quad';
          const r = await runTeam(s.spec, s.envSeed, s.cfg, arm === 'quad' ? { ...base, plannerOptions: () => quadPilotOptions(), separationBraking: QUAD_PICKER_BRAKING } : base, seed);
          ep = episodeOf(arm, seed, n, r, (Date.now() - t0) / 1000);
        } else if (arm === 'px4') ep = await flyPx4(seed, n);
        else throw new Error(`unknown arm ${arm}`);
        writeFileSync(join(outDir, `${key}.json`), `${JSON.stringify(ep)}\n`);
        const i = done.findIndex((e) => `${e.arm}-${e.cond}-${e.seed}-${e.drones}` === key);
        if (i >= 0) done[i] = ep;
        else done.push(ep);
        have.add(key);
        writeFileSync(summaryPath, `${JSON.stringify({ schema: 1, episodes: done }, null, 1)}\n`);
        const px = ep.px4 ? ` ekf ${Math.max(...ep.px4.map((d) => d.ekfErrRmsM)).toFixed(2)} m rms, walls ${ep.px4.reduce((a, d) => a + d.wallStops, 0)}, drops ${ep.px4.reduce((a, d) => a + d.offboardDrops, 0)}` : '';
        console.log(`${key}: coverage ${(ep.coverage * 100).toFixed(1)}% t80 ${ep.t80 ?? '-'} s contacts ${ep.droneContacts} collisions ${ep.collisions}${px} (${ep.wallSec} s wall)${ep.failed ? ` FAILED ${ep.failed}` : ''}`);
      }
    }
  }
}

void main().then(
  () => process.exit(0),
  (e) => {
    console.error(e);
    process.exit(1);
  },
);
