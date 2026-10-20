// Failure gallery for docs/REPORT.md: re-flies chosen episodes of the coverage study and draws each
// one top-down as an SVG (walls, shafts, furniture, flown voxels, the track, collisions).
//
//   vite-node scripts/study/gallery.ts            # reads docs/study.json, writes docs/report/
//
// The episodes are picked by rule from docs/study.json, not by hand, so a re-run of the study
// re-picks them.
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { CELL } from '../../src/shared/constants';
import type { BuildingSpec } from '../../src/shared/types';
import { loadPolicy, startEpisode, type Job, type Result } from './coverage-study';

interface Pick {
  id: string;
  title: string;
  why: string;
  job: Job;
}

interface Flown {
  track: [number, number, number][];
  hits: [number, number, number][];
  coverage: number;
  collisions: number;
  visited: Uint8Array;
}

const OUT = 'docs/report';
const study = JSON.parse(readFileSync('docs/study.json', 'utf8')) as { checkpoint: string; episodes: Result[] };
const policy = loadPolicy(study.checkpoint);

const where = (condition: string, pilot: string, lighting = 'day') =>
  study.episodes.filter((e) => e.condition === condition && e.pilot === pilot && e.lighting === lighting);
const worst = (rows: Result[], by: (r: Result) => number) => rows.reduce((a, b) => (by(b) > by(a) ? b : a));
const asJob = (r: Result, pilot = r.pilot): Job => ({ condition: r.condition, stage: r.stage, seed: r.seed, lighting: r.lighting, quad: r.quad, pilot });

const policyHits = worst(where('hard', 'policy'), (r) => r.collisions);
const plannerLow = worst(where('hard', 'classical'), (r) => -r.coverage);
const quadHits = worst(where('quad', 'classical'), (r) => r.collisions);
const floorsTeacher = worst(where('floors', 'teacher'), (r) => r.floors * 10 - r.coverage);

const picks: Pick[] = [
  {
    id: 'policy-collisions',
    title: 'Policy alone, most collisions on a hard building',
    why: 'The policy imitates a teacher that never collides and was never trained on the states just before a contact, so it brushes furniture and door jambs.',
    job: asJob(policyHits),
  },
  {
    id: 'policy-guarded',
    title: 'Same building, policy under the lidar guard',
    why: 'The guard filters the same actions through the lidar ring and holds 1.15 m above the floor, between the furniture heights.',
    job: asJob(policyHits, 'guarded'),
  },
  {
    id: 'planner-lowest',
    title: 'Sensor-only planner, lowest coverage on a hard building',
    why: 'The planner only routes through voxels its lidar has seen free. Rooms behind a doorway it never looked straight through stay unknown until the episode budget runs out.',
    job: asJob(plannerLow),
  },
  {
    id: 'quad-collisions',
    title: 'Sensor-only planner on quadrotor dynamics, most collisions',
    why: 'Motor lag, latency and overshoot make the kinematic braking distance wrong; the filter that is collision-free on the kinematic model is not on the quad.',
    job: asJob(quadHits),
  },
  {
    id: 'floors-teacher',
    title: 'Oracle teacher on a two-floor building, ground floor only',
    why: 'No pilot in the coverage env climbs: the teacher, the policy and the planner all chase ground-floor frontiers. Only the mission explorer uses shafts.',
    job: asJob(floorsTeacher),
  },
];

async function fly(job: Job): Promise<Flown> {
  const ep = await startEpisode(job, policy);
  let obs = ep.obs;
  const track: [number, number, number][] = [];
  const hits: [number, number, number][] = [];
  let coverage = 0;
  let collisions = 0;
  for (let t = 0; t < ep.cfg.maxSteps; t++) {
    const step = ep.env.step(ep.act(obs, ep.env));
    obs = step.obs;
    const p = ep.env.snapshot().pos;
    if (t % 4 === 0) track.push([p[0], p[1], p[2]]);
    if (step.info.collisions > collisions) hits.push([p[0], p[1], p[2]]);
    collisions = step.info.collisions;
    coverage = step.info.coverage;
    if (step.done) break;
  }
  const visited = ep.env.visitedGrid.slice();
  ep.env.dispose();
  return { track, hits, coverage, collisions, visited };
}

function svg(spec: BuildingSpec, flown: Flown): string {
  const f0 = spec.floors[0]!;
  const S = 22;
  const W = f0.w * spec.cellSize * S;
  const D = f0.d * spec.cellSize * S;
  const y = (v: number) => D - v * S;
  const parts: string[] = [];
  const vw = f0.w * 2;
  // Flown voxels on the ground floor.
  for (let k = 0; k < vw * f0.d * 2; k++) {
    if (!flown.visited[k]) continue;
    const vx = k % vw;
    const vy = Math.floor(k / vw);
    parts.push(`<rect x="${vx * 0.5 * S}" y="${y((vy + 1) * 0.5)}" width="${0.5 * S}" height="${0.5 * S}" fill="#cfe3f7"/>`);
  }
  for (let z = 0; z < f0.d; z++) {
    for (let x = 0; x < f0.w; x++) {
      const c = f0.cells[z * f0.w + x];
      const fill = c === CELL.WALL ? '#3b3f46' : c === CELL.SHAFT ? '#d08a3a' : null;
      if (fill) parts.push(`<rect x="${x * S}" y="${y(z + 1)}" width="${S}" height="${S}" fill="${fill}"/>`);
    }
  }
  for (const b of spec.furniture?.boxes ?? []) {
    if (b.floor !== 0) continue;
    const c = Math.cos(b.yaw);
    const s = Math.sin(b.yaw);
    const pts = [
      [-b.hx, -b.hy],
      [b.hx, -b.hy],
      [b.hx, b.hy],
      [-b.hx, b.hy],
    ].map(([u, v]) => `${((b.cx + c * u! - s * v!) * S).toFixed(1)},${y(b.cy + s * u! + c * v!).toFixed(1)}`);
    parts.push(`<polygon points="${pts.join(' ')}" fill="#a7a9ad" opacity="0.8"/>`);
  }
  const ground = flown.track.filter((p) => p[2] < spec.floorHeight);
  parts.push(
    `<polyline points="${ground.map((p) => `${(p[0] * S).toFixed(1)},${y(p[1]).toFixed(1)}`).join(' ')}" fill="none" stroke="#1f6fd1" stroke-width="2" stroke-linejoin="round"/>`,
  );
  const start = flown.track[0];
  if (start) parts.push(`<circle cx="${start[0] * S}" cy="${y(start[1])}" r="6" fill="#2a9d55"/>`);
  for (const h of flown.hits) parts.push(`<circle cx="${(h[0] * S).toFixed(1)}" cy="${y(h[1]).toFixed(1)}" r="5" fill="#d93a3a" stroke="#fff" stroke-width="1.5"/>`);
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${D}" width="${W}" height="${D}"><rect width="${W}" height="${D}" fill="#ffffff"/>${parts.join('')}</svg>\n`;
}

mkdirSync(OUT, { recursive: true });
const entries = [];
for (const pick of picks) {
  const ep = await startEpisode(pick.job, policy);
  const spec = ep.spec;
  ep.env.dispose();
  const flown = await fly(pick.job);
  writeFileSync(`${OUT}/${pick.id}.svg`, svg(spec, flown));
  entries.push({
    id: pick.id,
    title: pick.title,
    why: pick.why,
    job: pick.job,
    coverage: flown.coverage,
    collisions: flown.collisions,
    floors: spec.floors.length,
  });
  console.log(`${pick.id}: ${pick.job.condition} seed ${pick.job.seed} ${pick.job.pilot}, coverage ${(flown.coverage * 100).toFixed(1)}%, ${flown.collisions} collisions`);
}
writeFileSync(`${OUT}/gallery.json`, `${JSON.stringify(entries, null, 2)}\n`);
