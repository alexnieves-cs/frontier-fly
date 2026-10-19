import { describe, expect, it } from 'vitest';
import raw from '../fixtures/simpleRoom.json';
import { generateBuilding } from '../../src/gen/building';
import { isWalkable } from '../../src/gen/validate';
import { checkpointMismatch, parseCheckpoint } from '../../src/rl/checkpoint';
import { Curriculum } from '../../src/rl/curriculum';
import { InferPolicy } from '../../src/rl/infer';
import { MAP_SIZE, OBS, OBS_DIM, OBS_VERSION } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import { buildingFromJson } from '../../src/shared/serialize';
import type { BuildingSpec } from '../../src/shared/types';
import { createEnv, defaultEpisodeConfig } from '../../src/sim/env';
import { frontierAction } from '../../src/sim/frontier';
import { FrontierNav } from '../../src/sim/frontierFeatures';
import { heuristicAction } from '../../src/sim/heuristic';
import { lightingPreset } from '../../src/sim/lighting';
import { EgoMaps } from '../../src/sim/seenMap';

function stage3(seed: number): BuildingSpec {
  return generateBuilding(new Curriculum().genParams(3, new RNG(seed, 'eval-3')));
}

function expectSaneObs(obs: Float32Array): void {
  expect(obs).toHaveLength(OBS_DIM);
  for (let i = 0; i < obs.length; i++) {
    const v = obs[i]!;
    if (!Number.isFinite(v)) throw new Error(`obs[${i}] is ${v}`);
    if (i >= OBS.localVisited || (i >= OBS.lidar && i < OBS.vel)) {
      if (v < 0 || v > 1) throw new Error(`obs[${i}] = ${v} outside [0, 1]`);
    } else if (i >= OBS.nav) {
      if (v < -1 - 1e-6 || v > 1 + 1e-6) throw new Error(`obs[${i}] = ${v} outside [-1, 1]`);
    } else if (Math.abs(v) > 2.5) {
      throw new Error(`obs[${i}] = ${v} is badly scaled`);
    }
  }
  for (const flag of [OBS.hasFrontier, OBS.bfsTruncated]) expect([0, 1]).toContain(obs[flag]);
  if (obs[OBS.hasFrontier] === 1) {
    expect(Math.hypot(obs[OBS.wpDir]!, obs[OBS.wpDir + 1]!)).toBeCloseTo(1, 4);
  }
}

describe('observation v2', () => {
  it('has the versioned layout', () => {
    expect(OBS_VERSION).toBe(2);
    expect(OBS_DIM).toBe(OBS.end);
    expect(OBS_DIM).toBeLessThanOrEqual(1100);
    expect(OBS.localFrontier - OBS.localVisited).toBe(MAP_SIZE * MAP_SIZE);
    expect(OBS.coarseFrontier - OBS.coarseVisited).toBe(MAP_SIZE * MAP_SIZE);
    // Lidar-only code still reads v1 slots 0–47.
    expect([OBS.lidar, OBS.tilt, OBS.alt, OBS.ceil]).toEqual([0, 32, 46, 47]);
  });

  it('keeps every value finite and in range on a stage-3 flight', async () => {
    const env = await createEnv();
    const spec = stage3(4242);
    let obs = env.reset(spec, 3);
    expectSaneObs(obs);
    let sawFrontier = false;
    for (let t = 0; t < 300; t++) {
      const pose = env.snapshot();
      const step = env.step(t % 2 ? heuristicAction(obs) : frontierAction(spec, env.visitedGrid, pose.pos, pose.yaw));
      obs = step.obs;
      expectSaneObs(obs);
      if (obs[OBS.hasFrontier] === 1) sawFrontier = true;
      if (step.done) break;
    }
    expect(sawFrontier).toBe(true);
    const probe = env.observeAt(spec.spawn[0], spec.spawn[1], spec.spawn[2], 1);
    expectSaneObs(probe);
  });

  it('marks seen-free voxels only outside walls, including with glass and night dropouts', async () => {
    for (const [spec, lighting] of [
      [buildingFromJson(raw), 'day'],
      [stage3(5150), 'day'],
      [stage3(6160), 'night-dark'],
    ] as const) {
      const env = await createEnv();
      const nRooms = spec.floors.reduce((n, f) => n + f.rooms.length, 0);
      const cfg = defaultEpisodeConfig(3, nRooms);
      cfg.lighting = lightingPreset(lighting, nRooms);
      cfg.maxSteps = 400;
      let obs = env.reset(spec, 9, cfg);
      for (let t = 0; t < 400; t++) {
        const pose = env.snapshot();
        const step = env.step(t % 3 ? frontierAction(spec, env.visitedGrid, pose.pos, pose.yaw) : heuristicAction(obs));
        obs = step.obs;
        if (step.done) break;
      }
      const seen = env.seenGrid;
      const vw = spec.floors[0]!.w * 2;
      const vd = spec.floors[0]!.d * 2;
      let marked = 0;
      for (let f = 0; f < spec.floors.length; f++) {
        const floor = spec.floors[f]!;
        for (let vy = 0; vy < vd; vy++) {
          for (let vx = 0; vx < vw; vx++) {
            if (!seen[f * vw * vd + vy * vw + vx]) continue;
            marked += 1;
            const cell = floor.cells[(vy >> 1) * floor.w + (vx >> 1)]!;
            if (!isWalkable(cell)) throw new Error(`seen voxel (${vx}, ${vy}) on floor ${f} is inside a wall`);
          }
        }
      }
      expect(marked).toBeGreaterThan(40);
    }
  });

  it('resets the seen map per episode', async () => {
    const env = await createEnv();
    const spec = buildingFromJson(raw);
    env.reset(spec, 1);
    for (let t = 0; t < 30; t++) env.step(new Float32Array([0.5, 0, 0, 0.6]));
    const after = env.seenGrid.reduce((s, v) => s + v, 0);
    env.reset(spec, 1);
    const fresh = env.seenGrid.reduce((s, v) => s + v, 0);
    expect(fresh).toBeLessThan(after);
    expect(fresh).toBeGreaterThan(0);
  });

  it('aggregates 4x4 voxel blocks into the coarse map and rotates with yaw', () => {
    const vw = 48;
    const vd = 40;
    const rng = new RNG(7);
    const visited = new Uint8Array(vw * vd);
    const seen = new Uint8Array(vw * vd);
    for (let i = 0; i < visited.length; i++) {
      visited[i] = rng.bool(0.3) ? 1 : 0;
      seen[i] = visited[i] || rng.bool(0.5) ? 1 : 0;
    }
    const maps = new EgoMaps();
    const a = new Float32Array(OBS_DIM);
    // Drone on a voxel corner: (20, 16) in voxels.
    const x = 10;
    const y = 8;
    maps.write(a, OBS, visited, seen, vw, vd, 0, 1, x, y, 0);
    for (let iz = 0; iz < MAP_SIZE; iz++) {
      for (let ix = 0; ix < MAP_SIZE; ix++) {
        const x0 = 20 + (ix - 8) * 4;
        const y0 = 16 + (iz - 8) * 4;
        let v = 0;
        let fr = 0;
        for (let sy = 0; sy < 4; sy++) {
          for (let sx = 0; sx < 4; sx++) {
            const vx = x0 + sx;
            const vy = y0 + sy;
            if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) continue;
            const k = vy * vw + vx;
            if (visited[k]) v += 1;
            else if (seen[k]) fr += 1;
          }
        }
        const cell = iz * MAP_SIZE + ix;
        expect(a[OBS.coarseVisited + cell]).toBeCloseTo(v / 16, 6);
        expect(a[OBS.coarseFrontier + cell]).toBeCloseTo(fr / 16, 6);
      }
    }
    // Local cells cover single voxels; the cell at body (+0.25, +0.25) m is voxel (20, 16).
    expect(a[OBS.localVisited + 8 * MAP_SIZE + 8]).toBe(visited[16 * vw + 20]);
    // Yaw +90°: body forward is world +y, so body cell (ix, iz) is yaw-0 cell (15 - iz, ix).
    const b = new Float32Array(OBS_DIM);
    maps.write(b, OBS, visited, seen, vw, vd, 0, 1, x, y, Math.PI / 2);
    for (let iz = 0; iz < MAP_SIZE; iz++) {
      for (let ix = 0; ix < MAP_SIZE; ix++) {
        const src = ix * MAP_SIZE + (MAP_SIZE - 1 - iz);
        expect(b[OBS.coarseVisited + iz * MAP_SIZE + ix]).toBeCloseTo(a[OBS.coarseVisited + src]!, 6);
        expect(b[OBS.coarseFrontier + iz * MAP_SIZE + ix]).toBeCloseTo(a[OBS.coarseFrontier + src]!, 6);
      }
    }
  });

  it('points the BFS waypoint along a corridor toward unexplored space, not through unknown', () => {
    const vw = 40;
    const vd = 12;
    const visited = new Uint8Array(vw * vd);
    const seen = new Uint8Array(vw * vd);
    // Corridor rows 4–5 seen end to end; the west half (x < 20) already flown.
    for (const vy of [4, 5]) {
      for (let vx = 0; vx < vw; vx++) {
        seen[vy * vw + vx] = 1;
        if (vx < 20) visited[vy * vw + vx] = 1;
      }
    }
    // A seen pocket just south of the drone, but behind unknown voxels (rows 6–7 unseen).
    seen[9 * vw + 9] = 1;
    seen[9 * vw + 10] = 1;
    const nav = new FrontierNav();
    const out = new Float32Array(OBS_DIM);
    const x = 10 * 0.5 + 0.25;
    const y = 4 * 0.5 + 0.25;
    nav.write(out, visited, seen, vw, vd, 0, 1, x, y, 0, 0);
    expect(out[OBS.hasFrontier]).toBe(1);
    expect(out[OBS.wpDir]).toBeGreaterThan(0.95);
    expect(Math.abs(out[OBS.wpDir + 1]!)).toBeLessThan(0.3);
    expect(nav.last.tx).toBeCloseTo(20.5 * 0.5, 6);
    expect(out[OBS.pathLen]!).toBeCloseTo(nav.last.pathMeters / 20, 6);
    expect(nav.last.pathMeters).toBeGreaterThan(4.5);
    // Facing west, the same waypoint is behind the drone.
    nav.write(out, visited, seen, vw, vd, 0, 1, x, y, Math.PI, 0);
    expect(out[OBS.wpDir]).toBeLessThan(-0.95);
    // The largest frontier cluster is the east corridor as well.
    expect(out[OBS.clusterDir]).toBeLessThan(-0.9);
    // Diagonal corner cutting: a frontier reachable only by squeezing past two unknown voxels is not reachable.
    const v2 = new Uint8Array(9);
    const s2 = new Uint8Array(9);
    v2[0] = 1;
    s2[0] = 1;
    s2[4] = 1; // (1, 1) diagonal from (0, 0); (1, 0) and (0, 1) unknown
    nav.write(out, v2, s2, 3, 3, 0, 1, 0.25, 0.25, 0, 0);
    expect(out[OBS.hasFrontier]).toBe(0);
  });

  it('reports no frontier when everything seen is visited', () => {
    const vw = 20;
    const vd = 20;
    const visited = new Uint8Array(vw * vd);
    for (let vy = 5; vy < 15; vy++) for (let vx = 5; vx < 15; vx++) visited[vy * vw + vx] = 1;
    const seen = visited.slice();
    const nav = new FrontierNav();
    const out = new Float32Array(OBS_DIM).fill(0.5);
    nav.write(out, visited, seen, vw, vd, 0, 1, 5, 5, 0.3, 120);
    expect(out[OBS.hasFrontier]).toBe(0);
    expect(out[OBS.wpDir]).toBe(0);
    expect(out[OBS.wpDir + 1]).toBe(0);
    expect(out[OBS.pathLen]).toBe(1);
    expect(out[OBS.frontierTotal]).toBe(0);
    expect(out[OBS.clusterMass]).toBe(0);
    expect(out[OBS.stall]).toBe(1);
    expect(out[OBS.bfsTruncated]).toBe(0);
  });

  it('parses pre-v2 checkpoints as obsVersion 1 and refuses them with a reason', () => {
    const policy = new InferPolicy(306, 4, [256, 128], 1);
    const json = JSON.stringify({
      version: 1,
      obsDim: 306,
      actDim: 4,
      hidden: [256, 128],
      weights: policy.getWeights().map((buf) => Array.from(new Float32Array(buf))),
      obsMean: [],
      obsVar: [],
      obsCount: 1,
      rewVar: 1,
      stage: 3,
    });
    const old = parseCheckpoint(json);
    expect(old.obsVersion).toBe(1);
    expect(checkpointMismatch(old)).toMatch(/v1 \(306 inputs\).*v2 \(1088 inputs\)/);
    expect(checkpointMismatch({ obsDim: OBS_DIM, obsVersion: OBS_VERSION })).toBeNull();
  });

  it('runs batched inference on the wide input without corrupting the gemm scratch', () => {
    const n = 70;
    const obs = new Float32Array(n * OBS_DIM);
    const rng = new RNG(2);
    for (let i = 0; i < obs.length; i++) obs[i] = rng.next();
    // Reference rows from a separate, never-batched policy with the same seed:
    // an overflowing scratch would silently overwrite the batched policy's weights.
    const reference = new InferPolicy(OBS_DIM, 4, [256, 128], 3);
    const rows = [0, 33, 63, 64, 69];
    const single = rows.map((i) => reference.actWith(obs.subarray(i * OBS_DIM, (i + 1) * OBS_DIM), 1, new RNG(1), true));
    const policy = new InferPolicy(OBS_DIM, 4, [256, 128], 3);
    policy.actWith(obs, n, new RNG(1), true);
    const batch = policy.actWith(obs, n, new RNG(1), true);
    rows.forEach((i, r) => {
      const one = single[r]!;
      for (let j = 0; j < 4; j++) expect(batch.action[i * 4 + j]).toBeCloseTo(one.action[j]!, 4);
      expect(batch.value[i]).toBeCloseTo(one.value[0]!, 3);
    });
  });
});
