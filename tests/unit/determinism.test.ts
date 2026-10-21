import { describe, expect, it } from 'vitest';
import { isWalkable } from '../../src/gen/validate';
import { OBS_DIM, VOXEL } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import type { BuildingSpec } from '../../src/shared/types';
import { createEnv, ExplorerEnv } from '../../src/sim/env';
import { frontierDistance } from '../../src/sim/frontier';
import { initPhysics, PhysicsWorld } from '../../src/sim/world';
import { runTrace, traceSpec, TRACE_CASES } from '../support/envTrace';

/**
 * Trace hashes recorded on the code before the throughput work (frontier ring
 * search, env allocation fixes, pooled physics), by bundling
 * tests/support/printTraceHashes.ts against that checkout: 306 on the original
 * observation, 1088 on observation v2 + solid furniture (branch
 * claude/indoor-policy-generalize-w4bvyl at d62115d), re-recorded after the stuck/loiter/flip
 * reward penalties were added. They pin observations, rewards, done flags and coverage bit for bit.
 *
 * Valid only for the observation/reward definition they were recorded with
 * (keyed by OBS_DIM; the reward is not part of the key). After an intentional
 * observation or reward change, run `npx vite-node tests/support/printTraceHashes.ts`
 * and paste the new values, after checking a perf-only change does not move them.
 * The 1088 quad hash was re-recorded when the flight controller's rate cap went from 8 to
 * 4 rad/s (the other cases do not fly the quad and are unchanged), and the 1088 multifloor hash
 * when slabs on buildings with three or more floors stopped opening over shafts that lead up
 * from the floor above (gen/parts.ts, sim/world.ts). The 306 table predates both.
 */
const GOLDEN_BY_OBS_DIM: Record<number, Record<string, string>> = {
  306: {
    'stage0-kinematic': '8987d8b4',
    'stage2-kinematic': '6685ea79',
    'stage3-noise-latency': '562f291a',
    'stage4-multifloor': 'cad600fa',
    'stage4-quad': '8205deb1',
  },
  1088: {
    'stage0-kinematic': 'a8fcf8ba',
    'stage2-kinematic': 'efb78622',
    'stage3-noise-latency': '49221eb8',
    'stage4-multifloor': '7a7b7515',
    'stage4-quad': '761a5c35',
  },
};
const GOLDEN = GOLDEN_BY_OBS_DIM[OBS_DIM];

/** Verbatim copy of the original nearestOpen scan (before the ring search). */
function referenceNearestOpen(spec: BuildingSpec, visited: Uint8Array, pos: [number, number, number], radius: number): number {
  const vw = spec.floors[0]!.w * 2;
  const vd = spec.floors[0]!.d * 2;
  const cs = spec.cellSize;
  let best = 6;
  for (let f = 0; f < spec.floors.length; f++) {
    const floor = spec.floors[f]!;
    const gx = Math.floor(pos[0] / cs);
    const gz = Math.floor(pos[1] / cs);
    const z0 = Math.max(0, gz - radius);
    const z1 = Math.min(floor.d - 1, gz + radius);
    const x0 = Math.max(0, gx - radius);
    const x1 = Math.min(floor.w - 1, gx + radius);
    for (let z = z0; z <= z1; z++) {
      for (let x = x0; x <= x1; x++) {
        if (!isWalkable(floor.cells[z * floor.w + x]!)) continue;
        for (let dz = 0; dz < 2; dz++) {
          for (let dx = 0; dx < 2; dx++) {
            const vx = x * 2 + dx;
            const vy = z * 2 + dz;
            if (visited[f * vw * vd + vy * vw + vx]) continue;
            const cx = (vx + 0.5) * VOXEL;
            const cy = (vy + 0.5) * VOXEL;
            const dist = Math.hypot(cx - pos[0], cy - pos[1]);
            if (dist < best) best = dist;
            if (best < 0.4) return best;
          }
        }
      }
    }
  }
  return best;
}

function referenceFrontierDistance(spec: BuildingSpec, visited: Uint8Array, pos: [number, number, number]): number {
  const local = referenceNearestOpen(spec, visited, pos, 4);
  if (local < 6) return local;
  return referenceNearestOpen(spec, visited, pos, 64);
}

describe('bit-identical env after the throughput work', () => {
  it.skipIf(!GOLDEN)('matches the trace hashes recorded before the change', async () => {
    for (const c of TRACE_CASES) {
      const env = await createEnv();
      expect([c.name, runTrace(env, c).hash]).toEqual([c.name, GOLDEN![c.name]]);
      env.dispose();
    }
  });

  it('traces terminate and reset, so the goldens cover episode boundaries', async () => {
    const env = await createEnv();
    const record = runTrace(env, TRACE_CASES[1]!, { keep: true });
    expect(record.dones.filter(Boolean).length).toBeGreaterThan(0);
    env.dispose();
  });

  it('a reused (pooled) physics world gives the same obs/reward sequence as a fresh one', async () => {
    await initPhysics();
    for (const c of TRACE_CASES) {
      const fresh = runTrace(new ExplorerEnv(), c, { keep: true });
      const specs = new Map<number, BuildingSpec>();
      const specFor = (episode: number) => {
        if (!specs.has(episode)) specs.set(episode, traceSpec(c, episode));
        return specs.get(episode)!;
      };
      const worlds = new Map<BuildingSpec, PhysicsWorld>();
      let reused = 0;
      const physicsFor = (spec: BuildingSpec) => {
        let world = worlds.get(spec);
        if (world) reused += 1;
        else {
          world = new PhysicsWorld(spec);
          worlds.set(spec, world);
        }
        return world;
      };
      const pooledEnv = new ExplorerEnv();
      // Run 1 warms the worlds (raycasts, contacts); run 2 reuses every one of them after that history.
      runTrace(pooledEnv, c, { specFor, physicsFor });
      const pooled = runTrace(pooledEnv, c, { specFor, physicsFor, keep: true });
      expect(reused).toBeGreaterThan(0);
      expect(pooled.hash).toBe(fresh.hash);
      expect(pooled.rewards).toEqual(fresh.rewards);
      expect(pooled.obs.length).toBe(fresh.obs.length);
      for (let i = 0; i < fresh.obs.length; i++) expect(pooled.obs[i]).toEqual(fresh.obs[i]);
      pooledEnv.dispose();
      // The env borrowed the worlds, so they are still alive and ours to free.
      for (const world of worlds.values()) {
        expect(world.cast(1.5, 1.5, 1, 1, 0, 0, 8)).not.toBeUndefined();
        world.free();
      }
    }
  });

  it('frontierDistance equals the original full scan on random visited grids', () => {
    const rng = new RNG(99, 'frontier-eq');
    let checks = 0;
    let nonCap = 0;
    for (let episode = 0; episode < 6; episode++) {
      for (const c of TRACE_CASES) {
        const spec = traceSpec(c, episode);
        const floor0 = spec.floors[0]!;
        const n = spec.floors.length * floor0.w * 2 * floor0.d * 2;
        for (const density of [0, 0.3, 0.8, 0.97, 0.995, 1]) {
          const visited = new Uint8Array(n);
          for (let i = 0; i < n; i++) visited[i] = rng.next() < density ? 1 : 0;
          for (let k = 0; k < 25; k++) {
            const pos: [number, number, number] = [
              rng.range(-2, floor0.w * spec.cellSize + 2),
              rng.range(-2, floor0.d * spec.cellSize + 2),
              rng.range(0, spec.floors.length * spec.floorHeight),
            ];
            const got = frontierDistance(spec, visited, pos);
            const want = referenceFrontierDistance(spec, visited, pos);
            if (!Object.is(got, want)) expect({ pos, density, got }).toEqual({ pos, density, got: want });
            checks += 1;
            if (want < 6) nonCap += 1;
          }
        }
      }
    }
    expect(checks).toBeGreaterThan(4000);
    expect(nonCap).toBeGreaterThan(1000);
  });
});
