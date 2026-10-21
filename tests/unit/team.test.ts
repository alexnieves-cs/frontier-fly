import { beforeAll, describe, expect, it } from 'vitest';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { isFullyReachable } from '../../src/gen/validate';
import { CELL, SURFACE } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import type { BuildingSpec } from '../../src/shared/types';
import { SensorPlanner, type SensorFrame } from '../../src/sim/sensorPlanner';
import { initPhysics } from '../../src/sim/world';
import { ClaimPenalty, DEFAULT_CLAIMS } from '../../src/team/allocate';
import { generateLargeBuilding } from '../../src/team/largeBuilding';
import { exchange, MeshNode, type MeshStats } from '../../src/team/mesh';
import { basePosition, normalCdf, RadioModel } from '../../src/team/radio';
import { runTeam, separate } from '../../src/team/run';

beforeAll(async () => {
  await initPhysics();
});

/** One 20 x 5 m floor: open air, with a wall across x = 10 that has a door in rows 0-1. */
function corridor(surface: number = SURFACE.CONCRETE): BuildingSpec {
  const w = 20;
  const d = 5;
  const cells = new Uint8Array(w * d);
  const surfaces = new Uint8Array(w * d);
  for (let z = 2; z < d; z++) {
    cells[z * w + 10] = CELL.WALL;
    surfaces[z * w + 10] = surface;
  }
  return { seed: 1, cellSize: 1, floorHeight: 3, floors: [{ w, d, cells, rooms: [{ id: 0, x: 0, z: 0, w, d }], surfaces }], spawn: [2.5, 2.5, 1], reachableVoxels: 1 };
}

describe('radio', () => {
  it('loses more with distance and through walls, and nothing through a door', () => {
    const radio = new RadioModel(corridor());
    const near = radio.margin([2, 3.5, 1], [6, 3.5, 1]);
    const far = radio.margin([2, 3.5, 1], [9, 3.5, 1]);
    expect(near).toBeGreaterThan(far);
    // 6 dB per doubling of distance in free space.
    expect(radio.margin([2, 0.5, 1], [4, 0.5, 1]) - radio.margin([2, 0.5, 1], [6, 0.5, 1])).toBeCloseTo(20 * Math.log10(2), 5);
    // Same distance through the concrete wall versus through the door.
    expect(radio.wallLossDb([5, 3.5, 1], [15, 3.5, 1])).toBe(12);
    expect(radio.wallLossDb([5, 0.5, 1], [15, 0.5, 1])).toBe(0);
    expect(radio.margin([5, 0.5, 1], [15, 0.5, 1]) - radio.margin([5, 3.5, 1], [15, 3.5, 1])).toBeCloseTo(12, 5);
    // A wall is counted once however many of its cells the line grazes.
    expect(new RadioModel(corridor(SURFACE.DRYWALL)).wallLossDb([5, 2.2, 1], [15, 4.9, 1])).toBe(3);
  });

  it('delivers at the rate the shadowing predicts', () => {
    const radio = new RadioModel(corridor());
    const a = [2, 3.5, 1] as const;
    const b = [18, 3.5, 1] as const;
    const p = radio.deliveryProbability(a, b);
    expect(p).toBeGreaterThan(0.05);
    expect(p).toBeLessThan(0.95);
    const rng = new RNG(5);
    let ok = 0;
    for (let i = 0; i < 4000; i++) if (radio.deliver(a, b, rng)) ok += 1;
    expect(Math.abs(ok / 4000 - p)).toBeLessThan(0.03);
    expect(normalCdf(0)).toBeCloseTo(0.5, 6);
    expect(normalCdf(1.96)).toBeCloseTo(0.975, 3);
  });

  it('puts the base outside the wall nearest the entry', () => {
    const spec = corridor();
    const b = basePosition(spec);
    const inside = b[0] > 0 && b[1] > 0 && b[0] < 20 && b[1] < 5;
    expect(inside).toBe(false);
  });
});

describe('mesh', () => {
  const stats = (): MeshStats => ({ sent: 0, delivered: 0, bytes: 0 });
  const radio = new RadioModel(corridor(SURFACE.METAL), { shadowing: 0 });

  it('sends only what a peer has not received, and resends what was lost', () => {
    const a = new MeshNode(0, 100, 2);
    const b = new MeshNode(1, 100, 2);
    a.pos = [2, 0.5, 1];
    b.pos = [4, 0.5, 1];
    const seen = new Uint8Array(100);
    const visited = new Uint8Array(100);
    seen[3] = seen[4] = 1;
    visited[4] = 1;
    expect(a.absorbOwn(seen, visited)).toBe(3);
    expect(a.absorbOwn(seen, visited)).toBe(0);
    const s = stats();
    exchange([a, b], radio, new RNG(1), 10, 1000, s);
    expect(b.seen[3]).toBe(1);
    expect(b.visited[4]).toBe(1);
    expect(b.heard[0]!.pos).toEqual([2, 0.5, 1]);
    expect(a.pending(1, 1000).length).toBe(0);
    // Out of range (behind a metal wall, far): nothing arrives, and the entry stays queued.
    seen[7] = 1;
    a.absorbOwn(seen, visited);
    b.pos = [19, 4.5, 1];
    a.pos = [1, 4.5, 1];
    exchange([a, b], new RadioModel(corridor(SURFACE.METAL), { shadowing: 0, txPower: -40 }), new RNG(1), 20, 1000, stats());
    expect(b.seen[7]).toBe(0);
    expect(a.pending(1, 1000).length).toBe(1);
  });

  it('forwards over more than one hop', () => {
    const nodes = [0, 1, 2].map((i) => new MeshNode(i, 50, 3));
    nodes[0]!.pos = [1, 0.5, 1];
    nodes[1]!.pos = [10, 0.5, 1];
    nodes[2]!.pos = [19, 0.5, 1];
    // Weak enough that only the 9 m hops close.
    const weak = new RadioModel(corridor(), { shadowing: 0, txPower: -20 });
    expect(weak.margin(nodes[0]!.pos, nodes[1]!.pos)).toBeGreaterThan(0);
    expect(weak.margin(nodes[0]!.pos, nodes[2]!.pos)).toBeLessThan(0);
    const seen = new Uint8Array(50);
    seen[9] = 1;
    nodes[0]!.absorbOwn(seen, new Uint8Array(50));
    exchange(nodes, weak, new RNG(1), 10, 1000, stats());
    expect(nodes[1]!.seen[9]).toBe(1);
    expect(nodes[2]!.seen[9]).toBe(0);
    exchange(nodes, weak, new RNG(1), 20, 1000, stats());
    expect(nodes[2]!.seen[9]).toBe(1);
  });

  it('caps a packet and carries the rest next round', () => {
    const a = new MeshNode(0, 100, 2);
    const b = new MeshNode(1, 100, 2);
    a.pos = [2, 0.5, 1];
    b.pos = [3, 0.5, 1];
    const seen = new Uint8Array(100).fill(1);
    a.absorbOwn(seen, new Uint8Array(100));
    exchange([a, b], radio, new RNG(1), 10, 30, stats());
    expect(b.known).toBe(30);
    exchange([a, b], radio, new RNG(1), 20, 30, stats());
    expect(b.known).toBe(60);
  });
});

describe('frontier claims', () => {
  it('make frontiers near a teammate goal, or nearer a teammate, cost more', () => {
    const heard: MeshNode['heard'] = [null, { pos: [8, 1, 1.2], goal: [9, 1], at: 100 }];
    const source = { heard, selfId: 0, pos: [1, 1, 1.2] as [number, number, number], step: 120, floorHeight: 3, drones: 2 };
    const claim = new ClaimPenalty(DEFAULT_CLAIMS, source);
    claim.prepare(0, 40);
    const k = (x: number, y: number) => Math.floor(y / 0.5) * 40 + Math.floor(x / 0.5);
    expect(claim.cost(k(9, 1))).toBeGreaterThan(DEFAULT_CLAIMS.goalWeight * 0.8);
    expect(claim.cost(k(1.5, 1))).toBe(0);
    // Stale news claims nothing.
    source.step = 100 + DEFAULT_CLAIMS.maxAge + 1;
    claim.prepare(0, 40);
    expect(claim.cost(k(9, 1))).toBe(0);
  });

  it('steer the planner to the other frontier', () => {
    // A 20 x 2 voxel strip, seen free everywhere, flown in the middle: frontiers on both ends.
    const vw = 20;
    const vd = 3;
    const seen = new Uint8Array(vw * vd).fill(1);
    const visited = new Uint8Array(vw * vd);
    for (let x = 6; x <= 13; x++) for (let y = 0; y < vd; y++) visited[y * vw + x] = 1;
    const frame: SensorFrame = { seen, visited, vw, vd, floors: 1, floorHeight: 3, pos: [4.6, 0.75, 1.2], yaw: 0, lidar: new Float64Array(32).fill(8), alt: 1.2 };
    const free = new SensorPlanner();
    free.act(frame);
    // Without claims it takes the nearer end (x < 3 m).
    expect(free.goalPoint![0]).toBeLessThan(3);
    // A teammate claimed that end: it goes to the far one (x > 7 m).
    const claimed = new SensorPlanner({ penalty: { prepare: () => {}, cost: (k) => ((k % vw) * 0.5 < 3 ? 100 : 0) } });
    claimed.act(frame);
    expect(claimed.goalPoint![0]).toBeGreaterThan(7);
  });

  it('separation stops a drone closing on a teammate', () => {
    const [vx] = separate(2, 0, [0, 0, 1], [[0.5, 0, 1]]);
    expect(vx).toBeLessThanOrEqual(0.2);
    expect(separate(2, 0, [0, 0, 1], [[-0.5, 0, 1]])[0]).toBe(2);
  });
});

describe('team runs', () => {
  it('builds connected 40 m buildings deterministically', () => {
    const a = generateLargeBuilding(66001);
    const b = generateLargeBuilding(66001);
    expect(a.floors[0]!.w).toBe(40);
    expect(a.reachableVoxels).toBe(b.reachableVoxels);
    expect(a.reachableVoxels).toBeGreaterThan(2000);
    expect(isFullyReachable(a)).toBe(true);
  });

  it('is deterministic, and two drones cover more than one', async () => {
    const { spec, envSeed, cfg } = evalEpisodeSetup(3, 9101, 'day');
    cfg.maxSteps = 900;
    const solo = await runTeam(spec, envSeed, cfg, { drones: 1 }, 1);
    const duo = await runTeam(spec, envSeed, cfg, { drones: 2 }, 1);
    const again = await runTeam(spec, envSeed, cfg, { drones: 2 }, 1);
    expect(again.coverage).toBe(duo.coverage);
    expect(again.packets).toEqual(duo.packets);
    expect(duo.coverage).toBeGreaterThan(solo.coverage);
    expect(duo.perDrone.length).toBe(2);
    expect(duo.baseKnown).toBeLessThanOrEqual(duo.coverage + 1e-9);
    expect(duo.droneContacts).toBe(0);
    // Each drone draws at least hover power for its 45 s budget (~2 Wh at ~150 W), well inside a 74 Wh pack.
    expect(again.energyWh).toEqual(duo.energyWh);
    for (const wh of duo.energyWh) expect(wh).toBeGreaterThan(1);
    expect(Math.max(...duo.energyWh)).toBeLessThan(10);
  });
});
