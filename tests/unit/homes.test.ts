import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { BASELINES } from '../../src/bench/baselines';
import { benchEpisodeSetup } from '../../src/bench/run';
import { BENCH_TASKS } from '../../src/bench/tasks';
import { isFullyReachable, computeFlightMap } from '../../src/gen/validate';
import { floorsToBuilding } from '../../src/plans/convert';
import { HOME_HOLDOUT_PATH, HOME_SET_PATH, homeBuilding, homeEpisodeSetup, parseHomeSet } from '../../src/plans/homes';
import type { PlanStorey } from '../../src/plans/convert';
import { CELL } from '../../src/shared/constants';
import { createEnv } from '../../src/sim/env';
import { sensorFrame } from '../../src/sim/sensorPlanner';

const rect = (x0: number, y0: number, x1: number, y1: number) => [x0, y0, x1, y0, x1, y1, x0, y1];

/** A 6 x 4 m storey with 15 cm outer walls and, optionally, a staircase in one corner. */
function storey(stairs: boolean, door = false): PlanStorey {
  const t = 0.15;
  return {
    areas: [{ type: 'LIVING_ROOM', poly: rect(t, t, 6 + t, 4 + t) }],
    walls: [rect(0, 0, 6 + 2 * t, t), rect(0, 4 + t, 6 + 2 * t, 4 + 2 * t), rect(0, 0, t, 4 + 2 * t), rect(6 + t, 0, 6 + 2 * t, 4 + 2 * t)],
    doors: door ? [rect(2.5, 0, 3.4, t)] : [],
    windows: [],
    stairs: stairs ? [rect(4.2, 1.2, 6 + t, 3.4)] : [],
  };
}

describe('multi-storey plans', () => {
  it('open a shaft where the staircases of two storeys overlap, and only there', () => {
    const conv = floorsToBuilding([storey(true, true), storey(true)], 1)!;
    expect(conv).not.toBeNull();
    const [lo, hi] = conv.spec.floors;
    let shafts = 0;
    for (let i = 0; i < lo!.cells.length; i++) {
      expect(lo!.cells[i] === CELL.SHAFT).toBe(hi!.cells[i] === CELL.SHAFT);
      if (lo!.cells[i] === CELL.SHAFT) shafts += 1;
    }
    expect(shafts).toBeGreaterThanOrEqual(2);
    // The drone starts at the entrance on the ground floor, and both storeys are reachable.
    expect(Math.floor(conv.spec.spawn[2] / conv.spec.floorHeight)).toBe(0);
    expect(isFullyReachable(conv.spec, computeFlightMap(conv.spec))).toBe(true);
  });

  it('give up on a storey no staircase reaches', () => {
    expect(floorsToBuilding([storey(true, true), storey(false)], 1)).toBeNull();
  });
});

describe('real homes (Swiss Dwellings)', () => {
  const set = parseHomeSet(JSON.parse(readFileSync(HOME_SET_PATH, 'utf8')));

  it('ship 20 dev and 60 eval homes of two or three storeys, with the dataset license', () => {
    expect(set.dataset.license).toBe('CC BY 4.0');
    expect(set.homes.filter((h) => h.split === 'dev').length).toBe(20);
    expect(set.homes.filter((h) => h.split === 'eval').length).toBe(60);
    for (const h of set.homes) {
      expect(h.storeys.length).toBeGreaterThanOrEqual(2);
      expect(h.storeys.length).toBeLessThanOrEqual(3);
      expect(h.storeys.every((s) => s.stairs.length > 0)).toBe(true);
    }
  });

  it('every home converts into a building whose every storey is reachable through a shaft', () => {
    for (const h of set.homes) {
      const conv = homeBuilding(h);
      expect(conv, h.id).not.toBeNull();
      const spec = conv!.spec;
      expect(spec.floors.length).toBe(h.storeys.length);
      expect(spec.floors.every((f) => f.rooms.length > 0)).toBe(true);
      for (let k = 1; k < spec.floors.length; k++) {
        const a = spec.floors[k - 1]!;
        const b = spec.floors[k]!;
        expect(a.cells.some((c, i) => c === CELL.SHAFT && b.cells[i] === CELL.SHAFT), `${h.id} storeys ${k - 1}-${k}`).toBe(true);
      }
    }
  });

  it('the planner flies a real home and climbs its stairs', async () => {
    const home = set.homes.find((h) => h.split === 'dev')!;
    const { spec, envSeed, cfg } = homeEpisodeSetup(set, home, 'day');
    cfg.maxSteps *= spec.floors.length;
    const env = await createEnv();
    const pilot = BASELINES.planner!();
    pilot.reset({ speedH: 2, maxSteps: cfg.maxSteps, dynamics: 'kinematic' });
    let obs = env.reset(spec, envSeed, cfg);
    const floors = new Set<number>();
    let info = { coverage: 0, collisions: 0 };
    for (let t = 0; t < cfg.maxSteps; t++) {
      const s = env.step(pilot.act(obs, sensorFrame(env, obs)));
      obs = s.obs;
      info = s.info;
      floors.add(s.info.floor);
      if (s.done) break;
    }
    env.dispose();
    expect(floors.size).toBe(spec.floors.length);
    expect(info.coverage).toBeGreaterThan(0.6);
    expect(info.collisions).toBe(0);
  }, 60_000);
});

describe('shaft search', () => {
  it('takes NBVP, which maps a floor from its doorways, up the stairs it never flew under', async () => {
    // FrontierBench dev 81405 (two floors): without the search NBVP finished the ground floor and hovered.
    const task = BENCH_TASKS.find((t) => t.id === 'floors')!;
    const { spec, envSeed, cfg } = benchEpisodeSetup(task, { seed: 81405 });
    const env = await createEnv();
    const pilot = BASELINES.nbvp!();
    pilot.reset({ speedH: task.speedH, maxSteps: cfg.maxSteps, dynamics: task.dynamics });
    let obs = env.reset(spec, envSeed, cfg);
    const floors = new Set<number>();
    for (let t = 0; t < cfg.maxSteps; t++) {
      const s = env.step(pilot.act(obs, sensorFrame(env, obs)));
      obs = s.obs;
      floors.add(s.info.floor);
      if (s.done) break;
    }
    env.dispose();
    expect(floors.size).toBe(spec.floors.length);
  }, 60_000);
});

describe('held-out real homes', () => {
  it('are further eval homes from other buildings, none in the main set, and all convert', () => {
    const main = parseHomeSet(JSON.parse(readFileSync(HOME_SET_PATH, 'utf8')));
    const holdout = parseHomeSet(JSON.parse(readFileSync(HOME_HOLDOUT_PATH, 'utf8')));
    expect(holdout.homes.length).toBeGreaterThanOrEqual(10);
    const buildings = new Set(main.homes.map((h) => h.source.building));
    for (const h of holdout.homes) {
      expect(h.split).toBe('eval');
      expect(buildings.has(h.source.building), h.id).toBe(false);
      expect(homeBuilding(h), h.id).not.toBeNull();
    }
  });
});
