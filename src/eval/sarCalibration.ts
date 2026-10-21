// How well the planner's POD model predicts the camera detector. A person is planted on open ground,
// the gimbal camera looks from a random height, offset and speed, and the model's per-frame POD is
// compared with whether the detector reported the person. Development seeds only.
import { createPerceptionSource } from '../mission/autonomy/perception';
import { cellIndex, buildSearchGrid, sectorOf, skyVisible } from '../mission/autonomy/sar/grid';
import { plantPersonAt } from '../mission/autonomy/sar/lostPerson';
import { framePod, PERSON_CENTER_M } from '../mission/autonomy/sar/pod';
import { buildMissionWorld } from '../mission/autonomy/world';
import { RNG } from '../shared/rng';
import { sampleHeight } from '../world/outdoor/terrain';

export interface PodCalibration {
  frames: number;
  meanPredicted: number;
  meanObserved: number;
  brier: number;
  bins: { lo: number; hi: number; n: number; predicted: number; observed: number }[];
}

export function calibratePod(seedBase: number, worlds: number, peoplePerWorld = 30, framesPerPerson = 30): PodCalibration {
  const edges = [0, 0.02, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0001];
  const bins = edges.slice(0, -1).map((lo, i) => ({ lo, hi: Math.min(1, edges[i + 1]!), n: 0, predicted: 0, observed: 0 }));
  let frames = 0;
  let sumP = 0;
  let sumH = 0;
  let brier = 0;
  const sensor = { timeOfDay: 13, sensorMode: 'rgb' as const, flashlightOn: false };
  for (let w = 0; w < worlds; w++) {
    const seed = seedBase + w;
    const rng = new RNG(seed, 'pod-calibration');
    const base = buildMissionWorld({ seed, timeOfDay: 13, targets: [] });
    const grid = buildSearchGrid(base);
    for (let t = 0; t < peoplePerWorld; t++) {
      const k0 = grid.cells[rng.int(grid.cells.length)]!;
      const xy: [number, number] = [grid.x0 + ((k0 % grid.nx) + 0.5) * grid.cell, grid.y0 + (Math.floor(k0 / grid.nx) + 0.5) * grid.cell];
      const state = rng.pick(['lying', 'sitting', 'standing'] as const);
      const person = plantPersonAt(base, xy, state, seed * 1000 + t);
      if (!person) continue;
      const world = { ...base, outdoorScene: { ...base.outdoorScene, targets: [person] } };
      const source = createPerceptionSource(world);
      const k = cellIndex(grid, person.feet[0], person.feet[1]);
      for (let f = 0; f < framesPerPerson; f++) {
        const agl = rng.range(9, 18);
        const yaw = rng.range(-Math.PI, Math.PI);
        const along = rng.range(-12, 12);
        const across = rng.range(-16, 16);
        const x = person.feet[0] - (Math.cos(yaw) * along - Math.sin(yaw) * across);
        const y = person.feet[1] - (Math.sin(yaw) * along + Math.cos(yaw) * across);
        const z = sampleHeight(base.outdoor.terrain, x, y) + agl;
        const speed = rng.range(0, 5.5);
        const obs = source.observe({ pos: [x, y, z], yaw, vel: [speed, 0, 0] }, f * 0.2);
        const hit = obs.contacts.some((c) => c.classId === 'person') ? 1 : 0;
        const h = z - 0.08 - (person.feet[2] + PERSON_CENTER_M);
        const flat = Math.hypot(along, across);
        const seen = k < 0 || skyVisible(grid, k, sectorOf(Math.atan2(y - person.feet[1], x - person.feet[0])), h / Math.max(flat, 0.01));
        const p = framePod(h, along, across, speed, seen ? 0 : 1, sensor);
        const bin = bins.find((b) => p >= b.lo && p < b.hi) ?? bins[bins.length - 1]!;
        bin.n += 1;
        bin.predicted += p;
        bin.observed += hit;
        frames += 1;
        sumP += p;
        sumH += hit;
        brier += (p - hit) ** 2;
      }
    }
  }
  const r = (v: number) => Math.round(v * 1e4) / 1e4;
  return {
    frames,
    meanPredicted: r(sumP / frames),
    meanObserved: r(sumH / frames),
    brier: r(brier / frames),
    bins: bins.map((b) => ({ lo: b.lo, hi: r(b.hi), n: b.n, predicted: b.n ? r(b.predicted / b.n) : 0, observed: b.n ? r(b.observed / b.n) : 0 })),
  };
}
