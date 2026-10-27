import { describe, expect, it } from 'vitest';
import { ceUpdate, chiSquareQuantile, importanceRate, monteCarloRate } from '../../src/eval/atlas/estimate';
import { collisionScore, falseClearScore, flyCase, type AtlasFlight } from '../../src/eval/atlas/flight';
import { BINS, caseSetup, DIM_INDEX, DIMS, decode, mixtureWeight, nominalProposal, sampleCase, sampleMixture, weight, type AtlasCase, type Proposal } from '../../src/eval/atlas/space';
import { buildMissionWorld } from '../../src/mission/autonomy/world';

const missionDim = DIM_INDEX.mission!;
const faultDim = DIM_INDEX.fault!;
const windDim = DIM_INDEX.windMean!;

/** A synthetic flight: fails when the mission is a sweep with someone inside and the wind bin is high. */
function fake(c: AtlasCase): AtlasFlight {
  const fails = c.k[missionDim] === 1 && c.k[windDim]! >= 8;
  return {
    case: c,
    falseClear: fails,
    collision: c.k[faultDim] === 6,
    score: { falseClear: fails ? 1 : c.k[windDim]! / BINS, collision: c.k[faultDim] === 6 ? 1 : 0 },
  } as AtlasFlight;
}

describe('atlas search space', () => {
  it('draws the same case for the same seed and index', () => {
    const p = nominalProposal();
    expect(sampleCase(p, 5000, 17)).toEqual(sampleCase(p, 5000, 17));
    expect(sampleCase(p, 5000, 17)).not.toEqual(sampleCase(p, 5000, 18));
  });

  it('gives nominal draws weight 1 and keeps continuous values inside their bin', () => {
    const p = nominalProposal();
    for (let i = 0; i < 50; i++) {
      const c = sampleCase(p, 7000, i);
      expect(weight(p, c)).toBeCloseTo(1, 12);
      DIMS.forEach((d, j) => {
        if (d.continuous) expect(Math.floor(c.u[j]! * BINS)).toBe(c.k[j]);
      });
    }
  });

  it('decodes into a buildable world with the target in an existing building', () => {
    for (let i = 0; i < 20; i++) {
      const c = sampleCase(nominalProposal(), 9000, i);
      const d = decode(c);
      expect(d.targetBuilding).toBeLessThan(d.buildings);
      expect(d.flashlightOn).toBe(d.timeOfDay < 6.5 || d.timeOfDay >= 19.5);
      expect(caseSetup(c).world.targets!.length).toBe(d.mission === 'sweep-empty' ? 0 : 1);
    }
  });

  it('plants targets by the placement seed without moving the layout', () => {
    const a = buildMissionWorld({ seed: 37, buildings: 2, targets: [{ classId: 'person', where: 'indoor', building: 1 }] });
    const same = buildMissionWorld({ seed: 37, placementSeed: 37, buildings: 2, targets: [{ classId: 'person', where: 'indoor', building: 1 }] });
    const moved = buildMissionWorld({ seed: 37, placementSeed: 4242, buildings: 2, targets: [{ classId: 'person', where: 'indoor', building: 1 }] });
    expect(same.buildings.map((b) => b.scene)).toEqual(a.buildings.map((b) => b.scene));
    expect(moved.buildings.map((b) => b.footprint)).toEqual(a.buildings.map((b) => b.footprint));
    expect(moved.buildings[1]!.scene.targets[0]!.feet).not.toEqual(a.buildings[1]!.scene.targets[0]!.feet);
  });
});

describe('atlas estimators', () => {
  it('scores reach 1 exactly on the event', () => {
    expect(falseClearScore(true, { conf: 0.9, confirmed: false, clearedShare: 1 })).toBe(1);
    expect(falseClearScore(false, null)).toBe(0);
    // Seen but unconfirmed, with other buildings cleared, is closer than never seen, which is closer than found.
    const seen = falseClearScore(false, { conf: 0.75, confirmed: false, clearedShare: 0.5 });
    const unseen = falseClearScore(false, { conf: 0, confirmed: false, clearedShare: 0 });
    const found = falseClearScore(false, { conf: 0.95, confirmed: true, clearedShare: 0 });
    expect(seen).toBeLessThan(1);
    expect(seen).toBeGreaterThan(unseen);
    expect(unseen).toBeGreaterThan(found);
    expect(collisionScore(true, 0.5)).toBe(1);
    expect(collisionScore(false, 0.05)).toBeGreaterThan(collisionScore(false, 0.5));
  });

  it('moves the proposal toward the failing region', () => {
    const p = nominalProposal();
    const flights = Array.from({ length: 2000 }, (_, i) => fake(sampleCase(p, 1, i)));
    const step = ceUpdate(p, flights, 'falseClear');
    expect(step.next[missionDim]![1]!).toBeGreaterThan(p[missionDim]![1]!);
    expect(step.next[windDim]![9]!).toBeGreaterThan(p[windDim]![9]!);
    // Dimensions the failures do not depend on keep their probabilities.
    expect(step.refit).toContain('mission');
    expect(step.refit).not.toContain('timeOfDay');
    expect(step.next[DIM_INDEX.timeOfDay!]).toEqual(p[DIM_INDEX.timeOfDay!]);
    for (const row of step.next) expect(row.reduce((a, b) => a + b, 0)).toBeCloseTo(1, 12);
  });

  it('importance sampling from a learned proposal agrees with the true rate', () => {
    // True rate under the nominal: P(sweep-occupied) * P(top two wind bins) = 0.25 * 0.2 = 0.05.
    let p: Proposal = nominalProposal();
    let next = 0;
    for (let g = 0; g < 3; g++) {
      const flights = Array.from({ length: 1000 }, () => fake(sampleCase(p, 2, next++)));
      p = ceUpdate(p, flights, 'falseClear').next;
    }
    const draws = Array.from({ length: 4000 }, (_, i) => fake(sampleMixture(p, 0.3, 3, i)));
    const is = importanceRate(draws, 'falseClear', p, 0.3);
    for (const f of draws) expect(mixtureWeight(p, 0.3, f.case)).toBeLessThanOrEqual(1 / 0.3 + 1e-9);
    expect(is.ci[0]).toBeLessThan(0.05);
    expect(is.ci[1]).toBeGreaterThan(0.05);
    const mc = monteCarloRate(Array.from({ length: 4000 }, (_, i) => fake(sampleCase(nominalProposal(), 4, i))), 'falseClear');
    // The learned proposal puts most draws in the failing region, so its interval is the narrower one.
    expect(is.ci[1] - is.ci[0]).toBeLessThan(mc.ci[1] - mc.ci[0]);
  });

  it('approximates chi-square quantiles', () => {
    // Table values at 0.999: df 2 -> 13.82, df 9 -> 27.88.
    expect(chiSquareQuantile(2, 3.09)).toBeCloseTo(13.8, 0);
    expect(chiSquareQuantile(9, 3.09)).toBeCloseTo(27.9, 0);
  });

  it('bounds the rate by 3/(alpha n) when no flight fails', () => {
    const q = nominalProposal();
    const draws = Array.from({ length: 300 }, (_, i) => ({ ...fake(sampleMixture(q, 0.3, 5, i)), falseClear: false }));
    const r = importanceRate(draws, 'falseClear', q, 0.3);
    expect(r.p).toBe(0);
    expect(r.ci[1]).toBeCloseTo(3 / (0.3 * 300), 12);
  });
});

describe('atlas flight', () => {
  it('flies a case and records the closest approach to a solid', async () => {
    const p = nominalProposal();
    // Pin a short mission: find a person in one building, no fault, calm air, daytime.
    const c = sampleCase(p, 11, 0);
    const pin = (id: string, k: number, u = 0) => {
      c.k[DIM_INDEX[id]!] = k;
      if (DIMS[DIM_INDEX[id]!]!.continuous) c.u[DIM_INDEX[id]!] = u;
    };
    pin('mission', 0);
    pin('buildings', 0);
    pin('fault', 0);
    pin('windMean', 0, 0);
    pin('timeOfDay', 5, 0.5);
    const f = await flyCase(c, 'sensor');
    expect(f.minClearanceM).toBeLessThan(5);
    expect(f.clearance).not.toBeNull();
    expect(f.score.collision).toBeGreaterThanOrEqual(0);
    expect(f.targetConf).not.toBeNull();
  });
});

describe('atlas replay links', () => {
  it('round-trips a case exactly and rebuilds its world', async () => {
    const { decodeCase, encodeCase, loadAtlasCase } = await import('../../src/eval/atlas/replay');
    const c = sampleCase(nominalProposal(), 200001, 42);
    const back = decodeCase(encodeCase(c));
    expect(back).toEqual(c);
    expect(decodeCase('nonsense')).toBeNull();
    const loaded = loadAtlasCase(back!);
    expect(loaded.world.buildings.length).toBe(decode(c).buildings);
    expect(loaded.options.vehicle).toBe('quad');
  });
});
