// Two holes a 2026-11-09 audit found by flying them: an operator abort that a battery-swap return
// overrode (the drone took off again), and a GNSS loss during the final landing that nothing landed.
import { describe, expect, it } from 'vitest';
import { createMissionRunner, runMission } from '../../src/mission/autonomy/runner';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import { createFakeSource } from '../helpers/fakeSource';

describe('mission endings', () => {
  it('an operator abort during a battery-swap return ends the mission on the pad', async () => {
    const world = buildMissionWorld({ seed: 37, buildings: 2, timeOfDay: 10.5, interiorLights: 'on', targets: [], decoys: 2 });
    const mission = missionForWorld(world, {
      missionType: 'safety_sweep',
      targets: [{ id: 'tgt-clear', kind: 'clearance', label: 'Clear', minCount: 1, minConfidence: null }],
      requiredConfidence: 0.8,
      entryPolicy: 'auto',
      budgets: { batteryPct: 85, timeSec: 1200 },
    });
    const runner = await createMissionRunner(mission, world, await createFakeSource(world), { dt: 0.05 });
    let abortedAt = -1;
    for (let i = 0; i < 40000 && !runner.done; i++) {
      runner.step();
      if (abortedAt < 0 && runner.snapshot().log.some((l) => l.message.includes('to swap the battery'))) {
        runner.abort();
        abortedAt = runner.snapshot().tSec;
      }
    }
    const r = runner.record();
    expect(abortedAt).toBeGreaterThan(0);
    expect(runner.done).toBe(true);
    expect(r.outcome).toBe('aborted');
    expect(r.landedHome).toBe(true);
    const after = r.log.filter((l) => l.tSec >= abortedAt).map((l) => l.message);
    expect(after.some((m) => m.includes('Battery swap') || m.includes('Taking off'))).toBe(false);
  }, 240_000);

  it('a GNSS fix that never returns over the pad ends in a landing, not a hover to the time limit', async () => {
    const world = buildMissionWorld({ seed: 20, buildings: 1, timeOfDay: 13, interiorLights: 'on', targets: [{ classId: 'person', state: 'lying', where: 'indoor', building: 0 }], decoys: 1 });
    const mission = missionForWorld(world, {
      missionType: 'find_person',
      targets: [{ id: 'tgt-person', kind: 'person', label: 'Person', minCount: 1, minConfidence: null }],
      requiredConfidence: 0.8,
      entryPolicy: 'auto',
      budgets: { batteryPct: 80, timeSec: 900 },
    });
    const r = await runMission(mission, world, await createFakeSource(world), {
      dt: 0.05,
      faults: { plan: [{ id: 'gnss', kind: 'gnss_loss', trigger: { phase: 'land', afterSec: 1 }, durationSec: Infinity, driftMps: 0.15 }] },
    });
    // Before the fix this flew until the 900 s limit, drifting, and logged a contact on the way.
    expect(r.tSec).toBeLessThan(400);
    expect(r.track[r.track.length - 1]!.phase).toBe('done');
    expect(r.log.some((l) => l.message.includes('no GNSS fix for 60 s over the pad'))).toBe(true);
  }, 240_000);
});
