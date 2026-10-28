// @vitest-environment node
import { describe, expect, it } from 'vitest';
import { buildReport } from '../../src/mission/autonomy/report';
import { woodsSampleMission } from '../../src/mission/outdoorWorld';
import { FLIGHT_DT, prepareFlight } from '../../src/ui/mission/flight';

describe('planned missions fly the autonomy runner', () => {
  it('flies the woods sample, waits for the operator at an entry, and reports', async () => {
    const mission = woodsSampleMission(11);
    expect(mission.entryPolicy).toBe('ask');
    const flight = await prepareFlight(mission);
    const runner = flight.runner;
    let guard = 0;
    while (!runner.done && !runner.snapshot().pendingEntry && guard++ < 40_000) runner.step(FLIGHT_DT);
    const waiting = runner.snapshot();
    expect(waiting.pendingEntry).not.toBeNull();
    expect(waiting.track.length).toBeGreaterThan(2);
    runner.deny();
    while (!runner.done && guard++ < 40_000) {
      if (runner.snapshot().pendingEntry) runner.deny();
      runner.step(FLIGHT_DT);
    }
    expect(runner.done).toBe(true);
    const record = runner.record();
    expect(record.entries.length).toBeGreaterThan(0);
    expect(record.entries.every((entry) => entry.decision === 'denied')).toBe(true);
    expect(record.landedHome).toBe(true);
    const report = buildReport(record, flight.source, flight.world);
    expect(report.missionName).toBe(mission.name);
    expect(report.collisions).toBe(0);
  }, 120_000);

  it('notes when the planned start is away from the cleared pad', async () => {
    const mission = woodsSampleMission(11);
    expect((await prepareFlight(mission)).launchNote).toBeNull();
    const moved = { ...mission, startPose: { ...mission.startPose, x: mission.startPose.x + 10 } };
    expect((await prepareFlight(moved)).launchNote).toMatch(/cleared pad/);
  }, 60_000);
});
