import { describe, expect, it } from 'vitest';
import { sandboxedPilot } from '../../scripts/bench/sandbox';
import type { SensorFrame } from '../../src/sim/sensorPlanner';

const frame = (): SensorFrame => ({
  seen: new Uint8Array([0, 0, 0, 7, 0, 0, 0, 0]),
  visited: new Uint8Array(8),
  vw: 2,
  vd: 2,
  floors: 2,
  floorHeight: 3,
  pos: [1, 2, 0.5],
  yaw: 0.25,
  lidar: new Float64Array([4, 0.125, 4]),
  alt: 1.5,
  vel: [0.5, 0],
});

describe('bench pilot sandbox', () => {
  it('hands the pilot the step and nothing of the scorer', async () => {
    (globalThis as { __scorerSecret?: number }).__scorerSecret = 1;
    process.env.PILOT_TEST_SECRET = 'x';
    const pilot = await sandboxedPilot('tests/fixtures/pilots/echo.ts');
    try {
      expect(pilot.name).toBe('echo');
      pilot.reset({ speedH: 1.5, maxSteps: 10, dynamics: 'quad' });
      const a = pilot.act(new Float32Array([0.25, 9]), frame());
      // No global of the scorer, no environment variable; the step itself arrives intact.
      expect(Array.from(a)).toEqual([0, 0, 0.25 + 0.5 + 7 + 0.125, 1.5]);
    } finally {
      delete process.env.PILOT_TEST_SECRET;
      await pilot.close();
    }
  }, 60_000);

  it('fails the run when the pilot throws', async () => {
    const pilot = await sandboxedPilot('tests/fixtures/pilots/throws.ts');
    pilot.reset({ speedH: 1, maxSteps: 10, dynamics: 'kinematic' });
    expect(() => pilot.act(new Float32Array(2), frame())).toThrow(/pilot blew up/);
    expect(() => pilot.act(new Float32Array(2), frame())).toThrow();
    await pilot.close();
  }, 60_000);

  it('fails the run when the pilot hangs', async () => {
    const pilot = await sandboxedPilot('tests/fixtures/pilots/hangs.ts', { stepMs: 500 });
    pilot.reset({ speedH: 1, maxSteps: 10, dynamics: 'kinematic' });
    expect(() => pilot.act(new Float32Array(2), frame())).toThrow(/longer than/);
    await pilot.close();
  }, 60_000);

  it('reports a module that is not a pilot', async () => {
    await expect(sandboxedPilot('tests/fixtures/pilots/missing.ts')).rejects.toThrow();
  }, 60_000);
});
