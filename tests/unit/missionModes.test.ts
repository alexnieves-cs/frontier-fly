import { describe, expect, it } from 'vitest';
import type { ActingPolicy } from '../../src/mission/autonomy/planner/indoor';
import { MISSION_MODES, missionModeOptions, parseMissionModes } from '../../src/eval/missions';

describe('mission bench modes', () => {
  const policy = {} as ActingPolicy;

  it('give each mode the runner options it always had', () => {
    const got = Object.fromEntries(MISSION_MODES.map((m) => [m, missionModeOptions(m, policy)]));
    expect(got).toEqual({
      policy: { indoor: 'auto', policy },
      sensor: { indoor: 'frontier' },
      oracle: { indoor: 'oracle' },
      policyQuad: { indoor: 'auto', policy, vehicle: 'quad' },
      sensorQuad: { indoor: 'frontier', vehicle: 'quad' },
      policyLoc: { indoor: 'auto', policy, localization: 'estimated' },
      sensorLoc: { indoor: 'frontier', localization: 'estimated' },
      sensorQuadLoc: { indoor: 'frontier', vehicle: 'quad', localization: 'estimated' },
    });
  });

  it('reject an unknown mode instead of flying a default', () => {
    expect(parseMissionModes('sensor, sensorLoc')).toEqual(['sensor', 'sensorLoc']);
    expect(() => parseMissionModes('sensor,sensorquad')).toThrow(/sensorquad/);
  });
});
