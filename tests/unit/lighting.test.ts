import { describe, expect, it } from 'vitest';
import { isLowLight, lightingPreset, LIGHTING_PRESETS, sampleLighting, skyFromTime } from '../../src/sim/lighting';

describe('lighting', () => {
  it('is deterministic per seed', () => {
    const a = sampleLighting(99, 4, 5);
    const b = sampleLighting(99, 4, 5);
    expect(a.timeOfDay).toBe(b.timeOfDay);
    expect(Array.from(a.interiorLightsOn)).toEqual(Array.from(b.interiorLightsOn));
    expect(a.flashlightOn).toBe(b.flashlightOn);
    expect(a.sensorMode).toBe(b.sensorMode);
  });

  it('keeps early stages at noon', () => {
    const light = sampleLighting(1, 2, 1);
    expect(light.timeOfDay).toBe(12);
    expect(light.flashlightOn).toBe(false);
  });

  it('covers the eval presets', () => {
    expect(LIGHTING_PRESETS).toEqual(['day', 'dusk', 'night-lit', 'night-dark', 'ir']);
    const dark = lightingPreset('night-dark', 3);
    expect(dark.flashlightOn).toBe(true);
    expect(dark.interiorLightsOn.every((v) => v === 0)).toBe(true);
    expect(skyFromTime(12).night).toBeLessThan(skyFromTime(23).night);
    expect(isLowLight(lightingPreset('day', 1))).toBe(false);
  });
});
