import { RNG } from '../shared/rng';
import type { LightingState, SensorMode, SkySample } from '../shared/types';

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

export function defaultLighting(nRooms: number): LightingState {
  return {
    timeOfDay: 12,
    interiorLightsOn: new Uint8Array(Math.max(0, nRooms)).fill(1),
    flashlightOn: false,
    sensorMode: 'rgb',
  };
}

/** Seeded lighting draw. Stages 0–2 stay at noon so early curriculum is stationary. */
export function sampleLighting(seed: number, nRooms: number, stage: number): LightingState {
  const rng = new RNG(seed >>> 0, 'light');
  const lights = new Uint8Array(Math.max(0, nRooms));
  if (stage < 3) {
    lights.fill(1);
    return { timeOfDay: 12, interiorLightsOn: lights, flashlightOn: false, sensorMode: 'rgb' };
  }
  const full = stage >= 5;
  const timeOfDay = full ? rng.range(0, 24) : rng.range(7, 19);
  const prob = full ? rng.range(0, 1) : rng.range(0.45, 1);
  for (let i = 0; i < lights.length; i++) lights[i] = rng.bool(prob) ? 1 : 0;
  const night = timeOfDay < 6 || timeOfDay >= 18;
  const flashlightOn = night ? rng.bool(full ? 0.8 : 0.5) : false;
  const modes: SensorMode[] = ['rgb', 'ir', 'depth'];
  const sensorMode = full ? modes[rng.int(3)]! : 'rgb';
  return { timeOfDay, interiorLightsOn: lights, flashlightOn, sensorMode };
}

export function skyFromTime(timeOfDay: number): SkySample {
  const hour = ((timeOfDay % 24) + 24) % 24;
  const elev = Math.sin(((hour - 6) / 12) * Math.PI);
  const night = clamp((0.15 - elev) / 0.55, 0, 1);
  const az = ((hour - 6) / 12) * Math.PI;
  const horiz = Math.cos(Math.max(elev, -0.15));
  let sx = Math.cos(az) * horiz;
  let sy = Math.sin(az) * horiz;
  let sz = Math.sin(Math.max(elev, -0.08));
  const len = Math.hypot(sx, sy, sz) || 1;
  sx /= len;
  sy /= len;
  sz /= len;
  const warm = clamp((0.28 - Math.abs(elev)) / 0.28, 0, 1);
  const day = 1 - night;
  return {
    sunDir: [sx, sy, sz],
    sunIntensity: Math.max(0, elev) * (1 - night) * 6.5,
    sunColor: [1, 0.93 - warm * 0.28, 0.82 - warm * 0.5],
    ambient: 0.035 + 0.42 * Math.max(0, elev),
    skyColor: [0.25 + 0.45 * day, 0.32 + 0.4 * day - warm * 0.08, 0.08 + 0.62 * day],
    night,
  };
}

export function isLowLight(lighting: LightingState): boolean {
  return skyFromTime(lighting.timeOfDay).night > 0.65 && !lighting.flashlightOn && lighting.sensorMode === 'rgb';
}

export function nightFactor(lighting: LightingState): number {
  return skyFromTime(lighting.timeOfDay).night;
}
