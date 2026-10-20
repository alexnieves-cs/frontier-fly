import type { LightingState } from '../shared/types';

/** Formatting for the sim page's readouts and HUD. */

export function copyLight(light: LightingState): LightingState {
  return {
    timeOfDay: light.timeOfDay,
    interiorLightsOn: new Uint8Array(light.interiorLightsOn),
    flashlightOn: light.flashlightOn,
    sensorMode: light.sensorMode,
  };
}

export function battery(t: number): number {
  return Math.max(8, Math.round(100 - t * 1.5));
}

export function modeLabel(light: LightingState): string {
  if (light.sensorMode === 'ir') return 'IR';
  if (light.sensorMode === 'depth') return 'DEPTH';
  const night = light.timeOfDay < 6 || light.timeOfDay >= 18;
  if (!night) return 'DAY';
  return light.flashlightOn ? 'NIGHT' : 'DARK';
}

export function formatClock(hour: number): string {
  const h = Math.floor(hour) % 24;
  const m = Math.floor((hour % 1) * 60);
  return `${String(h).padStart(2, '0')}:${String(m).padStart(2, '0')}`;
}
