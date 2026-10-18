import type { QualityPreset } from '../shared/types';

export type QualityChoice = QualityPreset | 'auto';

/** High tries WebGPU and keeps the full post stack. Low stays on WebGL2 without SSAO. */
export function resolveQuality(choice: QualityChoice, hasWebGPU: boolean): QualityPreset {
  if (choice === 'low') return 'low';
  if (choice === 'high') return 'high';
  return hasWebGPU ? 'high' : 'low';
}
