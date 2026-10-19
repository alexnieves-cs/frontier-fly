import type { PersonState } from './types';

/**
 * Rigged humanoid packed from KayKit's CC0 Rig_Medium mannequin and clips.
 * See CREDITS.md and public/assets/targets/LICENSE-KayKit.txt.
 */
export const HUMANOID = {
  // Relative to the deployed base so the file also loads under the GitHub Pages subpath.
  url: `${import.meta.env.BASE_URL}assets/targets/humanoid.glb`,
  license: 'CC0-1.0',
  author: 'Kay Lousberg (KayKit)',
  /** Unscaled mesh height in the glTF file, meters. */
  sourceHeight: 2.204,
  targetHeight: 1.75,
  clips: {
    standing: 'Idle_A',
    sitting: 'Sit_Chair_Idle',
    lying: 'Lie_Idle',
    walking: 'Walking_A',
    hiding: 'Sit_Floor_Idle',
  } satisfies Record<PersonState, string>,
} as const;

export const HUMANOID_SCALE = HUMANOID.targetHeight / HUMANOID.sourceHeight;

export const CLIP_NAMES = Object.values(HUMANOID.clips);
