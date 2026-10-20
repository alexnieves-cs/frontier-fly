import type { EpisodeConfig, LightingState } from '../shared/types';
import { defaultEpisodeConfig, policyEpisodeLimits } from '../sim/env';

/** Building size per curriculum stage for the sim page's own episodes. */
export const STAGE_SIZE: Record<number, { floors: number; w: number; d: number }> = {
  0: { floors: 1, w: 8, d: 8 },
  1: { floors: 1, w: 10, d: 10 },
  2: { floors: 1, w: 16, d: 16 },
  3: { floors: 1, w: 22, d: 16 },
  4: { floors: 2, w: 18, d: 16 },
  5: { floors: 2, w: 18, d: 16 },
};

/**
 * Episode config for a sim-page episode: the stage's defaults, quadrotor
 * dynamics from stage 5, and the same step limits as training and eval, so
 * OBS.time reads the same when the policy flies.
 */
export function sessionEpisodeConfig(stage: number, nRooms: number, lighting: LightingState): EpisodeConfig {
  const cfg = defaultEpisodeConfig(stage, nRooms);
  cfg.dynamics = stage >= 5 ? 'quad' : 'kinematic';
  Object.assign(cfg, policyEpisodeLimits(stage));
  cfg.lighting = lighting;
  return cfg;
}
