import { OBS } from '../shared/constants';

/**
 * Observation ablations for the research write-up (docs/REPORT.md). Each one
 * zeroes a block of the observation in every env of the process, so training
 * (worker threads inherit the environment) and evaluation see the same input.
 * Selected with FRONTIER_OBS_ABLATION in Node; never set in the browser.
 */
export const OBS_ABLATIONS = {
  /** The navigation block: waypoint and frontier features from the sensor-only map. */
  'no-nav': [[OBS.nav, OBS.localVisited]],
  /** The local egocentric visited and frontier maps. */
  'no-local': [[OBS.localVisited, OBS.coarseVisited]],
  /** The coarse egocentric visited and frontier maps. */
  'no-coarse': [[OBS.coarseVisited, OBS.end]],
} as const satisfies Record<string, ReadonlyArray<readonly [number, number]>>;

export type ObsAblation = keyof typeof OBS_ABLATIONS;

function fromEnvironment(): ReadonlyArray<readonly [number, number]> {
  const name = (globalThis as { process?: { env?: Record<string, string | undefined> } }).process?.env?.FRONTIER_OBS_ABLATION;
  if (!name) return [];
  const ranges = (OBS_ABLATIONS as Record<string, ReadonlyArray<readonly [number, number]>>)[name];
  if (!ranges) throw new Error(`Unknown FRONTIER_OBS_ABLATION "${name}". Known: ${Object.keys(OBS_ABLATIONS).join(', ')}.`);
  return ranges;
}

const ACTIVE = fromEnvironment();

/** Zero the ablated blocks of an observation in place. A no-op unless an ablation is selected. */
export function applyObsAblation(obs: Float32Array): void {
  for (const [a, b] of ACTIVE) obs.fill(0, a, b);
}

