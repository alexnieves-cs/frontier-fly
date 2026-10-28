import type { LiveDetectionSource, MissionRunner, MissionWorld, RunnerOptions } from '../../mission/autonomy/contracts';
import { createPerceptionSource } from '../../mission/autonomy/perception';
import { policyFromFile, type NormalizedPolicy } from '../../mission/autonomy/policy';
import { createMissionRunner } from '../../mission/autonomy/runner';
import { buildMissionWorld } from '../../mission/autonomy/world';
import type { Mission } from '../../mission/types';
import { SHIPPED_POLICY_URL } from '../../rl/checkpoint';

/** Control tick the page steps the runner at, the same as the demo and the benches. */
export const FLIGHT_DT = 0.05;

/** A planned mission flown by the autonomy runner the demo, films and benchmarks fly. */
export interface Flight {
  runner: MissionRunner;
  world: MissionWorld;
  source: LiveDetectionSource;
  indoor: 'policy' | 'frontier';
  /** Set when the planned start is not where the drone takes off. */
  launchNote: string | null;
}

let shipped: Promise<NormalizedPolicy | null> | null = null;

/** The shipped checkpoint, or null when it cannot be fetched. */
function shippedPolicy(): Promise<NormalizedPolicy | null> {
  shipped ??= fetch(SHIPPED_POLICY_URL)
    .then(async (response) => (response.ok ? policyFromFile(await response.arrayBuffer()) : null))
    .catch(() => null);
  return shipped;
}

/**
 * Build the seed's world and fly `mission` on it. Entry requests wait for the
 * page (approve or deny); indoors the shipped policy flies first, as on the demo.
 */
export async function prepareFlight(mission: Mission): Promise<Flight> {
  const world = buildMissionWorld({ seed: mission.seed >>> 0 });
  const source = createPerceptionSource(world);
  const policy = await shippedPolicy();
  const options: RunnerOptions = {
    dt: FLIGHT_DT,
    operator: () => 'pending',
    indoor: policy ? 'auto' : 'frontier',
    policy: policy ?? undefined,
  };
  const runner = await createMissionRunner(mission, world, source, options);
  const [hx, hy] = world.home;
  const off = Math.hypot(mission.startPose.x - hx, mission.startPose.y - hy);
  // The generator clears the trees around its pad only, so the drone takes off there.
  const launchNote =
    off > 1
      ? `Takes off from the site's cleared pad at E ${hx.toFixed(1)} m, N ${hy.toFixed(1)} m, ${off.toFixed(0)} m from the planned start. The exported plan keeps your start.`
      : null;
  return { runner, world, source, indoor: policy ? 'policy' : 'frontier', launchNote };
}

/** Building label for an id, or the id itself. */
export function buildingLabel(world: MissionWorld, id: string | null): string | null {
  if (!id) return null;
  return world.buildings.find((building) => building.id === id)?.label ?? id;
}
