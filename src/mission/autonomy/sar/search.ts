import type { Airspace } from '../planner/geometry';
import { buildingBox } from '../planner/geometry';
import type { Contact, DronePose, MissionWorld, OutdoorSearchOptions } from '../contracts';
import { GIMBAL_MIN_AGL_M } from '../perception';
import { sampleHeight } from '../../../world/outdoor/terrain';
import { SearchBelief } from './belief';
import { buildSearchGrid, type SearchGrid } from './grid';
import { lostPersonPrior, type LostPersonPrior } from './lostPerson';
import { ProbabilityPlanner, type BuildingSegment, type SearchDecision } from './planner';
import type { SensorConditions } from './pod';

/** One sample of the belief over the flight, for the report's calibration curve. */
export interface PosSample {
  tSec: number;
  /** Cumulative probability of success the model claims. */
  pos: number;
}

/**
 * Probability-of-area search state for one mission: the grid, the prior
 * from the last known point, the belief the camera frames update, and the
 * planner that reads it. The runner owns flying; this owns deciding where.
 */
export class SarSearch {
  readonly grid: SearchGrid;
  readonly prior: LostPersonPrior;
  readonly belief: SearchBelief;
  readonly planner: ProbabilityPlanner;
  readonly segments: BuildingSegment[];
  readonly visited = new Set<string>();
  readonly history: PosSample[] = [];
  replans = 0;
  private nextSample = 0;

  constructor(
    readonly world: MissionWorld,
    air: Airspace,
    agl: number,
    readonly options: OutdoorSearchOptions,
  ) {
    const lkp = options.lastKnown ?? [world.home[0], world.home[1]];
    const sensor: SensorConditions = { timeOfDay: world.timeOfDay, sensorMode: world.sensorMode, flashlightOn: world.flashlightOn };
    this.grid = buildSearchGrid(world);
    this.prior = lostPersonPrior(world, this.grid, lkp, { shape: options.prior ?? 'behavior' });
    this.belief = new SearchBelief(this.grid, this.prior, sensor);
    this.planner = new ProbabilityPlanner(this.belief, air, sensor, { agl, ...options.planner });
    this.segments = world.buildings.map((b) => {
      const box = buildingBox(b);
      return { id: b.id, center: [box.cx, box.cy] as [number, number] };
    });
  }

  /** Fold one camera frame into the belief. Only gimbal frames from outside count. */
  observe(pose: DronePose, contacts: readonly Contact[], tSec: number): void {
    const [x, y, z] = pose.pos;
    const agl = z - sampleHeight(this.world.outdoor.terrain, x, y);
    if (agl >= GIMBAL_MIN_AGL_M) {
      const near = contacts.filter((c) => c.classId === 'person' && c.buildingId === null).map((c) => [c.estimate[0], c.estimate[1]] as [number, number]);
      const speed = Math.hypot(pose.vel[0], pose.vel[1], pose.vel[2]);
      this.belief.observeFrame(x, y, z - 0.08, pose.yaw, speed, near);
    }
    if (tSec >= this.nextSample) {
      this.nextSample = tSec + 5;
      this.history.push({ tSec: Math.round(tSec), pos: Math.round(this.belief.pos() * 1e4) / 1e4 });
    }
  }

  plan(pos: [number, number], heading: number): SearchDecision {
    this.replans += 1;
    return this.planner.plan(pos, heading, this.segments, this.visited);
  }

  /** A building the planner chose: its mass is searched by the indoor POD. */
  visit(id: string): void {
    this.visited.add(id);
    this.belief.observeStructure(id, this.planner.cfg.structurePod);
  }
}
