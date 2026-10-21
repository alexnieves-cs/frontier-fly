import { ACT_DIM, OBS_DIM } from '../../shared/constants';
import type { BuildingSpec } from '../../shared/types';
import { createEnv, type ExplorerEnv } from '../../sim/env';
import { endReason, pilotAct, raceEpisode, type Act, type EndReason, type PolicyFn, type RacePilot, type RaceSetup } from './pilots';

export interface Lane {
  pilot: RacePilot;
  env: ExplorerEnv;
  act: Act;
  obs: Float32Array;
  action: Float32Array;
  coverage: number;
  collisions: number;
  steps: number;
  floor: number;
  done: boolean;
  reason: EndReason;
}

/**
 * Every lane flies its own env on the same building, seed and episode config,
 * one policy step at a time in lockstep. Nothing is shared between lanes, so
 * each lane's flight is the one it would fly alone.
 */
export class RaceEngine {
  readonly lanes: Lane[] = [];
  spec!: BuildingSpec;
  maxSteps = 0;
  maxCollisions = 0;
  step = 0;

  constructor(
    readonly setup: RaceSetup,
    private readonly policy: PolicyFn | null,
  ) {}

  async init(): Promise<this> {
    const { spec, envSeed, cfg } = raceEpisode(this.setup);
    this.spec = spec;
    this.maxSteps = cfg.maxSteps;
    this.maxCollisions = cfg.maxCollisions ?? 3;
    for (const pilot of this.setup.pilots) {
      const env = await createEnv();
      const act = pilotAct(pilot, spec, this.policy);
      const obs = env.reset(spec, envSeed, cfg).slice();
      this.lanes.push({ pilot, env, act, obs, action: new Float32Array(ACT_DIM), coverage: 0, collisions: 0, steps: 0, floor: 0, done: false, reason: 'running' });
    }
    return this;
  }

  get finished(): boolean {
    return this.lanes.every((l) => l.done);
  }

  /** One policy step (50 ms of sim time) for every lane still flying. */
  advance(): void {
    if (this.finished) return;
    this.step += 1;
    for (const lane of this.lanes) {
      if (lane.done) continue;
      const action = lane.act(lane.obs, lane.env);
      lane.action.set(action.subarray(0, ACT_DIM));
      const r = lane.env.step(action);
      lane.obs.set(r.obs.subarray(0, OBS_DIM));
      lane.coverage = r.info.coverage;
      lane.collisions = r.info.collisions;
      lane.floor = r.info.floor;
      lane.steps += 1;
      if (r.done || lane.steps >= this.maxSteps) {
        lane.done = true;
        lane.reason = endReason(true, lane.steps, this.maxSteps, lane.coverage, r.truncated, lane.collisions, this.maxCollisions);
      }
    }
  }

  dispose(): void {
    for (const lane of this.lanes) lane.env.dispose();
    this.lanes.length = 0;
  }
}
